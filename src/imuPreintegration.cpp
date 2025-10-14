// IMU预积分与激光雷达融合的程序

#include "utility.h"


// gtsam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

//gtsam 
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>


// 位置 速度和偏置的符号定义
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class TransformFusion : public ParamServer
{
public:
    std::mutex mtx; // 线程锁

    ros::Subscriber subImuOdometry; // IMU里程计订阅者
    ros::Subscriber subLaserOdometry; // 激光雷达里程计订阅者

    ros::Publisher pubImuOdometry; // IMU里程计发布者
    ros::Publisher pubImuPath; // IMU轨迹发布者

    Eigen::Affine3f lidarOdomAffine; 
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;

    double lidarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;

    TransformFusion()
    { 
        // TransformFusion类的构造函数
        // 获取Lidar与base_link之间的变换关系
        if(lidarFrame != baselinkFrame) // 如果不是同一个坐标系，则需要获取变换关系
        {
            try
            {
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0)); // 等待变换关系建立，最多等3秒
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink); // 获取变换关系
            }
            catch (tf::TransformException ex) // 如果获取失败，则抛出异常
            {
                ROS_ERROR("%s",ex.what());
            }
        }

        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay()); // 订阅激光雷达里程计话题
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay()); // 订阅IMU里程计话题（增量）

        pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000); // 发布IMU里程计话题
        pubImuPath       = nh.advertise<nav_msgs::Path>    ("lio_sam/imu/path", 1); // 发布IMU轨迹话题
    }

    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {   
        // 使用Affine3f表示刚体空间变换矩阵
        // 读取里程计消息作为刚体变换矩阵，并返回该刚体变换矩阵
        double x, y, z, roll, pitch, yaw; // 定义刚体变换的元素

        // 从odom中读取刚体的位置和姿态信息，并转换为刚体变换矩阵的元素
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // 返回刚体变换矩阵
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {   
        // 激光雷达里程计处理回调函数

        // 线程锁
        std::lock_guard<std::mutex> lock(mtx);
        // 将ros的odom信息转换为刚体变换矩阵，并保存到lidarOdomAffine中
        lidarOdomAffine = odom2affine(*odomMsg);
        // 记录odom的时间信息
        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // IMU里程计处理回调函数
        // 将高频但存在漂移的 IMU 里程计数据，与低频但高精度的激光雷达位姿对齐融合，输出高频、高精度的融合位姿
        
        
        // static tf
        // 设置odom坐标系和map坐标系为一个坐标系
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        // thread lock
        std::lock_guard<std::mutex> lock(mtx);
        // 将imu的里程计放入到imuOdomQueue队列中
        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        // 获得距离激光雷达里程计最近的odom_imu的信息
        // 通过不断将小于激光雷达里程计的imu的信息pop out，直到队列中第一个odom_imu的时间戳大于激光雷达里程计的时间戳，
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }


        // 计算激光雷达时刻与当前时刻的IMU的相对运动增量，并且融合到激光雷达的位姿上面
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front()); // 激光雷达时刻的IMU的位姿
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back()); // 当前时刻的IMU的位姿
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack; // 相对运动的增量
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre; // 融合之后的位姿

        // 将最后的位姿进行发布
        float x, y, z, roll, pitch, yaw; // 定义变量
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw); // 从齐次矩阵变换到位置和姿态信息
        
        // publish latest odometry
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // publish tf
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // publish IMU path 发布IMU的轨迹
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec(); // 当前IMU时刻的位姿时间
        if (imuTime - last_path_time > 0.1) // 每隔0.1s发布一次IMU的轨迹
        {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)  // 只保留最近 1 秒的轨迹（与激光位姿时间对齐）
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

class IMUPreintegration : public ParamServer
{
public:

    std::mutex mtx; // IMU的线程锁

    ros::Subscriber subImu; // 订阅IMU话题
    ros::Subscriber subOdometry; // 订阅激光雷达里程计话题
    ros::Publisher pubImuOdometry; // 发布IMU里程计话题

    bool systemInitialized = false; // 系统是否初始化

    // 定义IMU里程计的噪声模型
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise; // 先验位置噪声模型
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise; // 先验速度噪声模型
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise; // 先验偏置噪声模型
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise; // 校正噪声模型 定义 激光雷达（或视觉）位姿观测（Correction） 的噪声模型 用于构建 BetweenFactor 或 PriorFactor，将激光 SLAM 输出的位姿作为“观测值”加入图中 激光位姿精度的信任程度（通常比 IMU 高，所以噪声较小）
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2; // 校正噪声模型2 另一个位姿校正噪声模型，通常用于不同场景或不同精度的观测
    gtsam::Vector noiseModelBetweenBias; // IMU 零偏随时间变化的随机游走（Random Walk）噪声 

    // GTSAM 库中用于 IMU 预积分
    // gtsam::PreintegratedImuMeasurements 对连续的 IMU 测量（加速度 + 角速度）进行预积分
    /*
      当新激光关键帧到来时：
        停止 imuIntegratorOpt_ 的积分；
        将其结果作为 ImuFactor 加入因子图；
        用优化后的状态（位姿、速度、bias）重置 imuIntegratorImu_ 和 imuIntegratorOpt_；
        开启新一轮积分。
        在两个激光关键帧之间：
        imuIntegratorImu_ 持续积分 IMU 数据；
        实时发布高频但未校正的增量位姿（供 TransformFusion 使用）；
        imuIntegratorOpt_ 同时也在后台积分，但只在关键帧到达时用于优化。
    */

    
    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_; // 用于状态因子图的构建
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_; // 用于实时的状态估计



    std::deque<sensor_msgs::Imu> imuQueOpt; // 用于状态因子图的构建
    std::deque<sensor_msgs::Imu> imuQueImu; // 用于实时的状态估计


    // 上一时刻状态量
    gtsam::Pose3 prevPose_; // 位置 
    gtsam::Vector3 prevVel_; // 速度
    gtsam::NavState prevState_; // 状态量（位姿、速度、偏置）
    gtsam::imuBias::ConstantBias prevBias_; // 偏置

    gtsam::NavState prevStateOdom; // 上一时刻激光雷达里程计的状态量（位姿、速度、偏置）
    gtsam::imuBias::ConstantBias prevBiasOdom; // 上一时刻激光雷达里程计的偏置

    // 开始积分的标志位
    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    // GTSAM 优化器与因子图
    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0; 

    int key = 1;
    

    // 定义雷达和IMU之间的刚体变换
    // T_bl: tramsform points from lidar frame to imu frame 
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    // T_lb: tramsform points from imu frame to lidar frame
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));



    IMUPreintegration()
    {   
        // IMU构造函数
        // IMU的队列长度较长
        // ros::TransportHints().tcpNoDelay() 降低IMU的通信延迟
        subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic, 2000, &IMUPreintegration::imuHandler, this, ros::TransportHints().tcpNoDelay()); // 订阅IMU话题
        subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay()); // 订阅激光雷达里程计话题

        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000); // 发布IMU里程计话题


        // 设置gtsam的噪声模型
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity); // IMU预积分参数
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
    }

    void resetOptimization()
    {   
      //  重置优化器与因子图
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams()
    {   
        // 重置参数
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {   
        // 将激光雷达里程计（低频、高精度）与 IMU（高频、有漂移）进行紧耦合图优化，输出高精度、高频、全局一致的状态估计（位姿、速度、IMU 零偏）
        // 程序的几个阶段：
        // 1. 初始化系统
        // 2. IMU预积分
        // 3. 构建因子图并进行优化
        // 4. 提取结果并且判断结果是否合理
        // 5. 重传播IMU预积分

        std::lock_guard<std::mutex> lock(mtx); // 互斥锁

        double currentCorrectionTime = ROS_TIME(odomMsg); // 获取激光雷达里程计的时间戳

        // make sure we have imu data to integrate 确保imu的有数据
        if (imuQueOpt.empty())
            return;

        // 获取激光雷达里程计的位姿信息
        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;

        // 程序的整体步骤



        // 判断激光雷达是否退化
        // 如果没有退化，使用较小的噪声模型，如果退化，使用较大的噪声模型
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false; // 判断激光雷达里程计的协方差矩阵是否退化

        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z)); // 将雷达的位置转化为gtsam的形式

        


        // 0. initialize system 初始化系统
        if (systemInitialized == false)
        {
            resetOptimization(); // 重置优化器

            // pop old IMU message
            while (!imuQueOpt.empty()) // 如果imu队列不为空，则弹出旧的IMU的数据
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }

            // initial pose 初始化位姿
            // 将激光雷达里程计的位姿转化为IMU的位姿
            prevPose_ = lidarPose.compose(lidar2Imu);

            // 添加先验因子
            // 添加位置的先验因子
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity 添加速度的先验因子
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias 添加零偏的先验因子
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);

            // add values 插入初始值
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once 接受初值，也就是仅进行一次优化
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();
            
            // 重置两个 IMU 预积分器
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            systemInitialized = true;
            return;
        }


        // reset graph for speed
        if (key == 100) // 因子图的定期重置，防止因子图过大
        {   
            // 每100帧重置一次因子图，防止因子图过大
            // get updated noise before reset
            // 清空因子图保留旧的数据
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();

            // 将上一个时间段的位姿数据放入到优化图的第一个节点，并对其进行优化
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1; // 重置key值
        }


        // 1. integrate imu data and optimize
        // 融合imu数据并且进行优化
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front(); //获取lidar时刻的imu数据
            double imuTime = ROS_TIME(thisImu); // 获取imu的时间戳

            // delat_t 同步的时间误差
            if (imuTime < currentCorrectionTime - delta_t) // 只处理“早于当前激光帧”的 IMU 数据，确保积分区间是 [t_{k-1}, t_k)，即两个关键帧之间
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt); // 计算当前IMU帧到上一个时间帧之间的时间间隔

                // imuIntegratorOpt_->integrateMeasurement(acc, gyro, dt); 将当前的加速度 和 角速度 和时间差 信息传入imu预积分器中
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime; //  更新lastImuT_opt
                imuQueOpt.pop_front();
            }
            else
                break;
        }



        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add pose factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // optimize 优化过程
        optimizer.update(graphFactors, graphValues); // 将因子图和值传入优化器中进行优化
        optimizer.update(); // 进行额外的迭代优化，提高精度
        graphFactors.resize(0); // 清空因子图
        graphValues.clear(); // 清空变量容器
        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate(); // 获得优化变量
        // 将优化的结果进行更新
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_); // 重置 IMU 预积分器内部状态
        // check optimization 检查优化的结果，判断优化是否失败，如果失败则重置参数
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        // 在完成图优化后，利用最新优化得到的状态（位姿、速度、IMU 零偏），对 IMU 队列中“当前时刻之后”的 IMU 数据进行重新预积分（re-propagation），
        // 以生成高精度的 IMU 里程计（IMU Odometry），用于后续激光帧的初始位姿预测或前端运动补偿。
        
        //  保存最新优化状态用于 IMU 里程计
        // 将优化得到的imu的数据作为imu里程计下一次的初始状态，用于后续的 IMU 里程计计算。
        prevStateOdom = prevState_; 
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        // 清除过期的imu的数据
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }


        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom); // 用最新 bias 重置 IMU 里程计预积分器
            // integrate imu message from the beginning of this optimization
            // 对 imuQueImu 中剩余的所有 IMU 数据（即 t ≥ currentCorrectionTime）进行积分
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {   
        // 获得imu的信息并且进行积分获得imu的里程计
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw); // 转换imu信息，去掉冗余的头信息

        // 将imu的信息放入到队列中
        imuQueOpt.push_back(thisImu); 
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false)
            return;

        // 获得时间戳
        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu); // 更新imu的时间
        lastImuT_imu = imuTime;

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt); // 进行imu的积分

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom); // 返回当前imu的里程计状态

        // publish odometry
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");
    
    IMUPreintegration ImuP;

    TransformFusion TF;

    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}
