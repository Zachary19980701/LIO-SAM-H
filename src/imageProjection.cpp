// 点云的预处理和去畸变
#include "../include/utility.h"
#include "lio_sam/cloud_info.h"
#include <mutex>

struct VelodynePointXYZIRT{
  // 存储Velodyne 16线激光雷达点的结构体
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint_16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16; //将结构体进行对齐修正
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT, // 解析结构体的函数
  (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
  (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t noise;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
  (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
  (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
  (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)



// 将威力登的点表示为一个普通的点
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000; // 定义最大的序列长度为2000

class ImageProjection : public ParamServer{
  private:
    // 线程锁
    std::mutex imulock;
    std::mutex odoLock;

    // ros pubs and subs
    // 点云的订阅与发布
    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    // imu
    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    // odom queue
    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    // cloud
    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    // 预定义一个数列
    double* imuTime = new double[queueLength];
    double* imuRotX = new double[queueLength];
    double* imuRotY = new double[queueLength];
    double* imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    // 定义点云的指针
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int deskewFlag; // 去畸变标志位
    cv::Mat rangeMat; // 范围矩阵

    bool odomDeskewFlag; // 去畸变标志位
    float odomIncreX; //
    float odomIncreY; // 
    float odomIncreZ; //

    lio_sam::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    std::vector<int> columnIdnCountVec;

  public:
    ImageProjection();
    ~ImageProjection();


    // 去畸变的相关初始化
    deskewFlag(0){
      subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
      subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
      subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

      pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);
      pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 1);
      allocateMemory(); // 分配内存
      resetParameters(); // 重置参数

      pcl::console::setVerbosityLevel(pcl::console::L_ERROR); // 设置打印级别
    }

    void allocateMemory(){
      // 分配内存，初始化点云指针
      // 初始化点云指针
      laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
      tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
      fullCloud.reset(new pcl::PointCloud<PointType>());
      extractedCloud.reset(new pcl::PointCloud<PointType>());
      // 初始化点云的大小
      fullCloud->points.resize(N_SCAN*Horizon_SCAN);
      // 初始化点云的头信息
      cloudInfo.startRingIndex.assign(N_SCAN, 0);
      cloudInfo.endRingIndex.assign(N_SCAN, 0);
      // 初始化点云的头信息
      cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
      cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);
      resetParameters(); // 重置参数
    }

    void resetParameters(){
      // 重置参数
      laserCloudIn->clear();
      extractedCloud->clear();
      // reset range matrix for range image projection
      rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX)); // 初始化范围矩阵

      imuPointerCur = 0;
      firstPointFlag = true;
      odomDeskewFlag = false;

      for (int i = 0; i < queueLength; ++i)
      {
          imuTime[i] = 0;
          imuRotX[i] = 0;
          imuRotY[i] = 0;
          imuRotZ[i] = 0;
      }

      columnIdnCountVec.assign(N_SCAN, 0);
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg){
      // imu处理函数
      sensor_msgs::Imu thisImu = *imuMsg;
      
      std::lock_guard<std::mutex> lock1(imulock); // 加锁
      imuQueue.push_back(thisImu);
      // debug IMU data
      // cout << std::setprecision(6);
      // cout << "IMU acc: " << endl;
      // cout << "x: " << thisImu.linear_acceleration.x << 
      //       ", y: " << thisImu.linear_acceleration.y << 
      //       ", z: " << thisImu.linear_acceleration.z << endl;
      // cout << "IMU gyro: " << endl;
      // cout << "x: " << thisImu.angular_velocity.x << 
      //       ", y: " << thisImu.angular_velocity.y << 
      //       ", z: " << thisImu.angular_velocity.z << endl;
      // double imuRoll, imuPitch, imuYaw;
      // tf::Quaternion orientation;
      // tf::quaternionMsgToTF(thisImu.orientation, orientation);
      // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
      // cout << "IMU roll pitch yaw: " << endl;
      // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;

    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg){
      // odom handler function
      std::lock_guard<std::mutex> lock2(odoLock); // lock the odometry queue
      odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
      // cloud handler function 对点云进行去畸变

      if(!cachePointCloud(laserCloudMsg))
        return; // 判断是否有缓存的点云

      if(!deskewInfo())
        return; // 判断是否有去畸变的信息

      projectPointCloud(); // 点云去畸变

      cloudExtraction(); // 特征提取

      publishClouds(); // 发布点云

      resetParameters(); // 下一帧点云重置
    }


    // 点云处理相关函数
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
      // 缓存点云，并且判断缓存区内部是否有点云
      cloudQueue.push_back(*laserCloudMsg);
      if(cloudQueue.size() <= 2){
        return false; // 如果缓存区少于阈值，范围无缓存的点云
      }

      // 点云转换
      currentCloudMsg = std::move(cloudQueue.front()); // 将队列中的点云数据移动到currentCloudMsg
      cloudQueue.pop_front();

      if(sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX){
        // 对于威力登和livox的雷达，直接进行处理
        pcl::moveFromROSMsg(currentCloudMsg, *laserCloudMsg);
      }
      else if(sensor == SensorType::OUSTER){
        // 对于ouster雷达，需要进行转换
        pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
        laserCloudIn->points.resize(tmpOusterCloudIn->size());
        laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
        for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
        {
            auto &src = tmpOusterCloudIn->points[i];
            auto &dst = laserCloudIn->points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.time = src.t * 1e-9f;
        }
      }
      else{
        ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
        ros::shutdown();
      }

      // 时间戳处理
      cloudHeader = currentCloudMsg.header;
      timeScanCur = cloudHeader.stamp.toSec();
      timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

      // check dense flag, 判断是否是dense点
      if (laserCloudIn->is_dense == false)
      {
          ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
          ros::shutdown();
      }

      // 检查线所在线数
      static int ringFlag = 0;
      if (ringFlag == 0)
      {
          ringFlag = -1;
          for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
          {
              if (currentCloudMsg.fields[i].name == "ring")
              {
                  ringFlag = 1;
                  break;
              }
          }
          if (ringFlag == -1)
          {
              ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
              ros::shutdown();
          }
      }

      // check point time
      if (deskewFlag == 0)
      {
          deskewFlag = -1;
          for (auto &field : currentCloudMsg.fields)
          {
              if (field.name == "time" || field.name == "t")
              {
                  deskewFlag = 1;
                  break;
              }
          }
          if (deskewFlag == -1)
              ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
      }

      return true;
    }

    bool deskewInfo(){
      // 检查当前的imu和odom信息是否能够用于点云的去畸变
      std::lock_guard<std::mutex> lock1(imulock);
      std::lock_guard<std::mutex> lock2(odoLock);

      // 检查imu信息能够用于当前的sscan
      if(imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd){
        // 判断当前的imu的buffer的开始的时间戳和结束时间戳在当前scan的时间段内部
        ROS_DEBUG("waiting for imu data ...");
        return false;
      }

      imuDeskewInfo(); // imu的去畸变的信息
      odomDeskewInfo(); // odom的去畸变信息

      return true;
    }

    void imuDeskewInfo(){
      // 对imu的进行去畸变的处理
      // 通过计算扫描周期内的imu的旋转信息，对点云进行运动补偿和时间同步
      cloudInfo.imuAvailable = true;

      // 讲当前imu队列中早于lidar扫描周期开始时间0.01s的imu的信息弹出
      while(!imuQueue.empty()){
        if(imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01){
          imuQueue.pop_front(); // 如果imu的开始时间戳小于当前scan的时间，则弹出队列中的imu信息
        }
        else {
          break;
        }
      }

      // 如果imu队列为空，则直接返回
      if(imuQueue.empty()){
        return;
      }

      // 遍历imu的队列，计算imu的旋转信息
      for(int i = 0; i < (int)imuQueue.size(); ++i){
        sensor_msgs::Imu thisImu = imuQueue[i];
        double currentImuTime = thisImu.header.stamp.toSec();

        // calculate imu rotation get roll pitch yaw estimate for this scan
        if(currentImuTime <= timeScanCur){
          imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
        }

        if(currentImuTime < timeScanEnd + 0.01){
          // 如果当前的imu的时间戳小于scan的结束时间，则跳出循环
          break;
        }

        if(imuPointerCur == 0){
          imuRotX[0] = 0;
          imuRotY[0] = 0;
          imuRotZ[0] = 0;
          imuTime[0] = currentImuTime;
          ++imuPointerCur;
          continue;
        }

        // 计算imu的角速度
        double angular_x, angular_y, angular_z;
        imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z); // 将当前的imu的角速度转换到ros的格式

        // 旋转量进行积分
        double timeDiff = currentImuTime - imuTime(imuPointerCur - 1);
        imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x*timeDiff;
        imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y*timeDiff;
        imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z*timeDiff;
        imuTime[imuPointerCur] = currentImuTime;
        ++imuPointerCur;
      }

      -- imuPointerCur;

      if(imuPointerCur <= 0){
        return;
      }

      cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo(){
      cluodInfo.odomAvailable = true;

      while(!odomQueue.empty()){
        if(odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01){
          odomQueue.pop_front();
        }
        else {
          break;
        }
      }

      if(odomQueue.empty()){
        return;
      }

      if (odomQueue.front().header.stamp.toSec() > timeScanCur)
        return;
    }





}