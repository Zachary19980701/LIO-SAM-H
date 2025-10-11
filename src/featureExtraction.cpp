// 从点云中提取平面特征和角特征

#include "utility.h"
#include "lio_sam/cloud_info.h"


// 定义平滑度的结构体
struct smoothness_t{
  float value;
  size_t ind;
}

struct by_value{
  // 对smoothness_t结构体按照value的值进行排序
  bool operator()(smoothness_t const &left, smoothness_t const &right){
    return left.value < right.value;
  }
}

class FeatureExtraction : public ParamServer{
  public:
    // pubs and subs
    ros::Subscriber subLaserCloudInfo;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;

    // define extract poincloud features
    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    // 定义降采样的相关点云
    pcl::VoxelGrid<PointType> downSizeFilter;

    lio_sam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;
    
    // define smoothness vector and value
    std::vector<smoothness_t> cloudSmoothness;
    float* cloudCurvature;
    int* cloudNeighborPicker; // 定义邻域拾取器
    int* cloudLabel;

    FeatureExtraction(){
      // init pubs and subs
      subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
      pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
      pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
      pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);

      intializationValue();
    }

    void initializationValue(){ 
      // 初始化相关变量
      cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

      downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

      extractedCloud.reset(new pcl::PointCloud<PointType>());
      cornerCloud.reset(new pcl::PointCloud<PointType>());
      surfaceCloud.reset(new pcl::PointCloud<PointType>());

      cloudCurvature = new float[N_SCAN*Horizon_SCAN];
      cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
      cloudLabel = new int[N_SCAN*Horizon_SCAN];
    }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn){
      // 点云的处理函数
      cloudInfo = *msgIn; 
      cloudHeader = msgIn->header;

      // 将点云从ros格式转化为pcl格式
      pcl::fromROSMsg(cloudInfo.cloud_deskewed, *extractedCloud);
      // 计算平滑度
      calculateSmoothness();
      // 计算邻域相关
      markOccludedPoints();
      // 提取特征点
      extractFeatures();
      // 发布特征点云
      publishFeatureCloud();
    }

    void calculateSmoothness(){
      // 计算每个点的平滑度
      int cloudSize = extractedCloud->points.size();

      for(int i = 5; i < cloudSize -5; i++){
        // 排除掉前面的几个点，进行曲率的计算
        // 将前五个点和后五个点进行比较，获得当前的点的平滑度
        float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                        + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                        + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                        + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                        + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                        + cloudInfo.pointRange[i+5]; 
        cloudCurvature[i] = diffRange * diffRange; // diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
        // cloudSmoothness for sorting
        cloudSmoothness[i].value = cloudCurvature[i];
        cloudSmoothness[i].ind = i;
      }
    }

    void markOccludedPoints(){
      // 标记被遮挡的点
      // 被遮挡点之间的的平滑度对于特征提取是没有意义的
      int cloudSize = extractedCloud->points.size();

      for(int i = 5; i < cloudSize - 6; ++i){
        // occluded points
        float depth1 = cloudInfo.pointRange[i]; // 当前点的深度信息
        float depth2 = cloudInfo.pointRange[i+1]; // 下一个点的深度信息
        int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i])); // 列方向的差值

        if(columnDiff < 10){
          // 计算附近十个点之间的插值
          if(depth1 - depth2 > 0.3){
            cloudNeighborPicked[i - 5] = 1;
            cloudNeighborPicked[i - 4] = 1;
            cloudNeighborPicked[i - 3] = 1;
            cloudNeighborPicked[i - 2] = 1;
            cloudNeighborPicked[i - 1] = 1;
            cloudNeighborPicked[i] = 1;
          }else if (depth2 - depth1 > 0.3) {
            cloudNeighborPicked[i + 1] = 1;
            cloudNeighborPicked[i + 2] = 1;
            cloudNeighborPicked[i + 3] = 1;
            cloudNeighborPicked[i + 4] = 1;
            cloudNeighborPicked[i + 5] = 1;
            cloudNeighborPicked[i + 6] = 1;
          }
        }
              // 计算点是否被遮挡
        float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
        float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

        if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
            cloudNeighborPicked[i] = 1;
      }
    }

    void extractFeatures(){
      // 提取特征点

      // clear old clouds
      cornerCloud->clear();
      surfaceCloud->clear();

      for(int i = 0; i < N_SCAN; i++){
        // 计算角点和平面点
        surfaceCloudScan->clear();

        for(int j = 0; j < 6; j++){
          // 将扫描线划分为六个方向，每个方向十个点
          // 提取六个方向的平面点
          // 获得每个方向的开始点和结束点
          int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;  // 开始点
          int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1; // 结束点

          if(sp > ep){
            // 跳过无效区间。如果 sp 大于或等于 ep，说明该区间没有有效的点云数据，直接跳过。
            continue;
          }

          std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value); // 按照平滑度排序

          int largestPickedNum = 0;
          for (int k = ep; k >= sp; k--){
              int ind = cloudSmoothness[k].ind; // 获得平滑度排序后的点的索引
              if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold) // 平滑度大于阈值，认为是角点
              {
                  largestPickedNum++; // 角点计数
                  if (largestPickedNum <= 20){
                      cloudLabel[ind] = 1; // 标记为角点
                      cornerCloud->push_back(extractedCloud->points[ind]); // 添加到角点云中
                  } else {
                      break;
                  }

                  cloudNeighborPicked[ind] = 1; // 标记相邻点为已处理,不在处理相关的邻居节点

                  // 检查当前角点的相邻点是否属于同一条扫描线。如果相邻点的列差小于 10，继续将其标记为邻居点。
                  for (int l = 1; l <= 5; l++)
                  {
                      int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1])); // 
                      if (columnDiff > 10)
                          break;
                      cloudNeighborPicked[ind + l] = 1;
                  }
                  for (int l = -1; l >= -5; l--)
                  {
                      int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                      if (columnDiff > 10)
                          break;
                      cloudNeighborPicked[ind + l] = 1;
                  }
              }
          }

          // 提取平面点
          for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {

                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0){
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
        }

        surfaceCloudScanDS->clear();
        downSizeFilter.setInputCloud(surfaceCloudScan); // 对面点进行降采样，减少计算量
        downSizeFilter.filter(*surfaceCloudScanDS);

        *surfaceCloud += *surfaceCloudScanDS;
      }
    }


    void freeCloudInfoMemory(){
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }

    void publishFeatureCloud(){
        // free cloud info memory
        freeCloudInfoMemory();
        // save newly extracted features
        cloudInfo.cloud_corner  = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        // publish to mapOptimization
        pubLaserCloudInfo.publish(cloudInfo);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");
   
    ros::spin();

    return 0;
}