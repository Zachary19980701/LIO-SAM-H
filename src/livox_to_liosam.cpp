#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
// #include <pcl/filters/filter.h>
#include <cmath>
#include <algorithm>
   // 需要在文件顶部包含一次


// 目标点型：XYZI + ring(uint16) + time(float) —— LIO-SAM常用
struct PointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)
  (uint16_t, ring, ring)
  (float, time, time)
)

class PC2RingTimeNode {
public:
  PC2RingTimeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("in_cloud", in_cloud_, "/lidar");
    pnh.param<std::string>("out_cloud", out_cloud_, "/points_raw");
    pnh.param<int>("N_SCAN", N_SCAN_, 32);               // 虚拟线束数（按LIO-SAM配置来）
    pnh.param<double>("ang_min_deg", ang_min_deg_, -25); // 俯仰角最小（度）
    pnh.param<double>("ang_max_deg", ang_max_deg_,  15); // 俯仰角最大（度）
    pnh.param<bool>("drop_out_of_fov", drop_out_of_fov_, true); // 超出俯仰范围的点：丢弃/夹紧
    pnh.param<double>("scan_period", scan_period_, 0.1); // 合成time时用，10Hz≈0.1s
    pnh.param<std::string>("time_field_name", time_field_name_, "time"); // 若输入里已有这个字段，就沿用

    sub_ = nh.subscribe(in_cloud_, 5, &PC2RingTimeNode::cb, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(out_cloud_, 5);
    ROS_INFO_STREAM("pc2_ring_time: sub=" << in_cloud_ << " pub=" << out_cloud_
                    << " N_SCAN=" << N_SCAN_ << " ang=[" << ang_min_deg_ << "," << ang_max_deg_ << "]");
  }

private:
  static bool hasField(const sensor_msgs::PointCloud2ConstPtr& msg, const std::string& name) {
    for (const auto& f : msg->fields) if (f.name == name) return true;
    return false;
  }

  void cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 用PCL把x/y/z/intensity取出来（简单稳定）
    pcl::PointCloud<pcl::PointXYZI> in;
    pcl::fromROSMsg(*msg, in);

    // 可选：读取输入里的 per-point time
    // const bool has_time_in = hasField(msg, time_field_name_);
    // sensor_msgs::PointCloud2ConstIterator<float> it_time(*msg, time_field_name_);
    
    const bool has_time_in = hasField(msg, time_field_name_);
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_time;
    if (has_time_in) {
      it_time.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*msg, time_field_name_));
    }
    
    // 俯仰角映射用到的参数
    const double ang_min = ang_min_deg_ * M_PI / 180.0;
    const double ang_max = ang_max_deg_ * M_PI / 180.0;
    const double ang_span = ang_max - ang_min;

    pcl::PointCloud<PointXYZIRT> out;
    out.reserve(in.size());

    size_t out_of_fov = 0;
    const size_t n = in.size();

    for (size_t i = 0; i < n; ++i) {
      const auto& p = in.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

      // 俯仰角（垂直角）：[-pi/2, pi/2]
      const double r_xy = std::hypot(p.x, p.y);
      const double elev = std::atan2((double)p.z, r_xy);

      int ring = -1;
      if (ang_span > 1e-6) {
        const double t = (elev - ang_min) / ang_span; // [0,1]内
        const double ridx = t * (N_SCAN_ - 1);
        if (t < 0.0 || t > 1.0) {
          if (drop_out_of_fov_) { out_of_fov++; continue; }
          // ring = std::clamp((int)std::lround(ridx), 0, N_SCAN_ - 1);
          ring = std::min(std::max((int)std::lround(ridx), 0), N_SCAN_ - 1);
        } else {
          ring = (int)std::lround(ridx);
        }
      } else {
        ring = 0; // 避免除零；等价1线束
      }

      PointXYZIRT q;
      q.x = p.x; q.y = p.y; q.z = p.z;
      q.intensity = p.intensity;
      q.ring = static_cast<uint16_t>(ring);

      if (has_time_in) {
        q.time = **it_time;   // 直接沿用输入time
        ++(*it_time);
      } else {
        // 没有time就均分合成（非严格，仅为让LIO-SAM能跑）
        q.time = (n > 1) ? (float)(scan_period_ * (double)i / (double)(n - 1)) : 0.0f;
      }

      out.push_back(q);
    }

    out.width  = static_cast<uint32_t>(out.size());
    out.height = 1;

    // 移除潜在 NaN
    // std::vector<int> dummy;
    // pcl::removeNaNFromPointCloud(out, out, dummy);
    // out.is_dense = true;

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(out, out_msg);
    out_msg.header = msg->header;
    pub_.publish(out_msg);

    if (out_of_fov > 0) {
      ROS_DEBUG("pc2_ring_time: dropped %zu out-of-FOV points (%.2f%%)",
                out_of_fov, 100.0 * out_of_fov / std::max<size_t>(1, n));
    }
  }

  // ROS
  ros::Subscriber sub_;
  ros::Publisher  pub_;
  // params
  std::string in_cloud_, out_cloud_, time_field_name_;
  int N_SCAN_;
  double ang_min_deg_, ang_max_deg_, scan_period_;
  bool drop_out_of_fov_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pc2_ring_time_livox");
  ros::NodeHandle nh, pnh("~");
  PC2RingTimeNode node(nh, pnh);
  ros::spin();
  return 0;
}
