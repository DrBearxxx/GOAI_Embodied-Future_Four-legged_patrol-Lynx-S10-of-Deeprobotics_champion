#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <glog/logging.h>
#include <tbb/global_control.h>
#include "core/lio/laser_mapping.h"

class LightningNode : public rclcpp::Node {
 public:
  LightningNode() : Node("lightning_lio") {
    lightning::LaserMapping::Options options;
    options.is_in_slam_mode_=false;
    lio_=std::make_unique<lightning::LaserMapping>(options);
    const auto config=declare_parameter<std::string>("config", "");
    if (!lio_->Init(config)) throw std::runtime_error("Invalid Lightning LIO config");
    pose_=create_publisher<nav_msgs::msg::Odometry>("odometry",20);
    counts_=create_publisher<std_msgs::msg::Float64MultiArray>("validation_counts",10);
    cloud_=create_subscription<sensor_msgs::msg::PointCloud2>("points",rclcpp::QoS(32).reliable(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr p){++cloud_count_;lio_->ProcessPointCloud2(p);drain();});
    imu_=create_subscription<sensor_msgs::msg::Imu>("imu",rclcpp::QoS(2000).reliable(),
      [this](sensor_msgs::msg::Imu::SharedPtr m){
        ++imu_count_;
        auto i=std::make_shared<lightning::IMU>();
        i->timestamp=rclcpp::Time(m->header.stamp).seconds();
        i->angular_velocity={m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z};
        i->linear_acceleration={m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z};
        lio_->ProcessIMU(i);drain();
      });
    timer_=create_wall_timer(std::chrono::seconds(1),[this]{
      std_msgs::msg::Float64MultiArray m;
      m.data={double(cloud_count_),double(imu_count_),double(pose_count_),last_stamp_};counts_->publish(m);
    });
  }
 private:
  void drain() {
    while (lio_->Run(true)) {
      const auto s=lio_->GetState();
      if (s.timestamp_<=last_stamp_) continue;
      nav_msgs::msg::Odometry m;
      m.header.stamp=rclcpp::Time(int64_t(std::llround(s.timestamp_*1e9)));
      m.header.frame_id=std::string(get_namespace())+"/odom_imu_origin";
      m.child_frame_id=std::string(get_namespace())+"/imu";
      m.pose.pose.position.x=s.pos_.x();m.pose.pose.position.y=s.pos_.y();m.pose.pose.position.z=s.pos_.z();
      const auto q=s.rot_.unit_quaternion();
      m.pose.pose.orientation.x=q.x();m.pose.pose.orientation.y=q.y();m.pose.pose.orientation.z=q.z();m.pose.pose.orientation.w=q.w();
      // Do not invent covariance or body-frame twist that the upstream API does not expose.
      pose_->publish(m);last_stamp_=s.timestamp_;++pose_count_;
    }
  }
  uint64_t cloud_count_=0,imu_count_=0,pose_count_=0;double last_stamp_=0;
  std::unique_ptr<lightning::LaserMapping> lio_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr counts_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv) {
  google::InitGoogleLogging(argv[0]);FLAGS_logtostderr=1;
  tbb::global_control workers(tbb::global_control::max_allowed_parallelism,4);
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<LightningNode>());}
  catch(const std::exception&e){LOG(ERROR)<<e.what();rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
