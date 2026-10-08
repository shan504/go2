#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <sensor_msgs/msg/laser_scan.hpp>
using namespace std::chrono_literals;
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("go2_filter_timeout_check");
  tf2_ros::Buffer buffer(node->get_clock());
  buffer.setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),node->get_node_timers_interface()));
  tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> filter(buffer,"map",50,
    node->get_node_logging_interface(),node->get_node_clock_interface(),600ms);
  filter.setTolerance(rclcpp::Duration::from_seconds(0.05));
  int sparse_inputs=0,live_inputs=0,sparse_ok=0,live_ok=0;
  bool live=false;
  filter.registerCallback([&](sensor_msgs::msg::LaserScan::ConstSharedPtr){
    (live?live_ok:sparse_ok)++;
  });
  auto start=node->now();
  int tick=0;
  auto timer=node->create_wall_timer(50ms,[&]{
    auto now=node->now();
    double elapsed=(now-start).seconds();
    // Allow pending timeout requests to settle before measuring the live case.
    if(elapsed>5.0 && !live){filter.clear();live=true;}
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id="odom";tf.child_frame_id="base_link";
    tf.transform.rotation.w=1.0;
    tf.header.stamp=now;
    buffer.setTransform(tf,"synthetic",false);
    if(live || tick%10==0){
      tf.header.frame_id="map";tf.child_frame_id="odom";
      tf.header.stamp=live?now:now-rclcpp::Duration::from_seconds(0.75);
      buffer.setTransform(tf,"synthetic",false);
    }
    if(elapsed<4.0 || (live && elapsed>5.5)){
      auto scan=std::make_shared<sensor_msgs::msg::LaserScan>();
      scan->header.frame_id="base_link";
      scan->header.stamp=now-rclcpp::Duration::from_seconds(0.08);
      (live?live_inputs:sparse_inputs)++;
      filter.add(scan);
    }
    tick++;
    if(elapsed>8.0)rclcpp::shutdown();
  });
  rclcpp::spin(node);
  std::cout<<"Sparse delayed TF: passed="<<sparse_ok<<" input="<<sparse_inputs
           <<"; odometry-stamped live TF: passed="<<live_ok<<" input="<<live_inputs<<std::endl;
  return sparse_inputs>sparse_ok && live_ok>20 && live_ok==live_inputs?0:1;
}
