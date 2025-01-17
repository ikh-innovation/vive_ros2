#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <random>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "vive_ros2/srv/vive_path_capture.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class VivePathCaptureServer : public rclcpp::Node
{
public:
  VivePathCaptureServer()
  : Node("VivePathCaptureServer")
  {
    // Declare and acquire parameters
    this->declare_parameter<std::string>("capture_origin_frame", "base_link");
    this->declare_parameter<std::string>("capture_tip_frame", "controller_tip_reversed");
    target_frame_ = this->get_parameter("capture_origin_frame").get_parameter_value().get<std::string>();
    controller_frame_ = this->get_parameter("capture_tip_frame").get_parameter_value().get<std::string>();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create captured path publisher
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Create a publisher for path markers
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/paths", 10);

    //Create a service server for path capturing
    capture_server_ = this->create_service<vive_ros2::srv::VivePathCapture>("/vive_pose_capture", std::bind(&VivePathCaptureServer::PoseCaptureCallback,this,
      std::placeholders::_1,
      std::placeholders::_2));

    //Create a service server for path reset
    reset_server_ = this->create_service<vive_ros2::srv::VivePathCapture>("/vive_pose_reset", std::bind(&VivePathCaptureServer::ResetCaptureCallback,this,
      std::placeholders::_1,
      std::placeholders::_2));
  }

  void CaptureLoop(){
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = target_frame_;
    std::string toFrameRel = controller_frame_;
    
    geometry_msgs::msg::PoseStamped pose ;
    pose.header.frame_id = target_frame_;
    while (rclcpp::ok()){
      // RCLCPP_INFO(this->get_logger(),"Spinning...");
      if (capture_on){
        // RCLCPP_INFO(this->get_logger(),"Capturing segments...");
        geometry_msgs::msg::TransformStamped t;
        // Look up for the transformation between target and source frames
        try {
          t = tf_buffer_->lookupTransform(fromFrameRel, toFrameRel, tf2::TimePointZero);
        } 
        catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
        pose.header.stamp = this->get_clock()->now() ;
        pose.pose.position.x = t.transform.translation.x ;
        pose.pose.position.y = t.transform.translation.y ;
        pose.pose.position.z = t.transform.translation.z ;
        pose.pose.orientation.x = t.transform.rotation.x ;
        pose.pose.orientation.y = t.transform.rotation.y ;
        pose.pose.orientation.z = t.transform.rotation.z ;
        pose.pose.orientation.w = t.transform.rotation.w ;

        current_segment.poses.push_back(pose); 
        path_publisher_->publish(current_segment);
        geometry_msgs::msg::Point point {};
        point.x = t.transform.translation.x ;
        point.y = t.transform.translation.y ;
        point.z = t.transform.translation.z ;
        marker_.points.push_back(point);
        marker_array_.markers.push_back(marker_);
        marker_publisher_->publish(marker_array_);
      }
      executor.spin_some();
    }
  }

private:

  void PoseCaptureCallback(const std::shared_ptr<vive_ros2::srv::VivePathCapture::Request>  request,
                                 std::shared_ptr<vive_ros2::srv::VivePathCapture::Response> response){
    capture_on = request->data ;
    current_segment.header.frame_id = target_frame_ ; 

    std::random_device rd; 
    std::mt19937 gen(rd()); 
    std::uniform_real_distribution<> distr(0,1);

    if (capture_on){
      marker_ = {};
      RCLCPP_INFO(this->get_logger(),"Capturing segments...");
      std::string ns = "path_" + std::to_string(segments.size());
      marker_.header.frame_id = target_frame_ ;
      marker_.header.stamp = this->get_clock()->now();
      rclcpp::Duration lt = rclcpp::Duration::from_seconds(0.0);
      marker_.lifetime = lt; 
      marker_.ns = ns; 
      marker_.type = 4; 
      marker_.action = 0; 
      marker_.pose.orientation.x = 0; 
      marker_.pose.orientation.y = 0; 
      marker_.pose.orientation.z = 0; 
      marker_.pose.orientation.w = 1; 
      marker_.scale.x = 0.02; 
      marker_.scale.y = 0.02;
      marker_.scale.z = 0.02;
      marker_.color.a = 1.0;
      marker_.color.r = distr(gen);
      marker_.color.g = distr(gen);
      marker_.color.b = distr(gen);
      mark_id ++ ;
      marker_.id = mark_id; 
      std::cout << marker_.id << std::endl ;
      // marker_array_.markers.push_back(marker_);
      response->message = "Capture has been started";
    }else{
      RCLCPP_INFO(this->get_logger(),"Capturing over...");
      response->message = "Capture has been stopped";
      segments.push_back(current_segment);
      current_segment = {};
      marker_array_.markers.push_back(marker_);
      marker_publisher_->publish(marker_array_);
    }
    response->segments = segments; 
  }

  void ResetCaptureCallback(const std::shared_ptr<vive_ros2::srv::VivePathCapture::Request>  request,
                                 std::shared_ptr<vive_ros2::srv::VivePathCapture::Response> response){
    reset_flag = request->data ;
    current_segment.header.frame_id = target_frame_ ;
    if (reset_flag){
      // nav_msgs::msg::Path current_segment {}; 
      RCLCPP_INFO(this->get_logger(),"Reset all captured paths.");
      response->message = "Reset all captured paths.";
      if (!segments.empty())
        segments.clear();
      current_segment = {};
      
      reset_marker_ = {};
      reset_marker_.id = mark_id + 1; 
      reset_marker_.action = 3; 
      marker_array_.markers.push_back(reset_marker_);
      marker_publisher_->publish(marker_array_);
      mark_id = 0;
    }else{
      if (mark_id > 0){
        RCLCPP_INFO(this->get_logger(),"Reset last captured path.");
        response->message = "Reset last captured path.";
        if (!segments.empty()){
          segments.pop_back();
        }
        current_segment = {};
        if (!marker_array_.markers.empty()){
          for (int32_t i = 0; i < marker_array_.markers.size(); i++){
            if (marker_array_.markers[i].id == mark_id ){
              marker_array_.markers[i].action = 2;
            }
          }
        }  
        mark_id--;          
        marker_publisher_->publish(marker_array_);
      }else{        
        RCLCPP_INFO(this->get_logger(),"No paths left to erase");
      }

    }
    response->segments = segments; 
  }

  // publisher and service forward declarations
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Service<vive_ros2::srv::VivePathCapture>::SharedPtr capture_server_;
  rclcpp::Service<vive_ros2::srv::VivePathCapture>::SharedPtr reset_server_;
  
  // path stuff
  std::vector<nav_msgs::msg::Path> segments {}; 
  nav_msgs::msg::Path current_segment;
  
  // visualization stuff
  visualization_msgs::msg::MarkerArray marker_array_ {};
  visualization_msgs::msg::Marker marker_ {};  
  visualization_msgs::msg::Marker reset_marker_ {}; 
  int32_t mark_id {0};

  // flags for capture and reset signals
  bool capture_on {false};
  bool reset_flag {false}; 

  // thread stuff. To delete.
  std::thread thread_obj_1;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  std::string controller_frame_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  VivePathCaptureServer Server;
  Server.CaptureLoop(); 
  rclcpp::shutdown();
  return 0;
}