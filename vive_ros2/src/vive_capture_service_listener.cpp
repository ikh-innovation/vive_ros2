#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vive_ros2/srv/vive_path_capture.hpp"
#include "vive_ros2/srv/vive_path_reset.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
// using std::placeholders::_1;

class ViveListener : public rclcpp::Node
{
  public:
    ViveListener(): Node("ViveListener")
    {
      rclcpp::Client<vive_ros2::srv::VivePathCapture>::SharedPtr capture_client = this->create_client<vive_ros2::srv::VivePathCapture>("/vive_pose_capture");
      rclcpp::Client<vive_ros2::srv::VivePathReset>::SharedPtr   reset_client = this->create_client<vive_ros2::srv::VivePathReset>("/vive_pose_reset");

      while (!capture_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the capture service. Exiting.");
          break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Capture service not available, waiting again...");
      }
      while (!reset_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the reset service. Exiting.");
          break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reset service not available, waiting again...");
      }     

      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(vive_controller_topic, 10, std::bind(&ViveListener::topic_callback, this, std::placeholders::_1));
      capture_client_ = this->create_client<vive_ros2::srv::VivePathCapture>("/vive_pose_capture");
      reset_client_ = this->create_client<vive_ros2::srv::VivePathReset>("/vive_pose_reset");
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      auto request = std::make_shared<vive_ros2::srv::VivePathCapture::Request>();
      if (msg->axes[0] == 1.0){
        button_state = true;
      }else{
        button_state = false;
      }
      if (!button_state == capturing){
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->axes[0]);
        if (button_state == true){
          RCLCPP_INFO(this->get_logger(), "I requested a call");
        }
        else{
          RCLCPP_INFO(this->get_logger(), "Stop capturing.");
        }
        capturing = button_state ; 
        request->data = button_state ;
        auto result = capture_client_->async_send_request(request);        
      }

      auto reset_request = std::make_shared<vive_ros2::srv::VivePathReset::Request>();
      if (msg->buttons[0]==1){
        if (reset_state){
          duration = this->now().seconds() - t_init.seconds();
        }
        else{
          RCLCPP_INFO(this->get_logger(), "Reset requested.");
          t_init = this->now();
          reset_state = true ; 
          reset_request->data = false ; //not yet sure of the press duration, signal to reset previous
          auto result = reset_client_->async_send_request(reset_request);
        }
        if (duration > time_reset_threshold) {
          RCLCPP_INFO(this->get_logger(), "Reset all requested.");
          reset_request->data = true ; //reset all
          auto result = reset_client_->async_send_request(reset_request);          
        }
      }else{
        reset_state = false ; 
        duration = 0.0 ; 
      }

    }
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    std::string vive_controller_name = "controller_1";
    std::string vive_controller_topic = "/joy_inputs";
    rclcpp::Client<vive_ros2::srv::VivePathCapture>::SharedPtr capture_client_ ;
    rclcpp::Client<vive_ros2::srv::VivePathReset>::SharedPtr reset_client_ ;

    bool capturing {false};
    bool button_state {false};

    bool reset_state {false}; //flag indicating if its the first time I received a reset command
    rclcpp::Time t_init;
    double duration {0.0};
    const double time_reset_threshold {3.0};
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViveListener>());
  rclcpp::shutdown();
  return 0;
}