#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vive_ros2/srv/vive_path_capture.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
// using std::placeholders::_1;

class ViveListener : public rclcpp::Node
{
  public:
    ViveListener(): Node("ViveListener")
    {
      capture_client_ = this->create_client<vive_ros2::srv::VivePathCapture>("/vive_pose_capture");
      reset_client_ = this->create_client<vive_ros2::srv::VivePathCapture>("/vive_pose_reset");

      this->declare_parameter<std::string>("buttons_topic", "joy_inputs");
      vive_controller_buttons_topic_ = this->get_parameter("buttons_topic").get_parameter_value().get<std::string>();

      while (!capture_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the capture service. Exiting.");
          break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Capture service not available, waiting again...");
      }
      while (!reset_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the reset service. Exiting.");
          break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reset service not available, waiting again...");
      }     

      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(vive_controller_buttons_topic_, 10, std::bind(&ViveListener::topic_callback, this, std::placeholders::_1));
      
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

      auto reset_request = std::make_shared<vive_ros2::srv::VivePathCapture::Request>();
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

    std::string vive_controller_buttons_topic_ ;
    rclcpp::Client<vive_ros2::srv::VivePathCapture>::SharedPtr capture_client_ ;
    rclcpp::Client<vive_ros2::srv::VivePathCapture>::SharedPtr reset_client_ ;

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