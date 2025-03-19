#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("peter_publisher"), count_(0)
    {
    
    
      publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);     
      subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "battery_voltage", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
    }

  private:
	void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
    	{
      		auto voltage = msg->data;
      
      		auto message = std_msgs::msg::Float32();
      		message.data = voltage;
      		float vysledok = (voltage-32.0)*10.0;
      		RCLCPP_INFO(this->get_logger(), "%f  %f",voltage, vysledok);
      		publisher_->publish(message);
    	}

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
