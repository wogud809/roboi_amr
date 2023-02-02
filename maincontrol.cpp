#include <memory>
#include <iostream>

#include "ini.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"



using std::placeholders::_1;

class maincontrol : public rclcpp::Node
{
  public:
    maincontrol()
    : Node("node_main")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery", 10, std::bind(&maincontrol::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Main voltage: '%f'", msg->voltage);
      RCLCPP_INFO(this->get_logger(), "Main percentage: '%f'", msg->percentage);
    }
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // mINI::INIFile file("test.ini");
  // mINI::INIStructure ini;

  // if( file.read(ini) == false )
  // {
  //   printf("InI file read fail \n");
  // }

  // std::string value1 = ini.get("PARAMETER").get("DrivingVelo");
  // std::string value2 = ini.get("PARAMETER").get("SteeringVelo");
  // std::string value3 = ini.get("PARAMETER").get("InitLED");
  // std::string value4 = ini.get("PARAMETER").get("FrontfLeftOffset");
  // std::string value5 = ini.get("PARAMETER").get("FrontfRightOffset");
  // std::string value6 = ini.get("PARAMETER").get("RearLeftOffset");
  // std::string value7 = ini.get("PARAMETER").get("RearRightOffset");

  // std::cout << std::stoi(value1) << "\n";
  // std::cout << std::stoi(value2) << "\n";
  // std::cout << value3 << "\n";
  // std::cout << std::stoi(value4) << "\n";
  // std::cout << std::stoi(value5) << "\n";
  // std::cout << std::stoi(value6) << "\n";
  // std::cout << std::stoi(value7) << "\n";

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<maincontrol>());
  rclcpp::shutdown();
  return 0;
}
