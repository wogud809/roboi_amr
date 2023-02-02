#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdint.h>

#include "ini.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <wiringPi.h>
#include <wiringSerial.h>
#include "sensor_msgs/msg/battery_state.hpp"

using namespace std::chrono_literals;

//using namespace fs = std::systemfilesystem;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class battery : public rclcpp::Node
{
public:
  battery()
  : Node("node_battery"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&battery::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
  size_t count_;
  sensor_msgs::msg::BatteryState Bat_Status = sensor_msgs::msg::BatteryState();

  void timer_callback()
  {
    //auto message = sensor_msgs::msg::BatteryState();
    //message.voltage = 53.4;
    GetBatteryStatus();
    RCLCPP_INFO(this->get_logger(), "Battery voltage: '%f, %d'", Bat_Status.voltage);
    RCLCPP_INFO(this->get_logger(), "Battery percentage: '%f'", Bat_Status.percentage);
    publisher_->publish(Bat_Status);
  }

public:
  int PortID;
  int ReciveTimeout;
  uint8_t BatterSendBuffer[11];
  uint8_t BatReciveBuffer[30];
  bool Open(void);
  void GetBatteryStatus(void);
};

bool battery::Open(void)
{
  mINI::INIFile file("test.ini");
  mINI::INIStructure ini;


  if( file.read(ini) == false )
  {
    RCLCPP_ERROR(this->get_logger(), "Battery System parameter read fail ");
    return false;
  }
  std::string Port = ini.get("SYSTEM").get("BatteryPort");
  std::string bardrate = ini.get("SYSTEM").get("BatteryBuadrate");

  if( Port.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Battery System No parameter : Port name ");
    return false;
  }

  if( bardrate.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Battery System No parameter : bardrate");
    return false;
  }

  int tempBardrate = std::stoi(bardrate);

  PortID = 0;
  if ((PortID = serialOpen(Port.c_str(), tempBardrate)) < 0)
  {
      RCLCPP_ERROR(this->get_logger(), "Battery Serial port open fail - [Port] : %s, [Bardpate] : %d ", Port.c_str(), tempBardrate);
      return false;
  }

  if (wiringPiSetup() == -1)
  {
      fprintf(stdout, "Unable to Battery start wiringPi: %s\n", strerror(errno));
      return false;
  }


  ReciveTimeout = 300;
  return true;
}

void battery::GetBatteryStatus(void)
{
    BatterSendBuffer[0] = 0xAF;
    BatterSendBuffer[1] = 0xFA;
    BatterSendBuffer[2] = 0x60;
    BatterSendBuffer[3] = 0x05;
    BatterSendBuffer[4] = 0x01;
    BatterSendBuffer[5] = 0x60;
    BatterSendBuffer[6] = 0x7F;
    BatterSendBuffer[7] = 0x07;
    BatterSendBuffer[8] = 0x4C;
    BatterSendBuffer[9] = 0xAF;
    BatterSendBuffer[10] = 0xA0;
    // 데이터 송신
    for (int i = 0; i < 11; i++)
        serialPutchar(PortID, BatterSendBuffer[i]);

    int StartTimeCheck = millis();
    int CurrentTime = 0;
    int TempBuffer;
    while (1)
    {
        CurrentTime = millis();
        if (CurrentTime - StartTimeCheck > ReciveTimeout)
        {
            printf("Battery Reqest : Time Over \n");
            return;
        }

        if (serialDataAvail(PortID))
        {
            int DataSize = 0;
            bool DataValid = false;
            uint8_t STX_01 = 0;
            while(1)
            {
                if( serialDataAvail(PortID) == false )
                    break;

                if( DataValid == false )
                {
                  STX_01 = serialGetchar(PortID);
                  if( STX_01 == 0xaf && serialGetchar(PortID) == 0xfa )
                  {
                      BatReciveBuffer[0] = 0xaf;
                      BatReciveBuffer[1] = 0xfa;
                      DataSize = 2;
                      DataValid = true;
                  }
                }
                else if(DataValid)
                {
                  BatReciveBuffer[DataSize] = serialGetchar(PortID);
                  //printf (" -> %1x", BatReciveBuffer[DataSize]) ;
                  DataSize++;

                  if( DataSize > 30 )
                    break;
                }
            }

            TempBuffer = BatReciveBuffer[6];
            TempBuffer = (TempBuffer << 8 ) | BatReciveBuffer[7];
            Bat_Status.voltage = TempBuffer * 0.01;

            TempBuffer = BatReciveBuffer[8];
            TempBuffer = (TempBuffer << 8 ) | BatReciveBuffer[9];
            Bat_Status.current = TempBuffer * 0.01;

            TempBuffer = BatReciveBuffer[10];
            TempBuffer = (TempBuffer << 8 ) | BatReciveBuffer[11];
            Bat_Status.percentage = TempBuffer;

            TempBuffer = BatReciveBuffer[14];
            TempBuffer = (TempBuffer << 8 ) | BatReciveBuffer[15];
            Bat_Status.charge = TempBuffer;

            TempBuffer = BatReciveBuffer[18];
            TempBuffer = (TempBuffer << 8 ) | BatReciveBuffer[19];
            Bat_Status.temperature = TempBuffer * 0.1;

            TempBuffer = BatReciveBuffer[22];
            TempBuffer = (TempBuffer << 8 ) | BatReciveBuffer[23];
            Bat_Status.capacity = TempBuffer;
            return;
        }
    }
}

//char *REMOTE_CONTROL_PORT = const_cast<char*>("/dev/ttyUSB2");
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto BatterCtrl = std::make_shared<battery>();
  RCLCPP_INFO(BatterCtrl->get_logger(), "Battery Serial port start");
  if( BatterCtrl->Open())
  {
    RCLCPP_INFO(BatterCtrl->get_logger(), "Battery Serial port open");
    rclcpp::spin(BatterCtrl);
    rclcpp::shutdown();
  }


  return 0;
}
