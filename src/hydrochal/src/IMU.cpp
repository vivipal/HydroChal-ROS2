#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/ypr.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

class IMUNode : public rclcpp::Node {
  public:
    IMUNode(): Node("imu_node"){
      publisher_ = this->create_publisher<interfaces::msg::YPR>("YPR",10);
    }

    rclcpp::Publisher<interfaces::msg::YPR>::SharedPtr publisher_;
};



int main(int argc, char const *argv[]) {

  rclcpp::init(argc,argv);
  auto n = std::make_shared<IMUNode>();



  const char* device = "/dev/ttyUSB1"; // Path to the serial device
  std::ifstream serialStream(device);

  if (!serialStream.is_open()) {
    std::cerr << "Error opening serial port" << std::endl;
    return 1;
  }else{
    std::cout << "Serial port openned" << '\n';
  }

  std::string line;
  std::getline(serialStream, line);
  while (rclcpp::ok()) {
    std::getline(serialStream, line);
    if (!line.empty()){

      std::vector<std::string> values;
      std::stringstream ss(line);
      std::string val;
      std::getline(ss, val, '='); // read the "YPR="

      interfaces::msg::YPR msg;

      std::getline(ss,val,',');
      msg.yaw = std::stof(val);
      std::getline(ss,val,',');
      msg.pitch = std::stof(val);
      std::getline(ss,val,',');
      msg.roll = std::stof(val);

      n->publisher_->publish(msg);



    }
  }

  serialStream.close();

  return 0;
}
