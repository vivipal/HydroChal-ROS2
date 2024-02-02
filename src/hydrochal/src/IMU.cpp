#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/ypr.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>


class IMUNode : public rclcpp::Node {
  public:
    IMUNode(): Node("imu_node"){
      publisher_ = this->create_publisher<interfaces::msg::YPR>("YPR",10);
    }

    rclcpp::Publisher<interfaces::msg::YPR>::SharedPtr publisher_;
};



int main(int argc, char const *argv[]) {

  if (argc != 2) {
    std::cerr << "Please provide IMU device path" << std::endl;
    return 1;
  }

  std::string device_path = argv[1];

  std::ifstream serialStream(device_path);

  if (!serialStream.is_open()) {
    std::cerr << "Error opening serial port" << std::endl;
    return 1;
  }else{
    std::cout << "Serial port openned: " << device_path << '\n';
  }


  rclcpp::init(argc,argv);
  auto n = std::make_shared<IMUNode>();

  int c = 0;

  std::string line;
  std::getline(serialStream, line);
  while (rclcpp::ok()) {

    std::getline(serialStream, line);
    std::cout << line << std::endl;
    if (!line.empty()){
      c++;

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


      if ( c >= 5){
        n->publisher_->publish(msg);
        c = 0;
      }



    }
  }

  serialStream.close();

  return 0;
}
