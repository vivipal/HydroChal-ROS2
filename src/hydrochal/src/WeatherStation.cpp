#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/heading.hpp"
#include "interfaces/msg/wind.hpp"

#include <iostream>
#include <fstream>
#include <sstream>


class WeatherStationNode : public rclcpp::Node {
  public:
     WeatherStationNode() : Node("weather_station_node") {
       gps_publisher_ = this->create_publisher<interfaces::msg::GPS>("GPS",10);
       wind_publisher_ = this->create_publisher<interfaces::msg::WIND>("WIND",10);
       heading_publisher_ = this->create_publisher<interfaces::msg::HEADING>("HEADING",10);
     }

     rclcpp::Publisher<interfaces::msg::GPS>::SharedPtr gps_publisher_;
     rclcpp::Publisher<interfaces::msg::WIND>::SharedPtr wind_publisher_;
     rclcpp::Publisher<interfaces::msg::HEADING>::SharedPtr heading_publisher_;
};

interfaces::msg::GPS getGPSmsg(std::vector<std::string> values){
  interfaces::msg::GPS msg;
  msg.utc_time.h = std::stoi( values[1].substr(0,2) );
  msg.utc_time.m = std::stoi( values[1].substr(2,2) );
  msg.utc_time.s = std::stoi( values[1].substr(4,2) );
  msg.date.day = std::stoi( values[9].substr(0,2) );
  msg.date.month = std::stoi( values[9].substr(2,2) );
  msg.date.year = std::stoi( values[9].substr(4,2) );

  msg.gps_status = values[2];
  msg.latitude = std::stof( values[3] )/100;
  if (values[4]=="S"){msg.latitude*=-1;}
  msg.longitude = std::stof( values[5] )/100;
  if (values[6]=="W"){msg.longitude*=-1;}
  msg.sog = std::stoi( values[7] );
  msg.cog = std::stoi( values[8] );

  return msg;

}

interfaces::msg::WIND getWINDmsg(std::vector<std::string> values){
  interfaces::msg::WIND msg;

  msg.true_wind_direction = std::stoi(values[13]);
  msg.wind_direction = std::stoi(values[15]);
  msg.wind_speed = std::stoi(values[19]);

  return msg;

}

interfaces::msg::HEADING getHEADINGmsg(std::vector<std::string> values){
  interfaces::msg::HEADING msg;

  msg.heading = std::stoi(values[1]);
  msg.variation = std::stoi(values[4]);
  if (values[5].substr(0,1) == "W"){ msg.variation*=-1.;}

  return msg;

}


int main(int argc, char const *argv[]) {

  rclcpp::init(argc,argv);
  auto n = std::make_shared<WeatherStationNode>();

  const char* device = "/dev/ttyUSB0"; // Path to your serial device
  std::ifstream serialStream(device);

  if (!serialStream.is_open()) {
    std::cerr << "Error opening serial port" << std::endl;
    return 1;
  }


  std::string line;
  std::getline(serialStream, line);
  while (rclcpp::ok()) {

    std::getline(serialStream, line);
    if (!line.empty()){
      std::cout << "Received data: " << line << std::endl;

      std::stringstream ss(line); // convert string to stream to use getline()
      std::vector<std::string> values;
      std::string val;
      while (std::getline(ss, val, ',')) {
        values.push_back(val);
      }

      if (values[0]=="$GPRMC") {
        interfaces::msg::GPS msg = getGPSmsg(values);
        n->gps_publisher_->publish(msg);
      }
      else if (values[0]=="$WIMDA") {
        interfaces::msg::WIND msg = getWINDmsg(values);
        n->wind_publisher_->publish(msg);
      }
      else if (values[0]=="$HCHDG") {
        interfaces::msg::HEADING msg = getHEADINGmsg(values);
        n->heading_publisher_->publish(msg);
      }

    }
  }

  // Close the serial port when done (optional for input streams)
  serialStream.close();

  return 0;
}
