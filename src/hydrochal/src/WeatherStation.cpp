#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/heading.hpp"
#include "interfaces/msg/wind.hpp"

#include <iostream>
#include <fcntl.h> 
#include <termios.h> 

#include <stdexcept>

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

  try {
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
  }catch(const std::invalid_argument& e){
    std::cout << "Error GPS" << std::endl;
  }

  return msg;
}

interfaces::msg::WIND getWINDmsg(std::vector<std::string> values){
  interfaces::msg::WIND msg;
  
  try{ 
    msg.true_wind_direction = std::stoi(values[13]);
    msg.wind_direction = std::stoi(values[15]);
    msg.wind_speed = std::stoi(values[19]);
  }catch(const std::invalid_argument& e){
    std::cout << "Error wind" << std::endl;
  }

  return msg;
}

interfaces::msg::HEADING getHEADINGmsg(std::vector<std::string> values){
  interfaces::msg::HEADING msg;

  try {
    msg.heading = std::stoi(values[1]);
    msg.variation = std::stoi(values[4]);
    if (values[5].substr(0,1) == "W"){ msg.variation*=-1.;}
  }catch(const std::invalid_argument& e){
    std::cout << "Error heading" << std::endl;
  }

  return msg;
}


void wait_new_message(int fd){
  char buffer[2];

  // wait for begin of a sentence ie '#'
  do {
    read(fd,buffer,1);
  } while(*buffer!='$');

}

void set_baudrate(int fd){
  struct termios options;

  //Get the current options for the port...
  tcgetattr(fd, &options);

  //Set the baud rates to 4800...
  cfsetispeed(&options, B4800);
  cfsetospeed(&options, B4800);

  // Enable the receiver and set local mode...
  options.c_cflag |= (CLOCAL | CREAD);

  // Set 8N1
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // Read data as raw
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  // Set the new options for the port...
  tcsetattr(fd, TCSANOW, &options);
}

void receive_message(int fd,char *receivedMessage){
  bool end_message = false;
  char buffer[2];
  while (!end_message){
    read(fd,buffer,1);
    if (*buffer=='*'){
      end_message = true;
    }else{
      strcat(receivedMessage,buffer);
    }
  }
}

int main(int argc, char const *argv[]) {

  if (argc != 2) {
    std::cerr << "Please provide Weather Station device path" << std::endl;
    return 1;
  }

  std::string device_path = argv[1];

  int fd = open(device_path.c_str(), O_RDONLY | O_NOCTTY  );
  if (fd < 0) {
    std::cerr << "Error opening serial port: " << device_path << std::endl;
    return 1;
  }
  
  fcntl(fd, F_SETFL,0);
  set_baudrate(fd);

  std::cout << "Serial port openned: " << device_path << '\n';



  rclcpp::init(argc,argv);
  auto n = std::make_shared<WeatherStationNode>();

  while (rclcpp::ok()) {
    wait_new_message(fd);

    char receivedMessage[200] = {0};
    receive_message(fd,receivedMessage);

    // std::cout << "Received data: " << receivedMessage << std::endl;
    // std::cout << strlen(receivedMessage) << "\n\n";

    std::stringstream ss(receivedMessage); // convert char[] to stream to use getline()
    std::vector<std::string> values;
    std::string val;
    while (std::getline(ss, val, ',')) {
      values.push_back(val);
    }

    if (values[0]=="GPRMC") {
      interfaces::msg::GPS msg = getGPSmsg(values);
      n->gps_publisher_->publish(msg);
    }
    else if (values[0]=="WIMDA") {
      interfaces::msg::WIND msg = getWINDmsg(values);
      n->wind_publisher_->publish(msg);
    }
    else if (values[0]=="HCHDG") {
      interfaces::msg::HEADING msg = getHEADINGmsg(values);
      n->heading_publisher_->publish(msg);
    }
  }

  // Close the serial port when done
  close(fd);

  return 0;
}
