#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/ypr.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <fcntl.h> 
#include <termios.h> 


class IMUNode : public rclcpp::Node {
  public:
    IMUNode(): Node("imu_node"){
      publisher_ = this->create_publisher<interfaces::msg::YPR>("YPR",10);
    }

    rclcpp::Publisher<interfaces::msg::YPR>::SharedPtr publisher_;
};

void set_baudrate(int fd){
  struct termios options;

  //Get the current options for the port...
  tcgetattr(fd, &options);

  //Set the baud rates to 4800...
  cfsetispeed(&options, B57600);
  cfsetospeed(&options, B57600);

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
    if (*buffer=='\n'){
      end_message = true;
    }else{
      strcat(receivedMessage,buffer);
    }
  }
}



void wait_new_message(int fd){
  char buffer[2];

  // wait for begin of a sentence ie '#'
  do {
    read(fd,buffer,1);
  } while(*buffer!='#');

}




int main(int argc, char const *argv[]) {

  if (argc != 2) {
    std::cerr << "Please provide IMU device path" << std::endl;
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
  auto n = std::make_shared<IMUNode>();

  double yaw, pitch,roll;

  while (rclcpp::ok()){
    wait_new_message(fd);

    char receivedMessage[200] = {0};
    receive_message(fd,receivedMessage);

    if (sscanf(receivedMessage, "YPR=%lf,%lf,%lf", &yaw, &pitch, &roll) == 3) {

      // std::cout << "Yaw: " << yaw << ", Pitch: " << pitch << ", Roll: " << roll << std::endl;
      
      interfaces::msg::YPR msg;
      msg.yaw = yaw;
      msg.pitch = pitch;
      msg.roll = roll;

      n->publisher_->publish(msg);
    }else{
      std::cerr << "Error parsing values from the line: " << std::endl;
      std::cout <<  std::endl << "----\n";
    }
  }

  return 0;
}
