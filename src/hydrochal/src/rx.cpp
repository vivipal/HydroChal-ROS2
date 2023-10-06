#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/command.hpp"

#include "SBUS.h"

#include <iostream>
#include <chrono>


struct Data_str {
  int flap = 0 ;
  int rudder = 0;
};

Data_str data;

class RXNode : public rclcpp::Node {
  public:
    RXNode(): Node("rx_node"){
      publisher_ = this->create_publisher<interfaces::msg::Command>("rx",10);
    }

    rclcpp::Publisher<interfaces::msg::Command>::SharedPtr publisher_;
};


static void onPacket(const sbus_packet_t &packet){
    static auto lastPrint = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();

    if (now - lastPrint > std::chrono::milliseconds(500))
    {
        std::cout << "  flap: " << packet.channels[0] <<  '\n';
        std::cout << "  rudder: " << packet.channels[1]  << '\n';

        data.flap = packet.channels[0];
        data.rudder = packet.channels[1];

        lastPrint = now;
    }
}


int main(int argc, char const *argv[]) {

  rclcpp::init(argc,argv);
  auto n = std::make_shared<RXNode>();



  std::string ttydevice = "/dev/ttyUSB2";

  // initialisation of sbus
  static SBUS sbus;
  sbus.onPacket(onPacket);
  sbus_err_t err = sbus.install(ttydevice.c_str(), true);  // true for blocking mode
  if (err != SBUS_OK){
      std::cerr << "SBUS install error: " << err << std::endl;
      return err;
  }

  std::cout << "SBUS installed" << std::endl;

  interfaces::msg::Command msg;


  while ((err = sbus.read()) != SBUS_FAIL){

    // desync means a packet was misaligned and not received properly
    if (err == SBUS_ERR_DESYNC){
      std::cerr << "SBUS desync" << std::endl;
    }else{
      msg.rudder = data.rudder;
      msg.flap = data.flap;
      n->publisher_->publish(msg);
    }
  }

  return 0;
}
