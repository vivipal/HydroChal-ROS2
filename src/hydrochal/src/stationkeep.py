#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from interfaces.msg import WIND
from interfaces.msg import GPS
from interfaces.msg import HEADING
from interfaces.msg import YPR

from utils import coord2cart


class stationNode(Node):

    def __init__(self, coord_station):
        super().__init__('station_node')

        ### BASE INITIALIZATION
        # Init sensors subscribers
        self.create_subscription(GPS, '/GPS', self.coord_callback, 10)
        self.create_subscription(WIND, '/WIND', self.wind_callback, 10)
        self.create_subscription(HEADING, '/HEADING', self.heading_callback, 10)
        self.create_subscription(YPR, '/YPR', self.yaw_callback, 10)

        # Init publisher vers topic "Plannif"
        self.create_publisher(Float32, '/cmd_flap', 10)
        self.create_publisher(Float32, '/cmd_rudder', 10)

        # Init main loop 5Hz
        self.create_timer(0.2, self.loop)
        
        # Init Variables de classe [x, y, vitesse, cap, ang_voile]
        self.X = [0., 0., 0., 0., 0.]

        # Init wind variables [twd, awd, aws]
        self.wind = [0., 0., 0.]

        # Init sensors status
        self.gps_received = False
        self.imu_received = False
        self.wind_received  = False 
        self.heading_received = False

        ### INITIALIZATION
        self.coord_station = coord_station



    def coord_callback(self, msg):
        self.gps_received = True
        x, y = coord2cart((msg.latitude, msg.longitude))
        self.X = [x, y, msg.sog]

    def wind_callback(self, msg : WIND):
        self.wind_received = True
        self[0] = msg.true_wind_direction   # twd
        self[1] = msg.wind_direction        # awd
        self[2] = msg.wind_speed            # aws

    def heading_callback(self, msg : HEADING):
        self.heading_received = True
        self.X[3] = msg.heading

    def yaw_callback(self, msg : YPR):
        self.imu_received = True
        self.X[4] = msg.yaw                 # ang_voile

    def loop(self):
        if not (self.gps_received and self.imu_received and self.wind_received and self.heading_received):
            print("Waiting for : " + "GPS"*self.coord_callback + "WIND"*self.wind_callback + "HEADING"*self.heading_callback + "YPR"*self.yaw_callback)
        else:
            print("GPS : ", self.X[:2])
            print("Heading : ", self.X[3])
            print("Wind Yaw : ", self.X[4])
            print("Wind : ", self.awd, self.twd, self.aws)

            # msg_flap_cmd = Float32()
            # msg_rudder_cmd = Float32()
            # msg_flap_cmd.data = float(self.delta_s_max)
            # self.flap_publisher.publish(m)
            # msg_rudder_cmd.data = float(self.delta_r)
            # self.rudder_publisher.publish(m)





def main(args=None):
    rclpy.init(args=args)

    WP = (48.1990631, -3.0158815)
    node = stationNode(WP)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':

    main()

