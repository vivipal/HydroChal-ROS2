#!/usr/bin/env python3
import numpy as np
from numpy import cos,sin
from numpy.linalg import norm,det

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from interfaces.msg import WIND
from interfaces.msg import GPS
from interfaces.msg import HEADING
from interfaces.msg import YPR


def sawtooth(x):
    x = x+np.pi
    return ((x + np.pi) % (2 * np.pi) - np.pi)/np.pi  # or equivalently   2*arctan(tan(x/2))


R = 6378000 # earth radius in meter
LAT_REF = 48.19924
LON_REF = -3.01461 



def coord2cart(coords,coords_ref=(LAT_REF,LON_REF)):
    '''
    in :
        coords = (lat,lon)
        coords_ref = centre du plan
    return :
        x_tilde et y_tilde coord sur le plan centre en coord_ref
    '''

    ly,lx = coords
    lym,lxm = coords_ref

    x_tilde = R * cos(ly*np.pi/180)*(lx-lxm)*np.pi/180
    y_tilde = R * (ly-lym)*np.pi/180

    return np.array([[x_tilde,y_tilde]]).T

def is_waypoint_passed(next_wp_coord,previous_wp_coord,cm):

    xy_nwp = coord2cart(next_wp_coord)
    xy_pwp = coord2cart(previous_wp_coord)
    xy_m = coord2cart(cm)

    return (((xy_nwp-xy_pwp)@(xy_m-xy_nwp).T)[0,0] > 0)



## Partie ROS

class regulationNode(Node):

    def __init__(self, WPs):
        super().__init__('regulation_node')

        self.WPs = WPs
        self.wp_passed = 0
        self.r = 10 # Taille couloirs:

        self.coord_subscriber = self.create_subscription(GPS, '/GPS', self.coord_callback, 10)
        self.wind_subscriber = self.create_subscription(WIND, '/WIND', self.wind_callback, 10)
        self.heading_subscriber = self.create_subscription(HEADING, '/HEADING', self.heading_callback, 10)
        self.ypr_subscriber = self.create_subscription(YPR, '/YPR', self.yaw_callback, 10)

        
        # Init publisher vers topic "Plannif"
        self.flap_publisher = self.create_publisher(Float32, '/cmd_flap', 10)
        self.rudder_publisher = self.create_publisher(Float32, '/cmd_rudder', 10)
        
        # Init Variables de classe

        self.X = [0,0,0]
        self.heading = 0.0 # Cap véhicule 
        self.sail_yaw = 0.0 # angle voile

        self.DELTA_R_MAX = 45
        self.BETA = 0.5
        self.ZETA = 0.2

        self.awd = 0 
        self.twd = 0
      
        self.lat = (0,0)
        self.lon = (0,0)
        self.a, self.b = coord2cart(WPs[0]), coord2cart(WPs[1])
        self.waypoint_passed = 0
        # self.update_line()

        self.gps_received = False
        self.imu_received = False
        self.wind_received  = False 
        self.heading_received = False

        

        print("Init done")
       
        self.ready = False

        print("waiting for GPS")
        while not self.gps_received:
            pass

        print("waiting for wind")
        while not self.wind_received:
            pass
        
        print("waiting for heading")
        while not self.heading_received: 
            pass
        
        print("waiting for imu")
        while not self.imu_received:
            pass

        print("ready")
        self.ready = True

        update_line()

    def publish_command(self):
        msg_flap_cmd = Float32()
        msg_rudder_cmd = Float32()

        msg_flap_cmd.data = float(self.delta_s_max)
        self.flap_publisher.publish(m);

        msg_rudder_cmd.data = float(self.delta_r)
        self.rudder_publisher.publish(m);


    
    def update_command(self):           
        psi_ap= self.awd
        sigma_r_max = 45 # angle max du gouvernail
        psi = self.twd

        theta = self.heading

        a,b = self.a.reshape((2,1)), self.b.reshape((2,1))

        m = self.X[:2]
        e=np.linalg.det( np.hstack(((b-a)/np.linalg.norm(b-a),m-a)) )

        phi=np.arctan2(b[1,0]-a[1,0],b[0,0]-a[0,0])

        if abs(e)>self.r :
            self.q=np.sign(e)
        theta_bar = phi - np.arctan(e/self.r)
        if ( ((np.cos(psi-theta_bar)+ np.cos(self.ZETA)) < 0) or ((abs(e)-self.r < 0) and (np.cos(psi-phi) + np.cos(self.zeta)) < 0) ):
            theta_bar = np.pi + psi-self.ZETA*self.q
        self.delta_r = self.DELTA_R_MAX / np.pi * sawtooth(theta-theta_bar)
        # self.delta_s_max = np.pi/2*((np.cos(psi-theta_bar)+1)/2)**(np.log(np.pi/2/self.BETA)/np.log(2)) / np.pi*180
        self.delta_s_max = 1 if (twd - heading) > 180 else - 1



    def coord_callback(self, msg):
        self.gps_received |= True
        self.lat, self.lon = msg.latitude, msg.longitude
        x, y = coord2cart((self.lat, self.lon))
        self.X = [x,y,msg.sog]
        self.update_line()

        if self.ready : self.update_command()

    def wind_callback(self, msg : WIND):
        wind_received |= True
        self.twd = msg.true_wind_direction
        self.awd = msg.wind_direction
        self.aws = msg.wind_speed
        
        if self.ready : self.update_command()


    def heading_callback(self, msg : HEADING):
        self.heading_received |= True
        self.heading=msg.heading

        if self.ready : self.update_command()

    def yaw_callback(self, msg : YPR):
        self.imu_received |= True
        self.sail_yaw = msg.yaw


   
    def update_line(self):
        
        if is_waypoint_passed(self.WPs[self.waypoint_passed+1], self.WPs[self.waypoint_passed], (self.lat, self.lon)) :
            self.waypoint_passed += 1;
            self.a, self.b = coord2cart(WPs[self.waypoint_passed]), coord2cart(WPs[self.waypoint_passed+1])

        



              
def main(args=None):
    rclpy.init(args=args)


    WPs = [[48.198861, -3.013927],[48.195524, -3.018772],[48.19881, -3.01542],[48.19923, -3.01474]]

    regulation_node = regulationNode(WPs)
    rclpy.spin(regulation_node)


    regulation.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':

    main()

