#!/usr/bin/env python3
import numpy as np
from numpy import cos,sin
from numpy.linalg import norm,det

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from interfaces.msg import WIND
from interfaces.msg import GPS
from interfaces.msg import HEADING
from interfaces.msg import YPR

R = 6378000 # earth radius in meter
LAT_REF = 48.19924
LON_REF = -3.01461 

def sawtooth(x):
    return ((x + np.pi) % (2 * np.pi) - np.pi) # or equivalently   2*arctan(tan(x/2))

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

    y_tilde = R * cos(ly*np.pi/180)*(lx-lxm)*np.pi/180
    x_tilde = R * (ly-lym)*np.pi/180

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
        self.r_lane_front = 7 # taille couloir louvoiement en vent avant
        self.r_lane_back = 4 # taille couloir louvoiement en vent arrière
        self.r_follow = 7 # coeff de suivi de ligne
        self.q_front = 1
        self.q_back = 1


        self.gps_received = False
        self.imu_received = False
        self.wind_received  = False 
        self.heading_received = False
        
        self.coord_subscriber = self.create_subscription(GPS, '/GPS', self.coord_callback, 10)
        self.wind_subscriber = self.create_subscription(WIND, '/WIND', self.wind_callback, 10)
        self.heading_subscriber = self.create_subscription(HEADING, '/HEADING', self.heading_callback, 10)
        self.ypr_subscriber = self.create_subscription(YPR, '/YPR', self.yaw_callback, 10)

        
        # Init publisher vers topic "Plannif"
        self.flap_publisher = self.create_publisher(Float32, '/cmd_flap', 10)
        self.rudder_publisher = self.create_publisher(Float32, '/cmd_rudder', 10)
        self.desired_heading_publisher = self.create_publisher(Float32, '/desired_heading', 10)
        
        # Init Variables de classe

        self.X = [0,0,0]
        self.heading = 0.0 # Cap véhicule 
        self.sail_yaw = 0.0 # angle voile

        # self.DELTA_R_MAX = 40
        self.ZETA_front = np.pi/6 # angle de louvoiement en vent avant
        self.ZETA_back = np.pi/8 # angle de louvoiement en vent arrière

        self.awd = 0 
        self.twd = 0
      
        self.lat = 0
        self.lon = 0
        self.a, self.b = coord2cart(WPs[0]), coord2cart(WPs[1])
        self.waypoint_passed = 0
        
        
    def publish_command(self):
        msg_flap_cmd = Float32()
        msg_rudder_cmd = Float32()
        msg_desired_heading = Float32()

        msg_flap_cmd.data = float(self.delta_s_max)
        self.flap_publisher.publish(msg_flap_cmd)

        msg_rudder_cmd.data = float(self.delta_r)
        self.rudder_publisher.publish(msg_rudder_cmd)

        msg_desired_heading.data = float(self.desired_heading)
        self.desired_heading_publisher.publish(msg_desired_heading)


    
    def update_command(self):  
        sigma_r_max = 40 # angle max du gouvernail
        # angles du vent (apparent et réel) renvoyé par la station météo, 0° Nord, +90° Est
        psi_ap = self.awd *np.pi/180
        psi = (self.twd*np.pi/180 )+np.pi   # angle du vecteur vent
        # angle renvoyé par la station météo, 0° Nord, +90° Est
        theta = self.heading*np.pi/180

        # coordonnées du robot
        a,b = self.a.reshape((2,1)), self.b.reshape((2,1))

        # position du robot dans le plan du lac x positif vers le Nord, y positif vers l'Est (comme la boussolle x positif vers l'avant, y positif vers tribord)
        m = self.X[:2]
        # distance entre le bateau et le segment reliant les deux waypoints
        e = np.linalg.det( np.hstack(((b-a)/np.linalg.norm(b-a),m-a)) )

        # angle du segment reliant les deux waypoints
        phi=np.arctan2(b[1,0]-a[1,0],b[0,0]-a[0,0])

        # q vaut -1 ou 1 selon si le bateau est à droite ou à gauche, change lorsque le bateau s'éloigne suffisamment de la ligne
        if abs(e)>self.r_lane_front :
            self.q_front=np.sign(e)
        if abs(e)>self.r_lane_back :
            self.q_back=np.sign(e)

        # cap de consigne
        theta_bar = phi - np.arctan(e/self.r_follow)

        # si le bateau est face au vent, on change theta_bar
        if ( ((np.cos(psi-theta_bar)+ np.cos(self.ZETA_front)) < 0) or ((abs(e)-self.r_lane_front < 0) and (np.cos(psi-phi) + np.cos(self.ZETA_front)) < 0) ):
            theta_bar = np.pi + psi-self.ZETA_front*self.q_front
            # self.get_logger().info("face au vent")

        # si le bateau est dos au vent, on change theta_bar aussi
        # if ( ((np.cos(psi+np.pi-theta_bar)+ np.cos(self.ZETA_back)) < 0) or ((abs(e)-self.r_lane_back < 0) and (np.cos(psi+np.pi-phi) + np.cos(self.ZETA_back)) < 0) ):
        #     theta_bar =  psi-self.ZETA_back*self.q_back
            # self.get_logger().info("face au vent")

        # publish desired heading
        self.desired_heading = theta_bar*(180/np.pi)

        # angle du safran (rudder)
        self.delta_r = (180. / np.pi) * (sawtooth(theta_bar-theta))
        self.delta_r = np.tanh(self.delta_r/45)*sigma_r_max     # commande en tangente hyperbolique valeur max à partir d'une erreur de 45°

        # angle de l'aileron (flap) -28° ou 28° selon l'orientation du vent par rapport au cap du bateau                            
        self.delta_s_max = 28 if sawtooth( psi- theta) > 0 else -28

        print(f"theta: {theta} \ttheta_bar: {theta_bar} \t saw: {sawtooth(theta-theta_bar)}")
        self.publish_command()

    def ready(self):
        return self.gps_received & self.wind_received & self.heading_received

    def coord_callback(self, msg):
        self.gps_received |= True
        # self.lat, self.lon = ((msg.latitude*100)/100, msg.longitude
        latitude_deg =  int((msg.latitude*100)/100)
        longitude_deg =  int((msg.longitude*100)/100)
        self.lat = latitude_deg  + (((msg.latitude*100)) - latitude_deg*100)/60
        self.lon = longitude_deg  + (((msg.longitude*100)) - longitude_deg *100)/60           
        x, y = coord2cart((self.lat, self.lon))
        self.X = [x,y,msg.sog]

        if self.ready() : 
            self.update_line()
            self.update_command()

    def wind_callback(self, msg : WIND):
        self.wind_received |= True
        # pour un vent à 0° (Nord) le vent vient du 0° (donc le vecteur vent est orienté vers le 180°)
        self.twd = msg.true_wind_direction
        self.awd = msg.wind_direction
        self.aws = msg.wind_speed
        
        if self.ready() : self.update_command()


    def heading_callback(self, msg : HEADING):
        self.heading_received |= True
        self.heading=msg.heading

        if self.ready() : self.update_command()

    def yaw_callback(self, msg : YPR):
        self.imu_received |= True
        self.sail_yaw = msg.yaw


   
    def update_line(self):
        
        if is_waypoint_passed(self.WPs[self.waypoint_passed+1], self.WPs[self.waypoint_passed], (self.lat, self.lon)) :
            self.waypoint_passed += 1
            self.a, self.b = coord2cart(self.WPs[self.waypoint_passed]), coord2cart(self.WPs[self.waypoint_passed+1])

        
        self.get_logger().info(f"going to wp n°{self.waypoint_passed}")



              
def main(args=None):
    rclpy.init(args=args)


    # WPs = [[48.198861, -3.013927],[48.195524, -3.018772],[48.19881, -3.01542],[48.19923, -3.01474]]

    # WPs = [[48.198904,-3.015555],[48.197997,-3.015126],[48.198290,-3.016366],[48.198904,-3.015555]]

    # WPs = [[48.198802, -3.013615], [48.198574, -3.012860], [48.198432, -3.012944], [48.198609, -3.013690]]      # waypoints parking

    # mission triangle 
    # WPs = [[48.198911, -3.015560], [48.198297, -3.016291], [48.198193, -3.015298], [48.198911, -3.015560]]

    # big mission
    WPs = [[48.198896, -3.015598],[48.196977, -3.015489],[48.196814, -3.019509],[48.197085, -3.021600],[48.198009, -3.018803],[48.198208, -3.016603],[48.198896, -3.015598]]

    regulation_node = regulationNode(WPs)
    rclpy.spin(regulation_node)


    regulation_node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':

    main()

