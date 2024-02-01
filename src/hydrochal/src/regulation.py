#!/usr/bin/env python3
import numpy as np
from numpy.linalg import norm,det
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from interfaces.msg import WIND
from interfaces.msg import GPS
from interfaces.msg import HEADING
from interfaces.msg import YPR
from interfaces.msg import regulation
#ROBLIB


def angle(x):
    x=x.flatten()
    return np.arctan2(x[1],x[0])

def add1(M):
    M=np.array(M)
    return np.vstack((M,np.ones(M.shape[1])))

def tran2H(x,y):
    return np.array([[1,0,x],[0,1,y],[0,0,1]])

def rot2H(a):
    return np.array([[cos(a),-sin(a),0],[sin(a),cos(a),0],[0,0,1]])

#-------------------------------
   
def f(x,u):
    x,u=x.flatten(),u.flatten()
    θ=x[2]; v=x[3]; w=x[4]; δr=u[0]; u2=u[1]; δs=x[5];
    w_ap = np.array([[awind*np.cos(ψ-θ) - v*np.cos(ψ-θ)],[awind*np.sin(ψ-θ)]])
    ψ_ap = angle(w_ap)
    a_ap=np.linalg.norm(w_ap)
    fr = p4*v*np.sin(δr)
    fs = p3*(a_ap)* np.sin(δs-ψ_ap)
    dx=v*np.cos(θ) + p0*awind*np.cos(ψ)
    dy=v*np.sin(θ) + p0*awind*np.sin(ψ)
    dv=(fs*np.sin(δs)-fr*np.sin(δr)-p1*v**2)/p8
    dw=(fs*(p5-p6*np.cos(δs)) - p7*fr*np.cos(δr) - p2*w*v)/p9
    ff=(a_ap)* np.sin(δs-u2-ψ_ap)
    dδs = p10*ff*np.cos(u2)+   fs/p3 #=> voile
    xdot=np.array([[dx],[dy],[w],[dv],[dw],[dδs]])
    return xdot,ψ_ap,a_ap,fr,fs,ff


def regulateur (x,q,a,b):
    psi=2
    zeta=np.pi/4
    theta=x[2]
    m=np.array([x[0],x[1]]).reshape(2,1)
    f=np.hstack(((b-a)/np.linalg.norm(b-a),m-a))
    e=np.linalg.det(f)   
    phi=np.arctan2(b[1,0]-a[1,0],b[0,0]-a[0,0])
    if abs(e)>r/2 :
        q=np.sign(e)
    #theta_etoile=phi - (2*0.35/3.14)*arctan(e/r)
    theta_etoile=phi - np.arctan(e/r)
    if (((np.cos(psi-theta_etoile)+ np.cos(zeta)) < 0) or (abs(e)-r < 0 and (np.cos(psi-phi) + np.cos(zeta)) < 0)):
        thetabar=np.pi+psi-zeta*q
    else :
        thetabar=theta_etoile    
    if np.cos (thetabar-theta)>0:
        sigma_r=sigma_r_max*np.sin(theta-thetabar)
    else :
        sigma_r=sigma_r_max*np.sign(np.sin(theta-thetabar))
    #sigma_max=1
    sigma_s_max = np.pi/2*((np.cos(psi-thetabar)+1)/2)**q
    
    u =np.array([sigma_r,[sigma_s_max],[q]]).reshape(3,1)
   # print(u)
    return u,m


## Partie ROS

class Plannif_regulation(Node):

    def __init__(self):
        super().__init__('sailboat_publisher')

        # Constants
        self.p0 = 0.1
        self.p1 = 50
        self.p2 = 6000
        self.p3 = 1000
        self.p4 = 2000
        self.p5 = 0.01
        self.p6 = 1
        self.p7 = 2
        self.p8 = 300
        self.p9 = 10000
        self.p10 = 1
        self.r = 10
        self.sigma_r_max = 5
    
        msg = Float64()

        self.coord_subscriber = self.create_subscription(GPS, 'GPS', self.coord_callback, 10)
        self.wind_subscriber = self.create_subscription(WIND, 'WIND', self.wind_callback, 10)
        self.heading_subscriber = self.create_subscription(HEADING, 'HEADING', self.heading_callback, 10)
        self.ypr_subscriber = self.create_subscription(YPR, 'YPR', self.yaw_callback, 10)

        
        # Init publisher vers topic "Plannif"
        self.u_publisher_ = self.create_publisher(
            Float64, 
            'Commande', 
            10)


        #self.subscription  # prevent unused variable warning
        
        # Init Variables de classe

        self.P = [0.0,0.0,0.0]# Position + vitesse du boat (x,y,v)
        
        self.vent = [0.0,0.0,0.0] # vent_true,vent_ap,awind
                          
        self.heading= 0.0 # Cap véhicule
        
        self.yaw=0.0 # angle voile



    def coord_callback(self, msg : GPS):
        self.P=msg.GPS
        self.P = [msg.latitude,msg.longitude,msg.sog]

    def wind_callback(self, msg : WIND):
        self.vent = [msg.true_wind_direction,msg.wind_direction,msg.wind_speed]

    def heading_callback(self, msg : HEADING):
        self.heading=msg.heading
    def yaw_callback(self, msg : YPR):
        self.yaw=msg.yaw

    def main_loop(self):

        q=1
        # VARIABLE PROGRAMME
        ax=init_figure(-150,150,-80,80)
        i=0
        x = np.array([P[0,0],P[1,0],heading,P[2,0],0,yaw]).reshape(6,1)   #x=(x,y,θ,v,w,δs)
        # COORDONEE
        a = np.array([[75],[-75]])   
        b = np.array([[-75],[-75]])
        c=np.array([-75,20]).reshape(2,1)
        d=np.array([-150,20]).reshape(2,1)
        coord=[a,b,c,d]
        n=len(coord)
        ψ_ap=vent[2,0]
        sigma_r_max =90 # angle max du gouvernail
        awind,ψ = vent[3,0],vent[1,0] # vitesse du vent, angle du vent 

    def control (i,x,q) :
        while True :
            u,m=regulateur(x,q,coord[i],coord[i+1])
            vector_director = coord[i+1] - coord[i]
            vector_point_B= coord[i+1]
            if vector_director[0] == 0:
                # Segment vertical
                vector_orthogonal = np.array([1, 0])
            elif vector_director[1] == 0:
                # Segment horizontal
                vector_orthogonal = np.array([0, 1])
            else:
                # Segment diagonal
                coord_y_norm = -(vector_director[0] ** 2) / vector_director[1]
                vector_orthogonal = np.array([vector_director[0], coord_y_norm])

            x2 = coord[i+1][0][0] + vector_orthogonal[0]
            y2 = coord[i+1][1][0] + vector_orthogonal[1]
            c=np.array([x2,y2]).reshape(2,1)
         
            vecteur_ab = coord[i+1] - c

            # Vecteur entre le point a et votre position
            vecteur_a_vous = m - c

            # Calcul du produit vectoriel entre le vecteur ab et le vecteur a_vous
            produit_vectoriel = np.cross(vecteur_ab.T, vecteur_a_vous.T)
            
            if produit_vectoriel <0:
               i=i+1
            if i==n-1:
                break

            u_msg = regulations()
            u_msg.u_rudder= u[0,0]
            u_msg.u_flap= u[1,0]
            self.u_publisher_.publish(u_msg)


def main(args=None):
    rclpy.init(args=args)
    regulation = Plannif_regulation()

    rclpy.spin(regulation)

    regulation.destroy_node()
    rclpy.shutdown()
