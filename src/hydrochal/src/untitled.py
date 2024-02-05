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

#ROBLIB

def clear(ax):
    plt.pause(0.001)
    plt.cla()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)

def init_figure(xmin,xmax,ymin,ymax): 
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal')   
    ax.xmin=xmin
    ax.xmax=xmax
    ax.ymin=ymin
    ax.ymax=ymax
    clear(ax)
    return ax

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

def draw_arrow(x,y,θ,L,col='darkblue',w=1):
    plot2D(tran2H(x,y)@rot2H(θ)@arrow2H(L),col,w)

def plot2D(M,col='black',w=1):
    plt.plot(M[0, :], M[1, :], col, linewidth = w) 


def arrow2H(L):
    e=0.2
    return add1(L*np.array([[0,1,1-e,1,1-e],[0,0,-e,0,e]]))

def draw_disk(ax,c,r,col,alph=0.7,w=1):
    #draw_disk(ax,array([[1],[2]]),0.5,"blue")
    e = Ellipse(xy=c, width=2*r, height=2*r, angle=0,linewidth = w)   
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(alph)  # transparency
    e.set_facecolor(col)

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

#-------------------------------------

def regulateur (x,q,a,b,r,sigma_r_max):
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

#------------------Visualization-------------------
def draw_saildrone(x,u,ψ,awind,ψ_ap,a_ap,fr,fs,ff):
    mx,my,θ,v,w,δs=list(x[0:6,0])
    u1,u2=list(u[0:2,0])
    hull=add1(np.array([[-1,5,7,7,5,-1,-1,-1],[-2,-2,-1,1,2,2,-2,-2]]))
    sail=np.array([[-5,2.5],[0,0],[1,1]])
    rudder=np.array([[-1,1],[0,0],[1,1]])
    R=tran2H(mx,my)@rot2H(θ)
    Rs=tran2H(3,0)@rot2H(δs)
    Rf=tran2H(-6,0)@rot2H(-u2)
    Rr=tran2H(-1,0)@rot2H(u1)
    draw_arrow(17,17,ψ,5*awind,'red')
    draw_arrow(17,17,ψ_ap+θ,5*a_ap,'green')
    plot2D(R@Rs@Rf@rot2H(-np.pi/2)@arrow2H(ff),'blue')
    plot2D(R@Rs@rot2H(-np.pi/2)@arrow2H(0.001*fs),'blue')
    plot2D(R@Rr@rot2H(np.pi/2)@arrow2H(0.001*fr),'blue')
    draw_disk(ax,R@np.array([[3],[0],[1]]),0.5,"red")
    plot2D(R@hull,'black');
    plot2D(R@Rs@sail,'red',2);
    plot2D(R@Rr@rudder,'red',2);
    plot2D(R@Rs@Rf@rudder,'green',2);


#--------------------------------------------------

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
    


    def control (self,i,x,q,coord) :
        while True :
            
            u,m=regulateur(x,q,coord[i],coord[i+1],self.r,self.sigma_r_max)
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
               xdot,ψ_ap,w_ap,fr,fs,ff=f(x,u)
               x = x + dt*xdot
               draw_saildrone(x,u,ψ,awind,ψ_ap,w_ap,fr,fs,ff)
            if i==3:
                break
  
    def main_loop(self):
        psi = 2
        zeta = 3.14/4
        q = 1
        ax = init_figure(-150, 150, -80, 80)
        i = 0
        x = np.array([100, -70, -3, 1, 0, 1]).reshape(6, 1)
        a = np.array([[75], [-75]])
        b = np.array([[-75], [-75]])
        c = np.array([-75, 20]).reshape(2, 1)
        d = np.array([-150, 20]).reshape(2, 1)
        coord = [a, b, c, d]
        n = len(coord)
        ψ_ap = 5
        sigma_r_max = 5
        awind, ψ = 3 * np.pi / 2, 2

        while rclpy.ok():
            self.control(i, x, q, coord)
            # Mettez à jour vos graphiques Matplotlib ici

            # Affichage de Matplotlib
            plt.pause(0.01)
            rclpy.spin_once()
            

"""

    def main_loop(self):

        psi=2
        zeta=3.14/4
       # ψ_ap=self.vent[1]
        q=1
       # awind= self.vent[2]
        #ψ = self.vent[0]


       # # VARIABLE PROGRAMME
        ax=init_figure(-150,150,-80,80)
        i=0
        x = np.array([100,-70,-3,1,0,1]).reshape(6,1)   #x=(x,y,θ,v,w,δs)
        # COORDONEE
        a = np.array([[75],[-75]])   
        b = np.array([[-75],[-75]])
        c=np.array([-75,20]).reshape(2,1)
        d=np.array([-150,20]).reshape(2,1)
        coord=[a,b,c,d]
        n=len(coord)
        ψ_ap=5
        sigma_r_max =5 # angle max du gouvernail
        awind,ψ = 3*np.pi/2,2 # vitesse du vent, angle du vent apparent

        self.control(i, x, q,coord)
"""
"""def ros_launch(plan):
    rclpy.spin(plan)
    plan.destroy_node()
    rclpy.shutdown()"""
    
def main(args=None):

    rclpy.init(args=args)
    sailboat_node = Plannif_regulation()
    sailboat_node.main_loop()  # Add your main control loop here
    rclpy.spin(sailboat_node)
    sailboat_node.destroy_node()
    rclpy.shutdown()
   

    # VARIABLE
    

if __name__ == '__main__':
    main()
