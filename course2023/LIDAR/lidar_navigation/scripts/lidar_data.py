#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray



class Control():
    def __init__(self):
        self.front_dist=Float32MultiArray()
        self.front_angle=[]
        self.side_dist=Float32MultiArray()

#We're listenning to the /LidarScan topic, processing the data and publishing to /front_data and /side_data

def interpolate(data, ind):
    
    s,e=ind,ind
    #rospy.loginfo(f"ind={i}")
    #on cherche debut
    while data[s]==float('inf'):
        s-=1

    #chercher fin du trou   
    while data[e]==float('inf'):
        e+=1

    #FAIRE ATTENTIO DES FOIS INTERPOLATION COMPLETE DES TROUS QUI SONT VRM PRESENTS DANS LE CIRCUIT
    #PAS CRITIQUE MAIS FAUT FAIRE ATTENTION-> CONDITION DE SECURITE 
    # -> P.EX. SI LA PENTE DY/DX EST TROP GRANDE => PROBABLEMENT VRAI TROU


    #construction droite lineaire
    dx=e-s
    dy=data[e]-data[s]
    x0,y0=s,data[s]

    #print(s,e,dx,dy)
    #interpolation du trou
    if dx!=0: #safety
        for j in range(s+1,e):
            data[j]=data[s] + dy/dx * (j-s)
    
    return data #interpolated data with no holes


def lidar_preprocess_callback(msg,c):
     
    scan=np.array(msg.data)
    n=len(scan)
    
    intv=[int(rospy.get_param("angle0",default=120)/90 * n/4),
         int(rospy.get_param("angle1",default=240)/270 * 3*n/4)] #angles interval
    
    
    angles=np.linspace(0,2*np.pi,n)
    c.front_angles=angles[intv[0]:intv[1]] #on va pas regarder tout le cadran avant mais, a nouveau, un sous ensmeble
    c.front_dist.data=[] #remise a zero du batch
    #utiliser que cadran avant du lidar -> [pi/2, 3*pi/2] = [len(scan)//4, 3*len(scan)//4]

    #PAS BESOIN DE X ET Y -> COORDONNEES POLAIRES SONT PLUS ADAPTEES
    #GARDER QUE UN SOUS ENSMEBLE POUR ACCELERER LE PROCESSUS
    for i in range(n)[intv[0]:intv[1]]:
        if scan[i]!=float('inf'):
            c.front_dist.data.append(np.clip(scan[i],0,3)) #on clip pour s'orienter par rapport a l'env "local"
        
        else:
            #chercher la bonne valeur a prendre
            #on a inf pour certaines sections du scan -> on va interpoler lineairement (approx) pour completer ces points
            scan=interpolate(scan,i)
            
            c.front_dist.data.append(np.clip(scan[i],0,3))
    
    #side data pour rester au milieu
    c.side_dist.data=[]
    for i in [n//4,3*n//4]:
        if scan[i]!=float('inf'):
            c.side_dist.data.append(np.clip(scan[i],0,3)) 
        else:
            scan=interpolate(scan,i)
            c.side_dist.data.append(np.clip(scan[i],0,3))


    #print(len(c.front_dist.data))       
            


            
    

if __name__=='__main__':

    try:
        #init le noeud
        rospy.init_node("LidarData_node")

        #topic d'ecoute
        lidar_topic = rospy.get_param("~lidar_topic",default="/LidarScan")    #en faire un parametre de launchfile

        #classe de controle, stock data etc
        c=Control()

        #subscriber sur lidar topic qui range dans control la valeur du front data
        rospy.Subscriber(lidar_topic, Float32MultiArray, lidar_preprocess_callback,c)


        #publisher de front data
        front_topic=rospy.get_param("~front_topic",default="/front_data")    
        front_pub=rospy.Publisher(front_topic,Float32MultiArray,queue_size=10)
        
        #publisher side_data
        side_topic=rospy.get_param("~side_topic",default="/side_data") 
        side_pub=rospy.Publisher(side_topic,Float32MultiArray,queue_size=10)

        rate=rospy.Rate(2)
        #COMMENT BIEN SYCHRONISER LA PUBLICATION? -> LIMITE PAR /LidarScan -> pas besoin de rate.sleep?

        while not rospy.is_shutdown():

            
            front_pub.publish(c.front_dist)
            side_pub.publish(c.side_dist)
            
            rate.sleep()
    
    except rospy.ROSInterruptException:
        pass
