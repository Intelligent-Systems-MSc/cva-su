#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import message_filters

from std_msgs.msg import Float32MultiArray, Float32

step_size=rospy.get_param("step_size",default=10)#step interval for front_data 

class Control:
    def __init__(self):
        self.angle_command=Float32()
        self.speed_command=Float32()
        self.dir=Float32()
        self.center=Float32()
        self.front_dist=Float32()
        self.offset=0.  #en faire parametre

def data_process_callback(msg_f,msg_s,c):
    #PARCOURIR LE TABLEAU DANS msg.data ET DETERMINER LA DIRECTION INSTANTANNEE A PRENDRE 
    #ET LA VITESSE QU'ON PEUT AVOIR

    #idee : parcourir le tableau et determiner dans quelle direction les distances sont les  plus
    #importantes-> moyenne ponderee des angles p.ex
    #-> parcourt le tableau des angles et on le multiplie par la distance, on fait la moyenne puis sur le nombre total
    #de distances et ca devrait nous donner la direction a prendre
    #
    front_data=msg_f.data #tableau contient dist autour du robot entre [a0,a1]
    
    a0,a1=rospy.get_param("angle0",default=120),rospy.get_param("angle1",default=240)
    angles=np.linspace(np.deg2rad(a0), np.deg2rad(a1), len(front_data))
    
    avg,steps=0,len(front_data)//step_size
    sum=0
    for i in range(steps):
        
        avg+=angles[step_size*i]*front_data[step_size*i]
        sum+=front_data[step_size*i]
    
    avg/=sum

    #si objet au milieu <-> front_data[diag_gauche]>front_data[n//2]<front_data[diag_droite]
    #probleme est qu'en faisant moyenne la direction a prendre est quand meme le milieu du a la symetrie
    #devier-> ajouter offfset a avg qui fera qu'en faisant moy on ira vers gauche ou droite
    #METTRE EN PLACE UN FLAG RECUPERE DANS nv_control QUI AJUSTE VITESSE ET COMMANDE DE BRAQUAGE
    
    n=len(front_data)
    front_dist=front_data[n//2]
    for i in range(1,step_size//2):
        front_dist+=front_data[n//2+i]
        front_dist+=front_data[n//2-i]
    
    front_dist/=step_size
    
    c.front_dist.data=front_dist #influence la vitesse

    

    #equivalent to orientation error
    direction=avg-np.pi #>0 : droite, <0 : gauche => donne la direction a prendre

    c.dir=direction-c.offset #-offset car defini t.q. offset>0 => gauche


    side_data=msg_s.data
    #equivalent to centering error
    c.center=side_data[0]-side_data[1] +c.offset #left-right + offset

    

if __name__=='__main__':
    try:
        
        #init node
        rospy.init_node("lidar_data_process_node")

        #init control
        c=Control()


        #subscribe to front_data and side_data
        front_data_topic=rospy.get_param("~front_data_topic", default="/front_data")
        side_data_topic=rospy.get_param("~side_data_topic", default="/side_data")

        front_sub=message_filters.Subscriber(front_data_topic,Float32MultiArray, queue_size=1)
        side_sub=message_filters.Subscriber(side_data_topic,Float32MultiArray,queue_size=1)

        ts = message_filters.ApproximateTimeSynchronizer([front_sub, side_sub], queue_size=1, slop=0.1, allow_headerless=True)
        ts.registerCallback(data_process_callback,c)
  


        #publish commands - test
        rate=rospy.Rate(10)


        #publish direction
        dir_pub=rospy.Publisher("/lidar_dir",Float32,queue_size=1)

        #publish center
        center_pub=rospy.Publisher("/lidar_center",Float32,queue_size=1)

        #publidhs front dist
        front_dist_pub=rospy.Publisher("/front_dist",Float32,queue_size=1)

        while not rospy.is_shutdown():

            dir_pub.publish(c.dir)
            center_pub.publish(c.center)
            front_dist_pub.publish(c.front_dist)

            rate.sleep()


    except rospy.ROSInterruptException:
        pass