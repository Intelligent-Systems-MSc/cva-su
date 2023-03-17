#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import numpy as np

from std_msgs.msg import Int16MultiArray, String

def bgr2hsv (colonne):
    """fonction qui convertit les valeurs d'une colonne de pixel bgr en leur valeur hsv"""
    colonne=colonne/255
    hsv=np.zeros(colonne.shape)
    for i,p in enumerate(colonne):
        b,g,r=p[0],p[1],p[2]
        v=np.max(p)
        if v != 0:
            s=(v-np.min(p))/v
        else:
            s=0
        if b==g and g==r:
            h=0
        elif v==r:
            h=60*(g-b)/(v-np.min(p))
        elif v==g:
            h=120+60*(b-r)/(v-np.min(p))
        elif v==b:
            h=240+60*(r-g)/(v-np.min(p))
        if h < 0:
            h+=360
        v=255*v
        s=255*s
        hsv[i]=[h,s,v]
        
    return hsv


class Dir_indicator : 

    """Ce noeud a pour but d'indiquer si le véhicule est dans le bon ou dans le mauvais sens par rapport au circuit et également d'indiquer la couleur qu'il a en face de lui.
Ce noeud publie sur deux topics:
    ->/Direction :
        Les messages publiés sont de type String()  "right" ou "wrong"

    ->/Wallcolor :
        Les messages publiés sont de type String()  "red" ou "green"
        Ce topic peut permettre de savoir dans quelle direction il faut tourner si on se retrouve face à un mur
    """

    def __init__(self, w, h) : 
        #subscriber
        sub_topic = rospy.get_param("image_datas", default="/ImageScan")
        self.sub=rospy.Subscriber(sub_topic, Int16MultiArray, self.callback)

        #publisher
        pubdir_topic = "/Direction"
        pubwcolor_topic = "/Wallcolor"
        self.pubdir=rospy.Publisher(pubdir_topic, String, queue_size=10)
        self.pubwcolor=rospy.Publisher(pubwcolor_topic, String, queue_size=10)
        self.dir=String()
        self.wcolor=String()

        #valeurs pour redimensionner l'image récupéré
        self.w, self.h = w, h
        self.left = np.zeros((self.h))
        self.right = np.zeros((self.h))
        
        
    def callback(self, msg) : 
        #On récupère deux lignes verticales à gauche et à droite
        scan = msg.data		
        leftscan = np.array(scan).reshape((self.h, self.w, 4))[:,10,0:3]
        rightscan = np.array(scan).reshape((self.h, self.w, 4))[:,630,0:3]
        middlescan = np.array(scan).reshape((self.h, self.w, 4))[:,320,0:3]

        #On convertie leurs valeur bgr en valeur hsv
        lefthsv = bgr2hsv(leftscan)
        righthsv = bgr2hsv(rightscan)
        middlehsv = bgr2hsv(middlescan)


        #On compte le nombre de pixel rouge selon leur valeur hsv
        count_red_left=0
        for p in lefthsv:
            if p[0]<14 or p[0]>280:
                if p[1]>90 and p[2]>50:
                    count_red_left+=1
        
        count_red_right=0
        for p in righthsv:
            if p[0]<14 or p[0]>330:
                if p[1]>90 and p[2]>50:
                    count_red_right+=1

        count_red_middle=0
        for p in middlehsv:
            if p[0]<14 or p[0]>330:
                if p[1]>90 and p[2]>50:
                    count_red_middle+=1

        
        
        #On compte le nombre de pixel vert à l'aide de leur valeur hsv
        count_green_left=0
        for p in lefthsv:
            if p[0]>90 and p[0]<160:
                if p[1]>50 and p[2]>50:
                    count_green_left+=1
        
        count_green_right=0
        for p in righthsv:
            if p[0]>90 and p[0]<160:
                if p[1]>50 and p[2]>50:
                    count_green_right+=1

        count_green_middle=0
        for p in middlehsv:
            if p[0]>90 and p[0]<160:
                if p[1]>50 and p[2]>50:
                    count_green_middle+=1


        #On détermine la direction prise par le véhicule

        if ((count_green_right < count_green_left) or (count_red_left < count_red_right)) :
            #Cette condition est pour éviter que la décision soit prise en fonction de seulement quelques pixels
            if count_green_left > 20 or count_red_right > 20:
                self.dir.data="wrong"
            
        elif (count_green_right > count_green_left) or (count_red_left > count_red_right):
            #Cette condition est pour éviter que la décision soit prise en fonction de seulement quelques pixels
            if count_green_right >20 or count_red_left > 20:
                self.dir.data="right"

        #On indique la couleur en face du véhicule

        if (count_green_middle > count_red_middle) and count_green_middle > 20:
             self.wcolor.data="green"
        
        elif (count_red_middle > count_green_middle) and count_red_middle > 20:
             self.wcolor.data="red"

        rospy.loginfo(f"red_l={count_red_left} red_r{count_red_right} green_l{count_green_left} green_r{count_green_right}")
        self.pubdir.publish(self.dir)
        self.pubwcolor.publish(self.wcolor)




if __name__ == '__main__':
    w, h = 640, 480
    try:
        rospy.init_node('dir_indicator', anonymous = True)
        d = Dir_indicator(w, h) 
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
