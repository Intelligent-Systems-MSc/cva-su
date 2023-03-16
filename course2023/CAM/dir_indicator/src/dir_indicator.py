#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Ce noeud a pour but d'indiquer si le véhicule dans le bon ou dans le mauvais sens par rapport au circuit et egalement d'indiquer la couleur qu'il a en face de lui.
Ce noeud publie sur deux topics:
    ->/Direction :
        Les messages publiées sont de type String()  "right" ou "wrong"

    ->/Wallcolor :
        Les messages publiées sont de type String()  "red" ou "green"
"""

import rospy
import numpy as np

from std_msgs.msg import Int16MultiArray, String

class Dir_indicator : 

    def __init__(self, w, h) : 
        #subsciber
        sub_topic = rospy.get_param("image_datas", default="/ImageScan")
        self.sub=rospy.Subscriber(sub_topic, Int16MultiArray, self.callback)

        #publisher
        pubdir_topic = "/Direction"
        pubwcolor_topic = "/Wallcolor"
        self.pubdir=rospy.Publisher(pubdir_topic, String, queue_size=10)
        self.pubwcolor=rospy.Publisher(pubwcolor_topic, String, queue_size=10)
        self.dir=String()
        self.wcolor=String()

        self.w, self.h = w, h
        self.left = np.zeros((self.h))
        self.right = np.zeros((self.h))
        
        
    def callback(self, msg) : 
        #We get 2 vertical lines on the right and left of the image
        scan = msg.data		
        leftscan = np.array(scan).reshape((self.h, self.w, 4))[:,10]
        rightscan = np.array(scan).reshape((self.h, self.w, 4))[:,630]
        middlescan = np.array(scan).reshape((self.h, self.w, 4))[:,320]


        #We make the ratio between the values of red and the other colors
        rgl = np.divide(leftscan[:,2],leftscan[:,1])
        rbl = np.divide(leftscan[:,2],leftscan[:,0])
        rgr = np.divide(rightscan[:,2],rightscan[:,1])
        rbr = np.divide(rightscan[:,2],rightscan[:,0])
        rgm = np.divide(middlescan[:,2],middlescan[:,1])
        rbm = np.divide(middlescan[:,2],middlescan[:,0])

        #On compte le nombre de pixel qui ont une majorité de rouge par rapport aux autres couleurs
        count_red_left=0
        for i in range(len(rgl)):
            if (rgl[i] >1.2) and (rbl[i] > 1.2):
                count_red_left+=1
        
        count_red_right=0
        for i in range(len(rgr)):
            if (rgr[i] >1.2) and (rbr[i] > 1.2):
                count_red_right+=1

        count_red_middle=0
        for i in range(len(rgm)):
            if (rgm[i] >1.2) and (rbm[i] > 1.2):
                count_red_middle+=1


        #We count the number of times the ratio is greater than 1.5
        # countrgl=np.count_nonzero(rgl>1.5)
        # countrbl=np.count_nonzero(rbl>1.5)
        # countrgr=np.count_nonzero(rgr>1.5)
        # countrbr=np.count_nonzero(rbr>1.5)

        #We make the ratio between the values of green and the other colors
        grl = np.divide(leftscan[:,1],leftscan[:,2])
        gbl = np.divide(leftscan[:,1],leftscan[:,0])
        grr = np.divide(rightscan[:,1],rightscan[:,2])
        gbr = np.divide(rightscan[:,1],rightscan[:,0])
        grm = np.divide(middlescan[:,1],middlescan[:,2])
        gbm = np.divide(middlescan[:,1],middlescan[:,0])

        #On compte le nombre de pixel qui ont une majorité de vert par rapport aux autres couleurs
        count_green_left=0
        for i in range(len(grl)):
            if (grl[i] >1.2) and (gbl[i] > 1.2):
                count_green_left+=1
        
        count_green_right=0
        for i in range(len(grr)):
            if (grr[i] >1.2) and (gbr[i] > 1.2):
                count_green_right+=1

        count_green_middle=0
        for i in range(len(grm)):
            if (grm[i] >1.2) and (gbm[i] > 1.2):
                count_green_middle+=1

        #We count the number of times the ratio is greater than 1.5
        # countgrl=np.count_nonzero(grl>1.5)
        # countgbl=np.count_nonzero(gbl>1.5)
        # countgrr=np.count_nonzero(grr>1.5)
        # countgbr=np.count_nonzero(gbr>1.5)
        
        
        # count_red_left=min(countrgl,countrbl)
        # count_red_right=min(countrgr,countrbr)
        # count_green_left=min(countgrl,countgbl)
        # count_green_right=min(countgrr,countgbr)

        #On détermine la direction prise par le véhicule
        if (count_green_left > 30) and (count_red_right > 30):
            self.dir.data="right"
            
        elif (count_green_right > 30) and (count_red_left >30):
            self.dir.data="wrong"

        #On indique la couleur en face du véhicule
        if (count_green_middle > count_red_middle) :
             self.wcolor.data="green"
        
        elif (count_red_middle > count_green_middle):
             self.wcolor.data="red"

        #rospy.loginfo(f"red_l={count_red_left} red_r{count_red_right} green_l{count_green_left} green_r{count_green_right}")
        #rospy.loginfo(leftscan)
        #rospy.loginfo(f"grl={grl} gbl={gbl}")

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
