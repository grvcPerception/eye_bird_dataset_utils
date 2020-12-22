#!/usr/bin/env python3

###########################################################################
#####                       eye_bird_flip_images                      #####
#####                            Raul Tapia                           #####
#####           GRVC Robotics Lab at University of Seville            #####
###########################################################################

# @file    eye_bird_flip_images.py
# @author  Raul Tapia (raultapia _at_ us.es | github.com/raultapia)
# @brief   Flip raw images and events

import os
import rosbag
import cv2
from cv_bridge import CvBridge

filenameList = [
    'Soccer_Base_1',
    'Soccer_Base_2',
    'Soccer_Base_3',
    'Soccer_ArUco_1',
    'Soccer_ArUco_2',
    'Soccer_People_1',
    'Soccer_People_2',
    'Soccer_Calibration',
    'Hills_Base_1',
    'Hills_Base_2',
    'Hills_Base_3',
    'Hills_ArUco_1',
    'Hills_ArUco_2',
    'Hills_Calibration',
    'Testbed_Base_1',
    'Testbed_Base_2',
    'Testbed_Base_3',
    'Testbed_ArUco_1',
    'Testbed_ArUco_2',
    'Testbed_ArUco_3',
    'Testbed_ArUco_4',
    'Testbed_ArUco_5',
    'Testbed_ArUco_6',
    'Testbed_Calibration'
]

for filename in filenameList:
    print(' Flipping ' + filename + '.bag ...')

    bag = rosbag.Bag(filename + '.bag')
    newBag = rosbag.Bag(filename + '_flipped.bag', 'w')
    bridge = CvBridge()

    for topic, msg, timestamp in bag.read_messages():
        if topic == "/dvs/image_raw":
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            img = cv2.flip(img, -1)
            newMsg = bridge.cv2_to_imgmsg(img, encoding="mono8")
            newMsg.header = msg.header
            msg = newMsg

        if topic == "/dvs/events":
            for i in range(len(msg.events)):
                msg.events[i].x = msg.width - 1 - msg.events[i].x
                msg.events[i].y = msg.height - 1 - msg.events[i].y

        newBag.write(topic, msg, timestamp)

    bag.close()
    newBag.close()
    os.system('rosbag reindex ' + filename + '_flipped.bag')
    os.system('rm ' + filename + '_flipped.orig.bag')
    print(' File ' + filename + '_flipped.bag saved\n')
