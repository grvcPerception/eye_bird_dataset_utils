#!/usr/bin/env python3

# @file    eye_bird_add_annotations.py
# @author  Juan Pablo Rodriguez Gomez (jrodriguezg _at_ us.es | github.com/jprodriguezg)
# @brief   Add bounding box annotations to input rosbag and flip frames

import os
import rosbag
import csv
import argparse
import cv2
from cv_bridge import CvBridge
import numpy as np
from eye_bird_dataset_msgs.msg import AnnotationArray
from eye_bird_dataset_msgs.msg import Annotation

def getInputFilesDir():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag_name", required=True, help='Bag file name without .bag .')
    parser.add_argument("--annotation_name", required=True, help='Annotation file name without .txt .')
    args = parser.parse_args()
    return args

def fillAnnotationMsg(label,xmin,ymin,xmax,ymax):
    msg = Annotation()
    msg.detection = label
    msg.xmax = xmax
    msg.xmin = xmin
    msg.ymax = ymax
    msg.ymin = ymin

    return msg

def main():
    inputRef = getInputFilesDir()
    fileBagName = inputRef.bag_name
    fileNameAnnotation = inputRef.annotation_name

    data = []
    with open(fileNameAnnotation + '.txt', mode='r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
                data.append(row)

    bag = rosbag.Bag(fileBagName + '.bag')
    newBag = rosbag.Bag(fileBagName + '_annotations.bag', 'w')
    bridge = CvBridge()

    lastIndex = 1
    for topic, msg, timestamp in bag.read_messages():
        if topic == "/dvs/image_raw":
            msgAnnoationArr = AnnotationArray()
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            img = cv2.flip(img, -1)
            img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
            validData  = True


            while (validData and lastIndex < len(data)):
                if (str(msg.header.stamp) == data[lastIndex][0]):
                    if(data[lastIndex][1]!='-1' and data[lastIndex][2]!= '-1' and data[lastIndex][3]!='-1' and data[lastIndex][4]!='-1'):
                        xmin = int(data[lastIndex][1]); ymin = int(data[lastIndex][2]); xmax = xmin+int(data[lastIndex][3]); ymax = ymin+int(data[lastIndex][4]);
                        img = cv2.rectangle(np.array(img),(xmin,ymin),(xmax,ymax),(0,255,0),1)
                        msgAnnoationArr.annotation_array.append(fillAnnotationMsg(1,xmin,ymin,xmax,ymax))
                    lastIndex += 1
                else:
                    validData = False;

            newMsg = bridge.cv2_to_imgmsg(img, encoding="rgb8")
            newMsg.header = msg.header
            newBag.write('/eyeBirdDataset/annotation/image_raw', newMsg, timestamp)

            msgAnnoationArr.header = msg.header
            newBag.write('/eyeBirdDataset/annotation/data', msgAnnoationArr, timestamp)

    bag.close()
    newBag.close()
    os.system('rosbag reindex ' + fileBagName + '_annotations.bag')
    os.system('rm ' + fileBagName + '_annotations.orig.bag')
    print(' File ' + fileBagName + '_annotations.bag saved\n')

if __name__ == "__main__":
    main()
