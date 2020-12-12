#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from numpy import int32
from std_msgs.msg import Int32

frequency = 10  # Hz

cap = cv2.VideoCapture(0)
whT = 320
confThreshold = 0.5
nmsThreshold = 0.01

classesFile = '/home/naudotvardis/Workspaces/ROS_ws/src/dviratis_robotas/detection_files/coco.names'
classNames = []

with open(classesFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

modelConfiguration = '/home/naudotvardis/Workspaces/ROS_ws/src/dviratis_robotas/detection_files/yolov3-tiny.cfg'
modelWeights = '/home/naudotvardis/Workspaces/ROS_ws/src/dviratis_robotas/detection_files/yolov3-tiny.weights'

net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

result_publisher = rospy.Publisher('DetectionResult_msg', Int32, queue_size=1)
result_message = Int32()


def findObjects(outputs, img):
    hT, wT, cT = img.shape
    bbox = []
    classIds = []
    confs = []

    for output in outputs:
        for det in output:
            scores = det[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold:
                w, h = int(det[2] * wT), int(det[3] * hT)
                x, y = int(det[0] * wT - w/2), int(det[1]*hT-h/2)
                bbox.append([x, y, w, h])
                classIds.append(classId)
                confs.append(float(confidence))

    rospy.loginfo("Found {}".format(len(bbox)))

    # Publish the number of found objects
    result_message.data = int32(len(bbox))
    result_publisher.publish(result_message)

    indices = cv2.dnn.NMSBoxes(bbox, confs, confThreshold, nmsThreshold)
    for i in indices:
        i = i[0]
        box = bbox[i]
        x, y, w, h = box[0], box[1], box[2], box[3]
        cv2.rectangle(img, (x, y), (x+w, y+h), (139, 168, 50), 2)
        cv2.putText(img, f'{classNames[classIds[i]].upper()} {int(confs[i]*100)}%',
                    (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (139, 168, 50), 2)


def talker():
    rospy.init_node('ObjectRecognition_node', anonymous=True)

    rate = rospy.Rate(frequency)  # x hz

    while not rospy.is_shutdown():
        success, img = cap.read()

        blob = cv2.dnn.blobFromImage(img, 1/255, (whT, whT), [0, 0, 0], 1, crop=False)
        net.setInput(blob)

        layerNames = net.getLayerNames()

        outputNames = [layerNames[i[0]-1] for i in net.getUnconnectedOutLayers()]

        outputs = net.forward(outputNames)

        findObjects(outputs, img)

        # cv2.imshow('Image', img)
        cv2.waitKey(1)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
