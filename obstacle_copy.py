# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32MultiArray


class ObstacleDetector:

    def __init__(self, topic):
        self.left = -1
        self.mid = -1
        self.right = -1
        self.filter_0 = []
        self.filter_1 = []
        self.filter_2 = []
        self.filter_3 = []
        self.filter_4 = []
        self.filter_5 = []
        self.filter_6 = []
        self.filter_7 = []
        self.weights = list(range(1, 6))
        rospy.Subscriber(topic, Int32MultiArray, self.read_distance)

    def read_distance(self, data):
        self.left = self.filter(self.filter_0, data.data[0], self.weights)
        self.mid = self.filter(self.filter_1, data.data[1], self.weights)
        self.right = self.filter(self.filter_2, data.data[2], self.weights)

    def for_parking(self, data):
        self.parking_0 = self.filter(self.filter_0, data.data[0], self.weights)
        self.parking_3 = self.filter(self.filter_3, data.data[3], self.weights)
        self.parking_4 = self.filter(self.filter_4, data.data[4], self.weights)
        self.parking_6 = self.filter(self.filter_6, data.data[6], self.weights)

    def get_distance(self):
        return self.left, self.mid, self.right

    def for_parking_distance(self):
        return self.parking_0, self.parking_3, self.parking_4, self.parking_6

    def filter(self, filterLst, data, weights):
        s = 0
        if data < 300:
            if len(filterLst) < 5:
                filterLst.append(data)
            else:
                filterLst = filterLst[1:] + [data]
            for i, x in enumerate(filterLst):
                s += x * weights[i]
                s = float(s) / sum(weights[:len(filterLst)])
        else:
            if len(filterLst) < 5:
                filterLst.append(300)
            else:
                filterLst = filterLst[1:] + [300]
            for i, x in enumerate(filterLst):
                s += x * weights[i]
                s = float(s) / sum(weights[:len(filterLst)])
        return s