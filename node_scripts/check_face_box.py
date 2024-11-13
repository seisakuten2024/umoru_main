#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

class PointCloudChecker:
    
    def __init__(self):
        self.publisher = rospy.Publisher('/point_cloud_check',
                                         Bool, queue_size=10)
        rospy.Subscriber('/extract_indices/output', PointCloud2, self.callback)

    def callback(self, msg):
        rospy.loginfo("Number of points {}".format(len(msg.data)))
        is_large = len(msg.data) > 10000
        self.publisher.publish(Bool(data=is_large))

if __name__ == '__main__':
    rospy.init_node('point_cloud_checker')
    checker = PointCloudChecker()
    rospy.spin()
