#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

class LaserScanMerger:
    def __init__(self):
        rospy.init_node('laser_scan_merger', anonymous=True)
        
        self.merged_scan = LaserScan()
        
        rospy.Subscriber('/rplidar/scan', LaserScan, self.callback_rplidar)
        rospy.Subscriber('/lanes/scan', LaserScan, self.callback_lanes)
        
        self.pub = rospy.Publisher('/zed/scan', LaserScan, queue_size=5)
        
        self.lane_ranges = None

    def callback_rplidar(self, data):
        self.merged_scan.header = data.header
        self.merged_scan.header.frame_id = "merged_laser"
        self.merged_scan.angle_min = data.angle_min
        self.merged_scan.angle_max = data.angle_max
        self.merged_scan.angle_increment = data.angle_increment
        self.merged_scan.time_increment = data.time_increment
        self.merged_scan.scan_time = data.scan_time
        self.merged_scan.range_min = data.range_min
        self.merged_scan.range_max = data.range_max*10
        self.merged_scan.ranges = list(data.ranges)
        self.merged_scan.intensities = data.intensities
        
        if self.lane_ranges is not None:
            for idx, range_value in enumerate(self.lane_ranges):
                lidx = int(5.4055555 * idx)
                #print(f"Lane index: {idx}, Lidar index: {lidx}, Range: {self.lane_ranges[idx]}, Increment: {data.angle_increment}")
                if self.lane_ranges[idx] != 1000 and self.lane_ranges[idx] < self.merged_scan.ranges[lidx]:
                    self.merged_scan.ranges[lidx] = self.lane_ranges[idx]
                    #print(f"Found override! {self.lane_ranges[idx]} < {self.merged_scan.ranges[lidx]}")
                    
#        for idx, range_value in enumerate(self.merged_scan.ranges):
#            self.merged_scan.ranges[idx] *= 2
        
        self.pub.publish(self.merged_scan)

    def callback_lanes(self, data):
        self.lane_ranges = data.ranges

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    #rospy.init_node()
    try:
        merger = LaserScanMerger()
        merger.run()
    except rospy.ROSInterruptException:
        pass
