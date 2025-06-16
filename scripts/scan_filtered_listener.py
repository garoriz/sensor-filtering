#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
import math
import numpy as np

APPROACHING = "APPROACHING"
RECEDING = "RECEDING"
STABLE = "STABLE"

class WallMotionDetector:
    def __init__(self):
        rospy.init_node('wall_motion_detector')

	# Публикация результатов в топики
        self.distance_pub = rospy.Publisher('/wall_distance', Float32, queue_size=10)
        self.motion_pub = rospy.Publisher('/wall_motion', String, queue_size=10)

        # Подписка на топик /scan_filtered
        rospy.Subscriber('/scan_filtered', LaserScan, self.scan_callback)

        self.last_distance = None
        self.state = STABLE
        self.delta_threshold = 0.03

        rospy.spin()

    # Вычисление расстояния до стены
    def scan_callback(self, scan: LaserScan):
        angle_increment = scan.angle_increment
        ranges = scan.ranges
        num_ranges = len(ranges)

        center_index = num_ranges // 2
        half_angle_range = math.radians(2.5)
        sector_half_count = int(half_angle_range / angle_increment)

        start_index = max(center_index - sector_half_count, 0)
        end_index = min(center_index + sector_half_count + 1, num_ranges)

        sector = ranges[start_index:end_index]
        valid = [r for r in sector if scan.range_min <= r <= scan.range_max and not math.isnan(r) and not math.isinf(r)]

        if not valid:
            return

        current_distance = float(np.mean(valid))

        self.distance_pub.publish(Float32(data=current_distance))

	# Вычисление направления движения
        if self.last_distance is not None:
            delta = current_distance - self.last_distance

            if abs(delta) < self.delta_threshold:
                self.state = STABLE
            elif delta < 0:
                self.state = APPROACHING
            else:
                self.state = RECEDING

            self.motion_pub.publish(String(data=self.state))

        self.last_distance = current_distance

if __name__ == '__main__':
    try:
        WallMotionDetector()
    except rospy.ROSInterruptException:
        pass

