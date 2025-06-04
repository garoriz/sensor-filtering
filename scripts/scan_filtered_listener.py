#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class WallDistanceEstimator:
    def __init__(self):
        rospy.init_node('wall_distance_estimator', anonymous=True)
        rospy.Subscriber('/scan_filtered', LaserScan, self.scan_callback)
        rospy.loginfo("Узел wall_distance_estimator подписан на /scan_filtered")
        rospy.spin()

    def scan_callback(self, scan: LaserScan):
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        num_ranges = len(scan.ranges)

        # Центральный индекс
        center_index = num_ranges // 2

        # Сектор ±2.5° = ±0.0436 рад
        half_angle_range = math.radians(2.5)
        sector_half_count = int(half_angle_range / angle_increment)

        # Границы сектора
        start_index = max(center_index - sector_half_count, 0)
        end_index = min(center_index + sector_half_count + 1, num_ranges)

        # Отбираем валидные значения
        sector_ranges = scan.ranges[start_index:end_index]
        valid_ranges = [r for r in sector_ranges if scan.range_min <= r <= scan.range_max and not math.isnan(r) and not math.isinf(r)]

        if valid_ranges:
            distance = float(np.mean(valid_ranges))  # можно заменить на median
            rospy.loginfo("Расстояние до стены в секторе ±2.5°: %.3f м (на %d лучах)", distance, len(valid_ranges))
        else:
            rospy.logwarn("Нет валидных данных в центральном секторе")

if __name__ == '__main__':
    try:
        WallDistanceEstimator()
    except rospy.ROSInterruptException:
        pass

