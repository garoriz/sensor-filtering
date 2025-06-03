#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class ScanFilteredListener:
    def __init__(self):
        rospy.init_node('scan_filtered_listener', anonymous=True)
        rospy.Subscriber('/scan_filtered', LaserScan, self.callback)
        rospy.loginfo("Подписка на /scan_filtered активна")
        rospy.spin()

    def callback(self, msg):
        # Выводим основную информацию о скане
        valid_ranges = [r for r in msg.ranges if not rospy.is_shutdown() and not (r != r or r == float('inf') or r == float('-inf'))]
        if valid_ranges:
            min_dist = min(valid_ranges)
            avg_dist = sum(valid_ranges) / len(valid_ranges)
            rospy.loginfo("Получен /scan_filtered: min = %.2f м, avg = %.2f м (%d значений)", min_dist, avg_dist, len(valid_ranges))
        else:
            rospy.logwarn("Нет валидных значений в /scan_filtered")

if __name__ == '__main__':
    try:
        ScanFilteredListener()
    except rospy.ROSInterruptException:
        pass

