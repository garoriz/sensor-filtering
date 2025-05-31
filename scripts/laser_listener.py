#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import copy

class HybridFilteredLaserListener:
    def __init__(self):
        rospy.init_node('hybrid_filtered_laser_listener_node', anonymous=True)
        
        self.window_size = 5  # Размер окна для фильтрации
        if self.window_size % 2 == 0:
            self.window_size += 1
        
        self.buffer = []  # Буфер последних сканов
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.scan_pub = rospy.Publisher("/scan_filtered", LaserScan, queue_size=10)

        rospy.loginfo("Узел запущен: подписка на /scan, публикация в /scan_filtered")

    def filter_scan(self, scan):
        filtered = []
        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r):
                filtered.append(float('nan'))
            elif r < scan.range_min or r > scan.range_max:
                filtered.append(float('nan'))
            else:
                filtered.append(r)
        return filtered

    def moving_average(self, data_buffers):
        length = len(data_buffers[0])
        averaged = []
        for i in range(length):
            vals = [buff[i] for buff in data_buffers if not math.isnan(buff[i])]
            if vals:
                averaged.append(sum(vals) / len(vals))
            else:
                averaged.append(float('nan'))
        return averaged

    def median_filter(self, data_buffers):
        length = len(data_buffers[0])
        medianed = []
        for i in range(length):
            vals = [buff[i] for buff in data_buffers if not math.isnan(buff[i])]
            if vals:
                medianed.append(float(np.median(vals)))
            else:
                medianed.append(float('nan'))
        return medianed

    def scan_callback(self, msg):
        filtered_scan = self.filter_scan(msg)
        self.buffer.append(filtered_scan)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)

        if len(self.buffer) == self.window_size:
            avg_filtered = self.moving_average(self.buffer)
            temp_buffer = [avg_filtered] + self.buffer[1:-1]
            median_filtered = self.median_filter(temp_buffer)
        else:
            median_filtered = filtered_scan

        # Публикация в /scan_filtered
        filtered_msg = copy.deepcopy(msg)
        filtered_msg.ranges = median_filtered
        filtered_msg.header.stamp = rospy.Time.now()
        self.scan_pub.publish(filtered_msg)

        valid_vals = [v for v in median_filtered if not math.isnan(v)]
        if valid_vals:
            rospy.loginfo("Публикация в /scan_filtered: %d валидных значений", len(valid_vals))
        else:
            rospy.logwarn("Публикация в /scan_filtered: нет валидных данных!")

def main():
    node = HybridFilteredLaserListener()
    rospy.spin()
    
# Частота /scan_filtered немного меньше, так как перед публикацией в /scan_filtered происходят множество вычислений

if __name__ == '__main__':
    main()

