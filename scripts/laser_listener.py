#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class HybridFilteredLaserListener:
    def __init__(self):
        rospy.init_node('hybrid_filtered_laser_listener_node', anonymous=True)
        
        self.window_size = 5  # Размер окна для обоих фильтров (нечетное)
        if self.window_size % 2 == 0:
            self.window_size += 1
        
        self.buffer = []  # Буфер для хранения последних отфильтрованных сканов
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.loginfo("Узел запущен: подписка на /scan с фильтрацией, скользящим средним и медианным фильтром")

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
            # Сначала скользящее среднее
            avg_filtered = self.moving_average(self.buffer)
            # Затем медианный фильтр по результатам скользящего среднего
            temp_buffer = self.buffer[1:-1]  # берём 3 центральных среза для медианного фильтра
            temp_buffer = [avg_filtered] + temp_buffer
            # Обновляем буфер временно для медианы
            median_filtered = self.median_filter(temp_buffer)
        else:
            median_filtered = filtered_scan  # Пока мало данных

        valid_vals = [v for v in median_filtered if not math.isnan(v)]
        if valid_vals:
            rospy.loginfo("Гибридный фильтр: валидных значений %d из %d, мин=%.2f, макс=%.2f, среднее=%.2f",
                          len(valid_vals), len(median_filtered), min(valid_vals), max(valid_vals), sum(valid_vals)/len(valid_vals))
        else:
            rospy.logwarn("Гибридный фильтр: нет валидных данных!")

def main():
    node = HybridFilteredLaserListener()
    rospy.spin()

if __name__ == '__main__':
    main()

