#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
from collections import deque

class FilteredLaserListener:
    def __init__(self):
        rospy.init_node('filtered_laser_listener_node', anonymous=True)
        self.window_size = 5  # Размер окна для скользящего среднего
        
        self.buffer = []  # Буфер для хранения последних N сканов
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.loginfo("Узел запущен: подписка на /scan с фильтрацией и сглаживанием")

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
        # data_buffers: список последних scan.ranges (списков)
        length = len(data_buffers[0])
        averaged = []
        for i in range(length):
            vals = []
            for buff in data_buffers:
                val = buff[i]
                if not math.isnan(val):
                    vals.append(val)
            if vals:
                averaged.append(sum(vals) / len(vals))
            else:
                averaged.append(float('nan'))
        return averaged

    def scan_callback(self, msg):
        filtered_scan = self.filter_scan(msg)

        # Добавляем новый отфильтрованный скан в буфер
        self.buffer.append(filtered_scan)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)

        # Применяем скользящее среднее по буферу
        if len(self.buffer) == self.window_size:
            smoothed = self.moving_average(self.buffer)
        else:
            smoothed = filtered_scan  # Пока мало данных — используем фильтр без сглаживания

        # Вывод статистики
        valid_vals = [v for v in smoothed if not math.isnan(v)]
        if valid_vals:
            rospy.loginfo("Сглаженный скан: валидных значений %d из %d, мин=%.2f, макс=%.2f, среднее=%.2f",
                          len(valid_vals), len(smoothed), min(valid_vals), max(valid_vals), sum(valid_vals)/len(valid_vals))
        else:
            rospy.logwarn("Сглаженный скан: нет валидных данных!")

def main():
    listener = FilteredLaserListener()
    rospy.spin()

if __name__ == '__main__':
    main()

