#!/usr/bin/env python
import rosbag
import csv
import os
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Extract data from ROS bags and save to CSV.')
parser.add_argument('--name_experiment', type=str, required=True, help='Name of the experiment')
parser.add_argument('--rx_bag', type=str, required=True, help='Path to RX bag file')
parser.add_argument('--tx1_bag', type=str, required=True, help='Path to TX1 bag file')
parser.add_argument('--tx2_bag', type=str, required=True, help='Path to TX2 bag file')
parser.add_argument('--output_csv', type=str, required=True, help='Path to output CSV file')

args = parser.parse_args()

name_experiment = args.name_experiment
output_csv = args.output_csv

bag_files = [
    args.rx_bag,
    args.tx1_bag,
    args.tx2_bag,
]

topics = [
    '/uav36/uvdar/blinkers_seen_left',
    '/uav36/estimation_manager/odom_main',
    '/uav39/estimation_manager/odom_main',
    '/uav40/estimation_manager/odom_main'
]

combined_data = []

for bag_path in bag_files:
    with rosbag.Bag(bag_path, 'r') as bag:
        data = []
        for topic, msg, t in bag.read_messages(topics=topics):
            entry = {
                'timestamp': t.to_sec(),
                'uav36_distance_x': None,
                'uav36_distance_y': None,
                'uav36_distance_z': None,
                'uav39_distance_x': None,
                'uav39_distance_y': None,
                'uav39_distance_z': None,
                'uav40_distance_x': None,
                'uav40_distance_y': None,
                'uav40_distance_z': None,
                'value': None,
                'point_x': None,
                'point_y': None
            }
            if topic == '/uav36/uvdar/blinkers_seen_left':
                entry['value'] = [point.value for point in msg.points]
                entry['point_x'] = [point.x for point in msg.points]
                entry['point_y'] = [point.y for point in msg.points]
            elif topic == '/uav36/estimation_manager/odom_main':
                entry['uav36_distance_x'] = msg.pose.pose.position.x
                entry['uav36_distance_y'] = msg.pose.pose.position.y
                entry['uav36_distance_z'] = msg.pose.pose.position.z
            elif topic == '/uav39/estimation_manager/odom_main':
                entry['uav39_distance_x'] = msg.pose.pose.position.x
                entry['uav39_distance_y'] = msg.pose.pose.position.y
                entry['uav39_distance_z'] = msg.pose.pose.position.z
            elif topic == '/uav40/estimation_manager/odom_main':
                entry['uav40_distance_x'] = msg.pose.pose.position.x
                entry['uav40_distance_y'] = msg.pose.pose.position.y
                entry['uav40_distance_z'] = msg.pose.pose.position.z

            data.append(entry)
        combined_data.extend(data)

# Sort combined data by timestamp to align the entries
combined_data.sort(key=lambda x: x['timestamp'])

# Write to CSV
with open(output_csv, 'w', newline='') as csvfile:
    fieldnames = [
        'timestamp',
        'uav36_distance_x', 'uav36_distance_y', 'uav36_distance_z',
        'uav39_distance_x', 'uav39_distance_y', 'uav39_distance_z',
        'uav40_distance_x', 'uav40_distance_y', 'uav40_distance_z',
        'value', 'point_x', 'point_y'
    ]
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    for entry in combined_data:
        writer.writerow(entry)

print(f"Data extracted and combined from multiple bag files and saved to {output_csv}")
