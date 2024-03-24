#!/usr/bin/env python
import rosbag
import csv
from geometry_msgs.msg import Point
import os


bag_path = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/test_standard_topics.bag')

# Output CSV file
output_csv = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/standard_test.csv')

# Define the topics to extract
topics = ['/uav1/uvdar/blinkers_seen_left','/uav1/uvdar/blinkers_seen_right','/uav1/control_manager/control_reference']

# Initialize an empty list to store data
data = []

# Open the ROS bag file
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == '/uav1/uvdar/blinkers_seen_left':
            # Assuming you're interested in the first point's value if multiple points exist
            value = msg.points[0].value if msg.points else None
            timestamp = t.to_sec()
            data.append({'timestamp': timestamp, 'value': value, 'distance_x': None})
        elif topic == '/uav1/uvdar/blinkers_seen_right':
            # Assuming you're interested in the first point's value if multiple points exist
            value = msg.points[0].value if msg.points else None
            timestamp = t.to_sec()
            data.append({'timestamp': timestamp, 'value': value, 'distance_x': None})
        elif topic == '/uav1/control_manager/control_reference':
            # Extract the x position as distance
            distance_x = msg.pose.pose.position.x
            timestamp = t.to_sec()
            data.append({'timestamp': timestamp, 'distance_x': distance_x})

# Sort data by timestamp to align the entries (if needed)
data.sort(key=lambda x: x['timestamp'])

# Now, write the extracted data to a CSV file
with open(output_csv, 'w', newline='') as csvfile:
    fieldnames = ['timestamp', 'distance_x', 'value']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    for entry in data:
        writer.writerow(entry)

print(f"Data extracted and saved to {output_csv}")
