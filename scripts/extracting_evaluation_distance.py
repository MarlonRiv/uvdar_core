#!/usr/bin/env python
import rosbag
import csv
import os


#bag_path = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/test_standard_topics_3.bag')

# List of ROS bag files
bag_files = [
    os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/multi/otsu_multiple_topics.bag'),

    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_8m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_11m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_14m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_17m.bag'),
    #os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/static_standard_topics_20m.bag'),
]

# Output CSV file
#output_csv = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/standard_test_3.csv')

# Output CSV file
output_csv = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/raw_csv/otsu_multiple_topics.csv')

# Define the topics to extract
topics = ['/uav1/uvdar/blinkers_seen_left','/uav1/uvdar/blinkers_seen_right','/uav1/control_manager/control_reference']


combined_data = []

# Process each bag file
for bag_path in bag_files:
    with rosbag.Bag(bag_path, 'r') as bag:
        data = []  # Temporary list to store data from the current bag
        for topic, msg, t in bag.read_messages(topics=topics):
            entry = {'timestamp': t.to_sec(), 'distance_x': None, 'value': None}
            if topic in ['/uav1/uvdar/blinkers_seen_left', '/uav1/uvdar/blinkers_seen_right']:
                #entry['value'] = msg.points[0].value if msg.points else None
                #Take all the values
                entry['value'] = [point.value for point in msg.points]
            elif topic == '/uav1/control_manager/control_reference':
                # Extract the x position as distance
                entry['distance_x'] = msg.pose.pose.position.x
            data.append(entry)
        combined_data.extend(data)  # Add the data from the current bag to the combined list

# Sort combined data by timestamp to align the entries
combined_data.sort(key=lambda x: x['timestamp'])

# Write the combined data to a CSV file
with open(output_csv, 'w', newline='') as csvfile:
    fieldnames = ['timestamp', 'distance_x', 'value']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    for entry in combined_data:
        writer.writerow(entry)

print(f"Data extracted and combined from multiple bag files and saved to {output_csv}")