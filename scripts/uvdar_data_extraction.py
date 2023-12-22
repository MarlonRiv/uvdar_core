#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import csv
import os
import cv2

# Initialize variables
bridge = CvBridge()
image_messages = []
points_messages = []
omta_messages = []

# Path to your ROS bag and output CSV file
bag_path = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/same_power_18m_topics.bag')
output_csv_path = os.path.expanduser('~/Desktop/MRS_Master_Project/Experiments_data/same_power_7m.csv')


# Directory to save images
image_output_dir = os.path.expanduser('~/Desktop/MRS_Master_Project/Experiments_data/images/same_power_18')
if not os.path.exists(image_output_dir):
    os.makedirs(image_output_dir)

# Read the bag file and store image and points messages in separate lists
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/uav7/uvdar_bluefox/left/image_raw', '/uav7/uvdar/blinkers_seen_back', '/uav7/uvdar/omta_all_seq_info_back']):
        if len(image_messages) > 1000 and len(omta_messages) > 1000:
            break
        if topic == '/uav7/uvdar_bluefox/left/image_raw':
            image_messages.append((t.to_nsec(), msg))
        elif topic == '/uav7/uvdar/blinkers_seen_back':
            points_messages.append((t.to_nsec(), msg))
        elif topic == '/uav7/uvdar/omta_all_seq_info_back':
            omta_messages.append((t.to_nsec(), msg))

# Sort messages by their timestamps
image_messages.sort(key=lambda tup: tup[0])
points_messages.sort(key=lambda tup: tup[0])
omta_messages.sort(key=lambda tup: tup[0])

print(f"Number of image messages: {len(image_messages)}")
print(f"Number of points messages: {len(points_messages)}")
print(f"Number of omta messages: {len(omta_messages)}")

# Process the messages to extract data
extracted_data = []


# Assuming that for each points message, the closest previous image message is the corresponding image
for pts_timestamp, points_msg in points_messages:
    corresponding_image_msg = None
    for img_timestamp, image_msg in image_messages:
        if img_timestamp <= pts_timestamp:
            corresponding_image_msg = image_msg
        else:
            break

    if corresponding_image_msg:
        try:
            cv_image = bridge.imgmsg_to_cv2(corresponding_image_msg, "mono8")
            # Save the image
            image_filename = f"{image_output_dir}/same_power_18{pts_timestamp}.png"
            cv2.imwrite(image_filename, cv_image)

            for point in points_msg.points:
                # Extract the pixel value at the point's x, y coordinates
                pixel_value = cv_image[int(point.y)][int(point.x)]
                # Store the data along with the timestamp
                extracted_data.append([pts_timestamp, point.x, point.y, point.value, pixel_value])
        except CvBridgeError as e:
            print(e)

print(f"Number of extracted data points: {len(extracted_data)}")
    
"""
# Write the extracted data to a CSV file
with open(output_csv_path, 'w', newline='') as csvfile:
    data_writer = csv.writer(csvfile, delimiter=',')
    # Write headers
    data_writer.writerow(['timestamp', 'x', 'y', 'value', 'pixel_value'])
    # Write data
    for data in extracted_data:
        data_writer.writerow(data)

print(f"Data extraction complete. CSV file created at {output_csv_path}")


"""