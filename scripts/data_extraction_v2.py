#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import csv
import os
import cv2

bridge = CvBridge()
sequence_counter = 0  # Counter for sequence index



bag_path = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/diff_power_5m_topics.bag')
output_csv_path = os.path.expanduser('~/Desktop/MRS_Master_Project/Experiments_data/exposure_5000.csv')

# Directory to save images
image_output_dir = os.path.expanduser('~/Desktop/MRS_Master_Project/Experiments_data/images/diff_power_5')


# Storage for image and omta messages
image_messages = []
omta_messages = []

# Read the bag file and store image and omta messages in separate lists
with rosbag.Bag(bag_path, 'r') as bag:
    last_timestamp = None
    
    for topic, msg, t in bag.read_messages(topics=['/uav7/uvdar_bluefox/left/image_raw', '/uav7/uvdar/omta_all_seq_info_back']):
        #Take only the first 5000 messages
        if len(image_messages) > 2000 and len(omta_messages) > 2000:
            break
        if topic == '/uav7/uvdar_bluefox/left/image_raw':
            image_messages.append((t.to_nsec(), msg))
        elif topic == '/uav7/uvdar/omta_all_seq_info_back':
            omta_messages.append((t.to_nsec(), msg))

# Sort messages by their timestamps
image_messages.sort(key=lambda tup: tup[0])
omta_messages.sort(key=lambda tup: tup[0])


print(f"Number of image messages: {len(image_messages)}")
print(f"Number of omta messages: {len(omta_messages)}")

extracted_data = []

# Assuming that for each omta message, the closest previous image message is the corresponding image

#Process only 12000 messages
#omta_messages = omta_messages[:12000]


def annotate_and_save_image(cv_image, omta_msg, image_filename):
    # Create a copy of the image to draw on
    annotated_image = cv_image.copy()
    # Convert to BGR for color drawing
    if len(annotated_image.shape) == 2:  # if grayscale, convert to BGR
        annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_GRAY2BGR)

    # Iterate over each sequence variable to draw the markers
    for seq_var in omta_msg:
        signal_id = seq_var.signal_id
        if seq_var.signal_id >0:
            for seq_point in seq_var.sequence:
                x, y = int(seq_point.point.x), int(seq_point.point.y)
                # Check if the pixel coordinates are within the image bounds
                if 0 <= x < annotated_image.shape[1] and 0 <= y < annotated_image.shape[0]:
                    # Draw a small red circle around the point
                    #cv2.circle(annotated_image, (x, y), 5, (0, 0, 255), 1)
                    # If the LED is on, color the pixel red
                    if seq_point.point.value:
                        annotated_image[y, x] = (0, 0, 255)

    # Save the annotated image
    cv2.imwrite(image_filename, annotated_image)
    
for omta_timestamp, omta_msg in omta_messages:
    corresponding_image_msg = None
    for img_timestamp, image_msg in image_messages:
        if img_timestamp <= omta_timestamp:
            corresponding_image_msg = image_msg
        else:
            break

    if corresponding_image_msg:
            try:
                cv_image = bridge.imgmsg_to_cv2(corresponding_image_msg, "mono8")

                # Save the image
                image_filename = f"{image_output_dir}/diff_power_5{omta_timestamp}.png"
                annotate_and_save_image(cv_image, omta_msg.sequences, image_filename)
                #cv2.imwrite(image_filename, cv_image)
                # Extract the pixel value for each sequence
                for seq_vars in omta_msg.sequences:
                    signal_id = seq_vars.signal_id
                    sequence_counter += 1
                    # Extract the pixel values for each point in the sequence
                    for seq_point in seq_vars.sequence:
                        point = seq_point.point
                        pixel_value = cv_image[int(point.y)][int(point.x)] if (0 <= int(point.y) < cv_image.shape[0] and 0 <= int(point.x) < cv_image.shape[1]) else None
                        # Append all the information to extracted_data
                        extracted_data.append([
                            seq_point.insert_time.secs + seq_point.insert_time.nsecs * 1e-9,  # Convert to seconds
                            sequence_counter,  # Sequence index
                            signal_id,
                            point.x,
                            point.y,
                            point.value,
                            pixel_value
                        ])
            except CvBridgeError as e:
                print(e)


# Release the video writer

"""
# Write the extracted data to a CSV file
with open(output_csv_path, 'w', newline='') as csvfile:
    data_writer = csv.writer(csvfile, delimiter=',')
    # Write headers
    data_writer.writerow(['timestamp', 'sequence_index', 'signal_id', 'x', 'y', 'value', 'pixel_value'])
    # Write data
    for data in extracted_data:
        data_writer.writerow(data)

print(f"Data extraction complete. CSV file created at {output_csv_path}")

"""