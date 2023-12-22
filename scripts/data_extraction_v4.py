#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import csv
import os
import cv2
import bisect


bridge = CvBridge()
sequence_counter = 0  # Counter for sequence index



bag_path = os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/diff_power_5m_topics.bag')
output_csv_path = os.path.expanduser('~/Desktop/MRS_Master_Project/Experiments_data/diff_power_15m_check.csv')

# Directory to save images
image_output_dir = os.path.expanduser('~/Desktop/MRS_Master_Project/Experiments_data/images/diff_power_5_clean')


# Storage for image and omta messages
image_messages = []
omta_messages = []
# Global set to keep track of unique points across all frames
unique_points = set()

# Read the bag file and store image and omta messages in separate lists
with rosbag.Bag(bag_path, 'r') as bag:
    last_timestamp = None
    
    for topic, msg, t in bag.read_messages(topics=['/uav7/uvdar_bluefox/left/image_raw', '/uav7/uvdar/omta_all_seq_info_back']):
        
        #Take only the first 5000 messages
        if len(image_messages) > 10 and len(omta_messages) > 10:
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


def annotate_and_save_image(cv_image, omta_msg, image_filename, frame_timestamp,unique_points):

    movement = False
    # Check if there's any point to annotate
    if any(seq_var.signal_id > 0 and any(seq_point.point.value for seq_point in seq_var.sequence) for seq_var in omta_msg):
        
        # Create a copy of the image to draw on
        annotated_image = cv_image.copy()
        # Convert to BGR for color drawing if necessary
        if len(annotated_image.shape) == 2:  # if grayscale, convert to BGR
            annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_GRAY2BGR)

        # Iterate over each sequence variable to collect points for drawing
        #points_to_draw = []
        #As a set
        #print(f"Frame Timestamp: {frame_timestamp}")
        points_to_draw = set()

        for seq_var in omta_msg:
            if seq_var.signal_id > 0:
                for seq_point in seq_var.sequence:
                    if seq_point.point.value:  # Check if LED is on
                        x, y = int(seq_point.point.x), int(seq_point.point.y)
                        if 0 <= x < annotated_image.shape[1] and 0 <= y < annotated_image.shape[0]:
                            #points_to_draw.append((x, y))
                            #Check if the point is already in the set
                            points_to_draw.add((x,y))
                            #print(f"Detected point at ({x}, {y})")
                            # Add the point to the global set
                            unique_points.add((x, y, frame_timestamp))

        # Draw all points in a batch
        #print(f"Number of points to draw: {len(points_to_draw)}")
        if len(points_to_draw) > 15:
            #print("Movement detected")
            movement = True
        else:
            movement = False

        """
        for x, y in points_to_draw:
            annotated_image[y, x] = (0, 0, 255)  # Color the pixel red

        # Save the annotated image if there were points to draw
        if points_to_draw:
            #cv2.imwrite(image_filename, annotated_image)
            #print("Timestamp: ", frame_timestamp, "Points: ", point_counter)
        """
        return movement

    """
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
    """


# Function to perform pooling around a point
def pooling_around_point(img, x, y, kernel_size, pooling_type='max'):
    half_k = kernel_size // 2
    # Ensure the neighborhood does not go out of image bounds
    neighborhood = img[max(y - half_k, 0):min(y + half_k + 1, img.shape[0]),
                       max(x - half_k, 0):min(x + half_k + 1, img.shape[1])]
    
    if pooling_type == 'max':
        return np.max(neighborhood)
    elif pooling_type == 'average':
        return np.mean(neighborhood)


# Preprocess image timestamps: Convert to seconds and store in a sorted list
image_timestamps_sec = [img_timestamp * 1e-9 for img_timestamp, _ in image_messages]
print("Finished preprocessing image timestamps")
    
# Variables to keep track of the last seen image message and its timestamp
last_seen_image_msg = None
last_seen_img_timestamp = None
corresponding_image_msg = None
frame_timestamp = None


for omta_timestamp, omta_msg in omta_messages:
    for seq_vars in omta_msg.sequences:
        signal_id = seq_vars.signal_id
        sequence_counter += 1  # Increment sequence counter for a new sequence


        for seq_point in seq_vars.sequence:
            point_timestamp = seq_point.insert_time.secs + seq_point.insert_time.nsecs * 1e-9
        
            # Find the closest preceding image message for each point
            # Find the index of the closest image timestamp
            index = bisect.bisect_right(image_timestamps_sec, point_timestamp) - 1

            # Check if a valid index is found
            if 0 <= index < len(image_messages):
                # Retrieve the corresponding image
                corresponding_image_msg = image_messages[index][1]
                # Save the image timestamp
                frame_timestamp = image_messages[index][0]
                #Convert to seconds
                frame_timestamp = frame_timestamp * 1e-9
                
            """   
            if corresponding_image_msg:
                try:
                    # Convert the image message to a cv_image
                    cv_image = bridge.imgmsg_to_cv2(corresponding_image_msg, "mono8")
                    # Save the image
                    image_filename = f"{image_output_dir}/diff_power_5{omta_timestamp}.png"
                    movement=annotate_and_save_image(cv_image, omta_msg.sequences, image_filename,frame_timestamp,unique_points)
                 
                except CvBridgeError as e:
                    print(e)
            """
           
            # Extract the pixel value for the current point
            if corresponding_image_msg:
                
                try:
                    cv_image = bridge.imgmsg_to_cv2(corresponding_image_msg, "mono8")

                    image_filename = f"{image_output_dir}/diff_power_5{omta_timestamp}.png"
                    #movement = annotate_and_save_image(cv_image, omta_msg.sequences, image_filename,frame_timestamp,unique_points)
                     # Save the image
                    movement = False
                    cv2.imwrite(image_filename, cv_image)
                    if movement:
                       continue
                
                    # Assuming the image is valid and the point is within the image bounds
                    point = seq_point.point
                    pixel_value = cv_image[int(point.y)][int(point.x)]
                    kernel_size = 5

                    # Perform max pooling around the point
                    max_pool_value = pooling_around_point(cv_image, int(point.x), int(point.y), kernel_size, pooling_type='max')
                    # Perform average pooling around the point
                    average_pool_value = pooling_around_point(cv_image, int(point.x), int(point.y), kernel_size, pooling_type='average')
                    
                    # Append all the information to extracted_data
                    extracted_data.append([
                        point_timestamp,  # Timestamp of the point
                        sequence_counter,  # Sequence index
                        signal_id,
                        point.x,
                        point.y,
                        point.value,  # The LED state (On/Off)
                        pixel_value, # The pixel value at the point's coordinates
                        max_pool_value,
                        average_pool_value
                    ])
                except CvBridgeError as e:
                    print(e)
      

print(f"Number of extracted data points: {len(extracted_data)}")

# Release the video writer
"""

# Write the extracted data to a CSV file
with open(output_csv_path, 'w', newline='') as csvfile:
    data_writer = csv.writer(csvfile, delimiter=',')
    # Write headers
    data_writer.writerow(['timestamp', 'sequence_index', 'signal_id', 'x', 'y', 'value', 'pixel_value', 'max_pool_value', 'average_pool_value'])
    # Write data
    for data in extracted_data:
        data_writer.writerow(data)

print(f"Data extraction complete. CSV file created at {output_csv_path}")

"""