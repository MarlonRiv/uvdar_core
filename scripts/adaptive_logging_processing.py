#!/usr/bin/env python
import rosbag
import csv
import rospy
import os
#from uvdar_core.msg import AdaptiveDataForLogging

def extract_data(bag_file, csv_file):
    topics = ['/uav1/uvdar/adaptive_logging_back', '/uav1/uvdar/adaptive_logging_left', '/uav1/uvdar/adaptive_logging_right']

    # Open the rosbag file
    with rosbag.Bag(bag_file, 'r') as bag:
        # Open the CSV file for writing
        with open(csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Write the CSV header
            writer.writerow(["camera_orientation", "timestamp", "num_rois", "roi_detected_points", "roi_threshold_used", "roi_kl_divergence", "roi_is_valid"])

            # Iterate through the specified topics
            for topic in topics:
                camera_orientation = topic.split('_')[-1]  # Extracts 'back', 'left', or 'right'
                for _, msg, t in bag.read_messages(topics=[topic]):
                    # Extract data for each message
                    timestamp = t.to_sec()
                    num_rois = msg.num_rois
                    for i in range(num_rois):
                        detected_points = msg.roi_detected_points[i] if i < len(msg.roi_detected_points) else "NA"
                        threshold_used = msg.roi_threshold_used[i] if i < len(msg.roi_threshold_used) else "NA"
                        kl_divergence = msg.roi_kl_divergence[i] if i < len(msg.roi_kl_divergence) else "NA"
                        is_valid = msg.roi_is_valid[i] if i < len(msg.roi_is_valid) else "NA"
                        
                        # Write a row for each ROI in the CSV, including camera orientation
                        writer.writerow([camera_orientation, timestamp, num_rois, detected_points, threshold_used, kl_divergence, is_valid])

if __name__ == "__main__":
    bag_file= os.path.expanduser('~/Desktop/MRS_Master_Project/rosbags/simulation/sim_otsu_topics.bag')
    csv_file = "sim_otsu_topics.csv"
    extract_data(bag_file, csv_file)
