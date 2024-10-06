#!/bin/bash

# Directories
raw_csv_dir=~/Desktop/MRS_Master_Project/paper/new_raw_csv
processed_csv_dir=~/Desktop/MRS_Master_Project/paper/processed_csv_automation

# Make sure the processed_csv directory exists
mkdir -p "$processed_csv_dir"

# Loop over all CSV files in the raw_csv directory
for filepath in "$raw_csv_dir"/*.csv; do
    # Get the filename without extension
    filename=$(basename -- "$filepath")
    extension="${filename##*.}"
    name_experiment="${filename%.*}"

    echo "Processing $name_experiment"

    # First subset: rx1_s1
    subset_rx='rx1_s1'
    center_x_min=190
    center_x_max=310

    # Run the Python script
    python processing_raw_csv.py --name_experiment "$name_experiment" --subset_rx "$subset_rx" --center_x_min "$center_x_min" --center_x_max "$center_x_max"

    # Second subset: rx2_s2
    subset_rx='rx2_s2'
    center_x_min=390
    center_x_max=510

    # Run the Python script
    python processing_raw_csv.py --name_experiment "$name_experiment" --subset_rx "$subset_rx" --center_x_min "$center_x_min" --center_x_max "$center_x_max"

done
