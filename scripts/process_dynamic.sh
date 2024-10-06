#!/bin/bash

# Directories
raw_csv_dir=~/Desktop/MRS_Master_Project/paper/dynamic_raw_csv
processed_csv_dir=~/Desktop/MRS_Master_Project/paper/processed_csv_automation_dynamic

# Make sure the processed_csv directory exists
mkdir -p "$processed_csv_dir"

# Loop over all CSV files in the raw_csv directory
for filepath in "$raw_csv_dir"/*.csv; do
    # Get the filename without extension
    filename=$(basename -- "$filepath")
    extension="${filename##*.}"
    name_experiment="${filename%.*}"

    echo "Processing $name_experiment"

    # Run the Python script
    python new_processing_raw_csv_dynamic.py --name_experiment "$name_experiment"

done
