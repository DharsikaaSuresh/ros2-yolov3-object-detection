#!/bin/bash

# URL of the file to download
FILE_URL="https://pjreddie.com/media/files/yolov3.weights"

SCRIPT_DIR="$(dirname "$0")"

# Destination file path
DEST_FILE="$SCRIPT_DIR/yolov3_data/yolov3.weights"

# Check if the file already exists
if [ ! -f "$DEST_FILE" ]; then
    echo "File does not exist. Downloading..."
    curl -o "$DEST_FILE" "$FILE_URL"
else
    echo "File already exists."
fi