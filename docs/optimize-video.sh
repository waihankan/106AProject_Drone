#!/bin/bash

# Video Optimization Script for Drone Control Project Website
# Usage: ./optimize-video.sh input-video.mp4 output-name
# Example: ./optimize-video.sh my-aruco-video.mp4 aruco-demo

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <input-video> <output-name>"
    echo "Example: $0 raw-footage.mp4 aruco-demo"
    exit 1
fi

INPUT="$1"
OUTPUT_NAME="$2"
ASSETS_DIR="website/assets"

# Check if input file exists
if [ ! -f "$INPUT" ]; then
    echo "Error: Input file '$INPUT' not found!"
    exit 1
fi

# Create assets directory if it doesn't exist
mkdir -p "$ASSETS_DIR"

echo "========================================="
echo "Video Optimization for Web"
echo "========================================="
echo "Input: $INPUT"
echo "Output: ${ASSETS_DIR}/${OUTPUT_NAME}.mp4"
echo ""

# Show original file info
echo "Original file size:"
ls -lh "$INPUT" | awk '{print $5}'
echo ""

# Optimize video
echo "Optimizing video..."
ffmpeg -i "$INPUT" \
  -vf scale=1280:720 \
  -r 30 \
  -c:v libx264 \
  -crf 25 \
  -preset medium \
  -movflags +faststart \
  -c:a aac -b:a 96k \
  "${ASSETS_DIR}/${OUTPUT_NAME}.mp4" \
  -y 2>&1 | grep -E 'Duration|Stream|frame=' | tail -5

echo ""
echo "Generating thumbnail..."

# Generate poster image (thumbnail at 3 seconds)
ffmpeg -i "${ASSETS_DIR}/${OUTPUT_NAME}.mp4" \
  -ss 00:00:03 \
  -vframes 1 \
  -q:v 2 \
  "${ASSETS_DIR}/${OUTPUT_NAME}-poster.jpg" \
  -y 2>&1 | grep -E 'Output|frame='

echo ""
echo "========================================="
echo "Optimization Complete!"
echo "========================================="
echo ""
echo "Optimized file size:"
ls -lh "${ASSETS_DIR}/${OUTPUT_NAME}.mp4" | awk '{print $5}'
echo ""
echo "Files created:"
echo "  - ${ASSETS_DIR}/${OUTPUT_NAME}.mp4"
echo "  - ${ASSETS_DIR}/${OUTPUT_NAME}-poster.jpg"
echo ""
echo "To use in website, add this HTML:"
echo ""
echo '<video class="w-full h-full object-cover"'
echo '       controls'
echo '       preload="metadata"'
echo "       poster=\"assets/${OUTPUT_NAME}-poster.jpg\">"
echo "    <source src=\"assets/${OUTPUT_NAME}.mp4\" type=\"video/mp4\">"
echo "    Your browser does not support the video tag."
echo "</video>"
echo ""
