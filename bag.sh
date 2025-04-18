#!/bin/bash

# Define an array of topics to exclude
EXCLUDED_TOPICS_ARRAY=(
  "/camera/(.*)"
  "/can/(.*)"
  "/can_(.*)"
  "/comms/(.*)"
  "/convoy/(.*)"
  "/front/(.*)"
  "/heartbeat/(.*)"
  "/iop/(.*)"
  "/jaus(.*)"
  "/lidar/(.*)"
  "/modes/(.*)"
  "/motion_execution/(.*)"
  "/navigation/(.*)"
  "/payload/(.*)"
  "/persistent_map/(.*)"
  "/radar/(.*)"
  "/rear/(.*)"
  "/vision/(.*)"
  "/web_ui/(.*)"
  "/world_model/(.*)"
)

# Convert array to regex string
EXCLUDED_TOPICS_REGEX=$(IFS='|'; echo "${EXCLUDED_TOPICS_ARRAY[*]}")

# Start recording
rosbag record -a --exclude="($EXCLUDED_TOPICS_REGEX)" -o radio-localization
