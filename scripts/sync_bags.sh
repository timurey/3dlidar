#!/bin/bash
# Sync ROS2 bag files from Orange Pi to local machine
# Usage: ./scripts/sync_bags.sh [--delete]

OPI_HOST="openclaw@192.168.1.108"
OPI_BAGS="/home/openclaw/bags/"
LOCAL_BAGS="$(dirname "$0")/../bags/"

mkdir -p "$LOCAL_BAGS"

RSYNC_OPTS="-avz --progress"
if [[ "$1" == "--delete" ]]; then
    RSYNC_OPTS="$RSYNC_OPTS --delete"
fi

echo "Syncing bags from $OPI_HOST:$OPI_BAGS → $LOCAL_BAGS"
rsync $RSYNC_OPTS "$OPI_HOST:$OPI_BAGS" "$LOCAL_BAGS"
echo "Done. Bags: $(ls "$LOCAL_BAGS" | wc -l | tr -d ' ')"
