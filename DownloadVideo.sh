#!/bin/bash
_now=$(date + "%m_%d_%Y")
ffmpeg -analyzeduration 100M -probesize 10M -c h264 -an -sn -i ftp://192.168.42.1/internal_000/recordedVideo.h264 -vcodec libx264 -ss 1 -map_chapters -1 -map_metadata -1 -an -sn ~/Videos/bebop_$now.mp4

