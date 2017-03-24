#!/bin/bash
PID=$(pgrep SLAM.exe)
sudo renice -20 -p $PID