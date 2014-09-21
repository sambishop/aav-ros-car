#!/bin/sh

v4l2-ctl -d /dev/video1 -c exposure_auto_priority=0
v4l2-ctl -d /dev/video1 -c exposure_auto=1
v4l2-ctl -d /dev/video1 -c exposure_absolute=200
