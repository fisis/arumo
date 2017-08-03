#!/bin/bash

defaultcamid=1
printf "\n"

# ----------------------------------------------------------

read -p $'Create AVI from image [y | (n)]? ' -n 1 createavi
printf "\n"
if [ "$createavi" == "y" ] || [ "$createavi" == "Y" ] ; then
    read -p $'Enter image file name: ' imfname
    avconv -loop 1 -i $imfname -c:v mpeg2video -t 1 -pix_fmt yuv420p loopback_video.avi
fi

# ----------------------------------------------------------

read -p $'Start v4l2loopback [(y) | n]? ' -n 1 domodprobe
printf "\n"
if [ "$domodprobe" != "n" ] && [ "$domodprobe" != "N" ] ; then
    sudo modprobe -v v4l2loopback video_nr=$defaultcamid
fi

# ----------------------------------------------------------

defaultcamdev=/dev/video$defaultcamid

read -p $"Enter camera device to stream to ($defaultcamdev): " camdev
camdevexists=$(ls "$camdev")
if [ "$camdevexists" == "" ]; then
    camdev=$defaultcamdev
fi

while true; do
    gst-launch filesrc location=./loopback_video.avi ! decodebin ! v4l2sink device=$camdev;
    IFS= read -r -t 0.01 -n 1 -s holder && key="$holder"
    if [ "$key" == "q" ] || [ "$key" == "Q" ] ; then break; fi
done

sudo modprobe -r v4l2loopback
printf "\n"

