#!/bin/bash

# ===
# === TELL WHAT'S GONNA HAPPEN ===
# ===
echo
echo "=== This script is installing GStreamer and makes"
echo "=== a symbolic link of GStreamer Python package" 
echo "=== to the Python virtual environment to make Gstreamer"
echo "=== work with Python packages in virtual environment."
echo
echo "=== Script will ask for sudo password."
echo


# ===
# === ASK TO INSTALL GSTREAMER ===
# ===
read -p "Install GStreamer? [y=yes, N=quit]  " response
if [[ $response =~ ^([yY])$ ]]
then
    echo
    echo "==== Installing GStreamer ===="
    echo
    sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
              gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
              gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa \
              gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
    
    # To uninstall packages run:
    # sudo apt remove libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    #                     gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    #                     gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa \
    #                     gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
    # sudo apt autoremove
fi


# ===
# === Make symbolic link of system's GStreamer package
# === to the virtual environment package folder.
# ===
echo
echo
read -p "Make GStreamer's symbolic link to virtual environment? [y=yes, N=quit]  " response
if [[ ! $response =~ ^([yY])$ ]]
then
    echo
    echo "==== Exiting ===="
    echo
    exit 1
fi

echo
echo "==== Making symbolic link ===="
echo
GSTREAMERFOLDER="/usr/lib/python3/dist-packages/gi"
VIRTUALENVFOLDER="venv/lib/python3.*/site-packages"
if [ ! -d $GSTREAMERFOLDER ]
then
    echo "=== GStreamer intallation folder not found at '$GSTREAMERFOLDER'."
    echo "=== Exiting..."
    exit 1
fi

if [ -d $VIRTUALENVFOLDER/gi ]
then
    echo "=== GStreamer already found at '$VIRTUALENVFOLDER/gi'."
else
    cd $VIRTUALENVFOLDER
    ln -s $GSTREAMERFOLDER
    echo "=== GStreamer symbolic link created to '$VIRTUALENVFOLDER/gi'."
fi

echo
echo "==== All done ===="
echo
