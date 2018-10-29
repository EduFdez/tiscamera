# Tcam Stereo Capture
This directory contains several sample programs to capture and save images from a stereo camera built from [TIS](https://www.theimagingsource.com/) cameras. The model DFK23UX174 was used to test the programs below.

## Prerequisites
The sample uses the the examples/cpp/common/tcamcamera.cpp and .h files of the *tiscamera* repository as wrapper around the [GStreamer](https://gstreamer.freedesktop.org/) code and property handling. 

You must also install [tiscamera](https://github.com/EduFdez/tiscamera) and [Xsens MT Software Suite 4.8](https://www.xsens.com/mt-software-suite/). 

## Compile
In order to build the sample, open a terminal, enter the sample's directory. Then enter
``` bash
mkdir build && cd build 
cmake ..
make
```

## Usage

### stereocapture_onKeyPress
``` bash
./stereocapture_onKeyPress <output_dir> 
```
This program starts the capture of stereo frames triggered by a keypress. This program is useful to acquire a calibration sequence. 

### imu_stereocapture
``` bash
./imu_stereocapture <output_dir> <fps> <imu_frequency=200>
```
This program starts the capture of stereo frames triggered by the IMU Xsens MTi-100. You must specify the fps which was set externally into the IMU.

### stereo_software_trigger
``` bash
./stereo_software_trigger <output_dir> <fps> <seq_length=100>
```
This program starts the capture of stereo frames triggered by software. 