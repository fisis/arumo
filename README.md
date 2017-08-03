ArUco-based Motion Capture System
---------------------------------

```
/** **************************************************************************************
*                                                                                        *
*    ArUco-based Motion Capture System (ArUMo)                                           *
*    Version 0.1b                                                                        *
*    ----------------------------------------------------------                          *
*    Copyright (C) 2017  Subhrajit Bhattacharya                                          *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify                *
*    it under the terms of the GNU General Public License as published by                *
*    the Free Software Foundation, either version 3 of the License, or                   *
*    (at your option) any later version.                                                 *
*                                                                                        *
*    This program is distributed in the hope that it will be useful,                     *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                      *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                       *
*    GNU General Public License for more details <http://www.gnu.org/licenses/>.         *
*                                                                                        *
*                                                                                        *
*    Contact:  subhrajit@gmail.com                                                       *
*              https://www.lehigh.edu/~sub216/ , http://subhrajit.net/                   *
*                                                                                        *
*                                                                                        *
*************************************************************************************** **/
```

### Description:
Scripts for setting up a motion capture system using overhead cameras.

Installation, Compilation and Running:
--------------------------------------

__Installation of OpenCV with contrib-modules (including ArUco):__
You need to install OpenCv v3.x with contrib modules v3.x in order to use this program. Please follow the instructions here: http://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html . It is suggested that you use `/usr/local` as `CMAKE_INSTALL_PREFIX`. The following sequence of commands will install OpenCv 3.2.0 with contrib modules:
```
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

cd /path/to/work_directory
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

cd /path/to/work_directory/opencv
git checkout 3.2.0
cd /path/to/work_directory/opencv_contrib
git checkout 3.2.0

cd /path/to/work_directory/opencv
mkdir build
cd build


cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/path/to/work_directory/opencv_contrib/modules  /path/to/work_directory/opencv/
make -j8
sudo make install
```


__Compilation:__ Run `make all`. This will create the executables in the 'bin' folder.

__Execution:__ Run the shell script `./mocap.sh`.

*******************************************************************************

Version history:
---------------

* August 2017: Version 0.1b releassed.

