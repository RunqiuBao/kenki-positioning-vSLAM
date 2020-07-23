# kenki-posi
This project is for real-time positioning for construction machines at dynamic construction sites using stereo visual SLAM.
![](https://media.giphy.com/media/QyPMp6Uyjc5vyCYv7b/giphy.gif)

----------------------------------------
## How to use?
- First install the dependencies.
- Download our dataset for the demo.
- Build with the script build.sh.
- Run the demo with the script Example/Stereo/auto10.sh

----------------------------------------
## Dependencies
**C++11 or C++0x Compiler**<br>
We use the new thread and chrono functionalities of C++11.

**Pangolin**<br>
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Install it like this:
```
cd /path/to/working/dir
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout ad8b5f83222291c51b4800d5a5873b0e90a0cf81
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    ..
make -j4
make install
```


**OpenCV**<br>
We use [OpenCV](http://opencv.org) **3.4.0**. You need to install with CUDA enabled. Install it like this:
```
cd /path/to/working/dir
wget -q https://github.com/opencv/opencv/archive/3.4.0.zip
unzip -q 3.4.0.zip
rm -rf 3.4.0.zip
cd opencv-3.4.0
mkdir -p build && cd build
cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \ 
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_TBB=ON \
    -D BUILD_NEW_PYTHON_SUPPORT=ON \
    -D WITH_V4L=ON \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D BUILD_EXAMPLES=ON \
    -D WITH_OPENGL=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D BUILD_TIFF=ON \
    ..  
make -j4
make install
```

**Eigen3**<br>
Required by g2o (see below). Install like this:
```
cd /path/to/working/dir
wget -q http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2
tar xf 3.3.4.tar.bz2
rm -rf 3.3.4.tar.bz2
cd eigen-eigen-5a0156e40feb
mkdir -p build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    ..
make -j4
make install
```

**DBoW2 and g2o (Included in Thirdparty folder)**<br>
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

----------------------------------------
## Dataset<br>
Large file alert (10GB)!!<br>
[mydataset](https://drive.google.com/file/d/1OPh0pPgMUlDg8l0BVCaS8fnzbx-rpWPL/view?usp=sharing)

----------------------------------------
## Citation
This work is based on ORB-SLAM2, please cite:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }
