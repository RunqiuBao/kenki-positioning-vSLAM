# kenki-posi
This project is for real-time positioning for construction machines at dynamic construction sites using stereo visual SLAM.<br>
![](https://media.giphy.com/media/Vi0IK2xH1oGWAy7fns/giphy.gif)
![](https://media.giphy.com/media/hQEldL313t8MT3GDzf/giphy.gif)

----------------------------------------
## How to use?
- First install the dependencies.
- Download our dataset for the demo.
- Build with the script build.sh.
- Run the demo with the script Example/Stereo/auto10.sh

----------------------------------------
## Dependencies
**C++11 Compiler**<br>
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
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DWITH_TBB=ON -DBUILD_NEW_PYTHON_SUPPORT=ON -DWITH_V4L=ON -DWITH_OPENGL=ON -DENABLE_FAST_MATH=1 -DCUDA_FAST_MATH=1 -DWITH_CUBLAS=1 -DBUILD_TIFF=ON ..  
make -j4
make install
```

**Eigen3**<br>
Required by g2o (see below). Install like this:
```
cd /path/to/working/dir
wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.tar.bz2
tar xf eigen-3.3.4.tar.bz2
rm -rf eigen-3.3.4.tar.bz2
cd eigen-3.3.4
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
example dataset (10GB)!!<br>
[mydataset(kitti00)](https://drive.google.com/file/d/1T1KrqSesag_-sO5D6IOZttP0yGVrRPhi/view?usp=sharing)<br>
More datasets to be released:<br>

----------------------------------------
## Citation
To use the code, please cite:
```
  @article{doi:10.1080/01691864.2020.1869586,
  author = { Runqiu   Bao  and  Ren   Komatsu  and  Renato   Miyagusuku  and  Masaki   Chino  and  Atsushi   Yamashita  and  Hajime   Asama },
  title = {Stereo camera visual SLAM with hierarchical masking and motion-state classification at outdoor construction sites containing large dynamic objects},
  journal = {Advanced Robotics},
  volume = {0},
  number = {0},
  pages = {1-14},
  year  = {2021},
  publisher = {Taylor & Francis},
  doi = {10.1080/01691864.2020.1869586},
  URL = {https://doi.org/10.1080/01691864.2020.1869586},
  eprint = {https://doi.org/10.1080/01691864.2020.1869586}
  }
```
<br>
You can read the preprint of the paper on Arxiv:)<br>
[preprint](https://arxiv.org/abs/2101.06563)<br>

