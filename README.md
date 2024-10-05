# rpg_svo_drive

This forked repo cointains a modified version of SVO's **monocular** visual-frontend that can incorporate external poses as **motion prior** from an **arbitrary** odometry sensor as long as the poses are published as a dynamic ros tf. Optimal for prototyping visual odometry for automotive use-cases with sensors that provide metric scale like wheel-encoders, radars or GPS.

# At a glance
## Table of content
- [Preface](#preface)
  - [Why SVO pro?](#why-svo-pro)
- [Install](#install)
- [Implementation details](#implementation-details)
- [Tuning and preparing your own data](#prepare-your-own-data-and-parameter-files)
  - [External odometry source](#external-odometry-source)
    - [Pose format](#pose-format)
    - [External pose coodrinate-frame convention](#coordinate-frame-convention-of-external-poses)
    - [Weighting of external poses](#weighting-of-external-poses)
    - [Extrinsic calibration from odometry source to lense](#extrinsic-calibration-from-odometry-sensor-to-camera-lense)
  - [Additional thoughts on tuning and implementation](#some-additional-details-on-implementation-sensors-and-tuning)
- [Usage](#usage)
- [Original readme](#original-readme)
## Limitations

- Monocular front-end only, when using motion prior from external odometry. IMU or stereo cannot be used in conjuction. When external odometry is toggled off, SVO should behave as the original version.
- No ceres backend when using motion prior from external odometry. Atleast not yet
- No global map and loop closure when using motion prior from external odometry.

# Preface

## Why SVO Pro?

Compared to feature-based methods, **direct** visual odometry is more robust in **repetitve** and **low textured** scenes like parking garages or other man-made structures. SVO Pro is a direct visual (inertial) odometry (VIO) ros-package that is well structured, easy to setup and easy to protype. Most importantly it can directly incorporate **motion priors** into its photometric front-end (read original paper for more details) opening up the possibility to **inject motion priors** from an **arbitrary odometry sensor** with **minimal coding effort** and **without breaking the consistency** of the system. 

With this type of interface, the **monocular front-end**, which is **still known to be a brittle** component of many visual odometry algorithms in real-world scenarios, can directly recover metric scale and quickly be robustified with odometry sensors available in the automotive-domain. This holds a lot of potential as generic building block to to rapidly prototype all sorts of automotive-centric applications. 

## Front-end motion prior vs fusing external odometry into the factor graph

When done properly, external odometry should be fused in the VIO back-end factor graph. This requires, to some extend, engineering time and know-how. And even then it can still be a struggle to wrap your head around why your front-end  won't properly initialize. If you simply want to utilize your dusting wheel-encoder or radar data for quick-and-dirty proof-of-concepts, hijacking SVOs motion prior is a great starting point.

## Why not just use IMU or stereo camera data with the original SVO Pro?

Original SVO Pro has definitely been proven in the past (not by me) to work with IMU or stereo cameras for automotive use-cases. Hovewer some considerations to make when rapid-prototyping on your own:

- Planar and linear motion of car-like vehicles can lead to degeneracy for monocular visual inertial configurations (<https://maplab.asl.ethz.ch/docs/master/pages/tutorials-rovioli/A_ROVIOLI-Introduction.html#known-issues>). For quick proof-of-concepts, stable metric scale can be obtained faster from more reliable odometry sources.
- Low-cost IMUs can be quite delicate. Unless you've got a bit of experience, calibrating and debugging the intrinsics of your IMU can be time consuming, error prone and **hard to validate**. It's faster to integrate less care-demanding sensors like wheel odometry to get your VO up and running.
- Stereo cameras are a nice way to directly obtain metric scale but without a bit of digging it is not easy to determine what baseline to choose for your scene. Furthermore, with stereo only you are still subjugated to the weaknesses of a vision only sensor source.

# Install

This adjusted repo has been tested on:

- Ubuntu 20.04 with ROS Noetic

## Install dependencies

Follow [install dependencies](https://github.com/uzh-rpg/rpg_svo_pro_open/tree/master?tab=readme-ov-file#install-dependences) from original SVO Pro.

## Clone

Follow [cloning instructions](https://github.com/uzh-rpg/rpg_svo_pro_open/tree/master?tab=readme-ov-file#clone-and-compile) from original SVO Pro **BUT** make sure to clone **THIS** repo (`rpg_svo_pro_drve`) instead of the original one.

## Compile

Follow [compile instructions](https://github.com/uzh-rpg/rpg_svo_pro_open/tree/master?tab=readme-ov-file#install-dependences) from original SVO Pro for building your workspace **without** the global map.

- After following instructions from original repo, a simple `build` in your workspace should do the trick.

```
catkin build
```

# Implementation details
Check the following code sections to see the inner workings of the pose fusion. More information is provided in the [Tuning and preparing your own data](#prepare-your-own-data-and-parameter-files) section.  
1. How external odometry tfs and extrinsic calib from odom sensor to camera lense is fetched by svo:
   - [svo_ros/src/svo_interface.cpp#L358](svo_ros/src/svo_interface.cpp#L358)
1. How an absolute pose from external odom sensor is being expressed as absolute cam pose using extrinsic calibration and assigned to each frame:
   - [svo/src/frame_handler_base.cpp#L236](svo/src/frame_handler_base.cpp#L236)
1. How 2 absolute camposes between 2 images are transformed into relative camera motion and used to override IMU motion prior from original codebase:
   - [svo/src/frame_handler_base.cpp#L1191](svo/src/frame_handler_base.cpp#L1191)
1. How external odometry is used during initizalization to bootstrap the metric map:
   - [svo/src/initialization.cpp#L383](svo/src/initialization.cpp#L383)  - stuffing external translation into 5 point essential matrix for initialization
   - [svo/src/initialization.cpp#L384](svo/src/initialization.cpp#L384)  - retaining rotation recovered from 5 point essential matrix for initialization
1. How monocular scale is bypassed during initialization when using external odometry:
   - [svo/src/initialization.cpp#L871](svo/src/initialization.cpp#L871)

# Prepare your own data and parameter files

All sensor data should be bundled into rosbag format or published live as ros stream. Example params for a setup with wheel-odometry and zed2i mono images can be found here:

- [pinhole_zed2i.yaml](svo_ros/param/pinhole_zed2i.yaml) (VIO parameters)
- [zed2i_left_rectified.yaml](svo_ros/param/calib/zed2i_left_rectified.yaml) (cam calib)
- [run_from_bag_zed2i.launch](svo_ros/launch/frontend/run_from_bag_zed2i.launch) including extrinsic calibration from odometry sensor to camera lense

Example for running the setup:

```
roslaunch svo_ros run_from_bag_zed2i.launch cam_name:=zed2i_left_rectified
```

## External odometry source

Any source can be used as long as it provides **absolute** poses with respect to some static reference frame, for example:

- Wheel odometry
- Radar odometry
- GPS
- LiDAR odometry / SLAM
- Motion capture poses

Ideally the poses come with higher frequency than the image data. When using slower sensors like GPS or LiDAR, the poses might need to be interpolated offline and rebagged.

### Pose format
  
Publish absolute poses of external odometry source as dynamic tf. If your pose is only being published as `odometry` or `pose` `msg` you can write a small python node that subscribes to these msgs and publish corresponding tfs accordingly. Make sure to feed the exact names of `child` and `parent` frame of your pose tf at [this section](https://github.com/kraxel-dev/rpg_svo_pro_drive/blob/inject_cam_pose_as_motion_prior/svo_ros/param/pinhole_zed2i.yaml#L141) of your parameter file. Otherwise SVO interface will fail to fetch your pose from the tf tree to use as motion prior.

### Coordinate frame convention of external poses

When using motion prior from external source, the fetched absolute poses between iterations are used to calculate relative motion. Therefore, the external absolute poses can be with respect to any static reference frame as long as they all refer to the **same** static frame that stays consistent over time.

### Availibility of external poses

During initialization of the front-end, external poses must be available. They are required to bootstrap the initial map with metric scale. If either the very first image or the images after minimal disparity has been reached fail to fetch an external pose, the system won't initialize.  

After succesfull initialization the visual front-end can run even without external pose tf messages being present. You can cut your external pose stream after initialization to check how the vision only front-end holds up in a metric map. 

### Weighting of external poses

Define how much the external poses should be trusted in the motion prior in [this section](https://github.com/kraxel-dev/rpg_svo_pro_drive/blob/inject_cam_pose_as_motion_prior/svo_ros/param/pinhole_zed2i.yaml#L137) of the param file.

### Extrinsic calibration from odometry sensor to camera lense

This is the not-so-fun part. You must provide the extrinsic calibration from your odometry sensor to the camera lense yourself. Extrinsics must be published as static ros tf and can be placed in [this section](https://github.com/kraxel-dev/rpg_svo_pro_drive/blob/inject_cam_pose_as_motion_prior/svo_ros/launch/frontend/run_from_bag_zed2i.launch#L11) of your launch file. Make sure that names of **source** and **target frame** in your launch file **match** the names of **body** and **camera frame** that you provide at [this section](https://github.com/kraxel-dev/rpg_svo_pro_drive/blob/inject_cam_pose_as_motion_prior/svo_ros/param/pinhole_zed2i.yaml#L148) of your parameter file. Otherwise extrinsics cannot be fetched and the motion of the external odometry sensor won't be transformed into the motion of the camera.

Provide the extrinsics such that the pose of the camera lense (z-axis pointing forwards) is expressed in the local body-frame of the odometry sensor, not the other way around.

Another important step is to set the extrinsic calibration from camera to IMU at [this section](https://github.com/kraxel-dev/rpg_svo_pro_drive/blob/inject_cam_pose_as_motion_prior/svo_ros/param/calib/zed2i_left_rectified.yaml#L24) of your calib file to the identity transform. Even though IMU is not being used, the motion prior lives in the body frame of the IMU, **not** in the camera frame. It is kept like that to keep consistency with the orgininal codebase.

## Some additional details on implementation, sensors and tuning 
### Initialization and minimal disparity
When operating with slow and linear moving vehicles, feature disparity between frames during initialization will grow slower than during drone take off. Reduce the `min disparity` parameter similar to [here](https://github.com/kraxel-dev/rpg_svo_pro_drive/blob/inject_cam_pose_as_motion_prior/svo_ros/param/pinhole_zed2i.yaml#L72) to allow the front-end to trigger triangulation during init. 

Initialization with external pose currently only implemented with the 5-point initializer. The translational component of the external pose is used to triangulate and bootstrap the map between 2 frames on metric scale. For the rotational component the original rotation from the 5-point essential matrix is kept for triangulation as it is more trustworthy in roll and pitch compared to a wheel-encoder orientation.
See this [section](svo/src/initialization.cpp#L384) in `initialization.cpp` for implementation details. 
### More detailed tuning and parameter explanation
For a more detailed explanation on each parameter of the params.yaml, check out this [header file](svo/include/svo/frame_handler_base.h#L41). 
### Rolling shutter cameras
Rolling shutter cameras as the zed2i should be avoided when using direct VSLAM methods. Though, for the data I tested on (vehicles operating on very low speeds: <5mph ), the zed2i produced satisfactory results.

# Usage
The data this package was validated on is company internal property. Unfortunately, a full working minimal example to run the algorithm on that real-life data cannot be provided. Below is still an example on how to run this package in a hypothetical setup.

## Run mono example with zed2i camera

- Run on zed2i mono rosbag data

  ```
  roslaunch svo_ros run_from_bag_zed2i.launch cam_name:=zed2i_left_rectified
  ```

- Play bag

  ```
  # Should contain absolute wheel odom poses to get the motion prior running
  rosbag play backwards_2_converted.bag -s 10 --clock -r 0.4 --pause
  ```

- Run dynamic tf completer to provide dynamic tfs to the wheel odom poses

  ```
  # Not included in this repo (yet)
  rosrun ros_helper dynamic_tf_completer.py
  ```

# Original Readme

# rpg_svo_pro

This repo includes **SVO Pro** which is the newest version of Semi-direct Visual Odometry (SVO) developed over the past few years at the Robotics and Perception Group (RPG). SVO was born as a fast and versatile visual front-end as described in the [SVO paper (TRO-17)](http://rpg.ifi.uzh.ch/docs/TRO17_Forster-SVO.pdf). Since then, different extensions have been integrated through various research and industrial projects. SVO Pro features the support of different [camera models](http://rpg.ifi.uzh.ch/docs/ICRA16_Zhang.pdf), [active exposure control](http://rpg.ifi.uzh.ch/docs/ICRA17_Zhang.pdf), a sliding window based backend, and global bundle adjustment with loop closure.

In summary, this repository offers the following functionalities:

- Visual-odometry: The most recent version of SVO that supports perspective and fisheye/catadioptric cameras in monocular or stereo setup. It also includes active exposure control.
- Visual-inertial odometry: SVO fronted + visual-inertial sliding window optimization backend (modified from [OKVIS](https://github.com/ethz-asl/okvis))
- Visual-inertial SLAM: SVO frontend + visual-inertial sliding window optimization backend + globally bundle adjusted map (using [iSAM2](https://gtsam.org/)). The global map is updated in real-time, thanks to iSAM2, and used for localization at frame-rate.
- Visual-inertial SLAM with loop closure: Loop closures, via [DBoW2](https://github.com/dorian3d/DBoW2), are integrated in the global bundle adjustment. Pose graph optimization is also included as a lightweight replacement of the global bundle adjustment.

An example of the visual-inertial SLAM pipeline on EuRoC dataset is below (green points - sliding window; blue points - iSAM2 map):

![](./doc/images/v102_gm.gif)

SVO Pro and its extensions have been used to support various projects at RPG, such as our recent work on [multiple camera SLAM](http://rpg.ifi.uzh.ch/docs/ICRA20_Kuo.pdf), [voxel map for visual SLAM](http://rpg.ifi.uzh.ch/docs/ICRA20_Muglikar.pdf) and [the tight-coupling of global positional measurements into VIO](http://rpg.ifi.uzh.ch/docs/IROS20_Cioffi.pdf). We hope that the efforts we made can facilitate the research and applications of SLAM and spatial perception.

## License

The code is licensed under GPLv3\. For commercial use, please contact `sdavide [at] ifi [dot] uzh [dot] ch`.

The visual-inertial backend is modified from OKVIS, and the license is retained at the beginning of the related files.

## Credits

If you use the code in the academic context, please cite:

- Christian Forster, Matia Pizzoli, Davide Scaramuzza. SVO: Fast Semi-Direct Monocular Visual Odometry. ICRA, 2014\. [bibtex](./doc/bib/Forster14icra.bib)
- Christian Forster, Zichao Zhang, Michael Gassner, Manuel Werlberger, Davide Scaramuzza. SVO: Semi-Direct Visual Odometry for Monocular and Multi-Camera Systems. TRO, 2017\. [bibtex](./doc/bib/Forster17tro.bib)

Additionally, please cite the following papers for the specific extensions you make use of:

- _Fisheye/catadioptric camera extension_: Zichao Zhang, Henri Rebecq, Christian Forster, Davide Scaramuzza. Benefit of Large Field-of-View Cameras for Visual Odometry. ICRA, 2016\. [bibtex](./doc/bib/Zhang16icra.bib)
- _Brightness/exposure compensation_: Zichao Zhang, Christian Forster, Davide Scaramuzza. Active Exposure Control for Robust Visual Odometry in HDR Environments. ICRA, 2017\. [bibtex](./doc/bib/Zhang17icra.bib)
- _Ceres-based optimization backend_: Stefan Leutenegger, Simon Lynen, Michael Bosse, Roland Siegwart, Paul Timothy Furgale. Keyframe-based visual–inertial odometry using nonlinear optimization. IJRR, 2015\. [bibtex](./doc/bib/Leutenegger15ijrr.bib)
- _Global map powered by iSAM2_: Michael Kaess, Hordur Johannsson, Richard Roberts, Viorela Ila, John Leonard, Frank Dellaert. iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree. IJRR, 2012\. [bibtex](./doc/bib/Kaess12ijrr.bib)
- _Loop closure_: Dorian Gálvez-López and Juan D. Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences. TRO, 2012\. [bibtex](./doc/bib/Galvez12tro.bib)

Our recent publications that use SVO Pro are:

- _Multiple camera SLAM_: Juichung Kuo, Manasi Muglikar, Zichao Zhang, Davide Scaramuzza. Redesigning SLAM for Arbitrary Multi-Camera Systems. ICRA, 2020\. [bibtex](./doc/bib/Kuo20icra.bib)
- _Voxel map for visual SLAM_: Manasi Muglikar, Zichao Zhang, Davide Scaramuzza. Voxel Map for Visual SLAM. ICRA, 2020\. [bibtex](./doc/bib/Muglikar20icra.bib)
- _Tight-coupling of global positional measurements into VIO_: Giovanni Cioffi, Davide Scaramuzza. Tightly-coupled Fusion of Global Positional Measurements in Optimization-based Visual-Inertial Odometry. IROS, 2020\. [bibtex](./doc/bib/Cioffi20iros.bib)

## Install

The code has been tested on

- Ubuntu 18.04 with ROS Melodic
- Ubuntu 20.04 with ROS Noetic

### Install dependences

Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) and [vcstools](https://github.com/dirk-thomas/vcstool) if you haven't done so before. Depending on your operating system, run

```sh
# For Ubuntu 18.04 + Melodic
sudo apt-get install python-catkin-tools python-vcstool
```

or

```sh
# For Ubuntu 20.04 + Noetic
sudo apt-get install python3-catkin-tools python3-vcstool python3-osrf-pycommon
```

Install system dependencies and dependencies for Ceres Solver

```sh
# system dep.
sudo apt-get install libglew-dev libopencv-dev libyaml-cpp-dev 
# Ceres dep.
sudo apt-get install libblas-dev liblapack-dev libsuitesparse-dev
```

### Clone and compile

Create a workspace and clone the code (`ROS-DISTRO`=`melodic`/`noetic`):

```sh
mkdir svo_ws && cd svo_ws
# see below for the reason for specifying the eigen path
catkin config --init --mkdirs --extend /opt/ros/<ROS-DISTRO> --cmake-args -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3
cd src
git clone git@github.com:uzh-rpg/rpg_svo_pro_open.git
vcs-import < ./rpg_svo_pro_open/dependencies.yaml
touch minkindr/minkindr_python/CATKIN_IGNORE
# vocabulary for place recognition
cd rpg_svo_pro_open/svo_online_loopclosing/vocabularies && ./download_voc.sh
cd ../../..
```

There are two types of builds that you can proceed from here

1. Build without the global map (**front-end + sliding window back-end + loop closure/pose graph**)

  ```sh
  catkin build
  ```

2. Build with the global map using iSAM2 (**all functionalities**)

  First, enable the global map feature

  ```sh
  rm rpg_svo_pro_open/svo_global_map/CATKIN_IGNORE
  ```

  and in `svo_cmake/cmake/Modules/SvoSetup.cmake`

  ```cmake
  SET(USE_GLOBAL_MAP TRUE)
  ```

  Second, clone GTSAM

  ```sh
  git clone --branch 4.0.3 git@github.com:borglab/gtsam.git
  ```

  and modify GTSAM compilation flags a bit:

  ```cmake
  # 1\. gtsam/CMakelists.txt: use system Eigen
  -option(GTSAM_USE_SYSTEM_EIGEN "Find and use system-installed Eigen. If 'off', use the one bundled with GTSAM" OFF)
  +option(GTSAM_USE_SYSTEM_EIGEN "Find and use system-installed Eigen. If 'off', use the one bundled with GTSAM" ON)
  # 2\. gtsam/cmake/GtsamBuildTypes: disable avx instruction set
  # below the line `list_append_cache(GTSAM_COMPILE_OPTIONS_PUBLIC "-march=native")`
  list_append_cache(GTSAM_COMPILE_OPTIONS_PUBLIC "-mno-avx")
  ```

  > Using the same version of Eigen helps avoid [memory issues](https://github.com/ethz-asl/eigen_catkin/wiki/Eigen-Memory-Issues). Disabling `avx` instruction set also helps with some segment faults in our experience (this can be however OS and hardware dependent).

  And finally build the whole workspace

  ```sh
  # building GTSAM may take a while
  catkin build
  ```

## Instructions

- Get started: running the pipeline

  - [The visual front-end](./doc/frontend/visual_frontend.md)
  - [Visual-inertial odometry](./doc/vio.md)
  - [VIO + global map](./doc/global_map.md)

- [Benchmarking](./doc/benchmarking.md)

- [Camera and sensor calibration](./doc/calibration.md)

- [Known issues and possible improvements](./doc/known_issues_and_improvements.md)

## Troubleshooting

1. **Weird building issues after some tinkering**. It is recommend to

  - clean your workspace (`catkin clean --all` at the workspace root) and rebuild your workspace (`catkin build`)
  - or `catkin build --force-cmake`

    after your have made changes to CMake files (`CMakeLists.txt` or `*.cmake`) to make sure the changes take effect.

    <details><summary>Longer explanation</summary>
    Catkin tools can detect changes in CMake files and re-build affected files only. But since we are working with a multi-package project, some changes may not be detected as desired. For example, changing the building flags in <code>svo_cmake/cmake/Modules/SvoSetup.cmake</code> will affect all the packages but the re-compiling may not be done automatically (since the files in each package are not changed). Also, we need to keep the linking (e.g., library version) and compiling flags consistent across different packages. Therefore, unless you are familiar with how the compilation works out, it is the safest to re-build the whole workspace. <code>catkin build --force-cmake</code> should also work in most cases.
    </details>

2. **Compiling/linking error related to OpenCV**: find `find_package(OpenCV REQUIRED)` in the `CMakeLists.txt` files in each package (in `rpg_common`, `svo_ros`, `svo_direct`, `vikit/vikit_common` and `svo_online_loopclosing`) and replace it with

  ```cmake
  # Ubuntu 18.04 + Melodic
  find_package(OpenCV 3 REQUIRED)
  # Ubuntu 20.04 + Noetic
  find_package(OpenCV 4 REQUIRED)
  ```

  <details><summary>Longer explanation</summary>
  First, ROS is built against OpenCV 3 on Ubuntu 18.04 and OpenCV 4 on Ubuntu 20.04\. It is desired to keep the OpenCV version linked in SVO consistent with the ROS one, since in <code>svo_ros</code> we need to link everything with ROS. Second, The original <code>CMakeLists.txt</code> files will work fine if you only have the default OpenCV installed. But if you have some customized version of OpenCV installed (e.g., from source), it is recommended to explicitly specify the version of OpenCV that should be used (=the version ROS uses) as mentione above.
  </details>

3. **Visualization issues with the PointCloud2**: Using `Points` to visualize `PointCloud2` in RVIZ seems to be [problematic](https://github.com/ros-visualization/rviz/issues/1508) in Ubuntu 20.04\. We use other visualization types instead of `Points` per default. However, it is good to be aware of this if you want to customize the visualization.

4. **Pipeline crashes with loop closure enabled**: If the pipeline crashes calling `svo::loadVoc()`, did you forgot to download the vocabulary files as mentioned above?

  ```sh
  cd rpg_svo_pro_open/svo_online_loopclosing/vocabularies && ./download_voc.sh
  ```

5. **Inconsistent Eigen versions during compilation**: The same Eigen should be used across the whole project (which should be system Eigen, since we are also using ROS). Check whether `eigen_catkin` and `gtsam` find the same version of Eigen:

  ```sh
  # for eigen_catkin
  catkin build eigen_catkin --force-cmake --verbose
  # for gtsam
  catkin build gtsam --force-cmake --verbose
  ```

  <details><summary>Longer explanation</summary>
  One common pitfall of using Eigen in your projects is have different libraries compiled against different Eigen versions. For SVO, eigen_catkin (<a href="https://github.com/ethz-asl/eigen_catkin">https://github.com/ethz-asl/eigen_catkin</a>) is used to keep the Eigen version same, which should be the system one (under /usr/include) on 18.04 and 20.04\. For GTSAM, system Eigen is found via a cumstomized cmake file (<a href="https://github.com/borglab/gtsam/blob/develop/cmake/FindEigen3.cmake#L66">https://github.com/borglab/gtsam/blob/develop/cmake/FindEigen3.cmake#L66</a>). It searches for <code>/usr/local/include</code> first, which may contain Eigen versions that are manually installed. Therefore, we explicitly specifies <code>EIGEN_INCLUDE_PATH</code> when configuring the workspace to force GTSAM to find system Eigen. If you still encounter inconsistent Eigen versions, the first thing to check is whether different versions of Eigen are still used.
  </details>

## Acknowledgement

Thanks to Simon Klenk, Manasi Muglikar, Giovanni Cioffi and Javier Hidalgo-Carrió for their valuable help and comments for the open source code.

The work is made possible thanks to the efforts of many contributors from RPG. Apart from the authors listed in the above papers, Titus Cieslewski and Henri Rebecq made significant contributions to the visual front-end. Jeffrey Delmerico made great efforts to apply SVO on different real robots, which in turn helped improve the pipeline. Many PhD and master students and lab engineers have also contributed to the code.

The Ceres-based optimization back-end is based on code developed at [Zurich-eye](https://www.wysszurich.uzh.ch/projects/completed-projects/zurich-eye), a spin-off from RPG. Jonathan Huber is the main contributor that integrated the back-end with SVO. Kunal Shrivastava (now CEO of [SUIND](https://suind.com/)) developed the loop closure module during his semester project and internship at RPG. The integration of the iSAM2-based global map was developed by Zichao Zhang.

We would like to thank our collaborators at [Prophesee](https://www.prophesee.ai/) for pointing out several bugs in the visual front-end. Part of the code was developed during a funded project with [Huawei](https://www.huawei.com/en/).
