# SLAMesh
## Real-time LiDAR Simultaneous Localization and Meshing

**Personal repository** of our work SLAMesh, please raise issues **here** so that I can get reminders immediately. The code may also more friendly to use and read, but I don't always guarantee it can replicate the result in out paper.

We are confident that this work introduced a novel approach to LiDAR SLAM, so we publish the code to benefit robotics society and welcome everyone to explore opportunities in this approach. :two_men_holding_hands:

### Update ###

17/Aug/2023, Code released. Spent some time to make my code more user-friendly :triumph:, feel free to contact me with any questions.

10/Mar/2023, Preprint of our paper can be found on: [**paper**](https://arxiv.org/pdf/2303.05252.pdf 'title text').

16/Jan/2023, The paper has been accepted for presentation on **ICRA 2023**,

## 1. Introduction

This work designs a **S**imultaneously **L**ocalization **A**nd **Mesh**ing system (SLAMesh). Mesh is a lightweight 3-D dense model. It can model complex structures and has the feasibility for rendering. We bridge localization with meshing at the same time to benefit each other.

### 1.1 Main features

- Build, register, and update the mesh maps in real time with CPU resources. The experiments show that our SLAMesh can run at around 40 Hz.
- Provide accurate odometry. The localization and meshing accuracy also outperforms the state-of-the-art methods.
- Different from point-cloud (LOAM), NDT, and Surfel map SLAM, this work has established a new approach to LiDAR SLAM.
- The key idea is that we conduct a reconstruction of the raw point cloud before registration. This strategy enables fast meshing, data-association without the kd-tree.

<img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/fig1_mesh_by_slamesh.png" alt="cover" width="60%" />

Author: Jianyuan Ruan, Bo Li, Yibo Wang, Yuxiang Sun.

### 1.2 Demo video

On public dataset:

<a href="https://www.youtube.com/watch?v=bm9u0-d4giw" target="_blank"><img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/cover.png" alt="video1" width="85%" /></a>

(or watch it on [bilibili](https://www.bilibili.com/video/BV1HB4y1J7k3/?vd_source=a7075e8cce0b5d3273610c2b2539377d))

On self-collected dataset:

<a href="https://www.youtube.com/watch?v=-zMNndGmUho" target="_blank"><img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/real_word_cover2.png" alt="video2" width="85%" /></a>

(or watch it on [bilibili](https://www.bilibili.com/video/BV1u84y1h7g3/?vd_source=a7075e8cce0b5d3273610c2b2539377d))

### 1.3 Find more detail

If you find our research helpful, please cite our [**paper**](https://arxiv.org/pdf/2303.05252.pdf 'title text'). :

[1] Jianyuan Ruan, Bo Li, Yibo Wang, and Yuxiang Sun, "SLAMesh: Real-time LiDAR Simultaneous Localization and Meshing" ICRA 2023.


Other related papers:

[2] Jianyuan Ruan, Bo Li, Yinqiang Wang and Zhou Fang, "GP-SLAM+: real-time 3D lidar SLAM based on improved regionalized Gaussian process map reconstruction," IROS 2020. [link](https://ieeexplore.ieee.org/abstract/document/9341028).

[3] Bo Li, Jianyuan Ruan, Yu Zhang, et al, 3D SLAM method based on improved regionalized Gaussian process map construction, 2020 International Conference on Guidance, Navigation and Control (ICGNC), 2022. [link](https://link.springer.com/chapter/10.1007/978-981-15-8155-7_296)

[4] Jianyuan Ruan, Zhou Fang, Bo Li, et al, Evaluation of GP-SLAM in real-world environments, 2019 Chinese Automation Congress (CAC), 2019. [link](https://ieeexplore.ieee.org/document/8996403)

[5] Bo Li, Yinqiang Wang, Yu Zhang. Wenjie Zhao, Jianyuan Ruan, and Pin Li, "GP-SLAM: laser-based SLAM approach based on regionalized Gaussian process map reconstruction". Auton Robot 2020.[link](https://link.springer.com/article/10.1007/s10514-020-09906-z)


If you understand Chinese, you can also refer to my [Master's thesis](https://connectpolyu-my.sharepoint.com/:b:/g/personal/21041552r_connect_polyu_hk/ESjrlb1oNbVMr4tFeG4bhY0BO0jmY-hlC61a3y67whp-Ww?e=pqOtjT) (now can download it without login) and an article on the WeChat platform: [SLAMesh: 实时LiDAR定位与网格化模型构建
](https://mp.weixin.qq.com/s/zYORZ1sOVkh-UnPkzzfh_g).


## 2. Install

### 2.1 Prerequisite

We tested our code in *Ubuntu18.04* with *ROS melodic* and *Ubuntu20.04* with *ROS neotic*.

**ROS**

Install ros following [ROS Installation](http://wiki.ros.org/melodic/Installation/Ubuntu). We use the PCL and Eigen library in ROS.

**Ceres**

Follow [Ceres Installation](http://ceres-solver.org/installation.html) to install Ceres Solver, tested version: 2.0, 2.1 (Error observed with V-2.2).

**mesh_tools**

We use mesh_tools to visualize the mesh map with the `mesh_msgs::MeshGeometryStamped` ROS message. mesh_tools also incorporates navigation functions upon mesh map. [Mesh tool introduction](https://github.com/naturerobots/mesh_tools)

Install mesh_tools by:

1. Install [lvr2](https://github.com/uos/lvr2):
```
sudo apt-get install build-essential \
     cmake cmake-curses-gui libflann-dev \
     libgsl-dev libeigen3-dev libopenmpi-dev \
     openmpi-bin opencl-c-headers ocl-icd-opencl-dev \
     libboost-all-dev \
     freeglut3-dev libhdf5-dev qtbase5-dev \
     qt5-default libqt5opengl5-dev liblz4-dev \
     libopencv-dev libyaml-cpp-dev
```
In Ubuntu18.04, use `libvtk6` because `libvtk7` will conflict with `pcl-ros` in melodic.
```
sudo apt-get install  libvtk6-dev libvtk6-qt-dev
```
In Ubuntu 20.04,
```
sudo apt-get install  libvtk7-dev libvtk7-qt-dev
```

then:
```
cd a_non_ros_dir
```
build:
```
git clone https://github.com/uos/lvr2.git
cd lvr2 
mkdir build && cd build
cmake .. && make
sudo make install
```
It may take you some time.

2. Install mesh_tools, (I can not install it from official ROS repos now, so I build it from source)

```
cd slamesh_ws/src
git clone https://github.com/naturerobots/mesh_tools.git
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

### 2.2 SLAMesh

Clone this repository and build:
```
cd slamesh_ws/src
git clone https://github.com/RuanJY/SLAMesh.git
cd ..
catkin_make
source ~/slamesh_ws/src/devel/setup.bash
```
### 2.3 Docker support

If you encounter some trouble with prerequisites, the problem may lay down on the prerequisite; we advise you to use our docker image:

```
docker pull pleaserun/rjy_slam_work:slamesh_18.04
```
After cloning the image, remember to run it with correct path of dataset via option `-v`, like:
```
docker run -it -p 5900:5900 -p 2222:22 -e RESOLUTION=1920x1080  \
-v path_in_your_PC:/root/dataset \
--name test_slamesh \
pleaserun/rjy_slam_work:slamesh_18.04
```
Then you can use VNC to enter a graphical interface via port 5900, or use ssh to connect container via port 2222.

Move to the dictionary `slamesh_ws/src`, and complete step after 2.1.

## 3. Usage

### 3.1 Run kitti dataset:

The dataset is available at [KITTI dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php).

Set the parameter `data_path` in `slamesh_kitti.launch` to your folder of kitti dataset path

The file tree should be like this:

```
file_loc_dataset
    ├── 00
    |   └──velodyne
    |       ├── 000000.bin
    |       └── ...
    └──01   

```
For example, if you want to run the 07 sequence:
```
roslaunch slamesh slamesh_kitti_meshing.launch seq:=/07
```
You should get:

<img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/kitti07_mesh.png" alt="kitti07_mesh" width="60%" />


If you can not see the mesh, check that the Rviz plugin is sourced correctly. When `mesh_visualization` is disabled, only vertices are published as a point cloud.



### 3.2 Run Mai City dataset:

The dataset is available at [Mai City Dataset](https://www.ipb.uni-bonn.de/data/mai-city-dataset/). Sequence 01 can be fed into SLAM and sequence 02 can be accumulated into a dense ground truth point cloud map.

Similarly, set the parameter `data_path` in `slamesh_maicity.launch` to your folder of the kitti dataset path.

```
roslaunch slamesh slamesh_maicity.launch seq:=/01
```
You should get:

<img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/maicity_mesh.png" alt="maicity_mesh" width="60%" />


### 3.3 Run online or ros bag
```
roslaunch slamesh slamesh_online.launch
```
And play your bag, in launch file, remap the topic "/velodyne_points" to your LiDAR topic like "/os_cloud_node/points".
```
rosbag play your_bag.bag 
```
The number of LiDAR channels does not matter because our algorithm does not extract features.

You can use our sample data recorded with an Ouster OS1-32 LiDAR: [SLAMesh dataset](https://connectpolyu-my.sharepoint.com/:f:/g/personal/21041552r_connect_polyu_hk/EjhEKl8-1GBLseA_2F7TOvEB3w7OyAJ_kS7DAaWoLay9ng?e=GK1fsd).

## 4. Evaluation

SLAMesh saves all its report to the path `result_path`. If you find ros warning: ` Can not open Report file`, create the folder of `result_path` first, or just follow the steps below.

### 4.1 Kitti odometry accuracy

The file `0x_pred.txt` is the KITTI format path. I use [KITTI odometry evaluation tool
](https://github.com/LeoQLi/KITTI_odometry_evaluation_tool) for evaluation:

```
cd slamesh_ws
mkdir result && cd result
git clone https://github.com/LeoQLi/KITTI_odometry_evaluation_tool
cd KITTI_odometry_evaluation_tool/
python evaluation.py --result_dir=./data/ --eva_seqs=07.pred
```

Run SLAMesh by:
```
roslaunch slamesh slamesh_kitti_odometry.launch seq:=/07
```
Currently, the result on the KITTI odometry benchmark is:

| Sequence         | 00     | 01     | 02     | 03     | 04     | 05     | 06     | 07     | 08     | 09     | 10     | Average |
| ---------------- | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ------- |
| Translation (%)  | 0.771  | 1.2519 | 0.7742 | 0.6366 | 0.5044 | 0.5182 | 0.5294 | 0.3607 | 0.8745 | 0.573  | 0.6455 | 0.6763  |
| Rotation (deg/m) | 0.0035 | 0.003  | 0.003  | 0.0043 | 0.0013 | 0.003  | 0.0022 | 0.0023 | 0.0027 | 0.0025 | 0.0042 | 0.0029  |

Notice that to achieve better KITTI odometry performance, the parameter in `slamesh_kitti_meshing.launch` are set as followed:
```
full_cover: false # due to discontinuity phenomenon between cells, shirnk the test locations can improve accuracy.
num_margin_old_cell: 500 # margin old cells, because KITTI evaluate odometry accuracy rather than consistency.
```

However, if you want to have better meshing result, they should be: (in launch `slamesh_kitti_meshing.launch`)
```
full_cover: true # so that there is no gap between cells.
num_margin_old_cell: -1  # do not margin old cells, the cell-based map will have implicit loop closure effect.
```

### 4.2 Mesh quality

To save the mesh map, in `slamesh_node.cpp`, function `SLAMesher::process()`, line 867, set variable `save_mesh_map` to `true`.

I use the `TanksAndTemples/evaluation` tool to evaluate the mesh. I slightly modify it (remove trajectory). You can find it here: [TanksAndTemples/evaluation_rjy](https://github.com/RuanJY/TanksAndTemples)

Then compare the mesh with the ground-truth point cloud map:
```
cd TanksAndTemples_direct_rjy/python_toolbox/evaluation/
python run.py \
--dataset-dir ./data/ground_truth_point_cloud.ply \
--traj-path ./ \
--ply-path ./data/your_mesh.ply
```

### 4.3 Time cost

`***_report.txt` record time cost and other logs.

## 5. Help you to read the code

TODO

## Contact

Author: Jianyuan Ruan, Bo Li, Yibo Wang, Yuxiang Sun.

Email: jianyuan.ruan@connect.polyu.hk, 120052@zust.edu.cn, me-yibo.wang@connect.polyu.hk, yx.sun@polyu.edu.hk

## Acknowledgement

Most of the code are build from scratch , but we also want to acknowledge the following open-source projects:

[TanksAndTemples/evaluation](https://github.com/isl-org/TanksAndTemples/tree/master/python_toolbox/evaluation): evaluation

[A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM): kitti dataset loader and tic-toc

[F-LOAM](https://github.com/wh200720041/floam): help me to write the ceres residuals

[VGICP](https://github.com/SMRT-AIST/fast_gicp): multi-thread




