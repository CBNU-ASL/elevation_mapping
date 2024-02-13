# Robot-Centric Elevation Mapping

## Overview

This is a [ROS] package developed for elevation mapping with a mobile robot. The software is designed for (local) navigation tasks with robots which are equipped with a pose estimation (e.g. IMU & odometry) and a distance sensor (e.g. structured light (Kinect, RealSense), laser range sensor, stereo camera). The provided elevation map is limited around the robot and reflects the pose uncertainty that is aggregated through the motion of the robot (robot-centric mapping). This method is developed to explicitly handle drift of the robot pose estimation.

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Péter Fankhauser<br />
Co-Author: Maximilian Wulf<br />
Affiliation: [ANYbotics](https://www.anybotics.com/)<br />
Maintainer: Maximilian Wulf, mwulf@anybotics.com, Magnus Gärtner, mgaertner@anybotics.com<br />**

This projected was initially developed at ETH Zurich (Autonomous Systems Lab & Robotic Systems Lab).

[This work is conducted as part of ANYmal Research, a community to advance legged robotics.](https://www.anymal-research.org/)

<img alt="Elevation Map Example" src="elevation_mapping_demos/doc/elevation_map.jpg" width="700">


Videos of the elevation mapping software in use:

<a alt="StarlETH Kinect elevation mapping" href="https://www.youtube.com/watch?v=I9eP8GrMyNQ"><img src="elevation_mapping_demos/doc/starleth_kinect.jpg" align="left" width="180" ></a>
<a alt="ANYmal outdoor terrain mapping" href="https://www.youtube.com/watch?v=iVMsQPTM65M"><img src="elevation_mapping_demos/doc/anymal_forrest.jpg" align="left" width="180" ></a>
<a alt="ANYmal rough-terrain locomotion planner" href="https://www.youtube.com/watch?v=CpzQu25iLa0"><img src="elevation_mapping_demos/doc/anymal_locomotion_planner.jpg" align="left" width="180" ></a>
<a alt="ANYmal outdoor stair climbing" href="https://www.youtube.com/watch?v=vSveQrJLRTo"><img src="elevation_mapping_demos/doc/anymal_outdoor_stairs.jpg" width="180" ></a>

## For ASL

### Note
1. Dependencies의 목록을 확인하고 실행 전에 설치할 것.
2. example_asl.launch 파일은 실행 가능한 파일이 아닌 예시 파일로 하기 내용을 추가적으로 확인하고 수정하고 사용하길 권장함.
3. 사용가능한 토픽은 `/elevation_mapping/elevation_map_raw`로 `grid_map_msgs/GridMap`타입.
4. 위 토픽은 다양한 레이어를 가지고 있으니 사용하기에 용이한 레이어를 선택하여 이용하기 바람.
5. 마지막으로 자세한 내용은 꼭 원 패키지의 reademe를 참고할 것.
   
### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionally, the Robot-Centric Elevation Mapping depends on following software:

- [Grid Map](https://github.com/anybotics/grid_map) (grid map library for mobile robots)
- [kindr](http://github.com/anybotics/kindr) (kinematics and dynamics library for robotics),
- [kindr_ros](https://github.com/anybotics/kindr_ros) (ROS wrapper for kindr),
- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing),
- [Eigen](http://eigen.tuxfamily.org) (linear algebra library).
- [message_logger](https://github.com/ANYbotics/message_logger)

  
### example

**예제파일**

example_asl.launch

elevation_mapping_demos/config/robots/example_asl.yaml

elevation_mapping_demos/config/postprocessing/postprocessor_pipeline.yaml

#### example_asl.launch

* **`Setting simulation`**
  
  해당 주석이 붙어 있는 부분의 경우, 로컬맵을 위한 구성이 아니며 시뮬레이션 실행을 위한 세팅임.

* **`pcl_manager`**
  
  point cloud를 전처리 기능을 가진 library로 예제에서는 pass_through_filter와 voxelgrid를 수행.
  '~input'에 처리할 point cloud의 토픽을 지정.
  '~output'에 전처리를 완료하여 발행할 토픽을 지정.(지정하지 않을 경우, 실행시킨 전처리기능에 따라 다르게 출력(ex./pass_through_filter/output)).
  pass_through_filter : point cloud를 사용자가 원하는 부분만 잘라냄.
  VoxelGird           : point cloud를 다운샘플링함.

* **'elevation_mapping'**
  
  robot과 postprocessing에 관련된 yaml 파일을 불러와 패키지를 실행시킴.

#### elevation_mapping_demos/config/robots/example_asl.yaml

  **Note**

  수정이 필요한 부분을 중점적으로 부가 설명. 하기되지 않은 내용은 원 패키지의 내용을 살펴볼 것.

* **`map_frame_id`**
  
  차량 frame_id로 설정.
  
  원 패키지의 경우, 매핑을 진행하기 위하여 `map_frame_id`를 따로 불러와 사용하였지만 로컬맵 사용시 차량 프레임으로 설정.

* **`robot_base_frame_id`**
  
  차량 frame_id로 설정.
  `map_frame_id` 내용과 동일.

* **`robot_pose_topic`**
  
  수신할 차량 위치데이터의 토픽을 지정.
  
  만약,`nav_msgs/Odometry`이외에 다른 메세지를 사용할 경우 `elevation_mapping/src/ElevationMaping.cpp`와 `elevation_mapping/include/elevation_mapping/ElvationMapping.hpp`에서 `Change your message`를 검색하여 수정할 것.

* **`input_sources`**
  
  **`topic`**  : 로컬맵에 사용할 pointcloud 토픽명을 지정.

* **`sensor_processor/ignore_points_above`**
  
  임계값을 초과하는 높이를 갖은 pointcloud는 무시함.(default : inf)

* **`sensor_processor/ignore_points_below`**
  
  임계값미만 높이를 갖은 pointcloud는 무시함.(default : -inf)

* **`track_point_frame_id`**
  
  pointcloud가 따라가야 하는 프레임을 지정. 디폴트의 경우, 차량의 프레임 사용.

* **`length_in_x`**
  
  map의 x방향 길이.
  
  30으로 지정할 경우, 전방 15m를 바라볼 수 있음.(default)

* **`length_in_y`**
  
  map의 y방향 길이.
  
  10으로 지정할 경우, 좌우 각각 5m를 바라볼 수 있음.(default)

* **`resolution`**
  
  resolution을 너무 작게 설정할 경우(0.1), 전방으로 멀어질수록 로컬맵의 값이 존재하지 않는 경우가 많음.
  
  그 때문에 0.3에서 0.5로 사용할 것을 권장.(0.5, default)

* **`enable_visibility_cleanup`**(default, true)
  
  센서에 표시되지 않는 요소를 지도에서 제거하는 별도의 스레드를 활성화/비활성화.
  
* **`enable_continuous_cleanup`**
  
  매핑되는 지도를 지속적으로 정리하여 새로운 센서 데이터가 수신될 때마다 모든 값을 지우고 새로운 데이터로 매핑.
  
  해당 내용이 활성화 될 경우, `enable_visibility_cleanup`은 자동으로 비활성화됨.
  
  로컬맵으로 사용할 경우, 해당 내용을 활성화.

* **`initialize_elevation_map`**
  
  매핑을 시작하기전 초기지도 세팅. false로 사용.

* **`target_frame_init_submap`**
  
  차량의 프레임과 같게 세팅.

#### elevation_mapping_demos/config/postprocessing/postprocessor_pipeline.yaml

  **Note**

  해당 파일은 수정하여 사용할 필요가 없음. 하지만 터미널에 지속적인 경고가 뜨는 것이 불편하다면 수정할 것.


  `resolution`값을 0.5보다 크게 설정하여 사용할 경우, 지속적인 경고가 발생.
  
  해당 내용은 전체맵을 매핑하는데에 있어서 발생하는 문제로 수정할 필요가 없지만 경고가 불편하다면 두 개의 `radius`값을 `resolution`값보다 크게 설정하여 사용.


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[grid_map_msgs/GridMap]: https://github.com/anybotics/grid_map/blob/master/grid_map_msgs/msg/GridMap.msg
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[geometry_msgs/PoseWithCovarianceStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[tf/tfMessage]: http://docs.ros.org/kinetic/api/tf/html/msg/tfMessage.html
[std_srvs/Empty]: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
[grid_map_msgs/GetGridMap]: https://github.com/anybotics/grid_map/blob/master/grid_map_msgs/srv/GetGridMap.srv
[grid_map_msgs/ProcessFile]: https://github.com/ANYbotics/grid_map/blob/master/grid_map_msgs/srv/ProcessFile.srv
