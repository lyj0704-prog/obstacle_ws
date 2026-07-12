# KF-Enhanced Obstacle Perception for F1TENTH

ROS 2 Humble용 LiDAR obstacle perception 패키지입니다. ForzaETH RACE_STACK의
`perception` 패키지를 독립적으로 빌드하고 시험할 수 있도록 분리했으며,
LaserScan 기반 detection과 Frenet Kalman Filter 기반 enhanced tracking을
제공합니다. 나중에 이 저장소의 `perception/` 디렉터리를 RACE_STACK의 같은
경로와 비교·병합할 수 있도록 원래 ROS 2 Python package 구조와 executable
계약을 유지합니다.

## 구조

```text
obstacle_ws/
├── perception/
│   ├── config/perception_params.yaml
│   ├── launch/
│   │   ├── perception_launch.xml
│   │   └── simulation/
│   │       ├── detect_bag_launch.xml
│   │       └── fake_perception_test_launch.py
│   ├── perception/
│   │   ├── __init__.py
│   │   ├── detect1.py
│   │   ├── tracking1.py
│   │   ├── frenet_utils.py
│   │   ├── simulation/
│   │   │   ├── __init__.py
│   │   │   ├── fake_perception_test.py
│   │   │   ├── perception_result_checker.py
│   │   │   └── static_path_detour.py
│   │   └── tools/
│   │       ├── __init__.py
│   │       └── obs_monitor.py
│   ├── resource/perception
│   ├── test/
│   ├── LICENSE
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── README.md
└── .gitignore
```

ROS 2 package 이름은 `perception`이며, 기본 executable은 `detect`와
`tracking`입니다. 소스 파일명은 기존 RACE_STACK 계약대로 `detect1.py`,
`tracking1.py`를 유지합니다.

`perception/perception/`의 최상위 세 파일은 실제 runtime 알고리즘입니다.
`simulation/`은 fake input과 결과 checker 등 통합시험 전용이고, `tools/`는
실행 중 상태를 관찰하는 진단 도구입니다. Simulation helper는 기본
`perception_launch.xml`에 자동 포함되지 않습니다.

## 핵심 알고리즘

### Detection

- LaserScan callback 기반 event-driven 처리 및 Hokuyo UST-10LX 40 Hz 대응
- scan timestamp와 beam timing을 이용한 LiDAR de-skew
- 실제 sensor range 기반 Adaptive Breakpoint Detection
- 거리에 따라 달라지는 minimum point filtering
- 인접 cluster 병합
- Frenet track boundary filtering
- PCA 기반 rotated bounding box fitting

### Tracking

- Frenet 상태 `[s, vs, d, vd]` 기반 Kalman Filter
- detection callback 기반 event-driven update
- global nearest-neighbor data association
- 폐곡선 `s` 좌표 lap wrap-around
- 초 단위 TTL과 coasting prediction
- 시간 기반 static/dynamic classification
- LiDAR 시야와 occlusion을 고려한 visibility-aware TTL
- 중복 track 병합과 `min_hits` 기반 ghost track 억제
- publish latency compensation

## 입력 토픽

### Detection (`detect`)

| Topic/TF | Message type | Provider | 의미 |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR driver 또는 simulator | De-skew와 Adaptive Breakpoint Detection에 사용하는 원본 scan |
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | Global planner/waypoint publisher | Cartesian↔Frenet 변환과 좌·우 track boundary 계산 |
| `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | Frenet converter | Ego 차량의 현재 `s` 위치와 전방 검출 범위 계산 |
| `map <- LaserScan.header.frame_id` | TF2 | Localization/sensor TF publisher | 각 scan point를 `map` 좌표로 변환하고 de-skew 수행 |

### Tracking (`tracking`)

| Topic | Message type | Provider | 의미 |
|---|---|---|---|
| `/perception/detection/raw_obstacles` | `f110_msgs/msg/ObstacleArray` | `detect` | Kalman Filter에 들어가는 frame별 obstacle 측정값 |
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | Global planner/waypoint publisher | Frenet 변환과 track length/lap wrap-around 계산 |
| `/car_state/frenet/odom` | `nav_msgs/msg/Odometry` | Frenet converter | Ego `s` 기준 전방 거리와 lap 진행 계산 |
| `/car_state/odom` | `nav_msgs/msg/Odometry` | State estimator/localization | Ego 전역 위치·yaw를 이용한 LiDAR visibility 판정 |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR driver 또는 simulator | 미검출 track이 실제로 보여야 하는 위치인지, 가려졌는지 판단 |

## 출력 토픽

### Detection (`detect`)

| Topic | Message type | Consumer | 의미 |
|---|---|---|---|
| `/perception/detection/raw_obstacles` | `f110_msgs/msg/ObstacleArray` | `tracking` | Cluster에서 계산한 Frenet 중심·경계·크기를 담은 추적 전 측정값 |
| `/perception/obstacles_markers_new` | `visualization_msgs/msg/MarkerArray` | RViz | 검출 장애물의 rotated bounding box |
| `/perception/breakpoints_markers` | `visualization_msgs/msg/MarkerArray` | RViz | ABD가 분리한 각 cluster의 시작점·끝점 |
| `/perception/detect_bound` | `visualization_msgs/msg/Marker` | RViz | Global waypoint로 계산한 좌·우 track boundary |
| `/perception/detection/latency` | `std_msgs/msg/Float32` | Monitor | `measure=true`일 때 한 scan의 detection 처리 시간(초) |

### Tracking (`tracking`)

| Topic | Message type | Consumer | 의미 |
|---|---|---|---|
| `/perception/obstacles` | `f110_msgs/msg/ObstacleArray` | Planner/controller/state machine | `min_hits`를 통과한 최종 static + dynamic/unclassified track |
| `/perception/raw_obstacles` | `f110_msgs/msg/ObstacleArray` | Planner/debug tools | 최종 결과 중 static이 아닌 dynamic/unclassified track |
| `/perception/static_dynamic_marker_pub` | `visualization_msgs/msg/MarkerArray` | RViz | KF 위치, track ID, static/dynamic 분류 결과 |
| `/perception/tracking/latency` | `std_msgs/msg/Float32` | Monitor | `measure=true`일 때 한 obstacle callback의 tracking 처리 시간(초) |

## TF 요구사항

Detection에는 다음 변환이 필요합니다.

```text
map <- LaserScan.header.frame_id
```

De-skew가 활성화된 경우 scan의 첫 beam과 마지막 beam 시점에 해당하는 TF가
필요하므로 TF buffer에 scan timestamp 구간의 history가 남아 있어야 합니다.
TF가 없거나 너무 늦게 도착하면 해당 scan은 처리되지 않을 수 있습니다.

## Message package 호환성

이 저장소는 ForzaETH ROS 2 Humble RACE_STACK의 표준인 `f110_msgs`를
사용합니다.

- `Wpnt`와 `WpntArray`는 AE-HYU 메시지와 필드 구성이 같지만 ROS 타입명은
  다르므로 직접 연결할 수 없습니다.
- `Obstacle`은 순정 `f110_msgs`를 기준으로 합니다.
- AE-HYU에만 있는 `position`, `theta`, `curr_lap`, covariance 필드는 ROS
  메시지로 발행하지 않습니다. Cartesian 위치, orientation, lap 및 covariance는
  detection/tracking 내부 상태와 RViz marker 계산에는 계속 사용됩니다.
- AE-HYU 노드와 연결해야 한다면 같은 토픽에 서로 다른 타입을 섞지 말고,
  별도 토픽을 사용하는 adapter node를 두어야 합니다.

`f110_msgs` 자체는 이 저장소에 복제하지 않습니다. ForzaETH RACE_STACK을
먼저 빌드해 overlay로 제공하거나, 동일한 순정 `f110_msgs` 패키지를 별도
workspace에 설치해야 합니다.

## 빌드

`f110_msgs`가 이미 설치된 환경에서는:

```bash
source /opt/ros/humble/setup.bash
# 필요한 경우 먼저 ForzaETH RACE_STACK overlay를 source합니다.
# source ~/ws/install/setup.bash

cd ~/ws/obstacle_ws
colcon build --symlink-install
source install/setup.bash
```

`colcon`이 `f110_msgs`를 찾지 못하면 RACE_STACK에서 최소한
`f110_msgs`를 먼저 빌드해야 합니다.

```bash
cd ~/ws/src/race_stack
source /opt/ros/humble/setup.bash
colcon build --packages-select f110_msgs
source install/setup.bash
```

## 실행

개별 실행:

```bash
ros2 run perception detect --ros-args \
  --params-file ~/ws/obstacle_ws/perception/config/perception_params.yaml

ros2 run perception tracking --ros-args \
  --params-file ~/ws/obstacle_ws/perception/config/perception_params.yaml
```

Detection과 tracking 동시 실행:

```bash
ros2 launch perception perception_launch.xml
```

Rosbag 재생:

```bash
ros2 launch perception simulation/detect_bag_launch.xml \
  bag_file:=/absolute/path/to/bag
```

Synthetic fake publisher는 실제 launch에 자동 포함되지 않습니다. 별도 시험할
때만 실행합니다.

```bash
ros2 launch perception simulation/fake_perception_test_launch.py
```

## Synthetic simulation 빠른 실행

이 테스트는 별도 맵이나 차량 simulator 없이 다음 입력을 40 Hz로 생성합니다.

- `/scan`: 차량 전방 2 m에 폭 0.30 m인 정적 장애물
- `/global_waypoints`: 반지름 5 m의 폐곡선 waypoint
- `/car_state/odom`, `/car_state/frenet/odom`
- TF `map <- laser`

터미널 1에서 빌드하고 detection, tracking, fake input을 함께 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash  # ForzaETH f110_msgs overlay

cd ~/ws/obstacle_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch perception simulation/fake_perception_test_launch.py
```

정상이면 실행 로그에 다음 결과가 출력됩니다.

```text
PASS: detection1 raw output=..., tracking1 output=...
```

터미널 2에서 RViz를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
source ~/ws/obstacle_ws/install/setup.bash
rviz2
```

RViz의 `Fixed Frame`을 `map`으로 설정하고 다음 display를 추가합니다.

| Display type | Topic | 표시 내용 |
|---|---|---|
| `MarkerArray` | `/perception/obstacles_markers_new` | Detection CUBE |
| `MarkerArray` | `/perception/breakpoints_markers` | Breakpoint/cluster |
| `Marker` | `/perception/detect_bound` | Track boundary |
| `MarkerArray` | `/perception/static_dynamic_marker_pub` | Tracking 분류 |

터미널 3에서 tracking ID와 분류 안정성을 확인할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
source ~/ws/obstacle_ws/install/setup.bash
ros2 run perception obs_monitor
```

토픽만 확인하려면 다음을 실행합니다.

```bash
ros2 topic hz /perception/detection/raw_obstacles
ros2 topic echo /perception/detection/raw_obstacles --once
ros2 topic hz /perception/obstacles
ros2 topic echo /perception/obstacles --once
```

이 fake simulation은 알고리즘 연결과 RViz marker를 검증하는 synthetic test입니다.
실제 맵 위 차량 주행은 F1TENTH simulator, map, localization, global waypoint
publisher를 별도로 실행한 뒤 기본 `perception_launch.xml`을 사용해야 합니다.

## 실행 전 입력 확인

```bash
ros2 topic hz /scan
ros2 topic hz /global_waypoints
ros2 topic hz /car_state/odom
ros2 topic hz /car_state/frenet/odom
ros2 topic type /global_waypoints
```

`/global_waypoints` 타입은 다음이어야 합니다.

```text
f110_msgs/msg/WpntArray
```

## 결과 확인

```bash
ros2 topic hz /perception/detection/raw_obstacles
ros2 topic echo /perception/detection/raw_obstacles --once
ros2 topic hz /perception/obstacles
ros2 topic echo /perception/obstacles --once
```

## RViz marker

RViz의 Fixed Frame은 `map`으로 설정합니다.

| Topic | Display | 모양/색 | 의미 |
|---|---|---|---|
| `/perception/detect_bound` | `Marker` | 빨간 `SPHERE_LIST`, 지름 0.04 m | Global waypoint에서 계산한 주행 가능 영역의 좌·우 경계 |
| `/perception/breakpoints_markers` | `MarkerArray` | 초록~청록 `SPHERE`, 지름 0.25 m | 각 LiDAR cluster의 첫 point와 마지막 point. 색 차이는 cluster 구분용 |
| `/perception/obstacles_markers_new` | `MarkerArray` | 청록 `CUBE`, alpha 0.5 | Rotated box fitting 결과. 중심은 장애물 위치, 회전은 주축 방향, 크기는 검출 크기(최소 0.35 m) |
| `/perception/static_dynamic_marker_pub` | `MarkerArray` | `SPHERE`, alpha 0.6 | KF로 보정된 최종 track 위치. Marker ID는 track ID |

Tracking sphere 색상:

- 초록색: static으로 확정된 track
- 빨간색: dynamic으로 확정된 track
- 핑크색: 관측 시간이 부족해 아직 분류되지 않은 track
- 크기 0.8 m: Ego 기준 전방 `tracking.dist_infront` 안의 track
- 크기 0.55 m: 그 외 유지 중인 track

`ObstacleArray` 토픽은 RViz 기본 display가 아니므로 수치 결과는 `ros2 topic
echo`로 보고, 공간 결과는 위 Marker/MarkerArray 토픽으로 확인합니다.

## 모니터

Tracking ID와 static/dynamic 분류 안정성을 1초 간격으로 확인하려면:

```bash
ros2 run perception obs_monitor
```

## 전체 RACE_STACK에 다시 합치기

먼저 dry-run으로 차이를 확인합니다.

```bash
diff -ruN ~/ws/src/race_stack/perception ~/ws/obstacle_ws/perception

rsync -avn --delete \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  ~/ws/obstacle_ws/perception/ \
  ~/ws/src/race_stack/perception/
```

실제 병합 전에는 반드시 다음 파일의 충돌을 별도로 검토합니다.

- `package.xml`: `f110_msgs` 및 runtime dependency
- `setup.py`: `detect`, `tracking` console scripts와 config 설치
- `launch/`: executable/node 이름과 파라미터 파일 경로
- `config/perception_params.yaml`: 차량 및 LiDAR 환경별 tuning 값

`rsync --delete`를 실제 적용 명령에 바로 사용하지 말고, Git branch에서
파일별 diff를 검토한 뒤 필요한 변경만 병합하는 것을 권장합니다.
