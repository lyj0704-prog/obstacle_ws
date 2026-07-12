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

| Topic | Message type | Provider | Purpose |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR driver 또는 simulator | Detection 입력과 tracking visibility 검사 |
| `/global_waypoints` | `f110_msgs/msg/WpntArray` | ForzaETH global planner/waypoint publisher | Frenet 변환, track boundary, lap 길이 |
| `/car_state/odom_frenet` | `nav_msgs/msg/Odometry` | Frenet state estimator | Ego 차량의 `s`, `d` 상태 |
| `/car_state/odom` | `nav_msgs/msg/Odometry` | State estimator | Tracking visibility 검사용 전역 pose |
| `/perception/detection/raw_obstacles` | `f110_msgs/msg/ObstacleArray` | `detect` | Tracking 측정값 |
| `/tf`, `/tf_static` | TF2 | Localization/sensor TF publisher | Scan frame을 `map`으로 변환 |

`tracking`은 위 입력을 모두 사용합니다. `detect`는 `/scan`,
`/global_waypoints`, `/car_state/odom_frenet`과 TF를 사용합니다.

## 출력 토픽

| Topic | Message type | Consumer | Purpose |
|---|---|---|---|
| `/perception/detection/raw_obstacles` | `f110_msgs/msg/ObstacleArray` | `tracking` | Detection 측정 장애물 |
| `/perception/obstacles_markers_new` | `visualization_msgs/msg/MarkerArray` | RViz | Rotated detection CUBE |
| `/perception/breakpoints_markers` | `visualization_msgs/msg/MarkerArray` | RViz | Breakpoint/cluster 디버그 표시 |
| `/perception/detect_bound` | `visualization_msgs/msg/Marker` | RViz | Detection track boundary |
| `/perception/detection/latency` | `std_msgs/msg/Float32` | Monitor | `measure=true`일 때 detection latency |
| `/perception/obstacles` | `f110_msgs/msg/ObstacleArray` | Planner/controller/state machine | 최종 static + dynamic track |
| `/perception/raw_obstacles` | `f110_msgs/msg/ObstacleArray` | Planner/debug tools | 최종 non-static track |
| `/perception/static_dynamic_marker_pub` | `visualization_msgs/msg/MarkerArray` | RViz | Track 분류 시각화 |
| `/perception/tracking/latency` | `std_msgs/msg/Float32` | Monitor | `measure=true`일 때 tracking latency |

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

## 실행 전 입력 확인

```bash
ros2 topic hz /scan
ros2 topic hz /global_waypoints
ros2 topic hz /car_state/odom
ros2 topic hz /car_state/odom_frenet
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

- Detection: rotated `CUBE`
- Static track: 초록색 sphere
- Dynamic track: 빨간색 sphere
- Unclassified track: 마젠타 sphere

RViz의 Fixed Frame은 기본적으로 `map`으로 설정합니다.

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
