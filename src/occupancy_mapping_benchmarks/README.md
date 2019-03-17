## Installation

```
mkdir -p ~/mapping_benchmark_ws/src
cd ~/mapping_benchmark_ws
catkin init
catkin config --extend /opt/ros/melodic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

cd src
git clone https://github.com/nicolov/mapping-benchmarks
wstool init . ./occupancy_mapping_benchmarks/ws_https.rosinstall
wstool update

catkin build
```
