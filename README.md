# Occupancy mapping benchmarks

<img src="https://nicolovaligi.com/occupancy_map_structure.png" style="max-width: 80%"
class="img-center" />

Code to benchmark three open source Occupancy Mapping libraries:
[OctoMap](https://octomap.github.io/),
[SkiMap](https://github.com/m4nh/skimap_ros), and
[OpenVDB](https://www.openvdb.org/) for an RGBD mapping task based on the TUM
RGBD dataset.

For discussion and instructions, read the [article on my blog](occupancy-mapping-benchmarks.html).

```
catkin init
catkin config --extend /opt/ros/melodic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build

rosrun occupancy_mapping_benchmarks benchmark \
src/occupancy_mapping_benchmarks/rgbd_dataset_freiburg3_long_office_household-2hz-with-pointclouds.bag  > bench_output.txt

rosrun occupancy_mapping_benchmarks make_plots.py
```

## Maintenance

To upgrade the Docker image:

```
( export TAG=nicolov/occupancy_mapping_benchmarks:0.0.1 && docker build -t $TAG . &&  docker push $TAG )
```
