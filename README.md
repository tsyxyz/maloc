# maloc

Mapping and localization for autonomous driving.

## Prerequisites

- ROS (tested with noetic under Ubuntu 20.04)
- [irp_sen_msgs](https://github.com/irapkaist/irp_sen_msgs)

## Build

Clone the repository and the dependant `irp_sen_msgs`:

```shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/irapkaist/irp_sen_msgs.git
$ git clone https://github.com/tsyxyz/maloc.git
$ cd ..
$ catkin_make
```

## Usage


## Roadmap

- [ ] Build a point cloud map with multi sensors
- [ ] Generate vector map from point cloud map
- [ ] Map API for querying the vector map
- [ ] Multi sensor localization, based on vector map
