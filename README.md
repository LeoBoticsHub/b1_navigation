# b1_navigation

The ros2 navigation stack adapted to b1 robot

## Mapping with slam_toolbox

For launching slam_toolbox 2D mapping on B1, be sure to have started ros2 controller and lidar and run the following.

```bash
ros2 launch b1_navigation mapping.launch.py
```

### Docker images

**TO CONCLUDE**

There are docker images on B1 computers to run the application. To launch dockers, use the [docker_factory](https://github.com/LeoBoticsHub/docker_factory) scripts as follows:

- on B1 main computer:
  
```bash
docker_factory_run -i b1/ros2_control
```

or, if you use [robot_setup](https://github.com/LeoBoticsHub/robots_setup) bashrcs, from your laptop,

```bash
launch_ros2_control
```

- on B1 Jetson NX2:

```bash
docker_factory_run -i b1/xavier/hesai_lidar
```

or, if you use [robot_setup](https://github.com/LeoBoticsHub/robots_setup) bashrcs, from your laptop,

```bash
launch_lidar
```

- on B1 Jetson NX3: (TO BE CHANGED ONCE FINISHED)
  
```bash
docker_factory_run -i b1/xavier/navigation
```

or, if you use [robot_setup](https://github.com/LeoBoticsHub/robots_setup) bashrcs, from your laptop,

```bash
launch_navigation
```
