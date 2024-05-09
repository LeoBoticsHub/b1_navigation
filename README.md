# b1_navigation

The ros2 navigation stack adapted to b1 robot. For using it be sure to have started ros2 controller and lidar on the robot. More information on setup and the use of this package can be found on [Navigation_hub](https://github.com/LeoBoticsHub/navigation_hub/blob/main/docs/B1/main.md) repo.

## Mapping with slam_toolbox

For launching slam_toolbox 2D mapping on B1, run the following.

```bash
ros2 launch b1_navigation mapping.launch.py
```

## Navigation with Nav2

The navigation can work either with its own SLAM or with a given map using a localization module. In the following the two alternatives documentation. Moreover, for using personal SLAM or localization, the navigation can be loaded as standalone.

### Standalone Nav2 Navigation

To launch the navigation stack without internal mapping or localization, be sure that there is an external node which is publishing at least a map and the tf map -> odom.

For launching the navigation type the following:

```bash
ros2 launch b1_navigation navigation.launch.py
```

### Nav2 with Mapping

To launch the navigation stack with slam_toolbox mapping, type the following:

```bash
ros2 launch b1_navigation navigation.launch.py mapping:=true
```

### Nav2 with AMCL Localization

To launch the navigation stack with AMCL localization, type the following:

```bash
ros2 launch b1_navigation navigation.launch.py localization:=true
```
