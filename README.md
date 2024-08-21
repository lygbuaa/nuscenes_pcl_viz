# NuScenes Lidar Visualized in PCL Visualizer

Quick and simple visualizer of NuScenes LIDAR_TOP sweeps in C++ with PCL. 

## Dependencies
Tested on Ubuntu 18.04.

Required system libs:
- JsonCpp  `apt install libjsoncpp-dev`
- YamlCpp  `apt install libyaml-cpp-dev`
- PCL      `installed with ROS`

## Build
CMake based:
```
mkdir build
cd build
cmake ..
make
```

## Run
Edit `config.yaml` to point to your [NuScenes dataset](https://www.nuscenes.org/). (WARNING: It's definitely slow for full NuScenes dataset but works good for `v1.0-mini` sizes).
```
./nuscenes_pcl_viz ../config.yaml
./build/nuscenes_pcl_viz config.yaml
```
Press `n` to switch between scenes. And default `h` key for help (PCLVisualizer).

![Viewer](./images/nuscenes_pcl_viz.gif)


## LIDAR_TOP extrinsics
```
nuscenes body-coord (imu-coord): X-forward, Y-left, Z-up
nuscenes lidar-coord: X-right, Y-forward, Z-up
```
{
"token": "d051cafdd9fe4d999b413462364d44a0",
"sensor_token": "dc8b396651c05aedbb9cdaae573bb567",
"translation": [
0.985793,
0.0,
1.84019
],
//left-hand, euler x: 0.1706383, y: 2.6496977, z: -90.0311332
"rotation": [  //w, x, y, z
0.706749235646644,
-0.015300993788500868,
0.01739745181256607,
-0.7070846669051719
],
"camera_intrinsic": []
},

## centerpoint class type
<!-- tasks = [
    dict(num_class=1, class_names=["car"]),
    dict(num_class=2, class_names=["truck", "construction_vehicle"]),
    dict(num_class=2, class_names=["bus", "trailer"]),
    dict(num_class=1, class_names=["barrier"]),
    dict(num_class=2, class_names=["motorcycle", "bicycle"]),
    dict(num_class=2, class_names=["pedestrian", "traffic_cone"]),
] -->
0: car,         
1: truck,       2: construction_vehicle
3: bus,         4: trailer,     
5: barrier
6: motorcycle   7: bicycle      
8: pedestrian   9: traffic_cone