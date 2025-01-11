# sky_vision2

## Package struture:

```
sky_vision2/
# --> package info, configuration, and compilation
├── CMakeLists.txt
├── package.xml
# --> Python stuff
├── sky_vision2
│   ├── __init__.py
│   └── module_to_import.py
├── scripts
│   └── py_node.py
# --> Cpp stuff
├── include
│   └── sky_vision2
│       └── cpp_header.hpp
└── src
    └── cpp_node.cpp
```

## ROS1-Bridge

A easy way for running ros1 bridge is through a docker container:

just run the command:
```
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
```

## Nodes

Here you can see what nodes are already implement and how they work. The "~" symbol represents /sky_vision/

### aruco_detector
Detect aruco marker and publish there id and pose

#### Parameters
- camera_name (string, default: "down_cam")
- debug (bool, default: true)
- marker_size (int, default: 25)
- dictionary_index (int, default: 6)
    - dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8,
    DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16, 
    DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20

#### Published Topics
- ~<camera_name>/aruco_id (std_msgs/Int32MultiArray)
    - Array of id's of the founded markers.
- ~<camera_name>/aruco_pose (geometry_msgs/PoseArray)
    - Array of position of the founded markers. Position is given in respect of BODY_NED.
- ~<camera_name>/debug_img (sensor_msgs/Image)
    - Image reserve for debuging code, just work if the debug parameter is true. 

#### Subscribed Topics
- ~<camera_name>/img_raw (sensor_msgs/Image)
    - Image from camera