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