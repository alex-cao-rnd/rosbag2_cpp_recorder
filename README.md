# rosbag2_cpp_recorder
A simple ros2 node to implement a programmatic Bag recording of all topics using C++.

## Why?
Since the current ROS2 releases (foxy, galactic), do not include a full C++ API to record all current topics, I've implemented a simple wrapper that uses the available python tools to access the same functionalities from C++.

## Usage
The node exposes a ROS service of the following type:

```
bool record          # 1: start, 0: stop
string save_folder  # folder where the bag will be saved
---
string res          # response from the recorder
```
