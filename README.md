
# Workspace  
## Create ROS workspace 
```
mkdir ros2_ws
cd ros2_ws
mkdir src

cd ..
colcon build

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

# Package

## Create ROS packgage 
```
cd ros2_ws/src

ros2 pkg create [my_py_pkg] --build-type ament_python --dependencies rclpy
```



# Node  

## Create Node  
```
create new file based on test_node.py template

chmod +x [node_name]


add new line to setup.py:
    entry_points={
        'console_scripts': [
         "test_node = test_pkg.test_node:main"
        ],
    },
```

## CLI  
### Nodes
#### Run Node  
```
ros2 run [my_pkg] [my_node]
```

#### List running nodes  
```
ros2 node list
```
#### Get info about node
```
ros2 node info [node_name]
```
#### Rename node in runtime
```
ros2 run [my_pkg] [my_node] --ros-args -remap __node:=[new_name]
```

### Interfaces
#### Get info about interface
```
ros2 interface show [name_of_interface]
```

### Topics
#### List active topics
```
ros2 topic list
```
#### See messages
```
ros2 topic echo [/topic_name]
```

#### Publish message
```
ros2 ropic pub -r 10 /[topic] [message_type] "{data:dasdasd}"
```
#### Remap topic
```
ros2 run [package] [node] --ros-args -r [old_topic]:=[new_topic]
```

### Building (Colcon)
#### Build Workspace 
```
colcon build
```
#### Build package  
```
colcon build --packages-select [my_py_pkg]
colcon build --packages-select [my_py_pkg] --symlink-install (after that you don't have to build the package before running everytime you make changes)
```


### Gui tools
```
rqt - get info about nodes
rqt_graph - get graph of running nodes
```

# Launch file
## Run
```
ros2 launch [pkg] [my_launch_file.launch.py]
```

# Parameters
```
ros2 param list
ros2 param get [/node] [parameter]
ros2 run [pkg] [node_name] --ros-args -p [param_name]:=1000

```
