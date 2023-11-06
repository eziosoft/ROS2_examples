
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

## Build package  
```
colcon build --packages-select [my_py_pkg]
```

# Node  

## Create Node  
```
create new file based on test_node.py template
add new line to setup.py:
    entry_points={
        'console_scripts': [
         "test_node = test_pkg.test_node:main"
        ],
    },
```

## Run Node  
```
ros2 run [my_pkg] [my_node]
```