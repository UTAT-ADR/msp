# Betaflight-ROS Communication Package

## Installation 
- Install ASIO and Ninja:
  ```
  sudo apt install -y --no-install-recommends ninja-build libasio-dev
  ```
- Clone Agilicious and its dependencies
- Clone MSP and build using Catkin
  ```
  cd src
  git clone https://github.com/UTAT-ADR/msp
  cd ..
  catkin build
  ```
  
## Usage
```
source devel/setup.bash
```
### Betaflight Node
This node requests IMU reading from the flight-controller at 1000 Hz and battery voltage at 10 Hz. It also converts collective-thrust and body rate command into appropriate format and sends it to the FC.

Example launch file:
```
roslaunch msp betaflight.launch
```

Launch file parameters:
- ```device```: Port name the FC is connected to
- ```baudrate```: 115200 - 1000000, leaving it empty defaults to 1000000
- ```command_topic```: Name of the command topic
- ```thrust_map_file```: File path and name to the .csv thrust map
- ```max_rate```: Max angular rate setting in Betaflight in [deg/s]

### Motor Test
Publishes control command at 100 Hz with a sine wave in the thrust channel. 

Topic name: ```/motor_command```

```
rosrun msp motor_test
```

## Command Specification
Message type: ```agiros_msgs/Command```

```
Header header 

# Time [s]
float64 t

# Collective thrust [N]
float64 collective_thrust

# Angular rates in body frame [rad/s]
geometry_msgs/Vector3 bodyrates
  float64 bodyrates.x # roll
  float64 bodyrates.y # pitch
  float64 bodyrates.x # yaw

```

### Compilation Guide
- In ```package.xml``` add
  ```
  <depend>geometry_msgs</depend>
  <depend>agiros_msgs</depend>
  ```
- When linking libraries in ```CMakeLists.txt``` add ```-lstdc++fs``` flag. E.g.
  ```
  target_link_libraries(betaflight_node
    ${catkin_LIBRARIES} -lstdc++fs
  )
  ```
- In the executable
  ```
  #include <geometry_msgs/Vector3.h>
  #include <agiros_msgs/Command.h>
  ```
