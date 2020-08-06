# Laser_sensor3

- This package uses ros-serial package for serial communication.

  U can install it in ROS by 

  `
  sudo apt-get install ros-kinetic-serial
  `

 - And change your Serial Port used by Laser Sensor in 

    ***launch/port.yaml***
    
    Don't forget give your port permission by 
    
    `sudo chmod 777 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2`
    
    for example.
 
 - Then just launch "laser.launch", you can see Topic "/laser_topic" if everything is okay.
 
   And use the laser data by echo this topic.
   
## Msg formation

 - float32 sensor1
 - float32 sensor2
 - float32 sensor3
 - uint16 state
 
 sensor1 -- sensor3 is sensor data, range of 80 -- 300 (mm)
 
 The state is indicating the laser sensor communicate. This state is 0 only when all of sensors are in the right state.
 You can use it to setting running flag.
 Because if three sensors not ready together, maybe cause algorithm getting a wrong result.
 
## Communication rate is 250Hz
