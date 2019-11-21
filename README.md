# MacMic_Robot

Macro-Micro Robot for the decoration project.
Author: likang
Time:20191030


If no serial 

```
sudo apt-get install ros-kinetic-serial
```

change Serial Port by #define XXX_PORT in {file}.cpp: 

1. stm32.cpp
2. laser_talker.cpp
3. motor_driver.cpp

ps. laser_talker PORT only set first port and second third port must be prioritized

[Click Here to watch the experiment video](http://219.223.251.59:5050/jiang/experiment-video/tree/master/MacMicRobot%20Project/Micro%20Parallel%20Manipulator)