## ppr statck

This is ppr stack.

### Todo list

- [ ]  SDF mesh

- [ ]  limit for controller

### How to use simulation

1. 下载代码

    `git clone `

    `cd {YOU-WORKSPACE}`

2. 安装依赖

    `rosdep install --from-paths src --ignore-src -r -y`

3. 编译

    `catkin_make`

4. 运行gazebo仿真

    `roslaunch ppr_gazebo ppr_world.launch`


### Topic

1. state topic 

    `rostopic echo parabot/joint_states`

2. cmd topic

    `rostopic pub /parabot/joint1_position_controller/command`

    `rostopic pub /parabot/joint2_position_controller/command`

    `rostopic pub /parabot/joint3_position_controller/command`

3. Change PID 

    `roslaunch ppr_control ppr_rqt.launch`
    
## MacMic_Robot

### Macro-Micro Robot for the decoration project.
*Author: likang*

*Created Time:20191030*

*Editor: Dayuan Chen*

*Version: 1.1*

------

### Configuration

1. Git clone Laser Driver by Reposiory

    `git clone gitlab@www.git-hitsz.cn:jiang/laser_sensor3.git`

2. change Serial Port by `port.yaml` in Laser_sensor package lanunch directory 

3. launch the luanch file

    `roslaunch pararob para_excution.launch`


[Click Here to watch the experiment video](http://www.git-hitsz.cn/jiang/experiment-video/tree/master/MacMicRobot%20Project)

------

### Some Issue
If need password, please follow this wiki to set your ssh keys.

[Ubuntu Git Config](http://www.git-hitsz.cn/jiang/wiki/wikis/GitConf_Ubuntu)

If no serial 

`sudo apt-get install ros-kinetic-serial`

or

 `sudo apt-get install ros-melodic-serial`

