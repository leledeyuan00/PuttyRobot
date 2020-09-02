## ppr statck

This is ppr stack.

### Todo list

 * [] SDF mesh

 * [] limit for controller

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