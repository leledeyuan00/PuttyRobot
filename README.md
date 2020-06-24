# MacMic_Robot

### Macro-Micro Robot for the decoration project.
*Author: likang*

*Created Time:20191030*

*Editor: Dayuan Chen*

*Version: 1.1*

------

Git clone Laser Driver by Reposiory

`git clone gitlab@www.git-hitsz.cn:jiang/laser_sensor3.git`

### If need password, please follow this wiki to set your ssh keys.

- [Ubuntu Git Config](http://www.git-hitsz.cn/jiang/wiki/wikis/GitConf_Ubuntu)

###  If no serial 

`
sudo apt-get install ros-kinetic-serial
`

or

`
sudo apt-get install ros-melodic-serial
`

change Serial Port by #define XXX_PORT in `parameter.h`: 

ps. laser_talker PORT only set first port and second third port must be prioritized

[Click Here to watch the experiment video](http://www.git-hitsz.cn/jiang/experiment-video/tree/master/MacMicRobot%20Project)