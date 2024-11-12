# 赛事包更新说明
####请按照先前教程完成ros的安装、工作空间的创建和仿真环境的搭建。

###1. 替换工作空间下的gazebo_pkg功能包并再次编译工作空间

**1.1 gazebo_pkg功能包中的logo更新**
> * 运行roslaunch前，请将start_plane、end_plane文件夹复制到.gazebo/models目录下

**1.2 gazebo_pkg功能包中的小车模型文件更新**
> * 小车模型的部分参数已做修改以及robotBaseFrame已修改为dummy

**1.3 gazebo_pkg功能包的赛道更新**
> * 小车赛道已添加障碍物，打开gazebo加载的为已添加锥筒的小车仿真环境

### 2. 运行gazebo仿真环境
> * 打开终端输入如下指令：

```bash
roslaunch gazebo_pkg race.launch       
```

> * 若打开的小车赛道中没有出现锥筒请按如下操作：
将construction_cone.zip压缩包解压，然后将解压后的construction_cone文件复制到.gazebo/models目录下，construction_cone文件为锥筒的模型包，复制后运行launch文件就可直接打开有障碍物的赛道。







