### 导航与视觉交互

小车每到达一个识别区，
```python
rospy.set_param('take_photo', 1)
```
即可开启视觉识别功能

### 识别顺序

恐怖分子->急救包->救援物品

### 启动视觉节点

1.命令行输入：
```bash
source ~/venv3.9/bin/activate
```
以启动虚拟环境；
2.启动节点
```bash
roslaunch ucar_cam 11_15_rec.launch
```

### 视觉部分后续工作

每完成一次识别，向导航发布参数，说明当前识别状态
