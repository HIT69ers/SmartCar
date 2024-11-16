# 11_17进度检查——视觉相关代码及参数
***
### 文件说明
- 11_15_rec.launch 视觉结点启动文件
- 11_15_rec.py 视觉识别代码
- classes.yml 需要识别的图像种类
- rec_params.yml 识别相关参数

### 文件安放位置
这些文件全部位于ucar_cam功能包中

1.进入ucar_cam
```bash
roscd ucar_cam
```
2.文件目录关系
```
|—— ucar_cam
|   |—— launch
|   |   |—— 11_15_rec.launch
|   |—— scripts
|   |   |—— 11_15_rec.py
|   |   |—— my_params
|   |   |   |—— classes.yml
|   |   |   |—— rec_params.yml
```

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

### 导航与视觉交互

小车每到达一个识别区，
```python
rospy.set_param('take_photo', 1)
```
即可开启视觉识别功能

### 视觉部分后续工作

1.该处代码需要补充：
```python
while not is_finish_first_aid_kit:
    ...
```
2.每完成一次识别，向导航发布参数，说明当前识别状态
