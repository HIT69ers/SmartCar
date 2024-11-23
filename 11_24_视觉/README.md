# 11月24日进度检查 视觉部分代码

### 功能简介

该部分代码用于小车识别指定物品并进行语音播报的任务：
- 识别恐怖分子数量
- 获取急救包
- 根据恐怖分子数量获取指定的救援物品

### 文件说明

该部分包含两个文件，使用时请将它们放置于指定的路径：

```
|—— ucar_ws
    |—— src
        |—— ucar_cam
            |—— scripts
                |—— 11_22_rec.py
            |—— launch
                |—— 11_22_rec.launch
```

- 11_22_rec.py        识别任务代码
- 11_22_rec.launch    识别节点启动文件

注意！！！放置文件后，必须为这两个文件添加可执行权限！！！
```bash
chmod +x 11_22_rec.py
chmod +x 11_22_rec.launch
```

### 使用方法

#### 启动识别功能

- 激活虚拟环境

```bash
cd
source venv3.9/bin/activate
```

- 终端进入工作区

```bash
cd ~/ucar_ws
```

- 启动节点

```bash
roslaunch ucar_cam 11_22_rec.launch
```

#### 视觉与导航交互

当小车到达需要识别物品的位置并停下时，在导航节点中通过该命令发布参数：

```python
rospy.set_param("take_photo", 1)
```

即可开启识别功能；

当小车完成识别时，会自动将"take_photo"参数置0，这时小车可以执行接下来的任务；

#### 救援物品识别逻辑

在本周的进度检查任务中，小车需要根据恐怖分子的数量识别对应的救援物品并播报。因此，小车需要判断识别到的是否为需要的物品；

具体逻辑如下：
- 识别到正确物品时，视觉代码会将"rescue"参数置4，表示识别完成；
- 识别到正确物品但**不符合面积要求**，视觉代码会将"rescue"参数置3，表示需要对小车姿态进行调整；
- 识别到的物品不是正确物品时，视觉代码会将"rescue"参数置1；
- 未识别到物品时，视觉代码会将"rescue"参数置0；

因此，在导航代码中通过获取"rescue"参数，即可获取小车识别救援物品的状态，并进行下一步操作：

```python
rospy.get_param("rescue", 0)
```

#### 识别救援物品的面积要求

视觉识别代码中在识别救援物品时包含如下内容：

```python
to_mid = result[2][0] / detect.width - 0.5  # 目标偏离中心点的像素距离-0.5 - 0.5 从左到右
area_rate = result[1] / (detect.width * detect.height)  # 目标框的面积与图片面积比例 0 - 1
rospy.set_param("rescue_x", to_mid) #发送中点距离
rospy.set_param("rescue_area", area_rate) #发送区域面积
```

当识别到正确物品时，只有"to_mid"和"area_rate"参数满足以下两个条件时，才能成功识别，否则均会判断为**不符合面积要求**

```python
area_rate_thre = config['area_rate'][target_rescue]  # target_rescue 为恐怖分子数量对应的救援物品
abs(to_mid) <= 0.4
abs(to_mid) < config["to_mid_thre"] and area_rate > area_rate_thre and rospy.get_param('nongoal_flag') == 6:
```

其中，**config["to_mid_thre"]** 和 **area_rate_thre**均为可调节的参数，位于**rec_params.yml**文件中，该文件位置：

```
|—— ucar_cam
    |—— scripts
        |—— my_params
            |—— rec_params.yml
```

在小车导航端，可以通过获取"rescue_x"和"rescue_area"参数来获取相关信息，帮助小车调整自身位姿；

"nongoal_flag"参数功能尚不明确，但测试过程中发现不影响使用，可忽略；
