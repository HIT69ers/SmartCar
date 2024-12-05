# 12_8 进度说明及视觉代码使用方法
更新于2024.12.4
***
## 进度说明
 - [ ] 模型推理
   - [x] RKNN加速平台的部署、环境配置；
   - [x] 模型格式的正确转换；
   - [ ] 模型的高质量推理；
 - [ ] 语音包
   - [ ] 环境配置、API函数使用方法；
   - [ ] 语音唤醒；
   - [ ] 指令脱困；
### 模型的高质量推理存在的问题：
- **现有的模型均无法正常识别急救包，其他均可正常识别（重点）；**
- 识别框标注质量低；

为了解决以上两点问题，目前考虑的解决思路：
1. 增加训练轮次，训练出性能更优良的模型以应对模型压缩导致的性能下降；
2. 修改**rknnlite_detect_model.py**文件中关于模型输出结果后处理的代码，提高识别框质量；
### 语音包啥都没做，一脸懵逼，唯一参考资料就是官方例程；
## 视觉代码使用方法
1. 相关文件
   1. 文件路径
   ```
   |—— ucar_cam
   |   |—— launch
   |   |   |—— 12_1_rec.launch                                               # ros节点相关设置
   |   |—— scripts
   |   |   |—— 12_1_rec.py
   |   |   |—— my_params
   |   |   |   |—— classes.yml                                               # 不必过于关心
   |   |   |   |—— rknn_rec_params.yml                                       # 指定视觉节点加载的权重文件，以及'to_mid'和'area_rate'的相关阈值
   |   |—— src
   |   |   |—— yolov5-master
   |   |   |   |—— detect.py                                                 # 官方例程，跑出来Segmentation Fault...
   |   |   |   |—— my_test.py                                                # 用于将.onnx模型转化为.rknn模型，并通过摄像头截取一帧图像进行推理
   |   |   |   |—— rknn_detect.py
   |   |   |   |—— rknnlite_detect_model.py
   |   |   |   |—— my_model
   |   |   |   |   |—— best.rknn                                             # epochs 10, batch-size 24, 的.pt转化而来，寄...
   |   |   |   |   |—— 2024_5_23_final_best_c.rknn                           # 学长的.pt权重转化而来，照样寄...
   ```
   2. 巡线相关设置
      如果需要启动节点的巡线功能，需要在**12_1_rec.launch**文件中设置**trace_line**参数为1：
      ```
      <param name="trace_line" value="1" />
      ```
   3. 获取、调整**to_mid**和**area_rate**的理想值及其他参数
      1. 获取相关值
         ![Image](https://github.com/HIT69ers/SmartCar/blob/main/12_8_视觉/1.jpg)
      2. 调整相关值
         1. rknn_rec_params.yml中**to_mid_thre**的值为成功识别救援物品的**to_mid**值的绝对值上限；
         2. rknn_rec_params.yml中**area_rate**下三种救援物品及对应值分别为成功识别三种救援物品的**area_rate**值的下限；
         3. rknn_rec_params.yml中**model_file**为视觉节点启动时加载的权重文件；
2. 启动视觉节点
   ```bash
   conda activate rknnlite                                                   # 启动虚拟环境
   roslaunch ucar_cam 12_1_rec.launch                                        # 启动识别节点
   ```
3. 视觉与导航交互
   1. 鉴于目前尚未得到可以正确识别急救包的模型，在识别急救包的部分，可以采用以下方法**跳过急救包识别直接进行语音播报**（反正急救包位置固定）：
      ```python
      rospy.set_param('first_aid_kit', 1)
      ```
      当然，该步操作必须在小车停稳在急救包识别区后进行，否则会穿帮...
   2. 在识别其他的物品时，则通过和之前同样的方法即可开始识别功能：
      ```python
      rospy.set_param('take_photo', 1)
      ```
   3. 在识别救援物品时，通过和之前同样的方法获取**to_mid**和**area_rate**值：
      ```python
      rospy.get_param('rescue_x', 0)                                         # to_mid
      rospy.get_param('rescue_area', 0)                                      # area_rate
      ```
   4. 识别超时，退出识别，直接进入巡线：
      ```python
      rospy.set_param('take_photo', 2)
      ```
   5. 启动巡线：
      ```python
      rospy.set_param('start_trace', 1)
      ```

***
[.pt->.onnx](https://github.com/ChuanSe/yolov5-PT-to-RKNN)
      
