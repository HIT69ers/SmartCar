from my_utils import calculate_iou
import yaml
import cv2
import copy


class FilterResult:
    """
        过滤结果的类， 对网络推理的结果进行处理
        里面每个过滤函数是对对应情况的特化，因此可以随意写，不用关系结构和可读性
    """

    def __init__(self, config, class_list):
        # 有关识别的外部参数
        self.config = config
        self.class_list = class_list  # 类别列表
        # 图片的一些属性
        self.width = 320
        self.height = 240
        self.area = self.width * self.height  # 图片的面积

        # rescue_2_3模式
        self.rescue_inf2 = {"bulletproof_vest": [], "spontoon": [], "teargas": []}
        self.rescue_inf3 = {"bulletproof_vest": [], "spontoon": [], "teargas": [], "first_aid_kit": []}

    def select_filter(self, result, what, mode, target=None):
        """
        类似与菜单选择，可能使用起来比较方便一点
        """
        final = None
        if result != None:  # 如果有结果
            if what is "terrorist":
                if mode == 1:
                    final = self.terrorist_1(result)
                else:
                    final = self.terrorist_2(result)
            elif what is "rescue":
                if mode == 1:  # 急救包
                    final = self.rescue_1(result)
                elif mode == 2:  # 其他物品
                    final = self.rescue_2_3(result, 1)
                elif mode == 3:
                    final = self.rescue_2_3(result, 2)
                elif mode == 4:
                    final = self.rescue_4(result, target)
        else:
            print("没有检测到目标")
            pass
        return final

    def terrorist_1(self, result):
        """
        恐怖分子的处理
        导航已经停泊在B区域
        车头正对着图片拍了一张照片
        照片已经经过网络输出了结果
        因为是小车停在指定区域,不用对box的绝对大小进行判断,但是会对交并比进行判断
        难点就是数量的判断
        """
        terrorist_num = {1: 0, 2: 0, 3: 0}  # 恐怖分子的数量
        box2s = []  # 恐怖分子2的框坐标
        box1s = []  # 恐怖分子1的框坐标
        for target in result:
            target_class = self.class_list[int(target[5])]
            if target[4] >= 0.4:  # 置信度要求
                if target_class == "terrorist3":
                    return 3  # 如果检测到3个恐怖分子，那就一定是3个恐怖分子
                elif target_class == "terrorist2":
                    box2s.append(target[0:4])  # 添加2框的坐标
                    terrorist_num[2] += 1
                elif target_class == "terrorist1":
                    box1s.append(target[0:4])
                    terrorist_num[1] += 1
                else:
                    print("这个目标没有恐怖分子")

        # 下面是非3个的判断逻辑
        if terrorist_num[2] != 0:  # 如果没检测到3个但是检测到两个恐怖分子
            if terrorist_num[2] > 1:  # 检测出来多个两个则可能是3个恐怖分子
                for box2_1 in box2s:
                    for box2_2 in box2s:
                        # 0.8是因为有交错的情况(但一定不为0， 因为神经网络不会分的那么开)
                        if calculate_iou(box2_1, box2_2) < 0.8 and not calculate_iou(box2_1, box2_2) == 0:
                            return 3  # 如果有重合度小于0.8， 那判断有第三个恐怖分子
            if terrorist_num[1] == 0:  # 如果没有一个恐怖分子的框
                return 2

            if terrorist_num[1] > 2:  # 如果有两个恐怖分子的框， 而且一个恐怖分子的框超过2个（我们就断言说一定有3个，因为此时应该概率很大, 但其实并不是百分之百）
                return 3
            else:
                for box1 in box1s:
                    for box2 in box2s:
                        if calculate_iou(box1, box2) < 0.2:  # 其实是将3个识别成两个和1个了
                            return 3
                return 2

        elif terrorist_num[1] != 0:  # 如果只检测到一个恐怖分子
            separate_num = 0  # 分离的数量
            if terrorist_num[1] == 1:  # 只有一个 一个的恐怖分子必定是一个恐怖分子
                return 1

            for box1_1 in box1s:  # 统计分离的框
                for box1_2 in box1s:
                    if calculate_iou(box1_1, box1_2) < 0.2:
                        separate_num += 1

            if separate_num == 0:  # 没有分离
                return 1
            elif separate_num == 1:  # 有一个分离
                return 2
            else:  # 有三个以上分离(这个判断其实也是概率问题，但是3个分离是3个的概率我们认为比两个大，所以返回的是3个)
                return 3

        else:
            print("这张图片没有恐怖分子")
            return 0

    def terrorist_2(self, result):
        """
        恐怖分子的处理
        这个的策略是根据置信度排序, 因为这个是顶点检测，因此也不需要进行面积啥的检测
        """

        def compare_second_column(row):
            return row[4]

        sort_result = sorted(result, key=compare_second_column)  # 按照置信度排序
        if sort_result[0][4] < 0.4:  # 最大的置信度都小于0.4了
            print("这张图片没有恐怖分子")
            return 0
        else:
            target_class = self.class_list[int(sort_result[0][5])]
            if target_class == "terrorist3":
                return 3
            elif target_class == "terrorist2":
                return 2
            else:
                return 1

    def rescue_1(self, result):
        """
        急救包探测
        因为急救包是放在固定位置的
        认为导航正确到达该位置，并且车头朝向图片
        车头正对着图片拍了一张照片
        照片已经经过网络输出了结果
        不需要判断数量，比较简单
        """
        for target in result:
            if self.class_list[int(target[5])] == "first_aid_kit" and target[4] > self.config['rescue_con']:
                return "first_aid_kit"  # 1表示已经识别到了
        return 0  # 没有识别到

    def rescue_2_3(self, result, mod=1):  # 2_3方法合并在一起用mod控制
        """
        急救物品探索
        分为1模式和2模式
        1模式是只探索非急救包的物品， 因为急救包已经用rescue探索完成
        2模式包括所有物品

        导航到位置，拍摄图片，经过网络后输入该函数
        不满足一定面积大小或中点偏移量的target被排除
        只返回满足一定面积大小和中点偏移量的target
        """
        if mod == 1:
            rescue_inf = copy.deepcopy(self.rescue_inf2)  # 救援物品的数量
        else:
            rescue_inf = copy.deepcopy(self.rescue_inf3)

        for target in result:
            target_class = self.class_list[int(target[5])]
            if target_class in rescue_inf.keys() and target[4] >= self.config['rescue_con']:  # 是否是规定的救援物品类别, 且置信度满足要求
                box = target[0:4]  # bbox的坐标点
                x_m, y_m = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0  # 中点坐标
                # 保证在一定区域中
                # if not (self.width * 0.1< x_m < self.width * 0.9 and self.height * 0.1 < y_m < self.height * 0.9):
                #     continue  # 如果过于偏则认为不是
                deviation = abs(x_m / self.width - 0.5) + abs(y_m / self.height - 0.5)  # 我们用来评价偏离量的方法
                area = (box[2] - box[0]) * (box[3] - box[1])  # box所占面积

                # 因为警棍喝催泪弹的体积都比较小，进行一定补偿
                if target_class == "spontoon":
                    area *= self.config['spontoon_area_c']
                elif target_class == "teargas":
                    area *= self.config['teargas_area_c']
                elif target_class == "bulletproof_vest":
                    area *= self.config['bulletproof_area_c']

                if area / self.area < 0.01:
                    continue  # 面积过于小了

                rescue_inf[target_class].append([deviation, area, (x_m, y_m)])  # 添加到结果列表当中来

        flag = 0  # 用来检验是否检测到
        max_area = [None, 1, 0, [0, 0]]  # 第一个是序号, 第二个是偏离量，第三个是面积, 第四个是中点坐标
        min_deviation = [None, 1, 0, [0, 0]]
        for key, values in rescue_inf.items():
            for value in values:  # 因为values又是一个列表, 能进这个循环也代表非空
                if value[0] < min_deviation[1]:  # 偏离量比较
                    min_deviation = [key, value[0], value[1], value[2]]  # 最小偏离量的目标
                if value[1] > max_area[2]:  # 面积比较
                    max_area = [key, value[0], value[1], value[2]]  # 最大面积的目标
                flag += 1

        if flag == 0:  # 没有任何类别
            return 0  # 0代表没有检测到

        if max_area[0] == min_deviation[0]:  # 如果都是同一个目标
            return [max_area[0], float(max_area[2]),
                    [float(max_area[3][0]), float(max_area[3][1])]]  # 正常返回值是一个列表，包含名称，最大目标框的面积， 中点坐标
        else:
            if abs(max_area[1] - min_deviation[1]) / (max_area[1] + min_deviation[1]) < \
                    abs(max_area[2] - min_deviation[2]) / (max_area[2] + min_deviation[2]):  # 进行最大和最接近中心的权衡
                return [max_area[0], max_area[2], [float(max_area[3][0]), float(max_area[3][1])]]
            else:
                return [min_deviation[0], min_deviation[2], [float(max_area[3][0]), float(max_area[3][1])]]
        
    def rescue_4(self, result, target_rescue):
        """
        急救物品探索
        输入：
            神经网络输出的结果
            目标的救援物品（已经通过恐怖分子判断了）

        如果有符合救援物品就不再输出其他救援物品的模式
        的最大置信度输出
        """
        rescue_inf = copy.deepcopy(self.rescue_inf2)  # 只有非急救包的物品

        for target in result:
            target_class = self.class_list[int(target[5])]
            if target_class in rescue_inf.keys() and target[4] >= self.config['resuce_con']:  # 是否是规定的救援物品类别, 且置信度满足要求
                box = target[0:4]  # bbox的坐标点
                x_m, y_m = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0  # 中点坐标

                deviation = abs(x_m / self.width - 0.5) + abs(y_m / self.height - 0.5)  # 我们用来评价偏离量的方法
                area = (box[2] - box[0]) * (box[3] - box[1])  # box所占面积

                if target_class == target_rescue and target[4] > self.config['rescue_con']:  # 找到一定置信度的物品直接返回
                    return [target_class, float(area), [float(x_m), float(y_m)]]

                if not (self.width * 0.1 < x_m < self.width * 0.9 and self.height * 0.1 < y_m < self.height * 0.9):
                    continue  # 如果过于偏则认为不是

                # 因为警棍喝催泪弹的体积都比较小，进行一定补偿
                if target_class == "spontoon":
                    area *= self.config['spontoon_area_c']
                elif target_class == "teargas":
                    area *= self.config['teargas_area_c']
                elif target_class == "bulletproof_vest":
                    area *= self.config['bulletproof_area_c']

                if area / self.area < 0.01:
                    continue  # 面积过于小了

                rescue_inf[target_class].append([deviation, area, (x_m, y_m)])  # 添加到结果列表当中来

        flag = 0  # 用来检验是否检测到
        max_area = [None, 1, 0, [0, 0]]  # 第一个是序号, 第二个是偏离量，第三个是面积, 第四个是中点坐标
        min_deviation = [None, 1, 0, [0, 0]]
        for key, values in rescue_inf.items():
            for value in values:  # 因为values又是一个列表, 能进这个循环也代表非空
                if value[0] < min_deviation[1]:  # 偏离量比较
                    min_deviation = [key, value[0], value[1], value[2]]  # 最小偏离量的目标
                if value[1] > max_area[2]:  # 面积比较
                    max_area = [key, value[0], value[1], value[2]]  # 最大面积的目标
                flag += 1

        if flag == 0:  # 没有任何类别
            return 0  # 0代表没有检测到

        if max_area[0] == min_deviation[0]:  # 如果都是同一个目标
            return [max_area[0], float(max_area[2]),
                    [float(max_area[3][0]), float(max_area[3][1])]]  # 正常返回值是一个列表，包含名称，最大目标框的面积， 中点坐标
        else:
            if abs(max_area[1] - min_deviation[1]) / (max_area[1] + min_deviation[1]) < \
                    abs(max_area[2] - min_deviation[2]) / (max_area[2] + min_deviation[2]):  # 进行最大和最接近中心的权衡
                return [max_area[0], max_area[2], [float(max_area[3][0]), float(max_area[3][1])]]
            else:
                return [min_deviation[0], min_deviation[2], [float(max_area[3][0]), float(max_area[3][1])]]
    

if __name__ == "__main__":
    """结果过滤器测试代码"""
    from my_detect_model import myPredict_single
    from my_detect import Detect
    import os
    folder_path = os.path.dirname(os.path.abspath(__file__))

    with open(f'{folder_path}/../../scripts/my_params/rec_params.yml', 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)  # 获取yolo识别策略的参数

    with open(f'{folder_path}/../../scripts/my_params/classes.yml', 'r') as f:  # 类别参数
        class_config = yaml.load(f, Loader=yaml.FullLoader)

    result_filter = FilterResult(config, class_config["classes"])

    img_path = [f"{folder_path}/data/my_imgs/frame99.jpg",
                f"{folder_path}/data/my_imgs/frame124.jpg"]

    d = Detect(config, class_config["classes"])

    for i in range(2):
        img = cv2.imread(img_path[i])
        results_bs = myPredict_single(model=d.model, im0=img, imgsz=d.imgsz, pt=d.pt, stride=32)

        if len(results_bs) > 0:
            results = results_bs[0]  # result: x1, y1, x2, y2, con, cls
        else:
            results = None

        print(results)

        # filter_result = result_filter.select_filter(results, "rescue", 1)
        filter_result = result_filter.select_filter(results, "terrorist", 2)
        print(filter_result)
