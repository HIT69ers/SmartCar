# -*- coding: utf-8 -*-
"""罗鑫 2024-03-27"""
import cv2
import time
import os
import sys
import yaml

folder_path = os.path.dirname(os.path.abspath(__file__))


def load_cap_yaml():
    with open(f'{folder_path}/my_params/cap_params.yml', encoding='utf-8') as f:
        res = yaml.load(f.read(), Loader=yaml.FullLoader)
    return res


def main():
    res = load_cap_yaml()  # 加载相机参数
    config_list = ['dark', 'backlight', 'bright', 'normal']  # 可选参数列表

    show_path = f'{folder_path}/../src/yolov5-master/data/my_imgs/imgs.jpg'  # 保存路径
    my_config = config_list[0]  # 选择参数
    save_show = True

    save_path = "/home/ucar/Desktop/my_picture/"
    
    # 如果文件夹存在的话就创建文件夹
    if not os.path.exists(save_path):
        print(f"make folder {save_path}")
        os.mkdir(save_path)
    else:
        for i in range(1000):
            save_path_2 = save_path[:-1] + f'{i + 1}' + '/'
            if not os.path.exists(save_path_2):
                print(f"make folder {save_path_2}")
                os.mkdir(save_path_2)
                save_path = save_path_2
                break

    # cap参数设置
    cap = cv2.VideoCapture(0)
    weight = 320
    height = 240
    cap.set(3, weight)
    cap.set(4, height)
    codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
    cap.set(cv2.CAP_PROP_BRIGHTNESS, res[my_config]['BRIGHTNESS'])
    cap.set(cv2.CAP_PROP_CONTRAST, res[my_config]['CONTRAST'])
    cap.set(cv2.CAP_PROP_SATURATION, res[my_config]['SATURATION'])
    cap.set(cv2.CAP_PROP_FOURCC, codec)

    # 打印当前参数
    print(f"save_path: {save_path}")
    print(f"my_config: {my_config}")
    print(f"-BRIGHTNESS-  {res[my_config]['BRIGHTNESS']}")
    print(f"-CONTRAST-  {res[my_config]['CONTRAST']}")
    print(f"-SATURATION-  {res[my_config]['SATURATION']}")
    print(f"code: {codec}")

    cv2.namedWindow('imgs')
    fps = cap.get(cv2.CAP_PROP_FPS)
    # cap.set(cv2.CAP_PROP_AUTOFOCUS, False)
    # cap.set(cv2.CAP_PROP_SETTINGS, 1)
    b_fps = time.time()
    i = 0
    while (True):
        # 计算fps
        f_fps = time.time()
        fps_now = str(round(1 / (f_fps - b_fps), 2))
        b_fps = f_fps

        # 取图片
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)

        key = cv2.waitKey(10) & 0xff  # 键盘输入

        if save_show == True:
            cv2.imwrite(show_path, frame)
        # 下面是键盘控制操作
        if key == 27:  # exc退出
            print("quit")
            break
        elif key == ord("s"):  # s单张图片保存
            i += 1
            path = os.path.join(save_path, f"frame{i}.jpg")
            print(f"save {path}")
            cv2.imwrite(path, frame)
        elif key == ord(" "):  # 空格暂停
            print("stop")
            while True:
                key = cv2.waitKey(0) & 0xff
                if key == ord("s"):
                    i += 1
                    path = os.path.join(save_path, f"frame{i}.jpg")
                    print(f"save {path}")
                    cv2.imwrite(path, frame)
                elif key == ord(" "):
                    break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
