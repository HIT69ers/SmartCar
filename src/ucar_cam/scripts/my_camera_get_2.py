"""罗鑫 2024-03-27"""
import cv2
import time
import os
import sys
import yaml
folder_path = os.path.dirname(os.path.abspath(__file__))

def str_to_bool(str):
    return True if str.lower() == 'true' else False


def load_cap_yaml():
    with open(f'{folder_path}/my_params/cap_params.yml') as f:
        res = yaml.load(f.read(), Loader=yaml.FullLoader)
    return res


def main():
    res = load_cap_yaml()  # 加载相机参数
    config_list = ['dark', 'backlight', 'bright', 'normal']  # 可选参数列表

    assert len(sys.argv) == 6, "You must input 6 args"
    save_path = sys.argv[1]  # 保存路径
    is_text = str_to_bool(sys.argv[2])  # 文字绘制
    is_print = str_to_bool(sys.argv[3])  # 打印
    my_config = sys.argv[4].lower()  # 选择参数
    is_series = str_to_bool(sys.argv[5])  # 是否是连续拍摄模式

    # 我的参数选择
    assert my_config in config_list, "input config not in configs list: 'Dark', 'Backlight', 'Bright', 'Normal'"

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
    print(f"is_text: {is_text}")
    print(f"is_print: {is_print}")
    print(f"save_path: {save_path}")
    print(f"my_config: {my_config}")
    print(f"-BRIGHTNESS-  {res[my_config]['BRIGHTNESS']}")
    print(f"-CONTRAST-  {res[my_config]['CONTRAST']}")
    print(f"-SATURATION-  {res[my_config]['SATURATION']}")
    print(f"is_series: {is_series}")
    print(f"code: {codec}")

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

        # 是否将FPS绘制到图片上
        if is_text:
            cv2.putText(frame, 'FPS:' + ' ' + fps_now, (10, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)

        # 命令行输出文字
        h, w = frame.shape[:2]
        if is_print:
            print(h, w)
            print("fps:", fps)

        cv2.imshow('lx_Camera', frame)  # 显示图片

        key = cv2.waitKey(5) & 0xff  # 键盘输入

        # 下面是键盘控制操作
        if not is_series:  # 非连续拍摄
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
        else:  # 连续读取保存
            if key == 27:  # exc退出
                print("quit")
                break
            elif key == ord(" "):  # 空格暂停
                print("stop")
                while True:
                    key = cv2.waitKey(0) & 0xff
                    if key == ord(" "):  # 退出
                        break
            i += 1
            path = os.path.join(save_path, f"frame{i}.jpg")
            print(f"save {path}")
            cv2.imwrite(path, frame)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
