import numpy as np
import cv2
import time
import rospy
import os
import sys

def str_to_bool(str):
    return True if str.lower() == 'true' else False

save_path = "/home/ucar/Desktop/my_picture/"
print(len(sys.argv))

is_text = 1
is_print = 1
if len(sys.argv) > 4:
    print("the number of args must be 0 ,1 ,2")
    sys.exit(1)
if len(sys.argv) == 2:
    save_path = sys.argv[1]

if len(sys.argv) == 3:
    save_path = sys.argv[1]
    is_text  = str_to_bool(sys.argv[2])

if len(sys.argv) == 4:
    save_path = sys.argv[1]
    is_text  = str_to_bool(sys.argv[2])
    is_print = str_to_bool(sys.argv[3])


if not os.path.exists(save_path):
    print(f"make folder {save_path}")
    os.mkdir(save_path)
else:
    for i in range(1000):
        save_path_2 = save_path[:-1] + f'{i+1}' + '/'
        if not os.path.exists(save_path_2):
            print(f"make folder {save_path_2}")
            os.mkdir(save_path_2)
            save_path = save_path_2
            break
            

print(is_print)
cap = cv2.VideoCapture(0)
weight=320
height=240
cap.set(3, weight)  
cap.set(4, height)
codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
print(codec)

cap.set(cv2.CAP_PROP_FOURCC, codec)

fps =cap.get(cv2.CAP_PROP_FPS) 
# cap.set(cv2.CAP_PROP_AUTOFOCUS, False)  
# cap.set(cv2.CAP_PROP_SETTINGS, 1) 
b_fps=time.time()  
i=0
while(True):
    f_fps=time.time() 
    fps_now=str(round(1/(f_fps-b_fps),2))   
    b_fps=f_fps 
    ret, frame = cap.read()
    frame = cv2.flip(frame,1)

    if is_text: 
        cv2.putText(frame,'FPS:'+' '+fps_now,(10, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1 ,(0,0,255),2,cv2.LINE_AA)
    h,w=frame.shape[:2]

    if is_print:
        print(h,w)
        print("fps:",fps)
    cv2.imshow('Camera_USB', frame)

    key = cv2.waitKey(5) & 0xff
    if key == 27:
        print("quit")
        break
    elif key == ord("s"):
        i += 1
        path = os.path.join(save_path, f"frame{i}.jpg")
        print(f"save {path}")
        cv2.imwrite(path, frame)
    elif key == ord(" "):
        print("stop")
        while(True):
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
