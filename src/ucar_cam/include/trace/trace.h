#ifndef _IMAGE_H
#define _IMAGE_H
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ros/ros.h"

#define MISS 255
#define CAMERA_H  120                            //图片高度
#define CAMERA_W  188                            //图片宽度
#define FAR_LINE 60//图像处理上边界
#define NEAR_LINE 115//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define white_num_MAX 10//每行最多允许白条数

extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
extern float  pre_mid; //中线位置

void THRE(const uint8_t* source);
int find_f(int a);
void search_white_range();
void find_all_connect();
void find_road();
uint8_t find_continue(uint8_t i_start, uint8_t j_start);
void ordinary_two_line(void);
float trace_main(const uint8_t* source);
void get_mid_line(void);

void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);
#endif //