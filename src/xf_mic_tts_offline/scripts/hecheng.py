#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import pyttsx3
import io
import sys
 
engine = pyttsx3.init()
 
# 获取语音包
voices = engine.getProperty('voices')
for voice in voices:
    print ('id = {}\tname = {} \n'.format(voice.id, voice.name))
# 设置使用的语音包
engine.setProperty('voice', 'zh') #开启支持中文
# engine.setProperty('voice', voices[0].id)
 
# 改变语速  范围为0-200   默认值为200
rate = engine.getProperty('rate')  #获取当前语速
engine.setProperty('rate', rate-40)
 
# 设置音量  范围为0.0-1.0  默认值为1.0
engine.setProperty('volume', 0.5)
 
# 预设要朗读的文本数据
line = "你好，世界！" #要播报的内容
engine.say(line)
 
# 朗读
engine.runAndWait()