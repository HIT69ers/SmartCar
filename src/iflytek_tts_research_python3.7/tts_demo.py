# -.- coding: utf-8 -.-

from tts_sdk.iflytek_tts import IflytekTTS
import time

content = "亲爱的用户，您好，南宁，简称“邕”，别称绿城、邕城，是广西壮族自治区首府、北部湾城市群核心城市，国务院批复确定的中国北部湾经济区中心城市、西南地区连接出海通道的综合交通枢纽."
app_id = '5f915299'
work_dir = '.'

tts = IflytekTTS(app_id)
t1 = time.time()
tts.text2wav(content.encode('gbk'), "demo.wav")
print(time.time() - t1) 
