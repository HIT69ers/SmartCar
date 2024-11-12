import sys
import os
folder_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(f"{folder_path}/../../../../smartCatTTS/")  # 添加语音播报文件夹的路径

from MyPlay.myPlaySound import MyPlaySound  # 语音播报模块

def voice(content):
    voice = MyPlaySound()
    if content == "first_aid_kit":
        voice.play("我已取到急救包")
    elif content == "teargas":
        voice.play("我已取到催泪瓦斯")
    elif content == "bulletproof_vest":
        voice.play("我已取到防弹衣")
    elif content == "spontoon":
        voice.play("我已取到警棍")
    elif content == "terrorist3":
        voice.play("恐怖分子的数量为三个")
    elif content == "terrorist2":
        voice.play("恐怖分子的数量为二个")
    elif content == "terrorist1":
        voice.play("恐怖分子的数量为一个")
    elif content == "over":
        voice.play("结束")
