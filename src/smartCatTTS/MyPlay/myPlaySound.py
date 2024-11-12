from playsound import playsound
from os import path


def readTxt():
    f = open("/home/ucar/ucar_ws/src/smartCatTTS/MyPlay/tts.txt", "r", encoding='utf-8')
    tts = {}
    for line in f:
        list = line[:-1].split(':')
        if len(list) == 1:
            break
        tts[list[0]] = list[1]
    return tts


class MyPlaySound:
    def __init__(self):
        self.ttsMap = readTxt()
        self.dic = self.ttsMap.keys()

    def forward_segment(self, text):
        word_list = []
        i = 0
        while i < len(text):
            longest_word = text[i]  # 当前扫描位置的单字
            for j in range(i + 1, len(text) + 1):  # 所有可能的结尾
                word = text[i:j]  # 从当前位置到结尾的连续字符串
                if word in self.dic:  # 在词典中
                    if len(word) > len(longest_word):  # 并且更长
                        longest_word = word  # 则更优先输出
            word_list.append(longest_word)  # 输出最长词
            i += len(longest_word)  # 正向扫描
        return word_list

    def play(self, text):
        word_list = self.forward_segment(text)
        for word in word_list:
            print(word)
            print(self.ttsMap[word])
            playsound(path.join(path.dirname(__file__), 'data/{0}'.format(self.ttsMap[word])))
