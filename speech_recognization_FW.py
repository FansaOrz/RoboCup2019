#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
"""
    Author: Yifei Ren
    Name: Off-line Speech Recognization
    Version: 1.0
    Date: 23/06/2019
    Description: Off-line speech recognization.
    Note: Using ALDialog Methods.
"""
import qi
import sys
import time

class offline_speech_recognization():
    def __init__(self, session):
        self.session = session
        # Initial Service
        self.Dialog = self.session.service("ALDialog")
        self.Memo = self.session.service("ALMemory")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.Dialog.setLanguage("English")
        # Initial parameter
        self.if_end = False
        self.ans = None
        # Loading the topic given by the user (absolute path is required)
        topic_path = "/home/nao/topic/aldialog_offline_topic_file.top"
        self.topf_path = topic_path.decode('utf-8')
        self.topic_name = self.Dialog.loadTopic(self.topf_path.encode('utf-8'))
        self.Dialog.activateTopic(self.topic_name)
        # Activating the loaded topic
        self.TextToSpe.say("What's your name, sir?")
        time.sleep(0.5)
        self.Dialog.subscribe('offline_dialog')
        # callback functions
        self.end_of_dialog = self.Memo.subscriber("stop_talking")
        self.end_of_dialog.signal.connect(self.callback_stoptalking)
        self.amelia_ans = self.Memo.subscriber("amelia")
        self.amelia_ans.signal.connect(self.callback_amelia)
        self.end_of_dialog = self.Memo.subscriber("angel")
        self.end_of_dialog.signal.connect(self.callback_angel)
        self.amelia_ans = self.Memo.subscriber("ava")
        self.amelia_ans.signal.connect(self.callback_ava)
        self.end_of_dialog = self.Memo.subscriber("charlie")
        self.end_of_dialog.signal.connect(self.callback_charlie)
        self.amelia_ans = self.Memo.subscriber("charlotte")
        self.amelia_ans.signal.connect(self.callback_charlotte)
        self.end_of_dialog = self.Memo.subscriber("hunter")
        self.end_of_dialog.signal.connect(self.callback_hunter)
        self.amelia_ans = self.Memo.subscriber("max")
        self.amelia_ans.signal.connect(self.callback_max)
        self.end_of_dialog = self.Memo.subscriber("mia")
        self.end_of_dialog.signal.connect(self.callback_mia)
        self.amelia_ans = self.Memo.subscriber("olivia")
        self.amelia_ans.signal.connect(self.callback_olivia)
        self.end_of_dialog = self.Memo.subscriber("parker")
        self.end_of_dialog.signal.connect(self.callback_parker)
        self.amelia_ans = self.Memo.subscriber("sam")
        self.amelia_ans.signal.connect(self.callback_sam)
        self.end_of_dialog = self.Memo.subscriber("jack")
        self.end_of_dialog.signal.connect(self.callback_jack)
        self.amelia_ans = self.Memo.subscriber("noah")
        self.amelia_ans.signal.connect(self.callback_noah)
        self.end_of_dialog = self.Memo.subscriber("oliver")
        self.end_of_dialog.signal.connect(self.callback_oliver)
        self.amelia_ans = self.Memo.subscriber("thomas")
        self.amelia_ans.signal.connect(self.callback_thomas)

        while not self.if_end:
            print '----', 'dialog lasting', '----'

    def get_result(self):
        return self.ans

    # Callback Functions
    def callback_stoptalking(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.TextToSpe.say("Thanks for your cooperation")
        # Saying "That's all"

    def callback_amelia(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'amelia'
        # self.TextToSpe.say("Thanks for your cooperation, amelia")

    def callback_angel(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'angel'
        # self.TextToSpe.say("Thanks for your cooperation, angel")

    def callback_ava(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'ava'
        # self.TextToSpe.say("Thanks for your cooperation, ava")

    def callback_charlie(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'charlie'
        # self.TextToSpe.say("Thanks for your cooperation, charlie")

    def callback_charlotte(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'charlotte'
        # self.TextToSpe.say("Thanks for your cooperation, charlotte")

    def callback_hunter(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'hunter'
        # self.TextToSpe.say("Thanks for your cooperation, hunter")

    def callback_max(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'max'
        # self.TextToSpe.say("Thanks for your cooperation, max")

    def callback_mia(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'mia'
        # self.TextToSpe.say("Thanks for your cooperation, mia")

    def callback_olivia(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'olivia'
        # self.TextToSpe.say("Thanks for your cooperation, olivia")

    def callback_parker(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'parker'
        # self.TextToSpe.say("Thanks for your cooperation, parker")

    def callback_sam(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'sam'
        # self.TextToSpe.say("Thanks for your cooperation, sam")

    def callback_jack(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'jack'
        # self.TextToSpe.say("Thanks for your cooperation, jack")


    def callback_noah(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'noah'
        # self.TextToSpe.say("Thanks for your cooperation, noah")

    def callback_oliver(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'oliver'
        # self.TextToSpe.say("Thanks for your cooperation, oliver")

    def callback_thomas(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'thomas'
        # self.TextToSpe.say("Thanks for your cooperation, thomas")

    def callback_william(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.ans = 'william'
        # self.TextToSpe.say("Thanks for your cooperation, william")
