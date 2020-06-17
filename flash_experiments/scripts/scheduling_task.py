#!/usr/bin/env python3

import math
import rospy

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont

from task import SchedulingTask
from utils.constants import *
from config import *

from flash_behaviors.msg import Speech


class Task(SchedulingTask):

    def __init__(self):

        # Initialize node.
        rospy.init_node('scheduling_task_node')

        # Speech publisher.
        self.speak_pub = rospy.Publisher('/flash_robot/say', Speech, queue_size=1)

        super().__init__()


    def show_message(self, msg, exception=False):

        if msg is not None:

            if self.succeeded:

                # Create popup window.
                popup = tk.Tk()

                # Determine window location based on message length.
                padx = 20
                pady = 20
                
                font = tkfont.Font(family='Verdana', size=16, weight='normal')
                msg_len = font.measure(msg) + padx + pady

                left_margin = math.floor((WIDTH - msg_len)/2)

                popup_x = SCREEN_X + left_margin # leftmost coordinate

                popup.geometry('+{}+250'.format(popup_x))
                popup.wm_attributes('-topmost', True)

                popup.wm_title('Congratulations!')

                label = ttk.Label(popup,
                                  text=msg+'\n',
                                  font=font)
                label.pack(padx=padx, pady=pady)

                button = ttk.Button(popup, text='Accept', command=popup.destroy)
                button.pack(pady=(0, pady))

                popup.mainloop()

            else:

                speech = Speech()

                speech.text = msg
                speech.intensity = 2

                self.speak_pub.publish(speech)

                self.conflict_ack = True


if __name__ == '__main__':

    game = Task()
    game.run()