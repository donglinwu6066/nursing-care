#!/usr/bin/env python
import rospy
import os

if __name__ =='__main__':
    for i in range(3):
        os.system("mpg123 ~/nursing_ws/nursing-care/src/move_group_control/src/mp3/Disinfecting_zh.mp3")
        os.system("mpg123 ~/nursing_ws/nursing-care/src/move_group_control/src/mp3/Disinfecting.mp3")
    print("fdsafdsafds")
    #Install the Sox Command Line Utility 
    #https://vitux.com/play-mp3-files-using-the-ubuntu-command-line/
