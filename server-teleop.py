#!usr/bin/python
import time
import math
import cv2
import numpy as np
#import sys
import pygame

import PiBot as PB


# write your command to initialise robot here


pygame.init()
pygame.display.set_mode((10,10))

try:
    print("TELEOP")

    while True:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                print event.type
                if event.key == pygame.K_UP:
                    print("FORWARD")
                    # write your command here
                if event.key == pygame.K_DOWN:
                    print("BACKWARD")
                    # write your command here
                if event.key == pygame.K_LEFT:
                    print "LEFT"
                    # write your command here
                if event.key == pygame.K_RIGHT:
                    print "RIGHT"
                    # write your command here

            if event.type == pygame.KEYUP:
                print "Done"
                # stop the robot on Key up

except KeyboardInterrupt:
    pb.stop()
