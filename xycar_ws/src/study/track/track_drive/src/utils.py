import cv2
import os
import numpy as np
import matplotlib.pyplot as plt
import glob
import time




def logging_time(original_fn):
    def wrapper_fn(*args, **kwargs):
        start_time = time.time()
        result = original_fn(*args, **kwargs)
        end_time = time.time()
        # print("WorkingTime[{}]: {} sec".format(original_fn.__name__, end_time-start_time))
        return result
    return wrapper_fn


def draw_dot(x_list,y_list):
    plt.scatter( x_list,  y_list, c='g',marker='.',label ="points")  # 각 점 표시
    plt.draw()
    plt.pause(0.001)