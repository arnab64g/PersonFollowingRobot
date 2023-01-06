import math


def get_max_size(top_left, top_right, bottom_left, bottom_right):
    
    a = math.sqrt(pow(top_left[0] - top_right[0], 2) + pow(top_left[1] - top_right[1], 2))
    b = math.sqrt(pow(bottom_left[0] - bottom_right[0], 2) + pow(bottom_left[1] - bottom_right[1], 2))
    c = math.sqrt(pow(top_left[0] - bottom_left[0], 2) + pow(top_left[1] - bottom_left[1], 2))
    d = math.sqrt(pow(top_right[0] - bottom_right[0], 2) + pow(top_right[1] - bottom_right[1], 2))

    ar = [int(a), int(b), int(c), int(d)]
    
    ar.sort()
    
    return ar[3]


def get_lef_line_right_line(width):
    
    left_line = (width * 7) / 16
    right_line = (width * 9) / 16

    return left_line, right_line


def get_center(a, b):
    
    c = int((a + b)/ 2.0)

    return c