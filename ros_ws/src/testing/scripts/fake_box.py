#!/usr/bin/env python

"""
@file fake_box.py
@brief Node to control a fake box

@author Sébastien Darche <sebastien.darche@polymtl.ca>
"""

import rospy

from detection import Detection, Detections

import sys
import select
import termios
import tty
import numpy as np

settings = termios.tcgetattr(sys.stdin)

inc = 0.5  # Increment at each keystroke

# Stored as x, y, rot
mappings_wasd = {'w': np.array([0, -inc, 0, 0]), 's': np.array(
    [0, inc, 0, 0]), 'd': np.array([inc, 0, 0, 0]), 'a':  np.array([-inc, 0, 0, 0]),
    'e': np.array([0, 0, inc, inc]), 'q':  np.array([0, 0, -inc, -inc])}

mappings_zqsd = {'z': np.array([0, -inc, 0, 0]), 's': np.array(
    [0, inc, 0, 0]), 'd': np.array([inc, 0, 0, 0]), 'q':  np.array([-inc, 0, 0, 0]),
    'e': np.array([0, 0, inc, inc]), 'a':  np.array([0, 0, -inc, -inc])}

mappings = mappings_zqsd

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

    key = None
    if rlist:
        key = sys.stdin.read(1)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rospy.init_node("turret_teleop")
    pub = rospy.Publisher('/detection/detections', Detections, queue_size=1)

    enemy = rospy.get_param('/decision/enemy_color', 0)
    order_init = np.array([208., 208., 10., 10.])

    order = order_init.copy()

    try:
        while True:
            key = get_key()
            if key is None:
                pass
            elif key in mappings.keys():
                order += mappings[key]
            elif key == '\x03':
                break
            else:
                order = order_init.copy()
            
            dets = Detections()
            dets.detections.append(Detection(x=order[0], y=order[1], w=order[2], h=order[3], clss=enemy, score=1))

            pub.publish(dets)

    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
