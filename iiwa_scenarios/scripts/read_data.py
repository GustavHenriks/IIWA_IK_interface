#!/usr/bin/env python
import numpy as np
import os

path = '/home/gustavhenriks/catkin_ws_ik_test/src/IIWA_IK_interface/iiwa_scenarios/scripts/data/'
files = os.listdir(path)
paths = [os.path.join(path, basename) for basename in files]
latest_file = max(paths, key=os.path.getctime)
print("Opening file : \n:" + str(latest_file))
X = np.load(latest_file)
print(len(X))
print(X[-10:])