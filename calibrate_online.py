import numpy as np
import cv2
import open3d as o3d
from calibrate import *
#from save_ply_for_calibration import *
import time

def empty_func(cam_1_fn, cam_2_fn, cam_1_id = '943222072534', cam_2_id = '912112072667'):
    pass

### Get a some ply files for calibration ###
t_start = 0
N_samples = 4
calibration_files = []

print("starting the calibration procedure......")

for i in range(N_samples):
    print("_________________________________________________")
    print("move the camera to the %s-th calibration position" % str(i+1))    

    print("[PRESS ENTER TO SAVE CALIBRATION DATA]")
    input()

    # do stuff
    cam_1_fn = "data/cam_1_calibration_%s.ply" % str(i+1)
    cam_2_fn = "data/cam_2_calibration_%s.ply" % str(i+1)
    empty_func(cam_1_fn,cam_2_fn, cam_1_id = '943222072534', cam_2_id = '912112072667')
    calibration_files.append([cam_1_fn,cam_2_fn])

    # done! 
    print("saved sample #%s of %s" %(i+1,N_samples))


print(calibration_files)



