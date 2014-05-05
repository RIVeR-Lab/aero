#!/usr/bin/env python
import subprocess
import sys

def main():
    subprocess.Popen(['rosrun image_view image_view image:=/aero/upper_stereo/left/image_raw'], shell=True)
    subprocess.Popen(['rosrun image_view image_view image:=/aero/upper_stereo/right/image_raw'], shell=True)
    subprocess.Popen(['rosrun image_view image_view image:=/aero/lower_stereo/left/image_raw'], shell=True)
    subprocess.Popen(['rosrun image_view image_view image:=/aero/lower_stereo/right/image_raw'], shell=True)

if __name__ == '__main__':
    main()
