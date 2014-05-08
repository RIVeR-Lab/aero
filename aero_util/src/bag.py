#!/usr/bin/env python
from subprocess import call
import sys
import readline

def rlinput(prompt, prefill=''):
   readline.set_startup_hook(lambda: readline.insert_text(prefill))
   try:
      return raw_input(prompt)
   finally:
      readline.set_startup_hook()

def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is one of "yes" or "no".
    """
    valid = {"yes":True,   "y":True,  "ye":True,
             "no":False,     "n":False}
    if default == None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "\
                             "(or 'y' or 'n').\n")

def main():
    tf_options = ' /tf /tf_static'
    lowerstereo_options = ' /aero/lower_stereo/left/image_raw ' \
                          '/aero/lower_stereo/left/camera_info ' \
                          '/aero/lower_stereo/right/image_raw ' \
                          '/aero/lower_stereo/right/camera_info'
    upperstereo_options = ' /aero/upper_stereo/left/image_raw ' \
                          '/aero/upper_stereo/left/camera_info ' \
                          '/aero/upper_stereo/right/image_raw ' \
                          '/aero/upper_stereo/right/camera_info'


    buffer_size = rlinput('Buffer Size: ', prefill='50000')
    bag_options =' record -q -b ' + str(buffer_size) + ' -o ~/aero'

    if query_yes_no('Bag TF?:', default="yes"):
        bag_options = bag_options + tf_options
    
    if query_yes_no('Bag Lower Stereo:', default="yes"):
        bag_options = bag_options + lowerstereo_options

    if query_yes_no('Bag Upper Stereo:', default="yes"):
        bag_options = bag_options + upperstereo_options

    call("rosbag" +  bag_options, shell=True) 

if __name__ == "__main__":
	main()
