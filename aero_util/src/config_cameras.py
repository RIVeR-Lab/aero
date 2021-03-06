#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client
import readline

def rlinput(prompt, prefill=''):
   readline.set_startup_hook(lambda: readline.insert_text(prefill))
   try:
      return raw_input(prompt)
   finally:
      readline.set_startup_hook()

def main():
    rospy.init_node('aero_config_cameras', anonymous=True)

    lower_left = dynamic_reconfigure.client.Client('/aero/lower_stereo/left_camera')
    lower_right = dynamic_reconfigure.client.Client('/aero/lower_stereo/right_camera')

    upper_left = dynamic_reconfigure.client.Client('/aero/upper_stereo/left_camera')
    upper_right = dynamic_reconfigure.client.Client('/aero/upper_stereo/right_camera')

    # Print current configuration
    
    lower_left_config = lower_left.get_configuration();
    print 'lower_left:'
    print '\texposure:',  lower_left_config['auto_exposure'], lower_left_config['exposure'] 
    print '\tgain:',  lower_left_config['auto_gain'], lower_left_config['gain'] 
    print '\twhitebalance:', lower_left_config['auto_whitebalance'], lower_left_config['whitebalance_red'], lower_left_config['whitebalance_blue']

    lower_right_config = lower_right.get_configuration();
    print 'lower_right:'
    print '\texposure:',  lower_right_config['auto_exposure'], lower_right_config['exposure'] 
    print '\tgain:',  lower_right_config['auto_gain'], lower_right_config['gain'] 
    print '\twhitebalance:', lower_right_config['auto_whitebalance'], lower_right_config['whitebalance_red'], lower_right_config['whitebalance_blue']

    upper_left_config = upper_left.get_configuration();
    print 'upper_left:'
    print '\texposure:',  upper_left_config['auto_exposure'], upper_left_config['exposure'] 
    print '\tgain:',  upper_left_config['auto_gain'], upper_left_config['gain'] 
    print '\twhitebalance:', upper_left_config['auto_whitebalance'], upper_left_config['whitebalance_red'], upper_left_config['whitebalance_blue']

    upper_right_config = upper_right.get_configuration();
    print 'upper_right:'
    print '\texposure:',  upper_right_config['auto_exposure'], upper_right_config['exposure'] 
    print '\tgain:',  upper_right_config['auto_gain'], upper_right_config['gain'] 
    print '\twhitebalance:', upper_right_config['auto_whitebalance'], upper_right_config['whitebalance_red'], upper_right_config['whitebalance_blue']

    # Prompt for new parameters
    lower_exposure = float(rlinput('Lower exposure: ', prefill=str(lower_left_config['exposure'])));
    lower_gain = float(rlinput('Lower gain: ', prefill=str(lower_left_config['gain'])));
    lower_whitebalance_red = float(rlinput('Lower whitebalance red: ', prefill=str(lower_left_config['whitebalance_red'])));
    lower_whitebalance_blue = float(rlinput('Lower whitebalance blue: ', prefill=str(lower_left_config['whitebalance_blue'])));

    upper_exposure = float(rlinput('Upper exposure: ', prefill=str(upper_left_config['exposure'])));
    upper_gain = float(rlinput('Upper gain: ', prefill=str(upper_left_config['gain'])));
    upper_whitebalance_red = float(rlinput('Upper whitebalance red: ', prefill=str(upper_left_config['whitebalance_red'])));
    upper_whitebalance_blue = float(rlinput('Upper whitebalance blue: ', prefill=str(upper_left_config['whitebalance_blue'])));
    
    # Set new parameters
    lower_params = { 'auto_exposure': False, 'exposure' : lower_exposure,
                     'auto_gain': False, 'gain' : lower_gain,
                     'auto_whitebalance': False, 'whitebalance_red' : lower_whitebalance_red, 'whitebalance_blue' : lower_whitebalance_blue }
    lower_left.update_configuration(lower_params)
    lower_right.update_configuration(lower_params)

    upper_params = { 'auto_exposure': False, 'exposure' : upper_exposure,
                     'auto_gain': False, 'gain' : upper_gain,
                     'auto_whitebalance': False, 'whitebalance_red' : upper_whitebalance_red, 'whitebalance_blue' : upper_whitebalance_blue }
    upper_left.update_configuration(upper_params)
    upper_right.update_configuration(upper_params)
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
