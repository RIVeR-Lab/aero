#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client

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

    lower_right_config = lower_right.get_configuration();
    print 'lower_right:'
    print '\texposure:',  lower_right_config['auto_exposure'], lower_right_config['exposure'] 
    print '\tgain:',  lower_right_config['auto_gain'], lower_right_config['gain'] 


    upper_left_config = upper_left.get_configuration();
    print 'upper_left:'
    print '\texposure:',  upper_left_config['auto_exposure'], upper_left_config['exposure'] 
    print '\tgain:',  upper_left_config['auto_gain'], upper_left_config['gain'] 

    upper_right_config = upper_right.get_configuration();
    print 'upper_right:'
    print '\texposure:',  upper_right_config['auto_exposure'], upper_right_config['exposure'] 
    print '\tgain:',  upper_right_config['auto_gain'], upper_right_config['gain'] 

    # Prompt for new parameters
    lower_exposure = float(input('Lower exposure: '));
    lower_gain = float(input('Lower gain: '));

    upper_exposure = float(input('Upper exposure: '));
    upper_gain = float(input('Upper gain: '));
    
    # Set new parameters
    lower_params = { 'auto_exposure': False, 'exposure' : lower_exposure,
                     'auto_gain': False, 'gain' : lower_gain }
    lower_left.update_configuration(lower_params)
    lower_right.update_configuration(lower_params)

    upper_params = { 'auto_exposure': False, 'exposure' : upper_exposure,
                     'auto_gain': False, 'gain' : upper_gain }
    upper_left.update_configuration(upper_params)
    upper_right.update_configuration(upper_params)
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass