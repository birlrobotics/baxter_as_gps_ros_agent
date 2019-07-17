import rospy

def callback(trial_command):
    rospy.logdebug('receive trial command: %s'%trial_command)
