
import argparse
import rospy
import sys
import os
import imp

def get_config():
    parser = argparse.ArgumentParser() 
    parser.add_argument("--config", dest='config', type=str, help='path to config file', default=None)

    removed_ros_args = rospy.myargv(sys.argv)
    args, unknown = parser.parse_known_args(removed_ros_args)

    if args.config is None:
        rospy.logerr("config is missing")
        parser.error("config is missing")

    dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
    config_path = os.path.join(dir_of_this_script, '..', '..', 'config', args.config)

    if not os.path.isfile(config_path):
        rospy.logerr("file %s doesn't exist"%config_path)
        parser.error("file %s doesn't exist"%config_path)

    
    config = imp.load_source('config', config_path)

    return config
