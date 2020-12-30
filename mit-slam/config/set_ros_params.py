#!/usr/bin/env python

"""
Set all necessary mit-slam parameters.
Note: some parameters are not set here and are assumed to be the default values.
The default values are defined in ParamUtils.cpp.
"""

import rospy

if __name__ == "__main__":

	# SLAM node parameters
	rospy.set_param('node_loop_rate', 62.5) 
	rospy.set_param('publish_frontend_estimates', True)
	rospy.set_param('downsample_thresh', 200)
	rospy.set_param('loop_match_thresh', 400)
	rospy.set_param('imu_dt', 0.016)


	


