
#
# Covariance matrices for the IMU and Wheel odometries
#
#
#
#

ODOM_COV_WHEEL= [ 1e-4, 0, 0, 0, 0, 0,
			      0, 1e-4, 0, 0, 0, 0,
			      0, 0, 1e-4, 0, 0, 0,
			      0, 0, 0, 1e-3, 0, 0,
			      0, 0, 0, 0, 1e-3, 0,
			      0, 0, 0, 0, 0, 1e-3]
			      
			      
TWIST_COV_WHEEL= [ 1e-9, 0, 0, 0, 0, 0,
			      0, 1e-4, 0, 0, 0, 0,
			      0, 0, 1e-4, 0, 0, 0,
			      0, 0, 0, 1e-1, 0, 0,
			      0, 0, 0, 0, 1e-1, 0,
			      0, 0, 0, 0, 0, 1e-1]
			      
# roll pitch yaw			      
ODOM_COV_IMU= [ 4e-4, 0, 0,
			    0, 4e-4, 0,
			    0, 0, 4e-4]
			    
ORIENTATION_COV_IMU = [-1, 0, 0,
						0, 0, 0,
						0 ,0, 0 ]

ANGULAR_VEL_COV_IMU= [ 4e-4, 0, 0,
				0, 4e-4, 0,
				0, 0, 4e-4]

ACCELERATION_COV_IMU= [ 4e-4, 0, 0,
			    0, 4e-4, 0,
			    0, 0, 4e-4]
