# this message contains a UBX-ESF-ALG (0x10 0x14) record
# IMU alignment information
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow             #  ms - GPS Time of week of the navigation epoch
uint8 version           # message version (0x02 for this version)

bool auto_mnt_alg_running   # Automatic IMU-mount alignment is running

uint8 ALG_STATUS_USER_DEFINED_ANGLES = 0
uint8 ALG_STATUS_ROLL_PITCH_ONGOING = 1
uint8 ALG_STATUS_ROLL_PITCH_YAW_ONGOING = 2
uint8 ALG_STATUS_COARSE_ALG = 3
uint8 ALG_STATUS_FINE_ALG = 4
uint8 alg_status    # Status of the IMU-mount alignment
                # 0: user-defined/fixed angles are used, 
                # 1: IMU-mount roll/pitch angles alignment is ongoing, 
                # 2: IMU-mount roll/pitch/yaw angles alignment is ongoing, 
                # 3: coarse IMU-mount alignment are used, 
                # 4: fine IMU-mount alignment are used

bool alg_tilt_error   # IMU-mount tilt (roll and/or pitch) alignment error
bool alg_yaw_error    # IMU-mount yaw alignment error
bool alg_euler_error  # IMU-mount misalignment Euler angle singularity error

float32 yaw         # IMU-mount yaw angle, radians [0, 2*pi]
float32 pitch       # IMU-mount pitch angle, degrees [-pi/2, pi/2]
float32 roll        # IMU-mount roll angle, degrees [-pi, pi]
