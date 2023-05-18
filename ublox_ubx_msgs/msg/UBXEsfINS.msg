# this message contains a UBX-ESF-INS (0x10 0x15) record
# Vehicle dynamics information
std_msgs/Header header # Header timestamp should be acquisition time

uint8 version           # message version (0x02 for this version)
bool x_ang_rate_valid      # Compensated x-axis angular rate data validity
bool y_ang_rate_valid      # Compensated y-axis angular rate data validity
bool z_ang_rate_valid      # Compensated z-axis angular rate data validity
bool x_accel_valid         # Compensated x-axis acceleration data validity
bool y_accel_valid         # Compensated y-axis acceleration data validity
bool z_accel_valid         # Compensated z-axis acceleration data validity

uint32 itow             #  ms - GPS Time of week of the navigation epoch

float32 x_ang_rate  # Compensated x-axis angular rate, rad/s
float32 y_ang_rate  # Compensated y-axis angular rate, rad/s
float32 z_ang_rate  # Compensated z-axis angular rate, rad/s

float32 x_accel     # Compensated x-axis acceleration (gravity-free), m/s^2
float32 y_accel     # Compensated y-axis acceleration (gravity-free), m/s^2
float32 z_accel     # Compensated z-axis acceleration (gravity-free), m/s^2
