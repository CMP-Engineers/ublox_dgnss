uint8 sensor_data_type      # see sensor data type in the integration manual
bool used                   # if set, sensor data is used for the current fusion solution
bool ready                  # if set, sensor is set up but not used for sensor fusion

uint8 CALIB_STATUS_NOT_CALIBRATED = 0b00
uint8 CALIB_STATUS_CALIBRATING = 0b01
uint8 CALIB_STATUS_CALIBRATED0 = 0b10
uint8 CALIB_STATUS_CALIBRATED1 = 0b11
uint8 calib_status          # 0b00: sensor not calibrated 0x01: sensor is calibrating,
                            # 0b10/0b11 sensor is calibrated. Good dead recking only possible
                            # when all used sensors are calibrated

uint8 TIME_STATUS_NO_DATA = 0b00
uint8 TIME_STATUS_FIRST_BYTE_USED = 0b01
uint8 TIME_STATUS_EVENT_INPUT = 0b10
uint8 TIME_STATUS_TTAG_PROVIDED = 0b11
uint8 time_status           # 0b00: No Data, 0x01: Reception of the first byte used to tag measurement
                            # 0b10: Event input used to tag the measurement
                            # 0b11: Time tag provided with the data

uint8 freq                  # observation frequency

bool fault_bad_meas         # bad measurements detected
bool fault_bad_ttag         # bad measurement time-tages detected
bool fault_missing_meas     # Missing or time-misaligned measurements detected
bool fault_noisy_meas       # high measurement noise-level detected