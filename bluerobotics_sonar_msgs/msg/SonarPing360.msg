# Header
std_msgs/Header header

# Units: m; The current return distance determined for the most recent acoustic measurement
float32 distance

# Operating mode (1 for Ping360)
uint8 mode

# Analog gain setting (0 = low, 1 = normal, 2 = high)
uint8 gain_setting

# Units: gradian; Head angle
uint16 angle

# Units: microsecond; Acoustic transmission duration (1~1000 microseconds)
uint16 transmit_duration

# Time interval between individual signal intensity samples in 25nsec increments (80 to 40000 == 2 microseconds to 1000 microseconds)
uint16 sample_period

# Units: kHz; Acoustic operating frequency. Frequency range is 500kHz to 1000kHz, however it is only practical to use say 650kHz to 850kHz due to the narrow bandwidth of the acoustic receiver.
uint16 transmit_frequency

# Units: m; The length of the scan region
float32 range

# 8 bit binary data array representing sonar echo strength
uint8[] profile_data