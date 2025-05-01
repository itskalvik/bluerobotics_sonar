# Header
std_msgs/Header header

# Units: m; The current return distance determined for the most recent acoustic measurement
float32 distance

# Units: %; Confidence in the most recent range measurement
uint16 confidence

# Units: us; The acoustic pulse length during acoustic transmission/activation
uint16 transmit_duration

# The pulse/measurement count since boot
uint32 ping_number

# Units: m; The beginning of the scan region in m from the transducer
float32 scan_start

# Units: m; The length of the scan region
float32 scan_length

# The current gain setting. 0: 0.6, 1: 1.8, 2: 5.5, 3: 12.9, 4: 30.2, 5: 66.1, 6: 144
uint32 gain_setting

# An array of return strength measurements taken at regular intervals across the scan region
uint8[] profile_data