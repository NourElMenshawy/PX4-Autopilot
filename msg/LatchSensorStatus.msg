# LatchSensorStatus.msg

# Message to report latch sensor readings used for determining if the drone is docked in the correct position.
# When docked in the correct position, all latch sensors should be turned off.

uint64 timestamp		# time since system start (microseconds)

# Sensor reading for the first latch sensor.
# 0 indicates no latch
uint8 latchsensor_1

# Sensor reading for the second latch sensor.
# 0 indicates no latch
uint8 latchsensor_2

# Sensor reading for the third latch sensor.
# 0 indicates no latch
uint8 latchsensor_3

# Sensor reading for the fourth latch sensor.
# 0 indicates no latch
uint8 latchsensor_4
