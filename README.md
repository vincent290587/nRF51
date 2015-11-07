# nRF51

This program lets the nRF51 chip send via serial interface messages
to another MCU.
These messages give information about HRM, bike C&S and ANCS (iPhone notifications).
They come in the following format:
$HRM,bpm,rr_interval\r\n (with rr_interval in milliseconds)
$CAD,crankset_rpm,wheel_speed\r\n (with whee_speed in km/h for a 2000mm circum wheel)
$ANCS,msg_type,msg_text\r\n
