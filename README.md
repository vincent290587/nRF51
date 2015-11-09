# nRF51422

This program lets the nRF51422 chip send via serial interface messages
to another MCU.<br>
These messages give information about HRM, bike C&S and ANCS (iPhone notifications).
They come in the following format:<br><br>
$HRM,bpm,rr_interval\r\n (with rr_interval in milliseconds)<br>
$CAD,crankset_rpm,wheel_speed\r\n (with wheel_speed in km/h for a 2000mm circum wheel)<br>
$ANCS,msg_type,msg_text\r\n<br>
