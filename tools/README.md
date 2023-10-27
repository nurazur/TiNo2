# Configuration Tools

## tino2cal_v02.py
This is a python3 script that allows to connect to a TiNo2 and to configure it. Configuration Data are stored in EEPROM and are password protected. The user can choose a password at compile time. The password ist set in the sensoe sketch. In order to ease access, I have stored the standard password in the python skript which is used when starting the script with the -pwd commandline option. Sensor sketches are configured with 57600Bd Baudrate and receiver sketches are configured with 230400Bd Baudrate. The MSDos .bat files demonstrate a typical command line call of this tool. 
The .cfg files are typical configurations for sensors and the receiver. 
Please see the Documentation for Details. 


## tino_log.py 
This python3 Script connects to a TiNo2 receiver and stores the received sensor data in a .csv file. 
