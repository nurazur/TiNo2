#!/usr/bin/env python

#Script evaluates FEC encoded messages with Vcc, counter, Temperature and Humidity

import serial
import time
import datetime
import sys
import os
import string
import struct


if os.name == 'nt':
    import msvcrt
else:
    import termios

import atexit


#t = time.strftime('%W %a %d.%m.%Y %H:%M:%S', time.localtime(time.time()))
#t = time.strftime('%a %d.%m.%Y %H:%M:%S', time.localtime(time.time()))


BAUDRATE= 230400
#SERIAL_PORT = '/dev/ttyAMA0'
SERIAL_PORT = 'COM4'


#default filename for valid records
filename = "tinyRx_record.csv"

#filename for error packets
errorlogfile = "tinyRx_error.csv"

#filename for the last log
last_log_filename = 'last_tinytx_log.txt'

#default directory for logfiles
#verzeichnis = '/var/www/logfiles/'
verzeichnis = ''


#default directory for temporary files
tmp_verzeichnis = ''

#list_of_nodes = [22,2,9, 7, 6, 78, 15, 17, 19, 23, 24, 25, 27, 28, 32, 60, 61,62,63,65,66,68,69, 143, 151, 155, 160, 188]
list_of_nodes = [6,7,22]

##########      Terminal Funktions - OS dependent       #########
if os.name == 'nt': # windows OS
    # switch to normal terminal
    def set_normal_term():
        pass

    # switch to unbuffered terminal
    def set_curses_term():
        pass

    def putch(ch):
        msvcrt.putch(ch)

    def getch():
        return msvcrt.getch()

    def getche():
        return msvcrt.getche()

    def kbhit():
        return msvcrt.kbhit()

else:
    # switch to normal terminal
    def set_normal_term():
        termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)

    # switch to unbuffered terminal
    def set_curses_term():
        termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)

    def putch(ch):
        if PY2:
            sys.stdout.write(ch)
        elif PY3:
            if type(ch) is str:
                sys.stdout.write(ch)
            elif type(ch) is bytes:
                sys.stdout.write(ch.decode())
        sys.stdout.flush()

    def getch():
        if PY2:
            return sys.stdin.read(1)
        else:
            return sys.stdin.read(1).encode()

    def getche():
        ch = getch()
        putch(ch)
        return ch

    def kbhit():
        dr,dw,de = select([sys.stdin], [], [], 0)
        #return dr <> []  # valid in Python 2.7 not valid in Python 3
        if not dr == []:
            return True
        return False


#key must be a bytes array
def extract_data(key, results):
    key_result_pair =''
    if key in results.keys():
        key_result_pair = ",%s,%s" % (key.decode() , results[key].decode())
    return key_result_pair


def write_logfile(logfilename, msg):
    try:
        logfile=open(logfilename,'a')
        logfile.write(msg)
        logfile.write('\n')
        logfile.close()
    except IOError as e:
        print(e)

def record_filename(loctime):
    #th_week = int ( time.strftime('%W', loctime) ) + 1
    iso = datetime.date.fromtimestamp(time.time()).isocalendar()
    th_week = iso[1]
    th_year = iso[0]
    filename = '%s%s_%02i_tinyrx_log.csv' % ( verzeichnis, th_year, th_week )
    return filename

def ftp_upload(filename, host, user, password, dest_dir):
    msg =''
    try:
        ftp = ftplib.FTP(host, user, password, timeout=5)
        msg = ftp.storlines("STOR " + filename, open(filename))
    except:
        return sys.exc_info()[1]
    ftp.quit()
    return msg



#### Main Program begins here ####
last_log_dict= {}
node = 'none'
rssi = 'none'
results ={}

os.environ['TZ'] = 'Europe/Paris'

######### Python version ########
PY2 = (sys.version_info.major==2)
PY3 = (sys.version_info.major==3)




#fill last_log_dict with existing entries
fn = '%s%s' % (tmp_verzeichnis, last_log_filename)
if os.path.exists(fn):
    f = open(fn, 'r')
    thefile = f.read().split('\n')
    f.close()
    for line in thefile:
        sensor = line.split(',')
        if len(sensor) > 3:
            last_log_dict[sensor[4]] = line


baudrate = BAUDRATE
argc=len(sys.argv)
if argc > 1:
    baudrate = sys.argv[1]
#Oeffne Seriellen Port
#port = serial.Serial('/dev/ttyAMA0', baudrate)
port = serial.Serial(SERIAL_PORT, baudrate)

# timeout?
if not port.isOpen():
    print ("cannot connect to serial port.")
    sys.exit(1)

while (True):

    if kbhit():
        ch = getch()
        if (ch ==b'q'):
            port.close()
            break # exit program

    if port.inWaiting()>0:
        line_raw = port.readline()
    else:
        time.sleep(0.05)
        continue

    line = line_raw[:-2]
    #print line
    loctime = time.localtime(time.time())
    zeit = time.strftime('%a,%d.%m.%Y,%H:%M:%S', loctime)
    is_a_valid_packet = True

    node =b''

    lst  = line.split(b",")
    #print(lst)
    # old style message format
    if len(lst) > 3:
        node = lst[1]
        token = lst[2].split('=')
        if len(token) > 1:
            rssi = float(token[1])
        else: # protocol error
            print (line.decode())
            continue
        #msg = lst[3].split(':')[1]
        msg = lst[3].split(':')
        if len(msg) >1:
            msg = msg[1]
        else: #no message body
            print ("no message body")
            continue
        res_str = "%s,n,%s,s,%.1f" % (zeit, node, rssi)
        if len(lst) >4:
            feistr = lst[4].split(b'=')
            if len(feistr) > 1:
                fei = int(feistr[1])
                res_str = res_str + ",a,%i" % (fei)
            else:
                pass

        payload = msg.split(';')[:-1] # strip off the last ''
        datalen = len(payload)


        if "OK" in payload[0]:
            is_a_valid_packet = True
            if datalen > 1:
                res_str = "%s,v,%s" % (res_str, payload[1])
            if datalen > 2:
                res_str = "%s,c,%s" % (res_str, payload[2])
            if datalen > 3:
                res_str = "%s,t,%s" % (res_str, payload[3])
            if datalen > 4:
                try:
                    hum = float(payload[4])
                    res_str = "%s,h,%.1f" % (res_str, hum)
                except:
                    pass
            if datalen > 5:
                flags = int(payload[5],16)
                res_str = "%s,f,%x" % (res_str, flags)
        else:
            is_a_valid_packet = False


        if int(node) in list_of_nodes:
            last_log_dict[node] = res_str
            last_log = open(tmp_verzeichnis + last_log_filename,'w')
            for key in last_log_dict:
                last_log.write(last_log_dict[key] + '\n')
            last_log.close()
            #ftp_upload(fn, '192.168.0.16', 'pi', 'andreas', '/var/tmp')




    #TiNo 2.0 and later
    else:
        try:
            lst = line.split(b' ')
            if len(lst) >1:
                node=lst[0].decode()
                items = lst[1].split(b'&')
                results.clear()
                for param in items:
                    param_value_Pair = param.split(b'=')
                    #print param_value_Pair
                    if len(param_value_Pair) >1:
                        results[param_value_Pair[0]] = param_value_Pair[1]

                #res_str = "%s,n,%s,s,%.1f"    % (zeit, node, float(results['rssi'])/10)

                res_str = "%s,n,%s"    % (zeit, node)
                #print (results.keys())
                if b'rssi' in results.keys():
                    res_str += ",s,%.1f" % (float(results[b'rssi'])/10)
                else:
                    res_str  += ",s,"

                fei_str=''
                if b'fo' in results.keys():
                    fei_str = ',a,' + results[b'fo']

                if b'f' in results.keys():
                    flags = int(results[b'f'],16)
                else:
                    is_a_valid_packet = False
                    print (line.decode())
                    #print("line is a invalid packet")
                    continue # string does not contain a flag byte ==> invalid

                hum_str = b''
                if b'h' in results.keys():
                    hum = float(results[b'h'])/100.0
                    hum_str = ",h,%.1f" %(hum)
                res_str = "%s%s%s%s%s%s%s%s" % (res_str,
                fei_str,
                #extract_data(b'fo', results),
                extract_data(b'v', results),
                extract_data(b'c', results),
                extract_data(b't', results),
                hum_str,
                #extract_data(b'h', results),
                extract_data(b'p', results),
                extract_data(b'f', results))

                if int(node) in list_of_nodes:
                    last_log_dict[node] = res_str
                    last_log = open(tmp_verzeichnis + last_log_filename,'w')
                    for key in last_log_dict:
                        last_log.write(last_log_dict[key] + '\n')
                    last_log.close()
        except:
            is_a_valid_packet = False
            print(line.decode())
            print (sys.exc_info())
            continue


    if node is not '':
        if int(node) > 0:
            print (res_str)

        if int(node) in list_of_nodes and is_a_valid_packet:
            print (line_raw.decode())
            write_logfile(record_filename(loctime), res_str)
        elif int(node) > 0:
            write_logfile(tmp_verzeichnis + errorlogfile, res_str)




