import tkinter as tk
from tkinter import ttk
from tkinter import scrolledtext
import tkinter.messagebox as msgbox
from tkinter.filedialog import asksaveasfile
from tkinter.filedialog import askopenfile
from tkinter.simpledialog import askstring
from tkinter import font
from functools import partial

import serial.tools.list_ports
from serial import Serial
import xml.etree.ElementTree as ET
import csv
import threading
import datetime
import struct
import time
import re
from subprocess import Popen, PIPE, STDOUT, DEVNULL,CREATE_NO_WINDOW
import os

#PassWord = b"WiNW_AzurdelaMer"
PassWord = b"TheQuickBrownFox"

#EEPROM map
mem_s =[
    ['NODEID','B'],
    ['NETWORKID', 'B'],
    ['GATEWAYID', 'B'],
    ['VCCATCAL', 'i'],
    ['VCCADC_CAL', 'i'],
    ['SENDDELAY','i'],
    ['SENSORCONFIG','i'],
    ['FREQ_CENTER','f'],
    ['TXPOWER','B'],
    ['RADIO_T_OFFSET','b'],
    ['USE_RADIO_T_COMP','B'],
    ['REQUESTACK','B'],
    ['LEDCOUNT','B'],
    ['LEDPIN','b'],
    ['LDRPIN','b'],
    ['PIRDATAPIN','b'],
    ['PIRDEADTIME','i'],
    ['TCCSPIN','b'],
    ['RTDPOWERPIN','b'],
    ['RTDCSPIN','b'],
    ['ONEWIREPOWERPIN','b'],
    ['ONEWIREDATAPIN','b'],
    ['I2CPOWERPIN','b'],
    ['PCI0PIN','b'],
    ['PCI0CONFIG','B'],
    ['PCI0GATEWAYID','b'],
    ['PCI1PIN','b'],
    ['PCI1CONFIG','B'],
    ['PCI1GATEWAYID','b'],
    ['PCI2PIN','b'],
    ['PCI2CONFIG','B'],
    ['PCI2GATEWAYID','b'],
    ['PCI3PIN','b'],
    ['PCI3CONFIG','B'],
    ['PCI3GATEWAYID','b'],
    ['USE_CRYSTAL_RTC','b'],
    ['ENCRYPTION_ENABLE','b'],
    ['FEC_ENABLE','b'],
    ['DUMMY','b','r'],
    ['EEPROM_VERSION','b','r'],
    ['SOFTWAREVERSION','i','r'],
    ['TXGAUSS_SHAPING','b'],
    ['SERIAL_ENABLE','b'],
    ['IS_RFM69HW','b'],
    ['PABOOST','b'],
    ['FDEV_STEPS','i'],
    ['CHECKSUM','i','r']
    ]



mem_help=[
    ['NODEID','die Identität des Nodes, mit der sich der Sender im Netwerk\nidentifiziert.Darf im Netwerk nur einmal verwendet werden.'],
    ['NETWORKID', 'technisch ist das das zweite Byte zur Synchronisation.\nMuss für das gesamte Netwerk, also alle Empfänger und Sender, die selbe ID haben.'],
    ['GATEWAYID', 'die Identität des Empfängers.\nNur Empfänger mit dieser ID erkennen diesen Node.'],
    ['VCCATCAL', 'Versorgungsspannung zum Zeitpunkt der Kalibrierung'],
    ['VCCADC_CAL', 'gemessener ADC Wert bei der Spannung die in VCCATCAL hinterlegt ist.'],
    ['SENDDELAY','definiert die Zeit zwischen zwei Sendepaketen in 8 Sekunden Intervallen.\nBeispiele:\n 2 Minuten = 120s/8 = 15\n10 MInuten: 75'],
    ['SENSORCONFIG','Bit Feld mit 16 bits. Jedes Bit definiert ob ein vorgesehener Sensor akiviert ist. Die ersten 8 Bits waren ursprünglich für I2C Sensoren gedacht. Inzwischen\
   werden I2C Sensoren automatisch erkannt. Deshalb sind die ersten 8 Bits nicht aktiv. Die zweiten 8 Bits definieren verschiedene Sensoren, wie Helligkeit, Themoelemente und PT100 Sensoren.\
    Die Liste kann in Zukunft noch weitere Sensoren aufnehmen.'],
    ['FREQ_CENTER','Mittenfrequenz des Radios in MHz.'],
    ['TXPOWER','Sendeleistung in 1dB Schritten\n.Die maximale Sendeleistung ist 31. Das sind in Europa 14 dBm.\nDie Sendeleistung sollte als Kompromiss zwischen Batterieverbrauch und Reichweite\
  optimiert werden. Typischer Wert ist 25 (7 dBm) womit die allermeisten Anwendungsfälle abgedeckt werden können.'],
    ['RADIO_T_OFFSET','Der Radio Chip (RFM69HCW) hat einen Temperatur Sensor in 1 degC Schritten. Die Ausgabe ist ohne Offset Einstellung sehr ungenau. Der Offset kann hier in 1/10 Grad Schritten eingestellt werden.'],
    ['USE_RADIO_T_COMP','Aktiviere den Offset RADIO_T_OFFSET für die Temperaturmessung des Radios.\nTemperaturkompensation ist in dieser Firmware nicht implementiert.'],
    ['REQUESTACK','1 = Der Sender möchte eine Rückmeldung vom Empfänger dass die Nachricht angekommen ist.\n0 = keine Rückmeldung vom Empfänger.'],
    ['LEDCOUNT','Anzahl der LED Doppel-Blinks nach dem Start.\n0 = nie\n 255 = immer\n\nDas ist eine Massnahme zum Strom sparen. Normalerweise ist die LED ausserhalb des Gehäuses nicht sichtbar. Ein Wert von 3 zeigt an dass die LED bei 3 Paketen nach dem Start blinkt, füe Testzwecke. Danach blinkt sie nichr mehr.'],
    ['LEDPIN','GPIO Pin nach Arduino für die LED. Normalerweise 19.'],
    ['LDRPIN','Dies muss ein analoger GPIO nach Arduino sein.\
    An diesem Pin wird ein LDR (Light Dependent Resistor) angeschlossen,\
    welcher in SENSORCONFIG aktiviert wird. Mögliche GPIOs sind: 15, 16, 17, 18. Theoretisch GPIOs 22,23,24,25, aber diese GPIOs sind bei nicht bei allen TiNo2 zugänglich.'],
    ['PIRDATAPIN','Digitaler GPIO Eingang des Bewegungsmelders, wenn angeschlossen.'],
    ['PIRDEADTIME','Zeit für die der Bewegungsmelder nach einer Auslösung stumm geschalten wird. Die Zeit wird in 8 Sekunden Intervallen angegeben. Typisch sind ca. 3 1/2 Minuten, damit ist der Wert ca. 25'],
    ['TCCSPIN','CS Pin für den SPI Bus eines ICs das Thermoelemente misst'],
    ['RTDPOWERPIN','VDD Pin für ein IC das PT100 Sensoren misst'],
    ['RTDCSPIN','CS Pin für den SPI Bus eines ICs das PT100 Sensoren misst'],
    ['ONEWIREPOWERPIN','DS18B20 VDD Pin'],
    ['ONEWIREDATAPIN','DS18B20 DATA Pin'],
    ['I2CPOWERPIN','VDD von I2C Komponenten'],
    ['PCI0PIN','GPIO Pin der einen Interrupt auslöst.'],
    ['PCI0CONFIG','Bedingung die einen Interrupt auslöst.'],
    ['PCI0GATEWAYID','ID des Empfängers zu dem das Paket im Falle eines Interrupts gesendet wird'],
    ['PCI1PIN','GPIO Pin der einen Interrupt auslöst.'],
    ['PCI1CONFIG','Bedingung die einen Interrupt auslöst.'],
    ['PCI1GATEWAYID','ID des Empfängers zu dem das Paket im Falle eines Interrupts gesendet wird'],
    ['PCI2PIN','GPIO Pin der einen Interrupt auslöst.'],
    ['PCI2CONFIG','Bedingung die einen Interrupt auslöst.'],
    ['PCI2GATEWAYID','ID des Empfängers zu dem das Paket im Falle eines Interrupts gesendet wird'],
    ['PCI3PIN','GPIO Pin der einen Interrupt auslöst.'],
    ['PCI3CONFIG','Bedingung die einen Interrupt auslöst.'],
    ['PCI3GATEWAYID','ID des Empfängers zu dem das Paket im Falle eines Interrupts gesendet wird'],
    ['USE_CRYSTAL_RTC','Wenn ein Quarz auf dem TiNo2 verbaut ist: 1, ansonsten 0'],
    ['ENCRYPTION_ENABLE','1 = Pakete verschlüsseln (empfohlen)\n0 = Pakete unverschlüsselt versenden.'],
    ['FEC_ENABLE','Forward Error Correction.\n1 = verwenden\n0 = nicht verwenden'],
    ['DUMMY','Compiler Flags'],
    ['EEPROM_VERSION','liefert die Version der EEPROM Struktur. Nur lesbar.'],
    ['SOFTWAREVERSION','Liefert die Version der Software. Wird nicht konsequent gepflegt, daher ohne Bedeutung. Nur lesbar.'],
    ['TXGAUSS_SHAPING','Filterung des Sendepakets mit einem Bandpass Filter. Normalerweise 0 (kein Shaping)'],
    ['SERIAL_ENABLE','Auf 1 setzen. Die Funktionalität ist nicht vollständig implementiert.'],
    ['IS_RFM69HW','Tino2 verwendet ausschliesslich den RFM69HCW, deshalb muss dieser Wert = 1 sein.'],
    ['PABOOST','0=normal mode\nExpert Modes:\n1= use PA1 and PA2 with activated high-Power registers\n2= use PA1 and PA2 but do not use high-power register'],
    ['FDEV_STEPS','Offset Kalibrierung der HF Frequenz. Auf 0 setzen, die Bandbreite des Empfängers ist auf jeden Fall ausreichend.'],
    ['CHECKSUM','Prüfsumme, nicht editierbar.']
]


mem_help_eng=[
    ['NODEID','The identity of the node with which the sender identifies itself in the network. May only be used once in the network.'],
    ['NETWORKID', 'Technically, this is the second byte used for synchronization.\nIt must have the same ID for the entire network, i.e., all receivers and senders.'],
    ['GATEWAYID', 'The identity of the receiver to which messages are sent.\nOnly receivers with this ID will recognize this node.'],
    ['VCCATCAL', 'Supply voltage at the time of calibration'],
    ['VCCADC_CAL', 'measured ADC value at the voltage that is stored in VCCATCAL.'],
    ['SENDDELAY','defines the time between transmit packets in 8seconds intervalls.\nExamples:\n 2 minutes = 120s/8 = 15\n10 minutes: 75'],
    ['SENSORCONFIG','Bit field with 16 bits.\nEach Bit enables/disables a specified sensor.\nSince TiNo2 detects I2C Sensors automatically,\nthe first 8 placeholders are ignored by the device.\nKeep them unchecked.'],
    ['FREQ_CENTER','Radio Center frequency in MHz.'],
    ['TXPOWER','Transmit power in 1dB steps.\nMaximum power level is 31, corresponding to 14 dBm.\nTransmit power should be optimized between range and current consumption.\nTypically TXPOWER is 25 (8 dBm).'],
    ['RADIO_T_OFFSET','For advanced users.\nThe Radio Chip (RFM69HCW) includes a temperatures sensor with a resolution of 1 degC.\
     Its output is by default inaccurate and needs offset calibration. Offset can be adjusted here in 1/10 degrees steps.'],
    ['USE_RADIO_T_COMP','For advanced users.\nEnables the offset compensation of the Radio Temperature sensor.'],
    ['REQUESTACK','1 = sender requests acknoledgement from receiver.\n0 = no acknoledgement from receiver.'],
    ['LEDCOUNT','number of LED double-blips after start.\n0 = never\n255 = always\n\nServes as measure to safe power. Usually the LED is not visible outside the enclosure.\n\
    For test purposes the number is typically set to 3'],
    ['LEDPIN','GPIO Pin (Arduino numbering) of the LED. Usually 19.'],
    ['LDRPIN','Analog GPIO Pin (Arduino numbering).\
    Connect a LDR (Light Dependent Resistor) vs. GND,\
    which is enabled in SENSORCONFIG register. Possible GPIOs are: 15, 16, 17, 18, 22, 23, 24, 25. Not all of these ports are available on some TiNo2 boards.'],
    ['PIRDATAPIN','Digital GPIO of the motion detector, if connected.'],
    ['PIRDEADTIME','period of time the motion detector is disabled after triggering, in 8 second steps.\n\
Typical time is approx. 3 1/2 minutes, so the value is 25'],
    ['TCCSPIN','Chip SElect (CS) Pin of the SPI Bus for a connected thermocouple IC'],
    ['RTDPOWERPIN','VDD Pin for a PT100/PT1000 sensor'],
    ['RTDCSPIN','CS Pin of the SPI Bus for a PT100/PT1000 sensor'],
    ['ONEWIREPOWERPIN','DS18B20 VDD Pin\nTo use DS18B20 Measurements, it must be enabled in the SENSORCONFIG register'],
    ['ONEWIREDATAPIN','DS18B20 DATA Pin\nTo use DS18B20 Measurements, it must be enabled in the SENSORCONFIG register'],
    ['I2CPOWERPIN','VDD of I2C components'],
    ['PCI0PIN','GPIO Pin triggering a interrupt.'],
    ['PCI0CONFIG','Conditions for triggering a interrupt.'],
    ['PCI0GATEWAYID','target ID of the receiver in case of PCI0 trigger'],
    ['PCI1PIN','GPIO Pin triggering a interrupt.'],
    ['PCI1CONFIG','Conditions for triggering a interrupt.'],
    ['PCI1GATEWAYID','target ID of the receiver in case of PCI1 trigger'],
    ['PCI2PIN','GPIO Pin triggering a interrupt.'],
    ['PCI2CONFIG','Conditions for triggering a interrupt.'],
    ['PCI2GATEWAYID','target ID of the receiver in case of PCI2 trigger'],
    ['PCI3PIN','GPIO Pin triggering a interrupt.'],
    ['PCI3CONFIG','Conditions for triggering a interrupt.'],
    ['PCI3GATEWAYID','target ID of the receiver in case of PCI3 trigger'],
    ['USE_CRYSTAL_RTC','if a crytal is populated on TiNo board: 1, else 0'],
    ['ENCRYPTION_ENABLE','1 = encrypt packets (recommended)\n0 = send packets unencrypted.'],
    ['FEC_ENABLE','Forward Error Correction.\n1 = enable\n0 = disable'],
    ['DUMMY','parameter not used'],
    ['EEPROM_VERSION','read only parameter'],
    ['SOFTWAREVERSION','read only parameter.'],
    ['TXGAUSS_SHAPING','Advanced use. Normally 0 (no Shaping)'],
    ['SERIAL_ENABLE','set to 1'],
    ['IS_RFM69HW','set to 1. Tino2 uses RFM69HCW only'],
    ['PABOOST','0=normal mode\nExpert Modes:\n1= use PA1 and PA2 and activate high-Power registers\n2= use PA1 and PA2 but do not use high-power registers'],
    ['FDEV_STEPS','Offset calibration of the center frequency.\nSet to 0, because the receiver bandwidth is sufficient in any case.'],
    ['CHECKSUM','read only parameter\nchecksum is automatically calculated.']
]


command_msg = 'currently supported commands:\n\
c    Measure ADC and store in EEPROM\n\
cs   Verify checksum\n\
m    Measure VCC with calibrated values\n\
r    Read from EEPROM. Syntax: "r(ead),<addr>"\n\
ri   Read 16 bit integer from EEPROM. Syntax: "ri(ead),<addr>"\n\
rf   Read float from EEPROM. Syntax: "ri(ead),<addr>"\n\
s    request checksum update and store in EEPROM.\n\
t    send a test RF packet\n\
tt   request temperature reading from radio chip\n\
w    Write value to to EEPROM. Syntax:\n      "w(rite),<addr>,<value>"\n\
wf   write float value to EEPROM. Syntax:\n      "wf,<addr>,<value>"\n\
wu   write unsigned int value to EEPROM. Syntax:\n      "wu,<addr>,<value>"\n\
x    exit calibration mode and continue with loop()\
'

sensors =[
            ["I2C_0"],
            ["I2C_1"],
            ["I2C_2"],
            ["I2C_3"],
            ["I2C_4"],
            ["I2C_5"],
            ["I2C_6"],
            ["PIR", "Motion Detector",'PIRDATAPIN','VBATT'],
            ["DS18B20","Temperature", 'ONEWIREDATAPIN', 'ONEWIREPOWERPIN'],
            ["MAX6675","Thermocouple", 'TCCSPIN','RTDPOWERPIN'],
            ["MAX31855","Thermocouple", 'TCCSPIN', 'RTDPOWERPIN'],
            ["MAX31856","Thermocouple", 'TCCSPIN', 'RTDPOWERPIN'],
            ["MAX31865","PT100 / PT1000",'RTDCSPIN','RTDPOWERPIN'],
            ["ADS1120","PT100",'RTDCSPIN', 'RTDPOWERPIN'],
            ['LDR', 'Brightness', 'LDRPIN'],
            ["ADS1120", "Thermocouple",'TCCSPIN', 'RTDPOWERPIN']
         ]

flash_cmd = ['avrdude',
        #'-Cavrdude.conf',
        '',
        '-carduino',
        '-P',
        '-b115200',
        '-D',
        '',
        '-v',
        '-pavr64dd32',
        '-Uflash:w:../../Sensor/.pio/build/AVR64DD32/firmware.hex:i'
       ]


def crc16(data):
    crc = 0xFFFF;

    if len(data) == 0:
        return 0

    for i in range(len(data)):
        #print data[i]
        dbyte = data[i]
        crc ^= (dbyte << 8)
        crc &= 0xFFFF
        for j in range(8):
            mix = crc & 0x8000
            crc = (crc << 1)
            crc &= 0xFFFF
            if (mix):
                crc = crc ^ 0x1021
            crc &= 0xFFFF
        #print ("%i, data: %i, crc: %x") % (i, data[i], crc)
    return crc;
    
    
    
class Help(tk.Toplevel):
    def __init__(self, parent, parameters, headline):
        super().__init__(parent)

        self.parent = parent
        self.geometry("+%d+%d" % (self.parent.winfo_rootx()+365, self.parent.winfo_rooty()))
        self.title(headline)
        
        #Create bold font from standard display font
        default_font = font.nametofont('TkTextFont').actual()
        bold_font=(default_font['family'], default_font['size'], "bold")

        #### Headline ###
        self.head = ttk.Label(self, text=headline, font=("Helvetica", 13))
        self.head.grid(row=0, column=0, columnspan=3, padx=0, pady=10)#, sticky='we')

        self.mainframe = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.mainframe.grid(row=1, column=0, padx=5, pady=10)#, sticky="ew")
        
        for i, p in enumerate(parameters):
            self.label = tk.Label(self.mainframe, text=p)
            self.label.grid(row=i, column=0, pady=2, padx=5, sticky='ne')

            idx,ok = mem.index_of_param(p)
            self.label = tk.Label(self.mainframe, text=mem_help[idx][1], justify='left', wraplength=340)
            self.label.grid(row=i, column=1, pady=2, padx=5, sticky='nw')


class WatchNodes(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.node_filter = askstring("Node Filter", 'Node to Monitor', parent=self)
        
        self.parent = parent
        self.geometry("+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+100 ))
        self.attributes('-topmost', True)
        self.title('Watch Messages from Nodes')
        
        #Create bold font from standard display font
        default_font = font.nametofont('TkTextFont').actual()
        self.bold_font=(default_font['family'], default_font['size'], "bold")

        #### Headline ###
        self.headertext='Last Message from Node '
        self.header = tk.StringVar(self, self.headertext)
        self.head = ttk.Label(self, textvariable=self.header, font=("Helvetica", 13))
        self.head.grid(row=0, column=0, padx=10, pady=10, sticky='n')

        self.mainframe = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.mainframe.grid(row=1, column=0, padx=5)#, sticky="w")

        # parent gets a new line from serial port
        # parent copies the line into StringVar self.device_message
        # self.localcopy is a Stringvar copy of self.paraent.device_message, wich changes accordingly. 
        # on change, I can trace the change and call the callback on_message_change
        self.localcopy = self.parent.device_message
        self.localcopy.trace_add('write', self.on_message_change) # wenn neue Message, rufe callback
        
        self.msglabel = tk.StringVar(self,'')
        self.label = tk.Label(self.mainframe, textvariable=self.msglabel)
        self.label.grid(row=0, column=0, pady=2, padx=5, columnspan=2, sticky='nw')
        
        self.labels = list() # container of all labels, so we can destroy them later
    
    def get_param_value_pair(self, token):
        pvp= token.split('=')
        if pvp[0] =='v':
            pvp[0] = 'Battery Voltage'
            pvp[1] += ' mV'
        elif pvp[0] =='t':
            pvp[0] = 'Temperature'
            pvp[1] = str(int(pvp[1])/100.0) + ' degC'
        elif pvp[0] =='h':
            pvp[0] = 'Humidity'
            pvp[1] = str(int(pvp[1])/100.0) + ' %rH'
        elif pvp[0] =='rssi':
            pvp[0] = 'RSSI'
            pvp[1] = str(int(pvp[1])/10.0) + ' dBm'
        elif pvp[0] =='f':
            pvp[0] = 'Trigger'
            flag = int(pvp[1]) & 0x1f
            if flag & 0x1:
                pvp[1] = 'Heartbeat'
            elif flag & 0x2:
                pvp[1] = 'PCI0'
            elif flag & 0x4:
                pvp[1] = 'PCI1'
            elif flag & 0x8:
                pvp[1] = 'PCI2'
            elif flag & 0x10:
                pvp[1] = 'PCI3'
        elif pvp[0] =='c':
            pvp[0] = 'Count'
        elif pvp[0] =='be':
            pvp[0] = 'Bit Errors corrected'
            
        return pvp[0],pvp[1]
        
    def update_message_data(self, msg):
        items= msg.split(' ')
        if len(items) >1 and items[0].isdigit():
            if not items[0] == self.node_filter:
                print("not my message, node is", items[0])
                return
            #strftime("%d.%m.%Y"), strftime("%H:%M:%S")
            print('node', items[0], 'message', items[1])
            self.header.set(self.headertext + items[0])
            self.msglabel.set(msg)
            parameterlist = items[1].split('&')
            for la in self.labels:
                la.destroy()
            self.labels.clear()

            zeit= datetime.datetime.now().strftime('%Y.%m.%d %H:%M:%S')
            print(zeit)
            self.l = tk.Label(self.mainframe, font=self.bold_font, text='Time')
            self.l.grid(row=1, column=0, pady=2, padx=5, sticky='ne')
            self.labels.append(self.l)
            self.l = tk.Label(self.mainframe, text=zeit)
            self.l.grid(row=1, column=1, pady=2, padx=5, sticky='nw')
            self.labels.append(self.l)
            for i, p in enumerate(parameterlist):
                param, val = self.get_param_value_pair(p)
                print (param, val)
                self.l = tk.Label(self.mainframe, font=self.bold_font, text=param)
                self.l.grid(row=i+2, column=0, pady=2, padx=5, sticky='ne')
                self.labels.append(self.l)
                self.l = tk.Label(self.mainframe, text=val)
                self.l.grid(row=i+2, column=1, pady=2, padx=5, sticky='nw')
                self.labels.append(self.l)
            self.geometry("")
                
            
            
    
    def on_message_change(self, *args):
        msg = self.localcopy.get().split('\r\n')
        if len(msg) > 0:
            msg = msg[-1] # select last line. Sometimes there come 2 lines from the serial port.
            #self.msglabel.set(msg)
            self.update_message_data(msg)
            
        

class Actions:
    def __init__(self, parent):
        self.parent = parent
    
    def check_actions_on_eeprom(self):
        if self.parent.connection_active and self.parent.calmode_active:
            #get total number of actions. 
            self.parent.cmd_sent = b'NumberOfActions'
            cmd = b'r,%i\n' % mem.adr_num_actions
            self.parent.ser.write(cmd)
            while (self.parent.cmd_sent != b''):
                time.sleep(0.01)
            
            #check validity of first entry
            if mem.number_of_actions > mem.max_num_actions: #invalid
                return False, 'invalid actions block or no actions defined' ,mem.number_of_actions 

            elif mem.number_of_actions ==0: #invalid
                return False, 'no actions defined', mem.number_of_actions
            else:
                return True, 'OK', mem.number_of_actions
                
        else:
            return False, 'Serial Port not open or Configuration Mode not active', 0   
    '''
    def check_actions_on_eeprom1(mainclass):
        if mainclass.connection_active and mainclass.calmode_active:
            #get total number of actions. 
            mainclass.cmd_sent = b'NumberOfActions'
            cmd = b'r,%i\n' % mem.adr_num_actions
            mainclass.ser.write(cmd)
            while (mainclass.cmd_sent != b''):
                time.sleep(0.01)
            
            #check validity of first entry
            if mem.number_of_actions > mem.max_num_actions: #invalid
                return False, 'invalid actions block or no actions defined' ,mem.number_of_actions 

            elif mem.number_of_actions ==0: #invalid
                return False, 'no actions defined', mem.number_of_actions
            else:
                return True, 'OK', mem.number_of_actions
                
        else:
            return False, 'Serial Port not open or Configuration Mode not active', 0 
    '''
    def copy_actions_from_eeprom_to_memory(self, num_actions):
        if self.parent.connection_active and self.parent.calmode_active:
            mem.actions.clear()
            adr=mem.adr_actions
            for i in range(num_actions):
                single_action = list()
                for k in range (4):
                    cmd = b'r,%i\n' % (adr)
                    self.parent.cmd_sent= b'ar'
                    self.parent.ser.write(cmd)
                    time.sleep(0.01)
                    while (self.parent.cmd_sent != b''):
                        time.sleep(0.01)
                    single_action.append(self.parent.membuf)
                    adr+=1
                mem.actions.append(single_action)
    
    def calculate_crc(self):
        data_arr= []
        for action in mem.actions:
            data_arr += action
        crc = crc16(data_arr)
        return crc
        
    def verify_CRC(self):
        if self.parent.connection_active and self.parent.calmode_active:
            cmd = b'ri,%i\n' % mem.adr_actions_crc
            self.parent.cmd_sent= b'ar'
            self.parent.ser.write(cmd)
            time.sleep(0.01)
            while (self.parent.cmd_sent != b''):
                time.sleep(0.01)
            # crc is now in membuf
            
            crc_from_eeprom = self.parent.membuf  & 0xffff
            crc_calcualted = self.calculate_crc() & 0xffff
            
            if (crc_from_eeprom ^ crc_calcualted) != 0:
                return False, 'CRC Checksum Error!', crc_from_eeprom ^ crc_calcualted
            else:
                return True, 'OK', crc_from_eeprom

        else:
            return False, 'Serial Port not open or Configuration Mode not active', 0
    
    def write_actions_to_eeprom(self):
        # write num actions
        self.parent.ser.write(b'w,%i,%i\n' %(mem.adr_num_actions, mem.number_of_actions))
        time.sleep(0.01)
        
        i=0
        for action in mem.actions:
            for k, item in enumerate(action):
                adr = mem.adr_actions + i*4 + k
                self.parent.ser.write(b'w,%i,%i\n' % (adr, item))
                time.sleep(0.01)
            i+=1
        
        # write CRC
        crc = self.A.calculate_crc()
        adr = mem.adr_actions_crc
        self.parent.ser.write(b'wi,%i,%i\n' % (adr, crc))
        
class ActionsWindow(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)

        self.parent = parent
        self.A = Actions(parent)
        
        self.geometry("+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+100 ))
        self.title('TiNo2 Receiver Actions Definition')

        #### Headline ###
        self.head = ttk.Label(self, text='Receiver Actions Definition', font=("Helvetica", 13))
        self.head.grid(row=0, column=0, padx=10, pady=10, sticky='n')

        self.mainframe = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.mainframe.grid(row=1, column=0, padx=5)#, sticky="w")
        
        self.okcancel_frame = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.okcancel_frame.grid(row=2, column=0, columnspan=3, padx=10, pady=15, sticky="w")
    
        #Create bold font from standard display font
        default_font = font.nametofont('TkTextFont').actual()
        bold_font=(default_font['family'], default_font['size'], "bold")
    
        #Table Head line
        table_head =['Origin Node Nr.', 'Trigger\nChannel', 'Pin\non Receiver', 'Action', 'Power On\nPin State', 'Pulse/Burst\nDuration']
        for i, p in enumerate(table_head):
            self.label =  tk.Label(self.mainframe, text=p, font=bold_font, justify='center')
            self.label.grid(row=0, column=i+1, padx=5, pady=0, sticky='s')

        #OK Button
        self.ok_btn = ttk.Button(self.okcancel_frame, text='Apply', width=10,  command=self.on_ok)
        self.ok_btn.pack(side='left')

        #Cancel Button
        self.cancel_btn = ttk.Button(self.okcancel_frame, text='Close', width=10,  command=self.on_cancel).pack(side='left')
        
        self.add_btn = ttk.Button(self.okcancel_frame, text='Add Action', width=10,  command=self.on_add_action).pack(side='left')
        
        ### Load Action Table from EEPROM ###
        if mem.actions == []:
            self.load_actions_from_eeprom()
        else:
            self.load_actions_from_eeprom(False)
        
        #if 0 < mem.number_of_actions and mem.number_of_actions <= mem.max_num_actions:
            #crc_from_eeprom = self.parent.membuf  & 0xffff
            #crc_calcualted = self.calculate_crc() & 0xffff
            #if (crc_from_eeprom ^ crc_calcualted) != 0:
                #msgbox.showinfo(message ='CRC Checksum Error!', parent= self)
        
        self.widget_ids= list()

        self.actions_trigger_channel_vals = ['PCI0','PCI1','PCI2','PCI3']
        self.action_modes_vals =            ['Turn OFF','Turn ON','TOGGLE','PULSE','BURST_1 (slow)', 'BURST_2', 'BURST_3', 'BURST_4', 'BURST_5', 'BURST_6', 'BURST_7 (fastest)']
        self.action_defaults_vals =         ['OFF', 'ON']
        self.action_pulseduration_vals=     ['0.125', '0.25', '0.5', '1','2','4','8','16','32','64','128','256','512','1024','2048','4096']
        
        self.last_row=0
        self.row_index_table = list() # if we delete a row, we need to keep the data synchronised, the row number can't act as index
        
        for i, action in enumerate(mem.actions):
            ids = self.add_action(action, i+1)
            self.widget_ids.append(ids) # collection of all widgets
            self.row_index_table.append(i+1) # collection of existing row inices
            self.last_row = i+1
           
    def add_action(self, action, rownr):
        widget_ids=list()
        
        #node
        self.v = tk.IntVar(self, action[0])
        self.entry = tk.Entry(self.mainframe, textvariable =self.v, width=4, font=('Courier New', 10))
        self.entry.grid(row=rownr, column=1, padx=5, pady=0)#, sticky='w')
        self.entry.bind("<Return>", self.check)
        widget_ids.append(self.entry)
        
        #pin
        self.v = tk.IntVar(self, action[1])
        self.entry = tk.Entry(self.mainframe, textvariable =self.v, width=4, font=('Courier New', 10), justify='center')
        self.entry.grid(row=rownr, column=3, padx=5, pady=0)#, sticky='w')
        self.entry.bind("<Return>", self.check)
        widget_ids.append(self.entry)
        
        #trigger channel [2]
        trig_channel_idx = self.get_action_trigger_channel_index(action[2])
        self.combo = ttk.Combobox(self.mainframe, values= self.actions_trigger_channel_vals, state="readonly", width=5)
        self.combo.current(trig_channel_idx)
        self.combo.grid(row=rownr, column=2, padx=5, pady=0)
        self.combo.bind("<<ComboboxSelected>>", self.on_combobox_trigger)
        widget_ids.append(self.combo)
        
        #Action on Pin [3]
        blink = action[2]>>5
        mode_idx = self.get_action_modes_index(action[3], blink)
        self.combo = ttk.Combobox(self.mainframe, values= self.action_modes_vals, height=12, state="readonly", width=12)
        self.combo.current(mode_idx)
        self.combo.grid(row=rownr, column=4, padx=5, pady=0)
        self.combo.bind("<<ComboboxSelected>>", self.on_combobox_trigger)
        widget_ids.append(self.combo)
        
        #Pin state at Power on [4]
        default_idx = (action[3] & 0x80) >>7
        self.combo = ttk.Combobox(self.mainframe, values= self.action_defaults_vals, state="readonly", width=5)
        self.combo.current(default_idx)
        self.combo.grid(row=rownr, column=5, padx=5, pady=0)
        self.combo.bind("<<ComboboxSelected>>", self.on_combobox_trigger)
        widget_ids.append(self.combo)
        
        #Pulse duration [5]
        pulse_idx = (action[3]>>2) &0xF
        self.combo = ttk.Combobox(self.mainframe, values= self.action_pulseduration_vals, state="readonly", width=5)
        self.combo.current(pulse_idx)
        self.combo.grid(row=rownr, column=6, padx=5, pady=0)
        self.combo.bind("<<ComboboxSelected>>", self.on_combobox_trigger)
        widget_ids.append(self.combo)
        
        # Delete Button
        self.b = tk.Button(self.mainframe, text='Delete', width = 8, command= partial(self.on_delete_row, rownr))
        self.b.grid(row=rownr, column=7, padx=5, pady=0)
        widget_ids.append(self.b)
        
        return widget_ids
        
    def on_add_action(self):
        # increment mem.number_of_actions
        mem.number_of_actions +=1
        
        # create new action with default settings
        new_action=[0,0,2,0]
        
        # insert action into mem.actions
        mem.actions.append(new_action)

        # add widgets row
        self.last_row +=1
        widgets = self.add_action(new_action, self.last_row)
        self.widget_ids.append(widgets) # add new row of widgets
        self.row_index_table.append(self.last_row)

        # resize window
        self.geometry("")
        
    def get_action_trigger_channel_index(self, mask):
        if mask & 0x2:
            return 0
        elif mask & 0x4:
            return 1
        elif mask & 0x8:
            return 2
        elif mask & 0x10:
            return 3
        else:
            msgbox.showinfo(title='Error', message='problem decoding trigger mask')
            return 0
    def get_action_modes_index(self, mode, blink):
        idx = mode & 0x3
        if (idx == 3) and blink>0:
            idx=3 + blink
        return idx
    
    def on_combobox_trigger(self, event):
        #print('row',event.widget.grid_info()['row'], 'col', event.widget.grid_info()['column'], 'selection', event.widget.current())
        pass
        
    def on_ok(self):
        # write num actions
        self.parent.ser.write(b'w,%i,%i\n' %(mem.adr_num_actions, mem.number_of_actions)) 
        
        # collect data
        mem.actions.clear()
        for i in range(mem.number_of_actions):
            action = []
            is_burst = False
            burst_frequency =0
            #node
            action.append(int(self.widget_ids[i][0].get()))
            #pin
            action.append(int(self.widget_ids[i][1].get()))
            
            #combined Byte
            action_mode = self.widget_ids[i][3].current()
            if action_mode > 3:
                is_burst = True
                burst_frequency = action_mode -3
                action_mode = 3       
            action_default = self.widget_ids[i][4].current()
            action_pulsedur = self.widget_ids[i][5].current()            
            combi_byte = (action_default<<7) | (action_pulsedur<<2) | (action_mode&0x3)
            
            #Trigger Byte
            trigger_channel= 2 << self.widget_ids[i][2].current()
            if is_burst is True:
                trigger_channel |= burst_frequency<<5 
            action.append(trigger_channel)
            action.append(combi_byte)
            
            #add row to actions
            mem.actions.append(action)
        print (mem.actions)
        
        # write actions
        i=0
        for action in mem.actions:
            for k, item in enumerate(action):
                adr = mem.adr_actions + i*4 + k
                self.parent.ser.write(b'w,%i,%i\n' % (adr, item))
                time.sleep(0.01)
            i+=1
        
        # write CRC
        crc = self.A.calculate_crc()
        adr = mem.adr_actions_crc
        self.parent.ser.write(b'wi,%i,%i\n' % (adr, crc))
    
    def on_delete_row(self, winrow): # remove a action
        # look up the right row number in the table
        action_row_nr = self.row_index_table.index(winrow)

        # decrease number of available actions
        mem.number_of_actions -=1
        
        # remove action data from mem.actions
        #print('mem.actions', mem.actions, 'action_row_nr', action_row_nr, 'mem.number_of_actions', mem.number_of_actions, '\n')
        mem.actions.pop(action_row_nr)
        
        #delete widgets
        for widget in self.widget_ids[action_row_nr]:
            widget.destroy()
        
        # delete widget id's
        self.widget_ids.pop(action_row_nr)
        
        #delete this index from the index table
        self.row_index_table.pop(action_row_nr)
        
        self.geometry('')          
        print (mem.actions)
        
        
    def on_cancel(self):
        self.destroy()
        
    def check(self, event):
        pass

    def load_actions_from_eeprom(self, reload= True):
        if reload:
           mem.actions_pop_done= False
           mem.actions.clear()
        
        if not mem.actions_pop_done:
            success, messagetext, mem.number_of_actions = self.A.check_actions_on_eeprom()
            if not success:
                msgbox.showinfo(title='Error', message=messagetext, parent = self)
                return
       
        if not mem.actions_pop_done:
            self.A.copy_actions_from_eeprom_to_memory(mem.number_of_actions)
            mem.actions_pop_done = True
        
        success, messagetext, crc = self.A.verify_CRC()
        if not success:
            msgbox.showinfo(title='Error', message=messagetext, parent = self)           
        
class HardwareSetup(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)

        self.parent = parent
        #self.geometry("365x270+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+150 ))
        self.geometry("+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+100 ))
        self.title('Hardware Environment Configuration')

        #### Headline ###
        self.head = ttk.Label(self, text='Hardware Environment Configuration', font=("Helvetica", 13))
        self.head.grid(row=0, column=0, padx=10, pady=10, sticky='n')

        self.mainframe = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.mainframe.grid(row=1, column=0, padx=5)#, sticky="w")

        self.helpframe = tk.Frame(self, width=10, height=10, highlightthickness=1, highlightbackground="gray")
        self.helpframe.grid(row=2, column=0, padx=5, pady=10, sticky="we")

        self.okcancel_frame = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.okcancel_frame.grid(row=3, column=0, columnspan=3, padx=10, pady=15, sticky="w")

        #Create bold font from standard display font
        default_font = font.nametofont('TkTextFont').actual()
        bold_font=(default_font['family'], default_font['size'], "bold")

        #self.parameters =['LEDPIN', 'LDRPIN', 'PIRDATAPIN', 'PIRDEADTIME', 'TCCSPIN', 'RTDPOWERPIN', 'RTDCSPIN', 'ONEWIREPOWERPIN', 'ONEWIREDATAPIN', 'I2CPOWERPIN' ]
        #self.paramlabels=['LED Pin', 'LDR Pin', 'PIR Data Pin', 'PIR Still Time', 'Thermocouple CS Pin', 'RTD Vcc Pin', 'RTD CS Pin', 'ONEWIRE Vcc Pin', 'ONEWIRE Data Pin', 'I2C Vcc Pin' ]
        #self.paramlabels_comments=['internal LED Pin = 19', 'must be a analog Pin', '', ' *8 seconds', 'Chip Select', 'PT100/PT1000 Vcc Pin', 'PT100/PT1000 CS Pin', 'DS18B20', 'DS18B20', 'internal I2C Vcc Pin = 25' ]

        self.parameters =['LEDPIN', 'LDRPIN', 'PIRDATAPIN', 'TCCSPIN', 'RTDPOWERPIN', 'RTDCSPIN', 'ONEWIREPOWERPIN', 'ONEWIREDATAPIN', 'I2CPOWERPIN' ]
        self.paramlabels=['LED', 'LDR (Brightness)', 'PIR Data', 'Thermocouple CS', 'RTD / TC  Vcc', 'RTD CS', 'DS18B20 Vcc', 'DS18B20 Data', 'I2C Vcc' ]
        self.paramlabels_comments=['internal LED Pin = 19', 'must be a analog Pin', '', 'Chip Select', 'PT100/PT1000 Vcc Pin', 'PT100/PT1000 CS Pin', 'Temperature Sensor', 'Temperature Sensor', 'internal I2C Vcc Pin = 25' ]


        self.parameters_values_str = list()
        self.parameters_enable = list()
        self.label=ttk.Label(self.mainframe, text='Pin Function', font= bold_font)
        self.label.grid(row=0, column=0, padx=5, pady=5, sticky ='e')

        self.label=ttk.Label(self.mainframe, text='Enable', font= bold_font)
        self.label.grid(row=0, column=1, padx=5, pady=5, sticky ='e')

        self.label=ttk.Label(self.mainframe, text='GPIO', font= bold_font)
        self.label.grid(row=0, column=2, padx=5, pady=5, sticky ='e')

        self.label=ttk.Label(self.mainframe, text='Remark', font= bold_font)
        self.label.grid(row=0, column=3, padx=5, pady=5, sticky ='w')


        for i, p in enumerate(self.parameters):
            idx, ok = mem.index_of_param(p)

            self.v = tk.StringVar(self, mem.paramvalue[idx])
            self.parameters_values_str.append(self.v)
            self.parameters_enable.append(tk.IntVar(self,mem.paramvalue[idx] & 0x80))

            self.label=ttk.Label(self.mainframe, text=self.paramlabels[i])
            self.label.grid(row=i+1, column=0, padx=5, pady=2, sticky ='e')

            self.chk = tk.Checkbutton(self.mainframe, text='', variable=self.parameters_enable[i], onvalue=0, offvalue=128, command= partial(self.on_checkbox, i))
            self.chk.grid(row=i+1, column=1, padx=5, pady=2)#, sticky ='e')
            self.chk._name = p

            self.entry = tk.Entry(self.mainframe, textvariable =self.parameters_values_str[i], width=4, font=('Courier New', 10))
            self.entry._name= p
            self.entry.grid(row=i+1, column=2, padx=5, pady=0)#, sticky='w')
            self.entry.bind("<Return>", self.check)

            self.label=ttk.Label(self.mainframe, text=self.paramlabels_comments[i])
            self.label.grid(row=i+1, column=3, padx=5, pady=2, sticky ='w')

        #Help text
        helptext='Setup Guide:\n\
- Valid GPIO Pin numbers range from 0 to 25.\n\
- negative numbers disable the GPIO function and hence the sensor (if any)\n\
- to use DS18B20, RTD or Themocouple sensors, the according software drivers\n   must be enabled in the Sensor Configuration Dialog Window'
        self.label=ttk.Label(self.helpframe, text=helptext, justify='left', wraplength=450)
        self.label.grid(row=0, column=0, padx=0, pady=0)
        #OK Button
        self.ok_btn = tk.Button(self.okcancel_frame, text='Finish', width=10,  command=self.on_ok)
        self.ok_btn.pack(side='left')

        #Cancel Button
        self.cancel_btn = tk.Button(self.okcancel_frame, text='Cancel', width=10,  command=self.on_cancel).pack(side = 'left')

        #Help Button
        self.help_btn = tk.Button(self.okcancel_frame, text='Help', width=10,  command=self.on_help).pack(side = 'left')

    def on_cancel(self):
        self.destroy()

    def validate (self, valstr, idx):
        value, ok = self.parent.validate_entry(valstr, idx)
        if not ok:
            msgbox.showinfo(title='Error', message='validation FAIL!')
        else:
            mem.paramvalue[idx] = value

    def on_ok(self):
        for i, p in enumerate(self.parameters):
            idx, ok = mem.index_of_param(p)
            valstr = self.parameters_values_str[i].get()
            self.validate(valstr, idx)

        self.parent.populate_eeprom()
        self.destroy()

    def check(self, event):
        name = event.widget._name
        idx, ok = mem.index_of_param(name)
        valstr = event.widget.get()
        self.validate(valstr, idx)

    def on_checkbox(self, indx):
        valstr = self.parameters_values_str[indx].get()
        port = int(valstr)
        state= self.parameters_enable[indx].get()
        if state == 128: # from checked to unchecked ==> new value = old value - 128
            valstr = str(port -128)
        else:
            valstr = str(port + 128)
        self.parameters_values_str[indx].set(valstr)
        name = self.parameters[indx]
        idx, ok = idx, ok = mem.index_of_param(name)
        self.validate(valstr, idx)

    def on_help(self):
        helpwin = Help(self,self.parameters, 'TiNo2 Hardware Peripherals Help')
        helpwin.grab_set()




class PCISetup(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)

        self.parent = parent
        #self.geometry("365x270+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+150 ))
        self.geometry("+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+150 ))
        self.title('Pin Change Interrupts Configuration')

        #### Headline ###
        self.head = ttk.Label(self, text='Pin Change Interrupts Configuration', font=("Helvetica", 13))
        self.head.grid(row=0, column=0, padx=10, pady=10, sticky='n')

        self.mainframe = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.mainframe.grid(row=1, column=0, padx=5)#, sticky="w")

        self.okcancel_frame = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.okcancel_frame.grid(row=2, column=0, columnspan=3, padx=10, pady=15, sticky="w")

        #Create bold font from standard display font
        default_font = font.nametofont('TkTextFont').actual()
        bold_font=(default_font['family'], default_font['size'], "bold")


        self.trigger_vals=['LOW','HIGH','FALLING','RISING','CHANGE']
        self.mode_vals = ['INPUT','OUTPUT','INPUT_PULLUP']


        self.pcis = ['PCI0PIN', 'PCI1PIN', 'PCI2PIN', 'PCI3PIN']
        self.pciports = list()
        self.pcimodes_str = list()
        self.pcitrigger_str = list()
        self.pcigateways= list()

        for i, p in enumerate(self.pcis):
            self.label=ttk.Label(self.mainframe, text=p)
            self.label.grid(row=i+1, column=0, padx=5, pady=5)

            idx, ok =  mem.index_of_param(p)
            self.v = tk.IntVar(self, mem.paramvalue[idx])
            self.pciports.append(self.v)

            self.c = mem.paramvalue[idx+1]
            self.opmode = self.c >> 3
            self.trigger = self.c &0x07

            self.opmode_str = tk.StringVar()
            self.pcimodes_str.append (self.opmode_str)
            self.trigger_str = tk.StringVar()
            self.pcitrigger_str.append(self.trigger_str)

            # Pin Mode Combobox
            self.combo = ttk.Combobox(self.mainframe, values=self.mode_vals, textvariable=self.pcimodes_str[i], state="readonly", width=15)
            self.combo.grid(row=i+1, column=2, padx=5, pady=0)
            self.combo.set(self.mode_vals[self.opmode])
            self.combo.bind("<<ComboboxSelected>>", self.on_combobox_mode)
            self.combo._name= p

            #Trigger Combo Box
            self.combo = ttk.Combobox(self.mainframe, values=self.trigger_vals, textvariable=self.pcitrigger_str[i], state="readonly", width=8)
            self.combo.grid(row=i+1, column=3, padx=5, pady=0)
            self.combo.set(self.trigger_vals[self.trigger])
            self.combo.bind("<<ComboboxSelected>>", self.on_combobox_trigger)
            self.combo._name= p

            # PCIxPIN
            self.entry = tk.Entry(self.mainframe, textvariable =self.pciports[i], width=4, font=('Courier New', 10))
            self.entry._name= p
            self.entry.grid(row=i+1, column=1, padx=5, pady=0)#, sticky='w')
            self.entry.bind("<Return>", self.check)

            # Gateway ID
            self.c = tk.IntVar(self, mem.paramvalue[idx+2])
            self.pcigateways.append(self.c)
            self.entry = tk.Entry(self.mainframe, textvariable =self.pcigateways[i], width=4, font=('Courier New', 10))
            self.entry._name= p[:4] + 'GATEWAYID'
            self.entry.grid(row=i+1, column=4, padx=5, pady=0)#, sticky='w')
            self.entry.bind("<Return>", self.check)


        table_head =['GPIO Nr.', 'Mode', 'Trigger', 'Gateway']
        for i, p in enumerate(table_head):
            self.label =  ttk.Label(self.mainframe, text=p, font=bold_font)
            self.label.grid(row=0, column=i+1, padx=5, pady=0)

        #OK Button
        self.ok_btn = tk.Button(self.okcancel_frame, text='Finish', width=10,  command=self.on_ok)
        self.ok_btn.pack(side='left')

        #Cancel Button
        self.cancel_btn = tk.Button(self.okcancel_frame, text='Cancel', width=10,  command=self.on_cancel).pack()

    def on_cancel(self):
        self.destroy()

    def validate (self, valstr, idx):
        value, ok = self.parent.validate_entry(valstr, idx)
        if not ok:
            msgbox.showinfo(title='Error', message='validation FAIL!')
        else:
            mem.paramvalue[idx] = value

    def on_ok(self):
        for i, p in enumerate(self.pcis):
            idx, ok = mem.index_of_param(p)
            # PCI POrt
            self.validate(str(self.pciports[i].get()), idx)
            # PCI Config
            mode = self.mode_vals.index(self.pcimodes_str[i].get()) <<3
            trig = self.trigger_vals.index(self.pcitrigger_str[i].get()) &0x07
            self.validate(str(mode + trig), idx+1)
            #PCI Gateway ID (Target)
            self.validate(str(self.pcigateways[i].get()), idx+2)

        self.parent.populate_eeprom()
        self.destroy()

    def check(self, event):
        name = event.widget._name
        idx, ok = mem.index_of_param(name)
        valstr = event.widget.get()
        self.validate(valstr, idx)




    def on_combobox_trigger(self, event):
        #name = event.widget._name
        #val = event.widget.current()
        #whichport = int(name[3])
        #print(name, whichport, val, event.widget.get())
        pass

    def on_combobox_mode(self, event):
        #name = event.widget._name
        #val = event.widget.current()
        #whichport = int(name[3])
        #self.pcimodes[whichport].set(val)
        #print(name, val, event.widget.get())
        pass



class Battery_Calibration(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent

        # place window relative to parent
        self.geometry("330x230+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+150 ))
        self.title('Battery Voltage Calibration')

        self.head = ttk.Label(self, text='Battery Voltage Calibration', font=("Helvetica", 13))
        self.head.grid(row=0, column=0, padx=10, pady=10, sticky='n')

        self.mainframe = tk.Frame(self, height=10, highlightthickness=0, highlightbackground="gray")
        self.mainframe.grid(row=1, column=0, padx=10, sticky="w")

        self.okcancelframe = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.okcancelframe.grid(row=2, column=0, padx=10, pady=25, sticky="e")



        ### LED disable / enable
        self.LED_enabled = tk.IntVar(self,0)
        self.LedPin_idx , ok = mem.index_of_param('LEDPIN')
        self.LedPin_value = mem.paramvalue[self.LedPin_idx]
        self.LedPin_value_original = self.LedPin_value
        if self.LedPin_value > 0:
            self.LED_enabled.set(1)
        else:
            self.LED_enabled.set(0)


        self.l = tk.Label(self.mainframe, text="1. Disable LED")
        self.l.grid(row=0, column=0, pady=0, padx=0, sticky='w')

        self.led_btn = tk.Button(self.mainframe, text='', width=10,  command=self.on_led_btn)
        self.led_btn.grid(row=0, column=1,pady=0, padx=10, sticky='w')
        if self.LED_enabled.get():
            self.led_btn['text'] = 'Disable LED'
        else:
             self.led_btn['text'] = 'Enable LED'

        ### Reference VCC Value (measured during Configuration phase)
        self.reference_vcc_idx,ok  = mem.index_of_param('VCCATCAL')
        self.reference_vcc = tk.IntVar(self, mem.paramvalue[self.reference_vcc_idx])
        self.l = tk.Label(self.mainframe, text="2. Measure Voltage in mV\nwith Digital Multimeter")
        self.l.grid(row=1, column=0, columnspan=2, pady=0, padx=0, sticky='w')

        self.VCC_AT_CAL_entry = tk.Entry(self.mainframe, textvariable =self.reference_vcc, width=7, font=('Courier New', 10))
        self.VCC_AT_CAL_entry._name='VCCATCAL'
        self.VCC_AT_CAL_entry.grid(row=1, column=2, padx=0, pady=0, sticky='w')
        self.VCC_AT_CAL_entry.bind("<Return>", self.check)

        self.reference_adc_idx,ok  = mem.index_of_param('VCCADC_CAL')
        #self.reference_adc = tk.IntVar(self, mem.paramvalue[self.reference_adc_idx])

        self.parent.Reference_ADC.set(mem.paramvalue[self.reference_adc_idx])

        self.l = tk.Label(self.mainframe, text="3. Measure ADC Value")
        self.l.grid(row=2, column=0, pady=0, padx=0, sticky='w')

        self.adc_cal_btn = tk.Button(self.mainframe, text='Measure ADC', width=10,  command=self.on_adc_cal)
        self.adc_cal_btn.grid(row=2, column=1, pady=0, padx=10, sticky='w')

        self.ADC_AT_CAL_entry = tk.Entry(self.mainframe, textvariable =self.parent.Reference_ADC, width=7, font=('Courier New', 10))
        self.ADC_AT_CAL_entry._name='VCCADC_CAL'
        self.ADC_AT_CAL_entry.grid(row=2, column=2, padx=0, pady=0, sticky='w')
        self.ADC_AT_CAL_entry.bind("<Return>", self.check)

        self.l = tk.Label(self.mainframe, text="4. Test VCC Measurement")
        self.l.grid(row=3, column=0, pady=0, padx=0, sticky='w')
        self.memtest_btn = tk.Button(self.mainframe, text='Test', width=10,  command=self.on_vcctest)
        self.memtest_btn.grid(row=3, column=1, pady=0, padx=10, sticky='w')

        self.l = tk.Label(self.mainframe, text="", bg='white', font=('Courier New', 10), textvariable= self.parent.Measured_VCC)
        self.l.grid(row=3, column=2, pady=0, padx=0, sticky='w')

        #OK Button
        self.ok_btn = tk.Button(self.okcancelframe, text='Finish', width=10,  command=self.on_ok)
        self.ok_btn.pack(side='left')

        #Cancel Button
        self.cancel_btn = tk.Button(self.okcancelframe, text='Cancel', width=10,  command=self.on_cancel).pack()

    def on_cancel(self):
        self.destroy()

    def on_ok(self):
        self.update_led_pin()
        self.check(0)
        self.parent.populate_eeprom()
        self.destroy()

    def on_vcctest(self):
        self.parent.measure_vdd()
        #print('on_memtest', mem.paramvalue[self.reference_adc_idx])
        #self.reference_adc.set(mem.paramvalue[self.reference_adc_idx])
        #self.parent.Reference_ADC.set(mem.paramvalue[self.reference_adc_idx])

    def on_adc_cal(self):
        self.parent.vddcal()



    def check(self, event):
        value, isok = self.parent.validate_entry(str(self.reference_vcc.get()), self.reference_vcc_idx)
        if not isok:
            msgbox.showinfo(title='Error', message='validation FAIL!')
        else:
             mem.paramvalue[self.reference_vcc_idx] = value

        value, isok = self.parent.validate_entry(str(self.parent.Reference_ADC.get()), self.reference_adc_idx)
        if not isok:
            msgbox.showinfo(title='Error', message='validation FAIL!')
        else:
             mem.paramvalue[self.reference_adc_idx] = value


    def update_led_pin(self):
        if self.LED_enabled.get(): # currently enabled
            pass
        else: # curently disable, now enable
            self.led_btn['text'] = 'Disable LED'
            self.LED_enabled.set(1)
            mem.paramvalue[self.LedPin_idx] = self.LedPin_value

        value, isok = self.parent.validate_entry(str(mem.paramvalue[self.LedPin_idx]), self.LedPin_idx)
        if not isok:
            msgbox.showinfo(title='Error', message='validation FAIL!')

    def on_led_btn(self):
        if self.LED_enabled.get(): # currently enabled, now disable
            self.led_btn['text'] = 'Enable LED'
            self.LED_enabled.set(0)
            mem.paramvalue[self.LedPin_idx] = -1
        else:                       # currently disabled, now enable
            self.led_btn['text'] = 'Disable LED'
            self.LED_enabled.set(1)
            mem.paramvalue[self.LedPin_idx] = self.LedPin_value

        value, isok = self.parent.validate_entry(str(mem.paramvalue[self.LedPin_idx]), self.LedPin_idx)
        if not isok:
            msgbox.showinfo(title='Error', message='validation FAIL!')
        else:
            mem.paramvalue[self.LedPin_idx] = value

        #self.parent.populate_eeprom()


class FirmwareSetup(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)

        self.parent = parent
        #self.geometry("365x270+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+150 ))
        self.geometry("+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+150 ))
        self.title('TiNo2 Firmware and Miscellaneous Setup')

        #### Headline ###
        self.head = ttk.Label(self, text='TiNo Firmware Setup', font=("Helvetica", 13))
        self.head.grid(row=0, column=0, columnspan=3, padx=0, pady=15)#, sticky='we')

        self.mainframe = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.mainframe.grid(row=1, column=0, padx=5, sticky="w")

        self.okcancel_frame = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.okcancel_frame.grid(row=2, column=0, columnspan=3, padx=5, sticky="w")

        #OK APPLY CANCEL Buttons

        self.apply_btn = tk.Button(self.okcancel_frame, text='Apply', command=self.on_apply, width=7)
        self.apply_btn.grid(row=0, column=0, padx=10, pady=20, sticky = 'se')
        self.ok_btn = tk.Button(self.okcancel_frame, text='OK', command=self.on_ok, width=7)
        self.ok_btn.grid(row=0, column=1, padx=0, pady=20, sticky = 'se')
        self.cancel_btn = tk.Button(self.okcancel_frame, text='Cancel', command=self.on_cancel, width=7)
        self.cancel_btn.grid(row=0, column=2, padx=10, pady=20, sticky = 'se')
        self.help_btn = tk.Button(self.okcancel_frame, text='Help', command=self.on_help, width=7)
        self.help_btn.grid(row=0, column=3, padx=2, pady=20, sticky = 'w')

        self.parameters =['SENDDELAY', 'LEDCOUNT', 'USE_CRYSTAL_RTC', 'SERIAL_ENABLE']
        self.parameter_values = list()

        #Label and values
        for i, p in enumerate(self.parameters):
            self.label = tk.Label(self.mainframe, text=p)
            self.label.grid(row=i, column=0, pady=2, padx=5, sticky='e')

            self.idx, ok = mem.index_of_param(p)
            self.v = tk.StringVar(self, mem.paramvalue[self.idx])
            self.parameter_values.append(self.v)

            if p == 'USE_CRYSTAL_RTC' or p == 'SERIAL_ENABLE':
                self.chk = tk.Checkbutton(self.mainframe, text='', variable=self.parameter_values[i], onvalue='1', offvalue='0', command=partial(self.on_checkbox, i))
                self.chk.grid(row=last_row, column=1, padx=0, pady=0, sticky ='w')
                self.chk._name = p                
            else:
                self.e = tk.Entry(self.mainframe, textvariable=self.parameter_values[i], width=6, state=tk.NORMAL)
                self.e.grid(row=i, column=1, sticky="w")
                self.e._name = p
                self.e.bind("<Return>", self.on_enter)
            last_row =i+1


    def on_cancel(self):
        self.destroy()

    def on_apply(self):
        # all parameters are stored as strings
        for i, p in enumerate(self.parameters):
            idx, ok = mem.index_of_param(p)
            v, isok = self.parent.validate_entry(self.parameter_values[i].get(), idx)
            if not isok:
                msgbox.showinfo(title='Error', message='validation FAIL!')
            else:
                mem.paramvalue[idx] = v

        self.parent.populate_eeprom()

    def on_ok(self):
        self.on_apply()
        self.on_cancel()

    def on_help(self):
        #helpwin= RadioSetupHelp(self)
        helpwin = Help(self,self.parameters, 'TiNo2 Firmware Setup Help')
        helpwin.grab_set()

    def on_enter(self, event):
        name = event.widget._name
        idx,ok = mem.index_of_param(name)
        valstr = event.widget.get()

        value, isok = self.parent.validate_entry(valstr, idx, True)
        if not isok:
           msgbox.showinfo(title='Error', message='validation FAIL!')

    def on_checkbox(self, i):
        name = self.parameters[i]
        idx,ok = mem.index_of_param(name)
        valstr = self.parameter_values[i].get()
        value, isok = self.parent.validate_entry(valstr, idx, True)
        if not isok:
           msgbox.showinfo(title='Error', message='validation FAIL!')


class RadioSetup(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)

        self.parent = parent
        #self.geometry("365x270+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+150 ))
        self.geometry("+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+150 ))
        self.title('TiNo Radio Setup')

        #### Headline ###
        self.head = ttk.Label(self, text='TiNo Radio Setup', font=("Helvetica", 14))
        self.head.grid(row=0, column=0, columnspan=3, padx=0, pady=15)#, sticky='we')

        self.mainframe = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.mainframe.grid(row=1, column=0, padx=5, sticky="w")

        self.okcancel_frame = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.okcancel_frame.grid(row=2, column=0, columnspan=3, padx=5, sticky="w")

        #OK APPLY CANCEL Buttons

        self.apply_btn = tk.Button(self.okcancel_frame, text='Apply', command=self.on_apply, width=7)
        self.apply_btn.grid(row=0, column=0, padx=10, pady=20, sticky = 'se')
        self.ok_btn = tk.Button(self.okcancel_frame, text='OK', command=self.on_ok, width=7)
        self.ok_btn.grid(row=0, column=1, padx=0, pady=20, sticky = 'se')
        self.cancel_btn = tk.Button(self.okcancel_frame, text='Cancel', command=self.on_cancel, width=7)
        self.cancel_btn.grid(row=0, column=2, padx=10, pady=20, sticky = 'se')
        self.help_btn = tk.Button(self.okcancel_frame, text='Help', command=self.on_help, width=7)
        self.help_btn.grid(row=0, column=3, padx=2, pady=20, sticky = 'w')


        self.parameters =['FREQ_CENTER', 'FDEV_STEPS', 'TXPOWER', 'IS_RFM69HW', 'PABOOST', 'TXGAUSS_SHAPING', 'USE_RADIO_T_COMP', 'RADIO_T_OFFSET']
        self.parameter_values = list()
        self.parameter_values_origin = list() # keep values in mind, for the CANCEL button

        #Label and values
        for i, p in enumerate(self.parameters):
            self.label = tk.Label(self.mainframe, text=p)
            self.label.grid(row=i, column=0, pady=2, padx=5, sticky='e')

            self.idx, ok = mem.index_of_param(p)
            self.v = tk.StringVar(self, mem.paramvalue[self.idx])
            self.v1 = tk.StringVar(self, mem.paramvalue[self.idx])
            self.parameter_values.append(self.v)
            self.parameter_values_origin.append(self.v1)


        # extra Label for tx Power
        i=  self.parameters.index('TXPOWER')
        self.label_power_dbm = tk.Label(self.mainframe, text='xxx dBm')
        self.label_power_dbm.grid(row=i, column=2, pady=0, padx=5, sticky='w')
        pl = int(self.parameter_values[i].get())
        self.label_power_dbm['text'] = '%i dBm' % (13 - 31 + pl)


        #Entry widgets
        for i in range(3):
            self.e = tk.Entry(self.mainframe, textvariable=self.parameter_values[i], width=6, state=tk.NORMAL)
            self.e.grid(row=i, column=1, sticky="w")
            self.e._name = self.parameters[i]
            self.e.bind("<Return>", self.on_enter)

        #Checkbox
        p_idx=3
        self.chk = tk.Checkbutton(self.mainframe, text='', variable=self.parameter_values[p_idx], onvalue='1', offvalue='0', command=self.on_checkbox)
        self.chk.grid(row=p_idx, column=1, padx=0, pady=0, sticky ='w')
        self.chk._name = p
        self.chk['state'] = tk.DISABLED

        #Comboboxes
        p_idx = p_idx+1
        self.pa_boost_choices = ['Normal', 'PA Boost Plus', 'PA Boost']
        self.combo_paboost = ttk.Combobox(self.mainframe, values=self.pa_boost_choices, state="readonly", width=12)
        current_choice = int(self.parameter_values[p_idx].get())
        self.combo_paboost.set(self.pa_boost_choices[current_choice])
        self.combo_paboost.grid(row=p_idx, column=1, columnspan=2, padx=0, pady=0,sticky="w")
        self.combo_paboost.bind("<<ComboboxSelected>>", self.on_combobox)
        self.combo_paboost._name = 'PABOOST'

        p_idx = p_idx+1
        self.txgauss_choices = ['No Shaping', 'BT= 1', 'BT= 0.5', 'BT= 0.3']
        self.combo_txgauss = ttk.Combobox(self.mainframe, values=self.txgauss_choices, state="readonly", width=12)
        current_choice = int(self.parameter_values[p_idx].get())
        self.combo_txgauss.set(self.txgauss_choices[current_choice])
        self.combo_txgauss.grid(row=p_idx, column=1, columnspan=2, padx=0, pady=0,sticky="w")
        self.combo_txgauss.bind("<<ComboboxSelected>>", self.on_combobox)
        self.combo_txgauss._name='TXGAUSS_SHAPING'

        p_idx = p_idx+1
        self.chk = tk.Checkbutton(self.mainframe, text='', variable=self.parameter_values[p_idx], onvalue='1', offvalue='0', command=self.on_checkbox)
        self.chk.grid(row=p_idx, column=1, padx=0, pady=0, sticky ='w')
        self.chk._name = self.parameters[p_idx]
        self.chk['state'] = tk.DISABLED

        # RADIO_T_OFFSET Entry 
        p_idx = p_idx+1
        self.e = tk.Entry(self.mainframe, textvariable=self.parameter_values[p_idx], width=6, state=tk.NORMAL)
        self.e.grid(row=p_idx, column=1, sticky="w")
        self.e._name = self.parameters[p_idx]
        self.e.bind("<Return>", self.on_enter)

        #extra Button for Radio_t_offset
        '''
        self.test_temp_btn = tk.Button(self.mainframe, text='Measure Radio\nTemperature', command=self.on_test_temperature)
        self.test_temp_btn.grid(row=p_idx, column=2, padx=2, pady=0, sticky = 'w')
        
        #Label for Temperature reading
        textvariable= self.parent.radio_temperature
        self.label = tk.Label(self.mainframe, textvariable= self.parent.radio_temperature)
        self.label.grid(row=p_idx, column=3, padx=2, pady=0, sticky = 'w')
        '''
        p_idx += 1
        self.test_temp_btn = tk.Button(self.mainframe, text='Measure Radio\nTemperature', command=partial(self.on_test_temperature, b'tt'))
        self.test_temp_btn.grid(row=p_idx, column=0, pady=2, padx=5, sticky='wens')
        
        #Label for Temperature reading
        textvariable= self.parent.radio_temperature
        self.label = tk.Label(self.mainframe, textvariable= self.parent.radio_temperature)
        self.label.grid(row=p_idx, column=1, padx=2, pady=0, sticky = 'w')
        
        p_idx += 1
        self.test_temp_btn = tk.Button(self.mainframe, text='Send Radio\nTest Packet', command=partial(self.on_test_temperature, b't'))
        self.test_temp_btn.grid(row=p_idx, column=0, pady=2, padx=5, sticky='wens')
        
    def on_test_temperature(self, cmd):
        self.parent.cmd_sent = cmd
        cmd = cmd + b'\n'
        self.parent.ser.write(cmd)

    
    def on_help(self):
        #helpwin= RadioSetupHelp(self)
        helpwin = Help(self,self.parameters, 'TiNo2 Radio Setup Help')
        helpwin.grab_set()

    def on_combobox(self, event):
        #here we need to update our local StringVar.
        # the parameter is a string like '0', '1', ...
        name = event.widget._name
        valstr = str(event.widget.current()) # get combobox selection as index, convert to string
        p_idx = self.parameters.index(name)  # get index of local windows parameters list
        self.parameter_values[p_idx].set(valstr)  # update local value
        #print(name, valstr)

    def on_checkbox(self):
        pass

    def on_enter(self, event):
        name = event.widget._name
        idx,ok = mem.index_of_param(name)
        valstr = event.widget.get()

        #print (name, idx, valstr)

        if name == 'TXPOWER':
            pl = int(valstr)
            dbm = 13 - 31 + pl
            labelstr = '%i dBm' % dbm
            self.label_power_dbm['text'] = labelstr
        
        # the idea is here that the user enters values, and at the end he validates with Apply button
        # therefore the 3rd Parameter is False, because otherwise the Cancel Button cannot undo any changes.
        value, isok = self.parent.validate_entry(valstr, idx, True)
        if not isok:
           msgbox.showinfo(title='Error', message='validation FAIL!')

    def get_index(self, param):
        idx, ok = mem.index_of_param(param)
        return idx

    '''
    def update(self, parameterlist): # parameterlist is a list of StrinVar's
        for i, p in enumerate(self.parameters):
            idx = self.get_index(p)
            v, isok = self.parent.validate_entry(parameterlist[i].get(), idx)
            if not isok:
                msgbox.showinfo(title='Error', message='validation FAIL!')
            else:
                mem.paramvalue[idx] = v
        self.parent.populate_eeprom()
    '''
    
    def on_cancel(self):
        # restore original data, discard any changes     
        for i, p in enumerate(self.parameters):
            idx = self.get_index(p)
            v, isok = self.parent.validate_entry(self.parameter_values_origin[i].get(), idx)
            if not isok:
                msgbox.showinfo(title='Error', message='validation FAIL!')
            else:
                mem.paramvalue[idx] = v
        self.parent.populate_eeprom()
        
        #self.update(self.parameter_values_origin)
        self.destroy()

    def on_apply(self):
        # all parameters are stored as strings
        for i, p in enumerate(self.parameters):
            idx = self.get_index(p)
            v, isok = self.parent.validate_entry(self.parameter_values[i].get(), idx)
            if not isok:
                msgbox.showinfo(title='Error', message='validation FAIL!')
            else:
                mem.paramvalue[idx] = v
        self.parent.populate_eeprom()

    def on_ok(self):
        self.on_apply()
        self.destroy()

class NetworkSetup(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)

        self.parent = parent
        self.geometry("+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+150))
        self.title('TiNo Network Setup')

        #### Headline ###
        self.head = ttk.Label(self, text='TiNo2 Network Setup', font=("Helvetica", 13))
        self.head.grid(row=0, column=0, columnspan=3, padx=0, pady = 15)#, sticky='we')

        self.mainframe = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.mainframe.grid(row=1, column=0, padx=5, sticky="w")

        self.parameters =['NODEID', 'NETWORKID','GATEWAYID', 'FREQ_CENTER', 'TXPOWER']
        self.parameters_chk =['ENCRYPTION_ENABLE', 'FEC_ENABLE']
        self.requestack_modes =['REQUESTACK', 'Receiver Mode NACK', 'Receiver Mode GATEWAY', 'ACK LED Blip']
        self.requestack_values =[]


        #Labels and Entry Widgets
        i=0

        self.parameter_values = list()
        for p in self.parameters:
            #Label
            self.label = tk.Label(self.mainframe, text=p)
            self.label.grid(row=i, column=0, pady=2, padx=5, sticky='e')
            #Value
            self.idx, ok = mem.index_of_param(p)
            self.v = tk.StringVar(self, mem.paramvalue[self.idx])
            self.parameter_values.append(self.v)
            #entry widget
            self.e = tk.Entry(self.mainframe, textvariable=self.v, width=6, state=tk.NORMAL)
            self.e.grid(row=i, column=1, sticky="w")
            self.e._name = p
            self.e.bind("<Return>", self.on_enter) # same as command= in constructor of entry
            i += 1
        last_row = i

        for i,p in enumerate(self.parameters_chk):
            #Label
            self.label = tk.Label(self.mainframe, text=p)
            self.label.grid(row=i+last_row, column=0, pady=2, padx=5, sticky='e')
            #Value
            self.idx, ok = mem.index_of_param(p)
            self.v = tk.StringVar(self, mem.paramvalue[self.idx])
            self.parameter_values.append(self.v)
            #Checkbox
            self.chk = tk.Checkbutton(self.mainframe, text='', variable=self.v, onvalue='1', offvalue='0')
            self.chk.grid(row=i+last_row, column=1, padx=0, pady=0, sticky ='w')
            self.chk._name = p
        last_row += i+1


        j=0
        self.idx, ok = mem.index_of_param('REQUESTACK')
        self.requestack_val =  mem.paramvalue[self.idx]
        #print((self.requestack_val>>0)&0x1, (self.requestack_val>>1)&0x1, (self.requestack_val>>2)&0x1)

        for i, p in enumerate(self.requestack_modes):
            #Label
            self.label = tk.Label(self.mainframe, text=p)
            self.label.grid(row=i+last_row, column=0, pady=2, padx=5, sticky='e')
            #value
            self.v = tk.IntVar(self, (self.requestack_val>>i)&0x1)
            self.requestack_values.append(self.v)
            #Checkbox
            self.chk = tk.Checkbutton(self.mainframe, text='', variable=self.requestack_values[i], onvalue=1, offvalue=0, command=self.on_checkbox)
            self.chk.grid(row=i+last_row, column=1, padx=0, pady=0, sticky ='w')


        # extra Label for tx Power
        i=  self.parameters.index('TXPOWER')
        self.label_power_dbm = tk.Label(self.mainframe, text='xxx dBm')
        self.label_power_dbm.grid(row=i, column=2, pady=0, padx=5, sticky='w')
        pl = int(self.parameter_values[i].get())
        self.label_power_dbm['text'] = '%i dBm' % (13 - 31 + pl)

        #self.line = tk.Frame(self.mainframe, height=2, highlightthickness=2, highlightbackground="gray")
        #self.line.grid(row=last_row, column=0, columnspan=5, padx=0, pady=5, sticky="ew")


        #OK CANCEL Buttons
        self.helpframe = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.helpframe.grid(row=2, column=0, padx=10, pady=25, sticky="e")

        self.apply_btn = tk.Button(self.helpframe, text='Apply', command=self.on_apply, width=7)
        self.apply_btn.pack(side='right')
        #self.apply_btn.grid(row= 0, column=1, padx=0, pady=20)#, sticky = 'se')
        self.help_btn = tk.Button(self.helpframe, text='Help', command=self.on_help, width=7).pack(side='left')
        #self.help_btn.grid(row= 0, column=0, padx=0, pady=20, sticky = 'se')
        self.ok_btn = tk.Button(self.helpframe, text='OK', command=self.on_ok, width=7)
        self.ok_btn.pack(side='left')
        #self.ok_btn.grid(row= 0, column=3, padx=0, pady=20, sticky = 'se')
        self.cancel_btn = tk.Button(self.helpframe, text='Cancel', command=self.on_cancel, width=7).pack(side='left')
        #self.cancel_btn.grid(row= 0, column=2, padx=0, pady=20, sticky = 'se')


    def on_help(self):
        helpwin = Help(self,self.parameters + self.parameters_chk, 'TiNo2 Network Setup Help')
        helpwin.grab_set()

    def on_checkbox(self):
        self.requestack_val =0
        for i in range (len(self.requestack_values)):
            self.requestack_val += self.requestack_values[i].get()<<i
        print(self.requestack_val)
        idx,ok = mem.index_of_param('REQUESTACK')
        self.validate (self.requestack_val, idx)

    def on_enter(self, event): # just make sure the entry is plausible
        name = event.widget._name
        idx,ok = mem.index_of_param(name)
        valstr = event.widget.get()

        print (name, idx, valstr)

        if name == 'TXPOWER':
            pl = int(valstr)
            dbm = 13 - 31 + pl
            labelstr = '%i dBm' % dbm
            self.label_power_dbm['text'] = labelstr

        value, isok = self.parent.validate_entry(valstr, idx, False)
        if not isok:
           msgbox.showinfo(title='Error', message='validation FAIL!')
        #else:
            #mem.paramvalue[idx] = value

    def validate(self, value, idx):
        v, isok = self.parent.validate_entry(str(value), idx)
        if not isok:
           msgbox.showinfo(title='Error', message='validation FAIL!')
        else:
            mem.paramvalue[idx] = v

    def get_index(self, param):
        idx, ok = mem.index_of_param(param)
        return idx

    def on_apply(self):
        for i, p in enumerate(self.parameters):
            idx = self.get_index(p)
            v, isok = self.parent.validate_entry(self.parameter_values[i].get(), idx)
            if not isok:
                msgbox.showinfo(title='Error', message='validation FAIL!')
            else:
                mem.paramvalue[idx] = v

        i = i+1
        for j, p in enumerate(self.parameters_chk):
            idx = self.get_index(p)
            v, isok = self.parent.validate_entry(self.parameter_values[j+i].get(), idx)
            if not isok:
                msgbox.showinfo(title='Error', message='validation FAIL!')
            else:
                mem.paramvalue[idx] = v



        self.parent.populate_eeprom()

    def on_cancel(self):
        self.destroy()

    def on_ok(self):
        self.on_apply()
        self.on_cancel()


class Flash(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)

        self.parent = parent
        """Start subprocess, make GUI widgets."""

        self.title("TiNo2 Firmware Flash Utility")
        self.proc = None

        # start button
        self.start_btn= tk.Button(self, text="Start", width=7, command=self.start)
        self.start_btn.grid(row=0, column=2, pady=5, padx=10, sticky='w')

        # stop subprocess using a button
        self.stop_btn = tk.Button(self, text="Exit", width=7, command=self.stop)
        self.stop_btn.grid(row=0, column=3, pady=5, padx=0, sticky ='w')

        self.Processorlabel = tk.Label(self, text='Processor:') # put subprocess output here
        self.Processorlabel.grid(row=0, column=0, pady=5, padx=5, sticky='w')

        self.processor_combobox = ttk.Combobox(self, values=["ATmega4808", "AVR64DD32", "AVR63DD28"], state="readonly", width=15)
        self.processor_combobox.set("ATmega4808")
        self.processor_combobox.grid(row=0, column=1, padx=0, pady=0,sticky="w")

        self.file_select_btn =  tk.Button(self, text="Datei:", command=self.select_file)
        self.file_select_btn.grid(row=1, column=0, padx=10, pady=0,sticky="w")
        #self.file_select_btn.grid(row=0, column=4, padx=0, pady=0)

        #self.Filenamelabel = tk.Label(self, text='filename:') # filename label
        #self.Filenamelabel.grid(row=1, column=0, pady=0, padx=10, sticky='w')

        self.Filenamepath = tk.Label(self, text='') # path to filename
        self.Filenamepath.grid(row=1, column=1, columnspan = 3, pady=0, padx=0, sticky='w')
        self.Filenamepath['text'] = flash_cmd[9].split(':')[2]

        self.log_text = scrolledtext.ScrolledText(self, wrap=tk.WORD, width=80, height=1)
        self.log_text.grid(row=2, column=0, columnspan=4, padx=10, pady=5, sticky="nsew")
        #self.columnconfigure (0, weight=0)
        #self.columnconfigure (1, weight=0)
        #self.columnconfigure (2, weight=0)
        self.columnconfigure (3, weight=1)
        #self.columnconfigure (4, weight=0) #does not work

        # Create a buffer for the stdout
        self.stdout_data = ""
        # Create a new thread that will read stdout and write the data to
        # `self.stdout_buffer`
        #thread = Thread(target=self.read_output, args=(self.proc.stdout, ))
        #thread.start()

        ### COM Port ###
        flash_cmd[3] = '-P' + self.parent.port_combobox.get()

    def select_file(self):
         files = [('Intel Hex file', '*.hex')]
         filename = askopenfile(filetypes = files, defaultextension = files)
         #print(filename.name)
         c = '-Uflash:w:' + filename.name + ':i'
         flash_cmd[9] = c
         self.Filenamepath['text'] = filename.name

    def start(self):
        # start subprocess
        processor = self.processor_combobox.get()
        if processor == 'ATmega4808':
            flash_cmd[8] = '-patmega4808'
        elif processor == 'AVR64DD32':
            flash_cmd[8] = '-pavr64dd32'
        elif processor == 'AVR63DD28':
            flash_cmd[8] = '-pavr64dd28'

        self.log_text.delete(1.0, tk.END)

        if self.parent.connection_active == True:
            self.parent.connect()

        self.proc = Popen(flash_cmd, stdout=PIPE, stderr=STDOUT, creationflags=CREATE_NO_WINDOW)
        flash_thread = threading.Thread(target=self.read_output, args=(self.proc.stdout, ))
        flash_thread.start()
        #print(f'active threads: {threading.active_count()}')


    def read_output(self, pipe):
        """Read subprocess' output and store it in `self.stdout_data`."""
        while True:
            data = os.read(pipe.fileno(), 1 << 20)
            if data:
                datadec = data.decode()
                if len(datadec) > 0:
                    self.stdout_data = datadec
                    self.show_stdout()
            else:  # clean up
                self.end_subprocess()
                return None

    def show_stdout(self):
        """Read `self.stdout_data` and put the data in the GUI."""
        #self.Mylabel.config(text=self.stdout_data.strip("\n"))
        if len(self.stdout_data) >0:
            if self.stdout_data == '#':
                self.log_text.insert(tk.END, self.stdout_data)
            else:
                self.parent.log_text.insert(tk.END, '\n' + self.stdout_data)
            self.parent.log_text.see(tk.END)
            self.stdout_data=''


    def stop(self):
        """Stop subprocess and quit GUI."""
        if self.proc:
            self.proc.terminate() # tell the subprocess to exit
            self.kill_after(countdown=5)
        self.destroy()

        # kill subprocess if it hasn't exited after a countdown
    def kill_after(countdown):
        if self.proc.poll() is None: # subprocess hasn't exited yet
            countdown -= 1
            if countdown < 0: # do kill
                print("killing")
                self.proc.kill() # more likely to kill on *nix
            else:
                self.after(1000, kill_after, countdown)
                return # continue countdown in a second

        self.proc.stdout.close()  # close fd
        self.proc.wait()          # wait for the subprocess' exit



    def end_subprocess(self):
        #print("end subprocess now")
        self.proc.terminate() # tell the subprocess to exit
        while self.proc.poll() is None:
            pass
        if self.proc.poll() != 0:
            msgbox.showerror(title='FEHLER', message='Flashen fehlgeschlagen')
        else:
            msgbox.showerror(title='Fertig', message='Flashen erfolgreich!')
        #rint(f'subprocess.poll: {self.proc.poll()}')
        self.proc.kill()
        self.proc.stdout.close()  # close fd
        self.proc.wait()          # wait for the subprocess' exit
        self.proc = None

class PassWordEntry(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)

        self.parent = parent
        self.geometry("250x150+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+150 ))
        self.title('Passwort')

        #### Headline ###
        self.head = ttk.Label(self, text='Password for DUT EEPROM', font=("Helvetica", 13))
        self.head.pack(pady=10)

        self.PwLabel = ttk.Label(self, text='Enter Password (exactly 16 characters):')
        self.PwLabel.pack()

        self.PwEntry = ttk.Entry(self, width =20)
        self.PwEntry.pack(pady=5)
        self.PwEntry.insert(0,'')
        self.PwEntry.bind("<Return>", self.on_enter_password)
        self.PwEntry.focus_set()

        self.ok_btn = ttk.Button(self, text='Submit', command=self.on_ok)
        self.ok_btn.pack(side='right', padx=10, pady=10, anchor='s')

    def on_enter_password(self, event):
        #PassWord = self.PwEntry.get().encode()
        self.on_ok()

    def on_ok(self):
        PassWord = self.PwEntry.get().encode()
        self.parent.on_password_entry_done(PassWord)
        self.destroy()


class PCIConfigWindow(tk.Toplevel):
    def __init__(self, parent, interruptpin):
        super().__init__(parent)

        self.parent = parent
        self.geometry("350x220+%d+%d" % (self.parent.winfo_rootx()+375, self.parent.winfo_rooty()+150 ))
        self.title('Pin Change Interrupt Configuration')

        #### Headline ###
        self.head = ttk.Label(self, text=interruptpin + ' Configuration', font=("Helvetica", 14))
        self.head.grid(row=0, column=0, columnspan=3, padx=0, sticky='e')

        ### some variables
        self.pcinumber = interruptpin[3]
        self.pciname = 'PCI' + self.pcinumber + 'PIN'

        self.idx, ok = mem.index_of_param(self.pciname)
        if not ok:
             msgbox.showerror(title='FATAL ERROR', message='No or wrong data!')
             self.destroy()

        self.pcipin = mem.paramvalue[self.idx]
        self.pciconfig = mem.paramvalue[self.idx+1]
        self.pcigatewayid =mem.paramvalue[self.idx+2]

        self.opmode = tk.IntVar()
        self.opmode.set(self.pciconfig >>4) #values of 0 or 2 are valid. the lower 3 bits represent possible modes. we shift one more, because bit 0 is not interesting.

        print(self.pciname, self.pcipin, self.pciconfig, self.pcigatewayid)

        self.pci_enabled = tk.IntVar(self,0)
        if (self.pcipin >= 0):
            self.pci_enabled.set(1)
        else:
            self.pci_enabled.set(0)

        ypadding=5


        ### WIDGETS ###

        # Enable Checkbox
        #self.pcienable_chk = tk.Checkbutton(self, text='enable PCI ' + self.pcinumber , variable=self.pci_enabled, onvalue=1, offvalue=0, command=self.on_pcienable)
        self.pcienable_chk = tk.Checkbutton(self, text='', variable=self.pci_enabled, onvalue=1, offvalue=0, command=self.on_pcienable)
        self.pcienable_chk.grid(row=1, column=1, columnspan=2, padx=0, pady=ypadding, sticky ='w')

        # Labels
        enablelabel = ttk.Label(self, text='enable PCI ' + self.pcinumber)
        enablelabel.grid(row=1, column=0, pady=ypadding, padx=10, sticky="e")

        self.pinlabel = ttk.Label(self, text='GPIO Pin')
        self.pinlabel.grid(row=2, column=0, pady=ypadding, padx=10, sticky="e")

        self.modelabel = ttk.Label(self, text='Mode')
        self.modelabel.grid(row=3, column=0, pady=ypadding, padx=10, sticky="e")

        self.triggerlabel = ttk.Label(self, text='Trigger')
        self.triggerlabel.grid(row=4, column=0, pady=ypadding, padx=10, sticky="e")

        self.gatewayidlabel = ttk.Label(self, text='Gateway ID')
        self.gatewayidlabel.grid(row=5, column=0, pady=ypadding, padx=10, sticky="e")

        # GPIO Pin number Entry
        self.pcipinentry = ttk.Entry(self, width =7)
        self.pcipinentry.grid(row=2, column=1, pady=ypadding, padx=0, sticky="w")
        self.pcipinentry._name = self.pciname
        self.pcipinentry.insert(0,self.pcipin)
        self.pcipinentry.bind("<Return>", self.on_enter_pcipin)
        self.pcipinentry["state"] = tk.NORMAL
        if self.pci_enabled.get() == 0:
            self.pcipinentry["state"] = tk.DISABLED

        #self.modeframe = tk.Frame(self, width=300, height=10, highlightthickness=1, highlightbackground="gray")
        #self.modeframe = tk.Frame(self, height=10)
        #self.modeframe.grid(row=3, column=1, columnspan=2, sticky="w")


        #ttk.Radiobutton(self.modeframe, text='INPUT',  variable=self.opmode, value=0).pack(side="left", padx=10, pady=1)
        #ttk.Radiobutton(self.modeframe, text='INPUT_PULLUP', variable=self.opmode, value=2).pack(side="left", padx=0, pady=1)
        ttk.Radiobutton(self, text='INPUT',  variable=self.opmode, value=0).grid(row=3, column=1, padx=0, sticky='w')
        ttk.Radiobutton(self, text='INPUT_PULLUP', variable=self.opmode, value=1).grid(row=3, column=2, padx=10, sticky='e')


        self.trigger = tk.IntVar()
        self.trigger.set(self.pciconfig &0x07)# the lower 3 bits are significant.
        self.triggerframe = tk.Frame(self, height=10)
        self.triggerframe.grid(row=4, column=1, columnspan=3, pady =ypadding, sticky="w")
        ttk.Radiobutton(self.triggerframe, text='FALLING', variable=self.trigger, value=2).pack(side='left', padx=0)
        ttk.Radiobutton(self.triggerframe, text='RISING', variable=self.trigger, value=3).pack(side='left')
        ttk.Radiobutton(self.triggerframe, text='CHANGE', variable=self.trigger, value=4).pack(side='left')

        self.pcigatewayid_entry = ttk.Entry(self, width =7)
        self.pcigatewayid_entry.grid(row=5, column=1, pady=ypadding, padx=0, sticky="w")
        self.pcigatewayid_entry.insert(0,self.pcigatewayid)
        self.pcigatewayid_entry.bind("<Return>", self.on_enter_pcipin)


        self.ok_btn = ttk.Button(self, text='Submit', command=self.on_ok)
        self.ok_btn.grid(row= 6, column=3, rowspan=6, padx=0, pady=ypadding, sticky = 'se')

        self.cancel_btn = ttk.Button(self, text='Cancel', command=self.preset_widgets)
        self.cancel_btn.grid(row= 6, column=2, padx=0, pady=ypadding, sticky = 'se')

    def preset_widgets(self): # cancel button: reset to previous
        self.pcipin = mem.paramvalue[self.idx]
        self.pciconfig = mem.paramvalue[self.idx+1]
        self.pcigatewayid =mem.paramvalue[self.idx+2]

        self.opmode.set(self.pciconfig >>4)
        self.trigger.set(self.pciconfig &0x07)

        if (self.pcipin >= 0):
            self.pci_enabled.set(1)
            self.pcipinentry["state"] = tk.NORMAL
            self.pcigatewayid_entry["state"] = tk.NORMAL

        else:
            self.pci_enabled.set(0)
            self.pcipinentry["state"] = tk.DISABLED
            self.pcigatewayid_entry["state"] = tk.DISABLED

        self.pcipinentry.delete(0, tk.END)
        self.pcipinentry.insert(0,self.pcipin)

        self.pcigatewayid_entry.delete(0, tk.END)
        self.pcigatewayid_entry.insert(0, self.pcigatewayid)


    def on_pcienable(self):# here we set the entry field to -1 or to its pin number
        if self.pci_enabled.get() == 0:
            self.pcipinentry.delete(0, tk.END)
            self.pcipinentry.insert(0,'-1')
            self.pcipinentry["state"] = tk.DISABLED
            self.pcigatewayid_entry["state"] = tk.DISABLED

        else:
            self.pcipinentry["state"] = tk.NORMAL
            self.pcipinentry.delete(0, tk.END)
            self.pcipinentry.insert(0,self.pcipin)
            self.pcigatewayid_entry["state"] = tk.NORMAL


    def on_enter_pcipin(self, event): # what to do with this?
        #self.pcipin = mem.paramvalue[self.idx]
        #self.pciconfig = mem.paramvalue[self.idx+1]
        #self.pcigatewayid =mem.paramvalue[self.idx+2]


        pass

    def on_ok(self): # take over the entries and store them into the copy of the eeprom
        # validate entries
        value, isok = self.parent.validate_entry(self.pcipinentry.get(), self.idx)
        if isok:
            mem.paramvalue[self.idx] = value

        mode = (self.opmode.get()<<4) | self.trigger.get()
        value, isok = self.parent.validate_entry(str(mode), self.idx+1)   # value is an int, therefore needs to become a str
        if isok:
            mem.paramvalue[self.idx+1] = value

        value, isok = self.parent.validate_entry(self.pcigatewayid_entry.get(), self.idx+2)
        if isok:
            mem.paramvalue[self.idx+2] = value

        self.parent.populate_eeprom()

'''
class SensorConfigWindow(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)

        self.parent = parent
        self.geometry("320x400+%d+%d" % (self.parent.winfo_rootx()+350, self.parent.winfo_rooty()+100 ))
        self.title('Sensor Selection')

        self.enable_mask = self.parent.compiler_flags<<8

        self.idx, ok  = mem.index_of_param('SENSORCONFIG')
        if not ok:
            msgbox.showinfo(title='Error', message='FAIL!')

        self.values=[]
        self.s = mem.paramvalue[self.idx]

        for i in range(len(sensors)):
            mask = 1<<i

            var = tk.IntVar(self, mask & mem.paramvalue[self.idx])

            self.values.append(var)
            self.chk = ttk.Checkbutton(self, text=sensors[i][0],variable=self.values[i], onvalue=1<<i, offvalue=0, command=self.update_sensorconfig)
            self.chk._name= str(i)
            self.chk.grid(row=i, column=0, padx=10, pady=1, sticky="nw")
            if not (self.enable_mask & mask):
                self.chk["state"] = tk.DISABLED
            if len(sensors[i]) >1:
                self.l = ttk.Label(self, text=sensors[i][1])
                self.l.grid(row=i, column=1, padx=10, sticky='sw')
            last_row=i

        self.text = ttk.Label(self, text="")
        self.text.grid(row= last_row+1, column=0, padx=10, sticky='sw')
        #self.text.place(rely=1.0, relx=0.5, x=0, y=-5, anchor='se')
        self.text['text'] = '0x%x'%(mem.paramvalue[self.idx])

        self.ok_btn = ttk.Button(self, text='OK', command=self.on_ok)
        self.ok_btn.grid(row= last_row+1, column=1, padx=0, sticky = 'se')
        #self.ok_btn.place(rely=1.0, relx=1.0, x=-10, y=-5, anchor='se')

        self.close_btn =  ttk.Button(self, text='Close', command=self.on_close)
        #self.close_btn.place(rely=1.0, relx=0.8, x=-10, y=-5, anchor='se')
        self.close_btn.grid(row= last_row+1, column=2, rowspan = last_row+1, padx=0, sticky = 'sw')

    def update_sensorconfig(self):
        self.s=0
        for i in range(16):
            self.s = self.s | self.values[i].get()
        #print(s)
        self.text['text'] = '0x%x'%(self.s)
        pass

    def on_ok(self):
        mem.paramvalue[self.idx] = self.s
        self.parent.on_sensor_config_window_close(self.s)

    def on_close(self):
        self.on_ok()
        self.destroy()
'''

class SensorConfigWindow(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)

        #self.grid_rowconfigure(1, weight=1)
        #self.grid_columnconfigure(0, weight=1)

        self.parent = parent
        self.geometry("+%d+%d" % (self.parent.winfo_rootx()+350, self.parent.winfo_rooty()+100 ))
        self.title('Sensor Selection')

        #### Headline ###
        self.head = ttk.Label(self, text='External Sensors Configuration', font=("Helvetica", 13))
        self.head.grid(row=0, column=0, columnspan=3, padx=0, pady=5,)

        self.head1frame = tk.Frame(self, highlightthickness=0, highlightbackground="gray", height=50)
        self.head1frame.grid(row=1, column=0, padx=0)

        self.head1 = ttk.Label(self.head1frame, text='List of sensors supported by firmware of the connected device')
        self.head1.grid(row=0, column=0, columnspan=3, padx=0, pady=5, sticky='n')

        self.mainframe = tk.Frame(self, highlightthickness=0, highlightbackground="gray")
        self.mainframe.grid(row=2, column=0, padx=10, sticky="nwse")


        self.okcancel_frame = tk.Frame(self, width=10, height=10, highlightthickness=0, highlightbackground="gray")
        self.okcancel_frame.grid(row=3, column=0, columnspan=3, padx=10, pady=15, sticky="w")

        #Create bold font from standard display font
        default_font = font.nametofont('TkTextFont').actual()
        bold_font=(default_font['family'], default_font['size'], "bold")

        #self.enable_mask = self.parent.compiler_flags<<8 | 0x80 #PIR is always part of the firmware
        self.enable_mask = self.parent.compiler_flags<<8

        self.idx, ok  = mem.index_of_param('SENSORCONFIG')
        if not ok:
            msgbox.showinfo(title='Error', message='FAIL!')
        self.s = mem.paramvalue[self.idx]

        # table headline
        headline= ['Device', 'Enable', 'Sensortype', 'CS/Data Pin', 'Vcc Pin']
        for i, p in enumerate(headline):
            self.label = ttk.Label(self.mainframe, text=p, font=bold_font, anchor=tk.CENTER)
            self.label.grid(row=0, column=i, padx=5, sticky='ew')


        self.enable_values=[]
        self.cs_data_entries = [] # [[pinname1, valstr1], [pinname2, valstr2]...]
        self.vcc_entries=[]

        r=1
        for i in range(len(sensors)):
            mask = 1<<i

            var = tk.IntVar(self, mask & mem.paramvalue[self.idx])
            self.enable_values.append(var)

            if self.enable_mask & mask:
                self.label = ttk.Label(self.mainframe, text=sensors[i][0])
                self.label.grid(row=r, column=0, padx=0, pady=1, sticky ='ens')
                self.chk = tk.Checkbutton(self.mainframe,variable=self.enable_values[i], onvalue=1<<i, offvalue=0, command=self.update_sensorconfig)
                self.chk._name= str(i)
                self.chk.grid(row=r, column=1, padx=5, pady=1, sticky="ew")
                if len(sensors[i]) >1:
                    self.l = ttk.Label(self.mainframe, text=sensors[i][1])
                    self.l.grid(row=r, column=2, padx=0, sticky='we')
                if len(sensors[i]) >2: # CS / DATA PIns
                    csidx, ok = mem.index_of_param(sensors[i][2])
                    if not ok:
                        self.l = ttk.Label(self.mainframe, text=sensors[i][2])
                        self.l.grid(row=r, column=3, padx=0)#, sticky='ew')
                    else:
                        cspin = mem.paramvalue[csidx]
                        self.strvar = tk.StringVar(self, cspin)
                        self.entry = tk.Entry(self.mainframe, textvariable =self.strvar, width=4, font=('Courier New', 10), relief='sunken')
                        self.entry._name= sensors[i][2]
                        self.entry.grid(row=r, column=3, padx=5, pady=0)#, sticky='we')
                        self.entry.bind("<Return>", self.check)
                        self.cs_data_entries.append([sensors[i][2],self.strvar])
                if len(sensors[i]) >3:
                    vccidx, ok = mem.index_of_param(sensors[i][3])
                    if not ok:
                        self.l = ttk.Label(self.mainframe, text=sensors[i][3])
                        self.l.grid(row=r, column=4, padx=0)#, sticky='we')
                    else:
                        vccpin = mem.paramvalue[vccidx]
                        self.strvar = tk.StringVar(self, vccpin)
                        self.entry = tk.Entry(self.mainframe, textvariable =self.strvar, width=4, font=('Courier New', 10), relief='sunken')
                        self.entry._name= sensors[i][3]
                        self.entry.grid(row=r, column=4, padx=0, pady=0)
                        self.entry.bind("<Return>", self.check)
                        self.vcc_entries.append([sensors[i][3],self.strvar])
                r = r+1
            last_row=r

        #self.label = ttk.Label(self.mainframe, text='retrigger\ndelay', font= bold_font)
        #self.label.grid(row=last_row, column=4, padx=0, pady=1, sticky ='e')

        self.label = ttk.Label(self.mainframe, text='PIR')
        self.label.grid(row=last_row+1, column=0, padx=0, pady=2, sticky ='e')

        self.l = ttk.Label(self.mainframe, text='Motion Sensor')
        self.l.grid(row=last_row+1, column=2, padx=0, sticky='w')

        self.l = ttk.Label(self.mainframe, text='VBATT')
        self.l.grid(row=last_row+1, column=4, padx=0)

        idx, ok = mem.index_of_param('PIRDATAPIN')
        self.pir_data_pin = tk.StringVar(self, mem.paramvalue[idx])
        self.pir_data_pin_entry = tk.Entry(self.mainframe, textvariable =self.pir_data_pin, width=4, font=('Courier New', 10))
        self.pir_data_pin_entry._name= 'PIRDATAPIN'
        self.pir_data_pin_entry.grid(row=last_row+1, column=3, padx=5, pady=0)#, sticky='w')
        self.cs_data_entries.append(['PIRDATAPIN',  self.pir_data_pin])

        self.pirframe = tk.Frame(self.mainframe, height=2, highlightthickness=2, highlightbackground="gray")
        self.pirframe.grid(row=last_row+2, column=0, columnspan=5, padx=0, pady=5, sticky="ew")

        self.label = ttk.Label(self.mainframe, text = 'PIR Retrigger Delay', justify='right')
        self.label.grid(row=last_row+3, column=0, columnspan = 2, padx=0, pady=2, sticky='ens')

        idx, ok = mem.index_of_param('PIRDEADTIME')
        self.pir_deadtime = tk.StringVar(self, mem.paramvalue[idx])
        self.pir_deadtime_entry = tk.Entry(self.mainframe, textvariable =self.pir_deadtime, width=4, font=('Courier New', 10))
        self.pir_deadtime_entry._name= 'PIRDEADTIME'
        self.pir_deadtime_entry.grid(row=last_row+3, column=3, padx=5, pady=2, sticky='s')
        self.pir_deadtime_entry.bind("<Return>", self.check)

        self.label = ttk.Label(self.mainframe, text = 'ticks')
        self.label.grid(row=last_row+3, column=4, padx=5, sticky = 'w')

        #self.pirframe = tk.Frame(self.mainframe, height=1, highlightthickness=2, highlightbackground="gray")
        #self.pirframe.grid(row=last_row+4, column=0, columnspan=5, padx=0, pady=0, sticky="ew")


        self.text = ttk.Label(self.okcancel_frame, text="", width = 10)
        self.text.grid(row= 0, column=0, padx=10, sticky='sw')
        self.text['text'] = '0x%x'%(mem.paramvalue[self.idx])

        self.ok_btn = tk.Button(self.okcancel_frame, text='OK', width=10, command=self.on_ok)
        self.ok_btn.grid(row= 0, column=1, padx=0, sticky = 'se')

        self.close_btn =  tk.Button(self.okcancel_frame, text='Close', width = 10, command=self.on_close)
        self.close_btn.grid(row=0, column=2, padx=0, sticky = 'sw')

    def validate (self, valstr, idx):
        value, ok = self.parent.validate_entry(valstr, idx)
        if not ok:
            msgbox.showinfo(title='Error', message='validation FAIL!')
        else:
            mem.paramvalue[idx] = value

    def update_sensorconfig(self):
        self.s=0
        for i in range(16):
            self.s = self.s | self.enable_values[i].get()
        self.text['text'] = '0x%x'%(self.s)
        self.validate(str(self.s), self.idx)

    def on_ok(self):
        for l in self.cs_data_entries:
            idx, ok = mem.index_of_param(l[0])
            valstr = l[1].get()
            self.validate(valstr, idx)
        for l in self.vcc_entries:
            idx, ok = mem.index_of_param(l[0])
            valstr = l[1].get()
            self.validate(valstr, idx)
            
        idx, ok = mem.index_of_param('PIRDEADTIME')
        self.validate(self.pir_deadtime.get(), idx)

        self.parent.populate_eeprom()

    def on_close(self):
        self.on_ok()
        self.destroy()

    def check(self, event):
        name = event.widget._name
        idx,ok = mem.index_of_param(name)
        valstr = event.widget.get()
        self.validate (valstr, idx)

        self.parent.populate_eeprom()


        #for l in self.cs_data_entries:
            #print(l[0], l[1].get())
        #for l in self.vcc_entries:
            #print(l[0], l[1].get())


class Eeprom:
    pop_done = False
    actions_pop_done = False
    def __init__ (self, eepromdefinition):
        #eepromdefinition is a list of [paramname, paramtype] pairs with optional third field
        self.parameter = list()  # names
        self.paramtype= list()   # types as string
        self.paramvalue = list() # array of numerical values
        self.paramindex = list() # pointer into raw structure for each item
        self.param_entry_widgets = list() # list of the Entry widgets for each parameter
        self.param_is_readonly = list() # future extention to mask non writable items

        idx =0
        for item in eepromdefinition:
            self.parameter.append(item[0])
            if len(item) > 2:
                if item[2] == 'r':
                    self.param_is_readonly.append(item[0])
            t = item[1]
            self.paramtype.append(t)
            self.paramindex.append(idx)
            self.paramvalue.append('')
            if t == 'i':
                idx = idx +2
            elif t == 'f':
                idx = idx+4
            else:
                idx = idx+1
        self.configuration_size = idx
        self.adr_num_actions = self.configuration_size
        self.adr_actions = self.adr_num_actions +1
        #self.max_num_actions= int((256-3-self.configuration_size)/4)
        self.max_num_actions=40
        #print(self.max_num_actions)
        self.adr_actions_crc = self.max_num_actions*4 + self.adr_actions
        self.actions = list()
        self.number_of_actions = 0
               
        # print('size of Config: ', self.configuration_size)

    def index_of_param(self, param):
        if param in self.parameter:
            return self.parameter.index(param), True
        else:
            return '', False

    def type_of_param(self, param):
        idx,ok = self.index_of_param(param)
        if ok:
            return  self.paramtype[idx], True
        else:
            return -1, False

    def load_values(self, values):
        #values is a comma-delimited string like '22,220,23, ...'
        raw = values.split(',')
        #print(len(raw))
        self.paramvalue.clear()
        try:
            for i in range(len(self.parameter)):
                idx = self.paramindex[i] # look up the pointer for each item
                if self.paramtype[i] == 'b':
                    v = struct.pack('B', int(raw[idx])) # char[255] as a bytes array
                    v = struct.unpack('b',v)[0]  # from char to signed char
                    self.paramvalue.append(v)

                elif self.paramtype[i] == 'B':
                    self.paramvalue.append(int(raw[idx]))

                elif self.paramtype[i] == 'i':
                    p= bytearray()
                    p.append(int(raw[idx]))
                    p.append(int(raw[idx+1]))
                    self.paramvalue.append(struct.unpack('H', p)[0])

                elif self.paramtype[i] == 'f':
                    p= bytearray()
                    p.append(int(raw[idx]))
                    p.append(int(raw[idx+1]))
                    p.append(int(raw[idx+2]))
                    p.append(int(raw[idx+3]))
                    f = struct.unpack('f', p)[0]
                    self.paramvalue.append(round(f,2))
                else:
                    print("there was an error")
        except Exception as e:
            print( f"Error loading values: {str(e)}\n")



    def value_of_param(self, param):
        #returns value of a parameter as a string
        idx,ok = self.index_of_param(param)
        if not ok:
            return -1000, False
        return self.paramvalue[idx], True

    def parameter_is_readonly(self, p):
        is_readonly = False
        if p in self.param_is_readonly:
            is_readonly = True
        return is_readonly


class SerialMonitor(tk.Tk):
    membuf=0
    
    def __init__(self):
        super().__init__()

        self.iconbitmap("tino_icon.ico")
        self.title("TiNo2 Configuration and Calibration")
        self.geometry("1050x600")
        #self.geometry("")
        
        self.create_widgets()

        # Flag to indicate if the serial connection is active
        self.connection_active = False

        # Flag to indicate whether the node is in calibration mode
        self.calmode_active = False

        # store the last command sent
        self.cmd_sent = b""

        self.new_window_is_open = False

        self.protocol("WM_DELETE_WINDOW", self.window_exit)

        #compiler flags indicating if certain sensor drivers are available
        self.compiler_flags = 0

        self.Reference_ADC = tk.IntVar(self,0)
        self.Measured_VCC  = tk.IntVar(self,0)
        
        self.radio_temperature = tk.StringVar(self,'')
        
        #make receiver messages available to child windows
        self.device_message = tk.StringVar(self,'')

    def create_widgets(self):
        ### ttk Styles ###
        ttk.Style().configure("TButton", padding=1, relief="raised", background="#fff")
        
        ### PORT LABEL #####
        port_label = ttk.Label(self.master, text="Port:")
        port_label.grid(row=0, column=0, padx=10, pady=2, sticky="e")
        port_label.bind("<Button-1>",self.port_combobox_refresh)

        ### PORT COMBOBOX ###
        self.populate_ports()

        ### MODE RADIOBUTTONS ###
        # frame to contain the radio buttons
        self.modeframe = tk.Frame(self.master, width=10, height=10, highlightthickness=1, highlightbackground="gray")
        self.modeframe.grid(row=1, column=0, columnspan=2, sticky="e")

        self.opmode = tk.IntVar()
        ttk.Radiobutton(self.modeframe, text='Monitor', variable=self.opmode, value=0).pack(side="right", padx=1, pady=1)
        ttk.Radiobutton(self.modeframe, text='Config',  variable=self.opmode, value=1).pack(side="left", padx=1, pady=1)
        self.opmode.set(1)

        ### BAUD COMBOBOX ###
        self.baud_combobox_label = ttk.Label(self, text="Baud:")
        self.baud_combobox_label.grid(row=0, column=2, padx=1, pady=2, sticky="e")

        self.baud_combobox = ttk.Combobox(self.master, values=["Sender", "Receiver", "9600","19200", "38400", "57600", "115200", "230400"], state="readonly", width=8)
        self.baud_combobox.set("Sender")
        self.baud_combobox.grid(row=0, column=3, padx=0, pady=0,sticky="w")

        ### CONNECT / DISCONNECT BUTTON ###
        self.connect_button = ttk.Button(self.master, text="Connect", command=self.connect, width=10)
        self.connect_button.grid(row=1, column=3, padx=0, pady=0, sticky="w")

        self.load_eeprom_btn = ttk.Button(self.master, text="Load<-EEPROM", command=self.load_eeprom, state=tk.DISABLED)
        self.load_eeprom_btn.grid(row=0, column=4, padx=0, pady=5, sticky='w')

        self.checksum_btn = ttk.Button(self.master, text="Checksum", command=self.calculate_checksum, state=tk.DISABLED)
        self.checksum_btn.grid(row=0, column=5, padx=0, pady=5, sticky = 'w')

        self.save_eeprom_btn = ttk.Button(self.master, text="Save EEPROM", command=self.save_eeprom, state=tk.DISABLED)
        self.save_eeprom_btn.grid(row=1, column=4, padx=0, pady=5, sticky='w')

        self.write_eeprom_btn = ttk.Button(self.master, text="Copy->EEPROM", command=self.write_eeprom, state=tk.DISABLED)
        self.write_eeprom_btn.grid(row=1, column=5, padx=0, pady=5, sticky='w')

        #self.vddcal_btn = ttk.Button(self.master, text="Vbatt Cal.", command=self.vddcal, state=tk.NORMAL)
        #self.vddcal_btn.grid(row=1, column=6, padx=0, pady=5, sticky='w')

        #self.vddmeas_btn = ttk.Button(self.master, text="Vbatt Measure", command=self.measure_vdd, state=tk.NORMAL)
        #self.vddmeas_btn.grid(row=1, column=7, padx=0, pady=5, sticky='w')

        self.flash_btn = ttk.Button(self.master, text="Flash Firmware", command=self.flash, state=tk.NORMAL)
        self.flash_btn.grid(row=1, column=7, columnspan=2, padx=0, pady=5, sticky='w')

        ### Test incomming Message Button
        self.incoming_btn = ttk.Button(self.master, text="View Incoming Messages", command=self.IncomingMsg, state=tk.NORMAL)
        self.incoming_btn.grid(row=0, column=8, padx=1, pady=5, sticky="e")
        
        ### Save Button
        self.save_file_btn = ttk.Button(self.master, text="Save to File", command=self.save_file, state=tk.NORMAL)
        self.save_file_btn.grid(row=0, column=9, padx=1, pady=5, sticky="e")

        ### open Button
        self.open_file_btn = ttk.Button(self.master, text="Open File", command=self.open_file, state=tk.NORMAL)
        self.open_file_btn.grid(row=0, column=10, padx=1, pady=5, sticky="w")


        self.frame_config_buttons = tk.Frame(self.master, width=50, height=540)#,highlightthickness=1, highlightbackground="gray")
        self.frame_config_buttons.grid(row=2, column=7, rowspan=6, sticky="nw")

        self.network_config_btn = tk.Button(self.frame_config_buttons, text="Network\nConfiguration", width=11, command=self.Network_Config_Window_open, state=tk.DISABLED)
        self.network_config_btn.grid(row=0, column=0, padx=1, pady=5, sticky="new")

        self.radio_config_btn = tk.Button(self.frame_config_buttons, text="Radio\nConfiguration", width=11, bd=2, command=self.Radio_Config_Window_open, state=tk.DISABLED)
        self.radio_config_btn.grid(row=1, column=0, padx=1, pady=5, sticky="nw")

        self.pci_btn = tk.Button(self.frame_config_buttons, text="Pin Interrupt\nConfiguration", width=11, height=2, bd=2, command=self.PCISetup_open, state=tk.DISABLED)
        self.pci_btn.grid(row=2, column=0, padx=1, pady=5, sticky="nw")

        self.vcccal_btn = tk.Button(self.frame_config_buttons, text="Battery Voltage\nCalibration", width=11, height=2, bd=2, command=self.Battery_Calibration_open, state=tk.DISABLED)
        self.vcccal_btn.grid(row=3, column=0, padx=1, pady=5, sticky="nw")

        self.hardware_btn = tk.Button(self.frame_config_buttons, text="Hardware\nConfiguration", width=11, height=2, bd=2, command=self.Hardware_Setup_open, state=tk.DISABLED)
        self.hardware_btn.grid(row=5, column=0, padx=1, pady=5, sticky="nw")

        self.sensor_btn = tk.Button(self.frame_config_buttons, text="Sensors\nConfiguration", width=11, height=2, bd=2, command=self.Sensor_Setup_open, state=tk.DISABLED)
        self.sensor_btn.grid(row=4, column=0, padx=1, pady=5, sticky="nw")

        self.firmware_btn = tk.Button(self.frame_config_buttons, text="Firmware\nSetup", width=11, height=2, bd=2, command=self.Firmware_Setup_open, state=tk.DISABLED)
        self.firmware_btn.grid(row=6, column=0, padx=1, pady=5, sticky="nw")
        
        self.actions_btn = tk.Button(self.frame_config_buttons, text="Actions", width=11, height=2, bd=2, command=self.ActionsWindowOpen, state=tk.DISABLED)
        self.actions_btn.grid(row=7, column=0, padx=1, pady=5, sticky="nw")

        #self.lora_config_btn['state'] = tk.DISABLED
        #self.hw_config_btn['state'] = tk.DISABLED
        #self.sw_config_btn['state'] = tk.DISABLED
        #self.vcc_cal_btn['state'] = tk.DISABLED

        ### Frames for configuration editor
        r=2
        self.frame_config_label1 = tk.Frame(self.master, width=50, height=540)
        self.frame_config_label1.grid(row=r, column=8, rowspan=5, sticky="nw")

        self.frame_config_values1 = tk.Frame(self.master, width=50, height=540)
        self.frame_config_values1.grid(row=r, column=9, rowspan=5, sticky="nw")

        self.frame_config_label2 = tk.Frame(self.master, width=50, height=540)
        self.frame_config_label2.grid(row=r, column=10, rowspan=5, sticky="nw")

        self.frame_config_values2 = tk.Frame(self.master, width=50, height=540)
        self.frame_config_values2.grid(row=r, column=11, rowspan=5, sticky="nw")


        ### Entry widget for commands ###
        self.testvar = tk.StringVar(self.master, "")
        self.testlabeltext = tk.StringVar(self.master, "Command:")
        self.testlabel = ttk.Label(self.master, text=self.testlabeltext.get())
        self.testlabel.grid(row=0, column=6, pady=5, sticky="e")
        self.testlabel.bind("<Button>", self.test_on_button)

        self.testentri = ttk.Entry(self.master, textvariable=self.testvar, state=tk.DISABLED)
        self.testentri.grid(row=0, column=7, sticky="w")
        self.testentri["width"] = 10
        self.testentri._name = "entry_command"
        self.testentri.bind("<Return>", self.test_on_enter) # same as command= in constructor of entry

        self.log_text = scrolledtext.ScrolledText(self.master, wrap=tk.WORD, width=60, height=31)
        #self.log_text = scrolledtext.ScrolledText(self.master, wrap=tk.WORD, height=31)
        self.log_text.grid(row=2, column=0, columnspan=7, padx=5, pady=10, sticky='w')

        #self.columnconfigure (10, weight=1)
        #self.columnconfigure (0, weight=1)
        
    def IncomingMsg(self):
        WatchMyNode = WatchNodes(self)
        
    def ActionsWindowOpen(self):
        ActionsWin = ActionsWindow(self)
        ActionsWin.grab_set()
        '''
        print('actions window open')
        if self.connection_active and self.calmode_active:
            self.cmd_sent = b'NumberOfActions'
            cmd = b'r,%i\n' % mem.adr_num_actions
            self.ser.write(cmd)

            while (self.cmd_sent != b''):
                time.sleep(0.01)
            
            if mem.number_of_actions > mem.max_num_actions: #invalid
                print('invalid actions block')
            elif not mem.actions_pop_done:
                
                for i in range(mem.number_of_actions*4): #range(mem.adr_actions, mem.adr_actions+mem.number_of_actions*4)
                    cmd = b'r,%i\n' % (i+mem.adr_actions)
                    self.cmd_sent= cmd
                    self.ser.write(cmd)
                    time.sleep(0.01)
                    while (self.cmd_sent != b''):
                        time.sleep(0.01)
                    mem.actions.append(self.membuf)   
                print (mem.actions)
                
                mem.actions.clear()
                j=mem.adr_actions
                for i in range(mem.number_of_actions):
                    single_action = list()
                    for k in range (4):
                        cmd = b'r,%i\n' % (j)
                        self.cmd_sent= cmd
                        self.ser.write(cmd)
                        time.sleep(0.01)
                        while (self.cmd_sent != b''):
                            time.sleep(0.01)
                        single_action.append(self.membuf)
                        j+=1
                    mem.actions.append(single_action)
                    
                mem.actions_pop_done = True
                print (mem.actions)
                
        else:
            print('connection inactive or not in calibration mode')
        '''
        
    def Firmware_Setup_open(self):
        FirmwareWindow = FirmwareSetup(self)
        FirmwareWindow.grab_set()

    def Sensor_Setup_open(self):
        SensorWindow = SensorConfigWindow(self)
        SensorWindow.grab_set()

    def Hardware_Setup_open(self):
        HardwareWindow = HardwareSetup(self)
        HardwareWindow.grab_set()

    def PCISetup_open(self):
        PciCfgWindow = PCISetup(self)
        PciCfgWindow.grab_set()

    def Battery_Calibration_open(self):
        BatteryCfgWin = Battery_Calibration(self)
        BatteryCfgWin.grab_set()

    def Network_Config_Window_open(self):
        NetworkCfgWin= NetworkSetup(self)
        NetworkCfgWin.grab_set()

    def Radio_Config_Window_open(self):
        RadioCfgWin= RadioSetup(self)
        RadioCfgWin.grab_set()

    def flash(self):
        flashwin = Flash(self)
        flashwin.grab_set()

    def notifyme(self):
        print("notify")

    def test_on_button(self, event):
        msg = command_msg
        msgbox.showinfo('HELP', msg)


    def test_on_enter(self, event):
        cmd = event.widget.get().encode()
        self.cmd_sent = cmd
        cmd = cmd + b'\n'
        self.ser.write(cmd)

    def on_password_entry_done(self, newPassWord):
        cmd = newPassWord + b'\n'
        PassWord=newPassWord
        self.cmd_sent = cmd
        self.ser.write(cmd)

    ### valuestr :  a str object that contains the value that we check with redex
    ### idx:        the index at which the valuestr object is located in the mem.parameter list
    ### write2ram:  if True the validated value is written to the RAM of the device under test
    def validate_entry(self, valuestr, idx, write2ram=True):
        typ= mem.paramtype[idx]
        pidx = mem.paramindex[idx]
        #print(pidx)
        isok = False
        value = None
        serialOK = self.connection_active and self.calmode_active
        if typ == 'B': # uint8_t
            pattern= '\\b(0x[0-9a-fA-F]+|[0-9]+)\\b'
            if re.match(pattern, valuestr) is not None:
                if valuestr[:2] == '0x':
                    value =  int(valuestr,16)
                else:
                    value =  int(valuestr)
            if value is not None:
                if 0 <= value  and value <= 255:
                    if serialOK and write2ram:
                        self.ser.write(b'w,%i,%i\n' %(pidx, value))
                    isok = True

        elif typ == 'b': #int8_t or char
            pattern = '^[-+]?[0-9]+$'
            if re.match(pattern, valuestr) is not None:
                value = int(valuestr)
            if value is not None:
                if -128 <= value  and value < 128:
                    v = ord(struct.pack('b', value))
                    if serialOK and write2ram:
                        self.ser.write(b"w,%i,%i\n" % (pidx, v))
                    isok = True

        elif typ == 'i': # uint16_t
            #pattern= '\\b(0x[0-9a-fA-F]+|[0-9]+)\\b'
            pattern = '^[-+]?[0-9x]+$'
            if re.match(pattern, valuestr) is not None:
                if valuestr[:2] == '0x':
                    value =  int(valuestr,16)
                else:
                    value =  int(valuestr)
            if value is not None:
                if serialOK and write2ram:
                    self.ser.write(b'wi,%i,%i\n' %(pidx, value))
                isok = True

        elif typ == 'f':    # float
            pattern = '^[-+]?[0-9.]+$'
            if re.match(pattern, valuestr) is not None:
                value = float(valuestr)
            if value is not None:
                if serialOK and write2ram:
                    self.ser.write(b'wf,%i,%f\n' %(pidx, value))
                    #print(value, b'wf,%i,%f\n' %(pidx, value))
                isok = True

        return value, isok

    # called when a value was entered in one of the the EEPROM List Entry widgets
    def on_enter(self, event):
        name = event.widget._name
        valstr = event.widget.get()
        idx,ok = mem.index_of_param(name)
        typ = mem.paramtype[idx]

        value, isok = self.validate_entry(valstr, idx)
        if not isok:
            msgbox.showinfo(title='Error', message='validation FAIL!')
        else:
            mem.paramvalue[idx] = value

        #print(name, valstr, value, idx, mem.paramindex[idx], typ)

    # called on click on one of the EEPROM List Text Labels
    def check(self, event):
        name = event.widget._name
        idx,ok = mem.index_of_param(name)
        if event.num == 1: #left mouse button click
            #self.log_text.insert(tk.END, f"label is {event.widget._name} \n")
            entrywidget = mem.param_entry_widgets[idx]
            valstr = entrywidget.get()
            print(f'{name} at index={idx}, value={valstr}')

            if name == 'SENSORCONFIG':
                if self.connection_active and self.calmode_active:
                    SensWin = SensorConfigWindow(self)
                    SensWin.grab_set()
            elif name == 'SENDDELAY':
                fwsetupwin = FirmwareSetup(self)
                fwsetupwin.grab_set()
            elif name == 'NODEID':
                nwsetup_win = NetworkSetup(self)
                nwsetup_win.grab_set()
            elif name== 'FREQ_CENTER':
                radio_win = RadioSetup(self)
                radio_win.grab_set()
            elif name[:4] =='PCI0':
                pciwin= PCIConfigWindow(self, 'PCI0')
            elif name[:4] =='PCI1':
                pciwin= PCIConfigWindow(self, 'PCI1')
            elif name[:4] =='PCI2':
                pciwin= PCIConfigWindow(self, 'PCI2')
            elif name[:4] =='PCI3':
                pciwin= PCIConfigWindow(self, 'PCI3')

        elif event.num == 3: # right mouse click
            msgbox.showinfo(name + ' Help', mem_help_eng[idx][1], icon="question")



    def on_sensor_config_window_close(self, sensorselection):
        idx,ok = mem.index_of_param('SENSORCONFIG')
        if ok:
            sval = sensorselection
            mem.paramvalue[idx] = sval
            p2eeprom = mem.paramindex[idx]

            if self.connection_active and self.calmode_active:
                if sval is not None:
                    if 0 <= sval  and sval <= 0xFFFF:
                        self.ser.write(b'wi,%i,%i\n' %(p2eeprom, sval))
                        e = mem.param_entry_widgets[idx]
                        e.delete(0, tk.END)
                        e.insert(0, str(sval))

                else:
                    msgbox.showerror(title='ERROR', message='wrong data!')
            else:
                msgbox.showerror(title='ERROR', message='COM Port nicht geöffnet oder Calibration Modus nicht aktiv')
        else:
           msgbox.showinfo(title='ERROR', message='item not found')
    '''
    def open_new_window(self, event):
        if not self.new_window_is_open:
            self.new_window_is_open = True
            self.new_window = tk.Toplevel(self.master)  # Create a new window
            self.new_window.title(event.widget._name)
            self.new_window.geometry("400x250")
            self.new_window.protocol("WM_DELETE_WINDOW", self.window_close_handler)
            ttk.Label(self.new_window, text="This is a new window").pack(pady=20)
            self.new_window.focus()

    def window_close_handler(self):
         self.new_window_is_open = False
         print("we are in the window close handler")
         self.new_window.destroy()
    '''
    
    def populate_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        ports.sort()
        self.port_combobox = ttk.Combobox(self.master, values=ports, state="readonly", width=10)
        if  len(ports) >0 :
                self.port_combobox.set(ports[0])
        self.port_combobox.grid(row=0, column=1, padx=3, pady=2, sticky = 'e')

    def port_combobox_refresh(self, event):
        #print("combobox selected")
        ports = [port.device for port in serial.tools.list_ports.comports()]
        ports.sort()
        self.port_combobox['values'] = ports
        if  len(ports) >0 :
                self.port_combobox.set(ports[0])

    def connect(self):
        port = self.port_combobox.get()
        baud_str = self.baud_combobox.get()
        if baud_str == 'Sender':
            baud_str = '57600'
        elif baud_str == 'Receiver':
            baud_str = '230400'
        baud = int(baud_str)
        try:
            if self.connection_active == False:
                self.connection_active = True
                self.ser = Serial(port, baud, timeout=1)
                self.log_text.delete(1.0, tk.END)

                self.connect_button["text"] = "Disconnect"
                #self.connect_button["command"] = self.disconnect
                #self.load_eeprom_btn["state"] = tk.NORMAL
                #self.save_eeprom_btn["state"] = tk.NORMAL
                #self.checksum_btn["state"] = tk.NORMAL
                #self.write_eeprom_btn["state"] = tk.NORMAL
                self.flash_btn["state"] = tk.DISABLED

                self.thread = threading.Thread(target=self.read_from_port)
                self.thread.start()
                #print(f'active threads: {threading.active_count()}')
            else:
                self.connection_active = False
                self.calmode_active = False
                if hasattr(self, 'ser') and self.ser.is_open:
                    self.ser.close()
                self.log_text.insert(tk.END, "Disconnected\n")
                self.connect_button["text"] = "Connect"
                self.load_eeprom_btn["state"] = tk.DISABLED
                self.checksum_btn["state"] = tk.DISABLED
                self.save_eeprom_btn["state"] = tk.DISABLED
                self.write_eeprom_btn["state"] = tk.DISABLED
                self.testentri["state"] = tk.DISABLED
                self.flash_btn["state"] = tk.NORMAL

                #enable Buttons for Configuration Dialogs
                self.network_config_btn['state'] = tk.DISABLED
                self.radio_config_btn['state'] = tk.DISABLED
                self.vcccal_btn['state'] = tk.DISABLED
                self.pci_btn['state'] = tk.DISABLED
                self.hardware_btn['state'] = tk.DISABLED
                self.sensor_btn['state'] = tk.DISABLED
                self.firmware_btn['state'] = tk.DISABLED
                self.actions_btn['state'] = tk.DISABLED

        except Exception as e:
            self.log_text.insert(tk.END, f"Error: {str(e)}\n")
    
    def read_raw(self, port):
        raw_line=b""
        monitor_mode = False
        if port.inWaiting() > 0:
            #time.sleep(0.01)
            if not monitor_mode:
                raw_line = port.read(port.inWaiting())
                if b'\r\n' not in raw_line[-2:]:
                    done = False
                    ts = time.time()
                    while not done:
                        if port.inWaiting() > 0:
                            raw_line += port.read(port.inWaiting())
                            if b'\r\n' in raw_line[-2:]: done = True
                        else:
                            #time.sleep(.001)
                            pass
                        if time.time() - ts > 1:
                            #print("rx timeout")
                            #raw_line += b'\r\n'
                            break;
            else:
                c = port.read(port.inWaiting())
                raw_line = c
        else:
            time.sleep(0.001)
        return raw_line

    def read_from_port(self):
        while self.connection_active:  # Check the flag in the reading loop
            try:
                line = self.read_raw(self.ser)
                try:
                    if line ==b'':
                        continue
                    elif type(line) is bytes:
                        line = line.decode("utf-8")
                    else:
                         print(line, type(line)) #should not ever happen
                except:
                    continue
                #line = self.ser.readline().decode("utf-8")
                line1 = line[:-2]
                #print(line1+'EOL')
                if line1:
                    if (line1 == '    ') and (self.opmode.get()==1):
                    #if ('    ' in line1) and (self.opmode.get()==1):
                        self.ser.write(b'y')
                        self.log_text.insert(tk.END, "-->switch to cal mode\n")
                        self.log_text.see(tk.END)
                    elif line1 == "calibration mode.":
                        self.ser.write(PassWord)
                        self.ser.write(b'\n')
                    
                    elif 'calibration mode.' in line1:
                        twolines = line.split('\r\n')
                        for l in twolines:
                            self.log_text.insert(tk.END, l+'\n')
                            self.log_text.see(tk.END)
                        self.ser.write(PassWord)
                        self.ser.write(b'\n')
                    
                    elif line1 == 'Pass OK':
                        self.calmode_active = True
                        #msgbox.showinfo(title='', message='Password correct')
                        self.load_eeprom_btn["state"] = tk.NORMAL
                        self.save_eeprom_btn["state"] = tk.NORMAL
                        self.checksum_btn["state"] = tk.NORMAL
                        self.write_eeprom_btn["state"] = tk.NORMAL
                        self.testentri["state"] = tk.NORMAL
                        self.log_text.insert(tk.END, line)
                        self.log_text.see(tk.END)
                        self.cmd_sent = b'a'
                        self.ser.write(b'a\n')

                    elif line1 == 'Pass not OK':
                        #print(line1)
                        self.log_text.insert(tk.END, line)
                        self.log_text.see(tk.END)
                        self.enter_password()


                    elif line1[:2] == 'a,' and (self.cmd_sent == b'a' or self.cmd_sent == b'ls'):
                        mem.load_values(line1[2:]) # get the eeprom class filled with values
                        #self.log_text.insert(tk.END, line)
                        #self.log_text.see(tk.END)
                        self.populate_eeprom()

                        #enable Buttons for Configuration Dialogs
                        self.network_config_btn['state'] = tk.NORMAL
                        self.radio_config_btn['state'] = tk.NORMAL
                        self.pci_btn['state'] = tk.NORMAL
                        self.vcccal_btn['state'] = tk.NORMAL
                        self.hardware_btn['state'] = tk.NORMAL
                        self.sensor_btn['state'] = tk.NORMAL
                        self.firmware_btn['state'] = tk.NORMAL
                        self.actions_btn['state'] = tk.NORMAL

                        #get compile flags
                        self.cmd_sent = b'd'
                        self.ser.write(b'd\n')

                    elif 'compiler flags: ' in line1 and self.cmd_sent == b'd':
                         self.compiler_flags = int(line1.split(':')[1])
                         #print('self.compiler_flags', self.compiler_flags)
                         self.log_text.insert(tk.END, "Compiler Flags: 0x%0x\n"%(self.compiler_flags))
                         self.log_text.see(tk.END)


                    elif self.cmd_sent == b's':
                        self.cmd_sent = b''
                        self.load_eeprom() # get new eeprom data from device (not yet saved), same as the button press

                    elif self.cmd_sent == b'c':
                        self.cmd_sent = b''
                        self.load_eeprom()
                        self.log_text.insert(tk.END, line)

                        measured_value = int(line.split(',')[1][:-2])
                        idx,ok = mem.index_of_param('VCCADC_CAL')
                        mem.paramvalue[idx] = measured_value
                        #print('on read from port line 1646', 'mem.paramvalue[idx]', mem.paramvalue[idx])
                        self.Reference_ADC.set(measured_value)
                        self.log_text.insert(tk.END, "Measured ADC value: %i\n"%(measured_value))
                        self.log_text.see(tk.END)

                    elif self.cmd_sent == b'm':
                        self.cmd_sent = b''
                        vcc = float(line1.split(',')[1])
                        messagetext = "Measured VCC value: %.0f mV\n" % (vcc)
                        self.log_text.insert(tk.END, messagetext)
                        self.log_text.see(tk.END)
                        self.Measured_VCC.set(int(vcc))
                    
                    elif self.cmd_sent == b'tt':
                        self.log_text.insert(tk.END, line)
                        self.log_text.see(tk.END)
                        for l in line1.split('\r\n'):
                            if 'degC' in l:
                                self.radio_temperature.set(l)
                        self.cmd_sent = b''
                    
                    elif self.cmd_sent == b'NumberOfActions':
                        #print('NUM ACTIONS: ', int(line))
                        mem.number_of_actions=int(line)
                        self.cmd_sent=b''        
                    
                    elif self.cmd_sent == b'ar':
                        self.membuf= int(line)
                        self.cmd_sent=b''
                        
                    elif self.cmd_sent[:1] == b'r':  #cmd_sent[0] is int, cmd_sent[:1] is bytes
                        #print (int(self.cmd_sent.split(b',')[1]), (line))
                        #self.membuf= int(line)
                        self.cmd_sent=b''
                        self.log_text.insert(tk.END, line) #not working in a for loop
                        self.log_text.see(tk.END)

                    else: 
                        self.log_text.insert(tk.END, line)
                        self.log_text.see(tk.END)
                        self.device_message.set(line1)
                        
            except Exception as e:
                if self.connection_active:  # Only log errors if the connection is still active
                    self.log_text.insert(tk.END, f"Error reading from port: {str(e)}\n")
                break

    def enter_password(self):
        self.pw_win = PassWordEntry(self)
        self.pw_win.grab_set()

        # does not work: window ".!_querystring" was deleted before its visibility changed
        # because askstring is not called from the main thread.
        #self.pw_str = askstring("Enter Password", 'Enter Password (exactly 16 characters)', parent=self)
        #if self.pw_str:
            #self.on_password_entry_done(self.pw_str)




    #this is for debug purposes only
    def output_params(self, line):
        #mem.load_values(line)
        for i in range(len(mem.parameter)):
            s = "%s = %s\n" %(mem.parameter[i], mem.paramvalue[i])
            self.log_text.insert(tk.END, s)
            self.log_text.see(tk.END)

    def populate_eeprom(self):
        one= int(len(mem.parameter) / 2) + 1
        if not mem.pop_done:
            self.tableLabel = tk.Label(self.master, text='Configuration Parameter Table',  font=("Helvetica", 11))
            self.tableLabel.grid(row=1, column=8,columnspan=4, pady=5, sticky="ew")
            for i in range(one):
                p= mem.parameter[i]
                #v= tk.StringVar(self.frame_config_values1, mem.paramvalue[i])

                self.l = ttk.Label(self.frame_config_label1, text=p)
                self.l.pack(pady=1, anchor="e")
                self.l.bind("<Button>",self.check)
                self.l._name=p

                self.e = ttk.Entry(self.frame_config_values1,  width=7)
                self.e.insert(0,mem.paramvalue[i])
                self.e._name=p
                self.e.pack(anchor="w")
                self.e.bind("<Return>", self.on_enter)
                if mem.parameter_is_readonly(p):
                    self.e["state"] = tk.DISABLED
                mem.param_entry_widgets.append(self.e)

            for i in range (one, len(mem.parameter)):
                p= mem.parameter[i]
                #v= tk.StringVar(self.frame_config_values2, mem.paramvalue[i])
                #mem.textentryvalue.append(v)
                self.l = ttk.Label(self.frame_config_label2, text=p)
                self.l.pack(pady=1, anchor="e")
                self.l.bind("<Button>",self.check)
                self.l._name=p

                self.e = ttk.Entry(self.frame_config_values2, width=7)
                self.e.insert(0,mem.paramvalue[i])
                self.e._name=p
                self.e.pack(anchor="w")
                self.e.bind("<Return>", self.on_enter)
                if mem.parameter_is_readonly(p):
                    self.e["state"] = tk.DISABLED
                mem.param_entry_widgets.append(self.e)

            mem.pop_done= True
        else: # reload eeprom
            for i in range(len(mem.param_entry_widgets)):
                e = mem.param_entry_widgets[i]
                e["state"] = tk.NORMAL
                e.delete(0, tk.END)
                e.insert(0,mem.paramvalue[i])
                if mem.parameter_is_readonly(mem.parameter[i]):
                    e["state"] = tk.DISABLED


    def load_eeprom(self):
        if self.connection_active and self.calmode_active:
            self.ser.write(b"a\n")
            self.cmd_sent = b'a'
        else:
            msgbox.showerror(title='Error', message='COM port nicht offen oder Calibration Modus nicht aktiv.')
            '''
            data = self.log_text.get(1.0, tk.END)
            filename = f"serial_log_{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}.txt"
            with open(filename, "w") as file:
                file.write(data)
            self.log_text.insert(tk.END, f"Log exported as TXT: {filename}\n")
            '''
    def write_eeprom(self): # copy values from screen to RAM of device
        for idx in range(len(mem.parameter)):
            value = mem.paramvalue[idx]
            v, isok = self.validate_entry(str(value), idx) # str(str(x)) is a string, so programming is simple here
            if not isok:
                msgbox.showinfo(title='Error', message='validation FAIL!')
            time.sleep(0.01) # delay when we save to a sender. otherwise its too fast


    def vddcal(self):
        if self.connection_active and self.calmode_active:
            self.ser.write(b"c\n")
            self.cmd_sent = b'c'

    def measure_vdd(self):
        if self.connection_active and self.calmode_active:
            self.ser.write(b"m\n")
            self.cmd_sent = b'm'

    def calculate_checksum(self):
        if self.connection_active and self.calmode_active:
            self.ser.write(b"s\n")
            self.cmd_sent = b's'
            #read value

    def save_eeprom(self):
        if self.connection_active and self.calmode_active:
            self.calmode_active = False
            self.opmode.set(0)
            self.ser.write(b"x\n")
            self.cmd_sent = b'x'
            self.load_eeprom_btn["state"] = tk.DISABLED
            self.checksum_btn["state"] = tk.DISABLED
            self.save_eeprom_btn["state"] = tk.DISABLED
            self.testentri["state"] = tk.DISABLED

    def export_csv(self):
        data = self.log_text.get(1.0, tk.END)
        filename = f"serial_log_{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}.csv"
        with open(filename, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerows([line.split() for line in data.splitlines()])
        self.log_text.insert(tk.END, f"Log exported as CSV: {filename}\n")

    def export_xml(self):
        data = self.log_text.get(1.0, tk.END)
        filename = f"serial_log_{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}.xml"
        root = ET.Element("LogData")
        lines = data.splitlines()
        for line in lines:
            entry = ET.SubElement(root, "Entry")
            ET.SubElement(entry, "Data").text = line
        tree = ET.ElementTree(root)
        tree.write(filename)
        self.log_text.insert(tk.END, f"Log exported as XML: {filename}\n")


    def save_file(self):
        files = [('Configuration', '*.cfg'),
                ('Text Document', '*.txt'),
                ('All Files', '*.*')]
        #file = asksaveasfile(filetypes = files, defaultextension = files)
        #with open(file.name, 'w') as f:
        with asksaveasfile(filetypes = files, defaultextension = files) as f:
            for i in range (len(mem.parameter)):
                if not mem.parameter_is_readonly(mem.parameter[i]):
                    item = '%s = %s\n' % (mem.parameter[i], mem.paramvalue[i])
                    f.write (item)

    '''
    def save_file(self):
        self.pw_win = PassWordEntry(self)
        self.pw_win.grab_set()
        self.ser.write(PassWord)
        self.ser.write(b'\n')
        print('now leave save_file')
    '''

    def open_file(self):
        files = [('Configuration', '*.cfg'),
                ('Text Document', '*.txt'),
                ('All Files', '*.*')]
        
        actions_list = list()
        with askopenfile(filetypes = files, defaultextension = files) as f:
            for line in f:
                #print(line)
                pair = line.split('=')
                if len(pair) > 1:
                    p = pair[0].strip()
                    v = pair[1].strip()
                    idx, ok = mem.index_of_param(p)
                    if ok:
                        val, valOK = self.validate_entry(v, idx, write2ram=False)
                        time.sleep(0.005)
                        if  valOK:
                            mem.paramvalue[idx] = val
                            #print(idx, p, val, type(val))
                        else:
                            print("validation fail", idx, p)

                if 'ACTION' in line:
                    s= re.findall(r"[\w]+",line)
                    actions_list.append(s[:3]) if len(s) > 2 else self.donothing
        self.populate_eeprom()
        self.treatactions(actions_list)
        msgbox.showinfo(title='Hinweis', message=f'Daten von der Datei {f.name.split('/')[-1:][0]} werden nicht automatisch zum\nDUT copiert.\nKlicke hierzu "Copy->EEPROM"')

    def donothing(self):
        pass
    
    def treatactions(self, listofactions):   
        action_indices = list()
        actions= []
        p=['NODE','PORT','MASK','ONOFF']
        for item in listofactions:
            item[0] = item[0][6:]
            if item[0] not in action_indices:
                action_indices.append(item[0])
                ai=[0]*4
                actions.append(ai)
            if item[1] in p:
                item[1] = p.index(item[1])
                actions[action_indices.index(item[0])][item[1]]= int(item[2])
            print(item) # ['0', 0, '24'] - Action number, p(Node, port, mask, onoff), value
        print(action_indices) # ['1','2','3','4']
        print(actions)
        
        A=Actions(self)
        success, text, mem.number_of_actions = A.check_actions_on_eeprom()
        #success, text, mem.number_of_actions = Actions.check_actions_on_eeprom1(self) # static possibility
        if not success:
            print (text)
            return
        #reload actions into mem.actions.
        mem.actions_pop_done= False
        mem.actions.clear()
        A.copy_actions_from_eeprom_to_memory(mem.number_of_actions)
        mem.actions_pop_done = True
        success, text, crc = A.verify_CRC()
        if not success:
            print (text)
            return 
        # now concatenate with actions from file
        mem.actions += actions
        mem.number_of_actions += len(action_indices)
        
    def window_exit(self):
        if self.connection_active:
            #close = msgbox.askyesno("Programm schliessen","COM Port Port schliessen und exit App?")
            close = True
            if close:
                self.connect()
                self.destroy()
        else:
            self.destroy()



if __name__ == "__main__":
    mem = Eeprom(mem_s)
    app = SerialMonitor()
    app.mainloop()
