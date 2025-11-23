import tkinter as tk
from tkinter import ttk
from tkinter import scrolledtext
import tkinter.messagebox as msgbox
from tkinter.filedialog import asksaveasfile
from tkinter.filedialog import askopenfile
from tkinter.simpledialog import askstring

import serial.tools.list_ports
from serial import Serial
import xml.etree.ElementTree as ET
import csv
import threading
import datetime
import struct
import time
import re
from subprocess import Popen, PIPE, STDOUT
import os

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
    ['INTERLEAVER_ENABLE','b'],
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
    ['RADIO_T_OFFSET','Nur für Fortgeschrittene. Der Radio Chip (RFM69HCW) hat einen Temperatur Sensor in 1 degC Schritten. Die Ausgabe ist ohne Offset Einstellung sehr ungenau. Der Offset kann hier in 1/10 Grad Schritten eingestellt werden.'],
    ['USE_RADIO_T_COMP','Nur füe Fortgeschrittene. Aktiviere den Offset RADIO_T_OFFSET für die Temperaturmessung des Radios'],
    ['REQUESTACK','1 = Der Sender möchte eine Rückmeldung vom Empfänger dass die Nachricht angekommen ist.\n0 = keine Rückmeldung vom Empfänger.'],
    ['LEDCOUNT','Anzahl der LED Doppel-Blinks nach dem Start.\n0 = nie\n 255 = immer\n\nDas ist eine Massnahme zum Strom sparen. Normalerweise ist die LED ausserhalb des Gehäuses nicht sichtbar. Ein Wert von 3 zeigt an dass die LED bei 3 Paketen nach dem Start blinkt, füe Testzwecke. Danach blinkt sie nichr mehr.'],
    ['LEDPIN','GPIO Pin nach Arduino für die LED. Normalerweise 19.'],
    ['LDRPIN','Dies muss ein analoger GPIO nach Arduino sein.\
    An diesem Pin wird ein LDR (Light Dependent Resistor) angeschlossen,\
    welcher in SENSORCONFIG aktiviert wird. Mögliche GPIOs sind: 15, 16, 17, 18. Theoretisch GPIOs 22,23,224,25, aber diese GPIOs sind bei nicht bei allen TiNo2 zugänglich.'],
    ['PIRDATAPIN','Digitaler GPIO Eingang des Bewegungsmelders, wenn angeschlossen.'],
    ['PIRDEADTIME','Zeit für die der Bewegungsmelder nach einer Auslösung stumm geschalten wird. Die Zeit wird in 8 Sekunden Intervallen angegeben. Typisch sind ca. 3 1/2 Minuten, damit ist der Wert ca. 25'],
    ['TCCSPIN','CS Pin für den SPI Bus eines ICs das Thermoelemente misst'],
    ['RTDPOWERPIN','VDD Pin für ein ICs das PT100 Sensoren misst]'],
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
    ['INTERLEAVER_ENABLE','Nur in Kombination mit FEC sinnvoll. Wenn FEC nicht verwendet wird, auf 0 setzen'],
    ['EEPROM_VERSION','liefert die Version der EEPROM Struktur. Nur lesbar.'],
    ['SOFTWAREVERSION','Liefert die Version der Software. Wird nicht konsequent gepflegt, daher ohne Bedeutung. Nur lesbar.'],
    ['TXGAUSS_SHAPING','Für fortgeschrittene. Normalerweise 0 (kein Shaping)'],
    ['SERIAL_ENABLE','Auf 1 setzen'],
    ['IS_RFM69HW','Tino2 verwendet ausschliesslich den RFM69HCW, deshalb muss dieser Wert = 1 sein.'],
    ['PABOOST','Nur für Insider.\n0=normal mode\n1=for PA1 and PA2 use and activate high-Power registers\n2=use PA1 and PA2 but do not use high-power register'],
    ['FDEV_STEPS','Offset Kalibrierung der HF Frequenz. Auf 0 setzen, die Bandbreite des Empfängers ist auf jeden Fall ausreichend.'],
    ['CHECKSUM','Prüfsumme, nicht editierbar.']
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
            "I2C_0",
            "I2C_1",
            "I2C_2",
            "I2C_3",
            "I2C_4",
            "I2C_5",
            "I2C_6",
            "I2C_7",
            "DS18B20",
            "MAX6675",
            "MAX31855",
            "MAX31856",
            "MAX31865",
            "ADS1120",
            "BRIGHTNESS",
            "RESERVED"
            ]
            
flash_cmd = ['avrdude',
        '-Cavrdude.conf',
        '-carduino',
        '-P',
        '-b115200',
        '-D',
        '',
        '-v',
        '-pavr64dd32',
        '-Uflash:w:../../Sensor/.pio/build/AVR64DD32/firmware.hex:i'
       ]

           
class ShowProcessOutputDemo(tk.Toplevel):
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
        
        self.proc = Popen(flash_cmd, stdout=PIPE, stderr=STDOUT)
        flash_thread = threading.Thread(target=self.read_output, args=(self.proc.stdout, ))
        flash_thread.start()
        #print(f'active threads: {threading.active_count()}')

        
    def read_output(self, pipe):
        """Read subprocess' output and store it in `self.stdout_data`."""
        while True:
            data = os.read(pipe.fileno(), 1 << 20)
            # Windows uses: "\r\n" instead of "\n" for new lines.
            #data = data.replace(b"\r\n", b"\n")
            #data = data.replace(b"\r\n", b"")
            if data:
                #data = data.replace(b"\r\n", b"")
                datadec = data.decode()
                if len(datadec) > 0:
                    self.stdout_data = datadec
                    #print(datadec)
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
        PassWord = self.PwEntry.get().encode()
           
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


class SensorConfigWindow(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)
        
        self.parent = parent
        self.geometry("280x400+%d+%d" % (self.parent.winfo_rootx()+350, self.parent.winfo_rooty()+100 ))
        self.title('Sensor Selection')

        self.idx, ok  = mem.index_of_param('SENSORCONFIG')
        if not ok:
            msgbox.showinfo(title='Error', message='FAIL!')

        self.values=[]
        self.s = mem.paramvalue[self.idx]

        for i in range(len(sensors)):
            mask = 1<<i

            var = tk.IntVar(self, mask & mem.paramvalue[self.idx])

            self.values.append(var)
            self.chk = ttk.Checkbutton(self, text=sensors[i],variable=self.values[i], onvalue=1<<i, offvalue=0, command=self.update_sensorconfig)
            self.chk._name= str(i)
            self.chk.grid(row=i, column=0, padx=10, pady=1, sticky="nw")
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
        self.close_btn.grid(row= last_row+1, column=2, rowspan = last_row+1, padx=10, sticky = 'se')

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


class Eeprom:
    pop_done = False
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
                    self.paramvalue.append(struct.unpack('f', p)[0])
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
    def __init__(self):
        super().__init__()

        self.iconbitmap("tino_icon.ico")
        self.title("TiNo2 Configuration and Calibration")
        self.geometry("900x600")

        self.create_widgets()

        # Flag to indicate if the serial connection is active
        self.connection_active = False

        # Flag to indicate whether the node is in calibration mode
        self.calmode_active = False

        # store the last command sent
        self.cmd_sent = b""

        self.new_window_is_open = False

        self.protocol("WM_DELETE_WINDOW", self.window_exit)

    def create_widgets(self):
        ### PORT LABEL #####
        port_label = ttk.Label(self.master, text="Port:")
        port_label.grid(row=0, column=0, padx=10, pady=2, sticky="e")
        port_label.bind("<Button-1>",self.port_combobox_selected)

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

        self.vddcal_btn = ttk.Button(self.master, text="Vbatt Cal.", command=self.vddcal, state=tk.NORMAL)
        self.vddcal_btn.grid(row=1, column=6, padx=0, pady=5, sticky='w')

        self.vddmeas_btn = ttk.Button(self.master, text="Vbatt Measure", command=self.measure_vdd, state=tk.NORMAL)
        self.vddmeas_btn.grid(row=1, column=7, padx=0, pady=5, sticky='w')
        
        self.flash_btn = ttk.Button(self.master, text="Flash Firmware", command=self.flash, state=tk.NORMAL)
        self.flash_btn.grid(row=1, column=8, columnspan=2, padx=0, pady=5, sticky='w')

        ### Save Button
        self.save_file_btn = ttk.Button(self.master, text="Save to File", command=self.save_file, state=tk.NORMAL)
        self.save_file_btn.grid(row=0, column=9, padx=1, pady=5, sticky="e")

        ### open Button
        self.open_file_btn = ttk.Button(self.master, text="Open File", command=self.open_file, state=tk.NORMAL)
        self.open_file_btn.grid(row=0, column=10, padx=1, pady=5, sticky="w")


        ### Frames for configuration editor
        r=2
        self.frame_config_label1 = tk.Frame(self.master, width=50, height=540)
        self.frame_config_label1.grid(row=r, column=7, rowspan=5, sticky="nw")

        self.frame_config_values1 = tk.Frame(self.master, width=50, height=540)
        self.frame_config_values1.grid(row=r, column=8, rowspan=5, sticky="nw")

        self.frame_config_label2 = tk.Frame(self.master, width=50, height=540)
        self.frame_config_label2.grid(row=r, column=9, rowspan=5, sticky="nw")

        self.frame_config_values2 = tk.Frame(self.master, width=50, height=540)
        self.frame_config_values2.grid(row=r, column=10, rowspan=5, sticky="nw")


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
        #print(self.testentri.get(), type(self.testentri.get())) # mein 2. <class 'str'>

        self.log_text = scrolledtext.ScrolledText(self.master, wrap=tk.WORD, width=60, height=31)
        #self.log_text = scrolledtext.ScrolledText(self.master, wrap=tk.WORD, height=31)
        self.log_text.grid(row=2, column=0, columnspan=7, padx=5, pady=10, sticky='w')
        
        #self.columnconfigure (10, weight=1)
        #self.columnconfigure (0, weight=1)

    def flash(self):
        flashwin = ShowProcessOutputDemo(self)
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
                    self.ser.write(b'wf,%i,%i\n' %(pidx, value))
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
            elif name[:4] =='PCI0':
                pciwin= PCIConfigWindow(self, 'PCI0')
            elif name[:4] =='PCI1':
                pciwin= PCIConfigWindow(self, 'PCI1')
            elif name[:4] =='PCI2':
                pciwin= PCIConfigWindow(self, 'PCI2')
            elif name[:4] =='PCI3':
                pciwin= PCIConfigWindow(self, 'PCI3')
                
        elif event.num == 3: # right mouse click
            msgbox.showinfo(name + ' Help', mem_help[idx][1])



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

    def populate_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        ports.sort()
        self.port_combobox = ttk.Combobox(self.master, values=ports, state="readonly", width=10)
        if  len(ports) >0 :
                self.port_combobox.set(ports[0])
        self.port_combobox.grid(row=0, column=1, padx=3, pady=2, sticky = 'e')
        self.port_combobox.bind('<<ComboboxSelected>>', self.port_combobox_selected)

    def port_combobox_selected(self, event):
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
            baud_str == '230400'
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


        except Exception as e:
            self.log_text.insert(tk.END, f"Error: {str(e)}\n")

    def read_from_port(self):
        while self.connection_active:  # Check the flag in the reading loop
            try:
                line = self.ser.readline().decode("utf-8")
                line1 = line[:-2]
                if line1:
                    if (line1 == '    ') and (self.opmode.get()==1):
                        self.ser.write(b'y')
                        self.log_text.insert(tk.END, "-->switch to cal mode\n")
                        self.log_text.see(tk.END)
                    elif line1 == "calibration mode.":
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
                        
                    elif self.cmd_sent == b's':
                        self.cmd_sent = b''
                        self.load_eeprom() # get new eeprom data from device (not yet saved), same as the button press

                    elif self.cmd_sent == b'c':
                        self.cmd_sent = b''
                        #msgbox.showinfo(title='', message=line1)
                        self.load_eeprom()
                        self.log_text.insert(tk.END, line)
                        self.log_text.insert(tk.END, "Measured ADC value: %s\n"%(line.split(',')[1]))
                        self.log_text.see(tk.END)

                    elif self.cmd_sent == b'm':
                        self.cmd_sent = b''
                        vcc = float(line1.split(',')[1])
                        messagetext = "Measured VCC value: %.0f mV" % (vcc)
                        self.log_text.insert(tk.END, "Measured ADC value: %.0fmV\n"%(vcc))
                        self.log_text.see(tk.END)
                        #msgbox.showinfo(title='', message=messagetext)

                    else:
                        self.log_text.insert(tk.END, line)
                        self.log_text.see(tk.END)
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
        #file = askopenfile(filetypes = files, defaultextension = files)
        #with open(file.name, 'r') as f:
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
        self.populate_eeprom()
        msgbox.showinfo(title='Hinweis', message=f'Daten von der Datei {f.name.split('/')[-1:][0]} werden nicht automatisch zum\nDUT copiert.\nKlicke hierzu "Copy->EEPROM"')


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
