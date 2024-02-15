# TiNo
![](https://github.com/nurazur/TiNo2/blob/master/Tino2_logo.png)

## Hinweis
Der Master-Branch ist als Entwicklungsbranch gedacht, die Verwendung, auch einzelner Dateien, ist auf eigenes Risiko. Die Releases sind jedoch gründlich getetstet. Doch auch hier gilt: ich übernehme keine Garantie dass die Software frei von Fehlern ist.

# Einführung
"**TI**ny **NO**de" : Batteriebetriebener Funksensor oder Funk-Aktor.
Ziel dieses Projekts ist die Entwicklung schnurloser Funk Sensoren,
    die über Batterien versorgt werden und z.B. mit dem Raspberry Pi kommunizieren.
    Die Entwicklung hat zum Ziel:

- geringst mögliche Kosten (Stückkosten unter 5 EUR, Stand 2020, ca. 8 EUR Stand 2023)
- kleinst möglicher Form Faktor (Streichholzschachtel)
- geringst möglicher Stromverbrauch,  maximale Batterielebensdauer (5 Jahre oder mehr)
- möglichst grosse Reichweite
- moeglichst einfachster Nachbau
- Plug&Play Firmware

Als Sensor kann man so ziemlich alles verwenden, ob Temperatur, Luftfeuchtigkeit, Luftdruck, Höhenmesser, Lichtintensität, UV Index,
Anwesenheitssensoren, Magnetschalter, Erschütterungs-Sensoren, Feuchtigkeitsmesser usw also im Prinzip alle Arten von Sensoren. Voraussetzung ist dass der Sensor bis mindestens 2.2V Betriebsspannung spezifiziert ist. Sonst kann die Batterieladung nicht voll ausgenutzt werden.

Die Leiterplatten passen zu im Handel erhältlichen PVC Gehäusen, welche in etwa die Grösse einer Streichholzschachtel haben. Die verwendeten Komponenten sind am Markt eingeführt, jederzeit erhältlich und
dadurch kostengünstig zu beschaffen.


# Features
## Allgemein
- Spannung von ca. 1.8V bis 3.6V
- Betrieb mit CR2032 Zelle bis zu 5 Jahren Lebensdauer
- verschiedene Leiterplatten passend zum im Handel erhältlichen Gehäusen

## Sensoren
- HTU21D
- SHT21, SHT20, SHT25
- SHT30, SHT31, SHT35
- SHTC3
- SHT40, SHT41, SHT43, SHT45
- BME280 Luftdruck Sensor
- DS18B20 (beliebter Temperatursensor)
- MAX31865 (PT100 Temperatur Sensor)
- AM312 PIR Bewegungssensor
- MAX31865 PT100 Temperatur Sensor
- [](https://github.com/nurazur/TiNo2/blob/e5b521a594be324584e5fc79e4e9750f60dd1295/New_smaller.png) MAX6675, MAX31855K, MAX31856, ADS1120 ADC IC's für Thermoelemente
- LDR als Helligkeitssensor
- I2C oder SPI basierte Sensoren leicht konfigurierbar
- genug digitale GPIOs
- genug analoge GPIOs


## Radio
- RFM69HCW Modul oder RFM95 (LoRa) Modul
- bidirektionale Kommunikation
- ISM Band (Europa: 433MHz oder 868MHz, US:315MHz oder 915Mhz)
- 2GFSK Modulation oder LoRa
- Frequenz abstimmbar
- Frequenzkorrektur kalibrierbar
- Sendeleistung -18 dBm (typ.) bis 20dBm (max)
- Link Budget bis 125dB
- Empfindlichkeit -105 dBm typ.
- Reichweite t.b.d., ist aber sehr weit!
- HF Kommunikation verschlüsselt
- FEC (Forward Error Correction)
- Interleaver

## Basisband
- Microchip ATMega4808-au
- ![](https://github.com/nurazur/TiNo2/blob/e5b521a594be324584e5fc79e4e9750f60dd1295/New_smaller.png) auch für Micrpchip AVR64DD28 and AVR64DD28 MCU's
- 48kByte Flash (64kByte Flash on AVR64DD)
- Ruhestrom < 2uA
- Ruhestrom ca. 1uA mit TPL5110 (extern)
- 1 MHz oder 4 MHz Takt Sender erlaubt Betriebsspannung bis 1.8V
- 16 MHz Takt Empfänger
- I2C oder SPI für Sensoren
- 11 weitere GPIO's
- Serielles Interface zum Programmieren und Konfigurieren

## System / Software
- Open Source Software C++
- Software kann einfach individuell angepasst werden
- Programmierung mit Arduino IDE
- Konfiguration der Nodes über serielles Interface (FTDI Adapter)
- Konfigurations- und Kalibrierdaten im EEPROM gespeichert.
- EEPROM verschlüsselt
- Flashen
  - mit UPDI Programmer oder
  - seriell mit FTDI Adapter über Bootloader
- bis zu 4 externe Interrupts (z.B. 4 Tasten) konfigurierbar
- [Interface zu IOBroker](https://github.com/bowao/ioBroker.tino/)

# Installation
1. Installiere MegaCoreX
2. Installiere TiNo2 Bibliotheken

## Installation von MegaCoreX über Boards Manager
* Arduino IDE starten.
* `File->Preferences` öffnen.
* Unter `Additional Boards Manager URL's` diesen Link eintragen:
    ```
    https://mcudude.github.io/MegaCoreX/package_MCUdude_MegaCoreX_index.json
    ```
* Einträge mit einem Komma ( **,** ) trennen, wenn es mehr als einen URL Eintrag gibt.
* Boards Manager öffnen: Auf **Tools > Board > Boards Manager...** Klicken.
* Jetzt werden die vorhandenen Platformen heruntergeladen. Warte bis der Vorgang beendet ist.
* Suche den Eintrag **MegaCoreX** und klicke drauf.
* Klick **Install**.
* Nachdem der Installationsvorgang beendet ist, schliesse das **Boards Manager** Fenster.

## Manuelle Installation der MegacoreX Bibliothek
Click on the "Download ZIP" button. Extract the ZIP file, and move the extracted folder to the location "**~/Documents/Arduino/hardware**". Create the "hardware" folder if it doesn't exist.
Open Arduino IDE and a new category in the boards menu called "MegaCoreX" will show up.

## Installation des TiNo2 Pakets
* Open Arduino IDE.
* Open the **File > Preferences** menu item.
* Enter the following URL in **Additional Boards Manager URLs**:
    ```
    https://raw.githubusercontent.com/nurazur/TiNo2/master/package_nurazur_TiNo2_index.json
    ```
* Separate the URLs using a comma ( **,** ) if you have more than one URL
* Open the **Tools > Board > Boards Manager...** menu item.
* Wait for the platform indexes to finish downloading.
* Scroll down until you see the **nurazur TiNo2 Boards** entry and click on it.
* Click **Install**.
* After installation is complete close the **Boards Manager** window.


## Alte Dokumentation in Deutsch
[deutsche Dkumentation](https://github.com/nurazur/TiNo/blob/master/dokumentation.md)
