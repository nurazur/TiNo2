# TiNo
![](https://github.com/nurazur/TiNo2/blob/master/Tino2_logo.png)

## Hinweis
Der Master-Branch ist als Entwicklungsbranch gedacht, die Verwendung, auch einzelner Dateien, ist auf eigenes Risiko. Die Releases sind jedoch gründlich getetstet. Doch auch hier gilt: ich übernehme keine Garantie dass die Software frei von Fehlern ist.

# Einführung
"**TI**ny **NO**de" : Batteriebetriebener Funksensor oder Funk-Aktor.
Dies ist die zweite Generation des TiNo mit zahlreichen Verbesserungen. Hauptsächlich hat der TiNo2 mehr Flash Speicher, und der Stromverbrauch wurde weiter reduziert. Die Antenne ist jetzt auf der leiterplatte untergebracht. TiNo2 basiert auf der neuen Generation von AVR Prozessoren, zunaechst dem ATmega4808.
Die Kernpunkte des TiNo2 sind (immer noch)

- geringst mögliche Kosten (Stückkosten unter 5 EUR, Stand 2020, ca. 8 EUR Stand 2023)
- kleinst möglicher Form Faktor (Streichholzschachtel)
- geringst möglicher Stromverbrauch,  maximale Batterielebensdauer (5 Jahre oder mehr)
- möglichst grosse Reichweite
- moeglichst einfachster Nachbau
- Plug&Play Firmware

Als Sensor kann man so ziemlich alles verwenden, ob Temperatur, Luftfeuchtigkeit, Luftdruck, Höhenmesser, Lichtintensität, UV Index,
Anwesenheitssensoren, Magnetschalter, Erschütterungs-Sensoren, Feuchtigkeitsmesser usw also im Prinzip alle Arten von Sensoren. Voraussetzung ist dass der Sensor bis mindestens 2.2V Betriebsspannung spezifiziert ist. Sonst kann die Batterieladung nicht voll ausgenutzt werden.

Die Leiterplatten passen zu im Handel erhältlichen PVC Gehäusen, welche in etwa die Grösse einer Streichholzschachtel haben. Die verwendeten Komponenten sind am Markt eingeführt und jederzeit erhältlich.


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
- DS18B20 (beliebter Temperatursensor, da auch in wasserdichten Versionen verfügbar)
- MAX31865 (PT100 Temperatur Sensor)
- AM312 PIR Bewegungssensor
- MAX6675, MAX31855K, MAX31856, ADS1120 ADC IC's für Thermoelemente
- [](https://github.com/nurazur/TiNo2/blob/e5b521a594be324584e5fc79e4e9750f60dd1295/New_smaller.png) ADS1120 und ADS1220 ADC für PT100
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
- FEC (Forward Error Correction) und Interleaver

## Basisband
- Microchip ATMega4808-au AVR64DD28 und AVR64DD28 MCU's
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
- Programmierung mit Arduino IDE (bis Version 2.6.1), und/oder mit PLatformIO
- Konfiguration der Nodes über serielles Interface (FTDI Adapter)
- Konfigurations- und Kalibrierdaten im EEPROM gespeichert (verschlüsselt)
- Flashen
  - mit UPDI Programmer (z.B. ATMEL ICE) oder
  - seriell mit FTDI Adapter über Bootloader
- bis zu 4 externe Interrupts (z.B. 4 Tasten) konfigurierbar
- [Interface zu IOBroker](https://github.com/bowao/ioBroker.tino/)

# Installation mit der Arduino IDE
Die Arduino IDE Installation wird von mir ab Version 2.7 nicht mehr unterstützt, aber ist prinzipiell möglich. Das TiNo2 Paket ab Version 2.7 ist nicht rückwärts kompatibel.
Die gesamte Installation geht in 3 Schritten und benötigt nur einige Minuten. MegaCoreX ist der Kernel den man für den Atmega4808 braucht und der leider nicht in der Arduino IDE mitgeliefert wird.

1. Installiere MegaCoreX
2. Installiere TiNo2 Bibliotheken
3. Installiere Sensor Bibliotheken

## Installation von MegaCoreX über Boards Manager (empfohlen)
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

## Manuelle Installation der MegacoreX Bibliothek (für Experten)
 - Öffne die Seite [https://github.com/mcudude/megacorex](https://github.com/mcudude/megacorex)
 - Im `Code` Menu klicke auf `Download ZIP`
 - Extrahiere die ZIP Datei.
 - Bewege den extrahierten Ordner zu "**~/Documents/Arduino/hardware**". Wenn der "hardware" Ordner noch nicht existiert, erstelle ihn.
- Öffne  die Arduino IDE, im Boards Menu erscheint eine neue Kathegorie "MegaCoreX". Dies zeigt die erfolgreiche Installation an.

## Installation des TiNo2 Pakets
* Öffne die Arduino IDE.
* Öffne das **File > Preferences** Menu.
* Unter `Additional Boards Manager URL's` diesen Link eintragen:
    ```
    https://raw.githubusercontent.com/nurazur/TiNo2/master/package_nurazur_TiNo2_index.json
    ```
* Einträge mit einem Komma ( **,** ) trennen, wenn es mehr als einen URL Eintrag gibt.
* Boards Manager öffnen: Auf **Tools > Board > Boards Manager...** Klicken.
* Jetzt werden die vorhandenen Platformen heruntergeladen. Warte bis der Vorgang beendet ist.
* Suche den Eintrag **nurazur TiNo2 Boards**.
* Klicke **Install**.
* Nachdem der Installationsvorgang beendet ist, schliesse das **Boards Manager** Fenster.

## Installation der Sensor Bibliotheken
Kopiere folgende ZIP Dateien in den `libraries` Ordner deines Arduino Sketch Verzeichnisses:
<br>
[BME280](https://github.com/nurazur/BME280/archive/refs/heads/master.zip)<br>
[SHTxx](https://github.com/Sensirion/arduino-sht/archive/refs/tags/v1.2.4.zip)<br>
[OneWire](https://github.com/nurazur/OneWire/archive/refs/heads/master.zip)<br>
[DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library/archive/refs/tags/3.9.1.zip)<br>
Wenn ein Thermoelement eingebunden werden soll, muss man die folgenden Bibliotheken <u>ALLE</u> installieren: <br>
[ADS1120](https://github.com/nurazur/ADS1120-Library/archive/refs/heads/master.zip)<br>
[MAX6675](https://github.com/RobTillaart/MAX6675/archive/refs/tags/0.3.0.zip)<br>
<!-- [MAX31855](https://github.com/RobTillaart/MAX31855_RT/arc.hive/refs/tags/0.6.0.zip)<br> -->

Installiere die Bibliotheken mit der Arduino IDE:<br>
`sketch -> Include Library -> Add .Zip library`

# Installation von TiNo2 für PLatformIO
1. Installiere PlatformIO IDE mit Visual Studio Code oder Installiere nur die PlatformIO CLI (Command-Line-Interface).
2. Auf Windows: füge folgende zwei Pfade zur `path` Variable der Umgebungsvariablen hinzu:<br>
`<User>.platformio\penv\Scripts`<br>
`<User>.platformio\penv\Lib\site-packages`<br>

wobei "User" der Pfad zum PLatformIO Installationsordner ist.<br>

3. Herunterladen des TiNo2 Bereichs von [Github](https://github.com/nurazur/TiNo2) als zip Datei. Kopieren der Ordner `libraries`, `sensor` und `receiver` in einen Projektordner freier Wahl.
4. Wenn Verwendung von VS Code mit PlatformIO IDE, neues Projekt eröffnen, navigiere zum `sensor` oder `receiver` Ordner. Arbeite weiter von hier aus mit der PlatformIO.
5. Bei Verwendung der PlatformIO CLI vom Command-Prompt, wie folgt vorgehen:<br>
  a. Konsole öffnen (Windows: cmd.exe)<br>
  b. Zum `sensor` oder `receiver` Ordner des TiNo2 Projekts wechseln mit `cd <path>`.<br>
  c. Zum Kompilieren `pio run` eingeben.
      Der `atmelmegaavr` Kernel wird installiert falls noch nicht auf dem PC vorhanden.
      Alle vom Projekt benoetigten Bibliotheken werden automatisch heruntergeladen und installiert. Dies ist einer der grossen Vorteile gegenüber der Arduino IDE.<br>
  d. Um einen Sketch auf den TiNo2 zu übertragen, das Kommando `pio run -t upload` eingeben.
  <br>Falls ein anderer Port benutzt wird als derjenige, der in der `platform.ini` voreingestellt ist, Option `-upload-port <port>` zum Kommando hinzufuegen.<br>
  e. Die PLatformIO Dokumentation konsultieren um die `platformio.ini` Konfigurationsdatei zu personalisieren.

## Fuses
Kommando `pio run -e fuses_bootloader -t fuses` in der Konsole eingeben. Wie das in  VS Code funktioniert, weiß ich nicht.
## Bootloader programmieren
Kommando `pio run -e fuses_bootloader -t fuses` in der Konsole eingeben. Wer Platformio vor dem 20.Februar2024 installiert hat, beachtet bitte diesen Bug: [https://github.com/platformio/platform-atmelmegaavr/pull/67](https://github.com/platformio/platform-atmelmegaavr/pull/67)
