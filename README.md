# JMRI-Nodes
All of my current, live, JMRI nodes. 

All are written in the Arduino IDE but are intended to run on different devices - Arduino Mega, ESP32 and Intel Galileo. The intended hardware is indicated in the filename, if none is stated it's likely to be for an Arduino Mega.

My layout originally used CMRI and RS485 for accessory communication but I never managed to make it reliable and stable, so I switched to MQTT and TCP/IP over ethernet and wifi.
https://youtube.com/playlist?list=PL7Nf8RnEsYmSdl8QI9ElILIPk0n1USRwE&si=tskidrLFsRzOB36V

These sketches handle subscriptions to an MQTT server, through which they receive update messages from JMRI. 

Messages from JMRI usually indicate the change of state of a turnout, which means that the sketches need to move servo motors accordingly, as well as set electrofrog polarity. Additionally they may receive updates to signal LED status which leads to updates to LEDs through shift out registers.

They also monitor locally connected sensors - current detectors (usually directly connected to the device), Infrared sensors (usually connected through a shift-in register), LDRs (connected to an analog pin), reporting state updates to JMRI (through MQTT).

Some of the sensor classes include debounce code which handles 'flapping' and prevents spamming of the MQTT server (and therefore JMRI) with flapping sensor states.

All servo turnout control uses a 'slow' method where the servo gradually moves the servo to the required position.
https://www.youtube.com/watch?v=ViDmebzVHoY

The ESPs use wifi which requires an SSID and password. These are stored in separate files called arduino_secrets.h. These are excluded from the repo for security reasons.
To replace them, create a new project file called arduino_secrets.h and fill it with the following lines

#define SECRET_SSID "yourSSID"

#define SECRET_PASS "yourPassword"

DISCLAIMER - these sketches are written very much for my requirements and will very probably not directly transfer to another layout. They're here more as a reference, how I'm doing things, and hopefully will provide something useful to someone.

Map of where nodes are situated on the layout

https://github.com/Vintage80sModelRailway/Nodes-for-JMRI/blob/main/nodes%20map.jpg

<h2>1 - MQTT_Client_PS_Node_2</h2>
Arduino Mega R3

The first node created for the layout, controlling the four crossovers (8 turnouts) at the top of the map image.

Controls the servos for the layouts and has shift out registers connected that control the LEDs for nearby signals.

Electrofrog relay is controlled through the PCA9685 board as it had 8 connectors free after connecting 8 servos (16 ports)

https://youtu.be/bpWt6p9PFpw?si=DcQliWyzrZVmNKaC

Basic LED signals through shift out registers, responding to on / off messages from MQTT
https://youtu.be/x6u-Y4YNqIA

2 RFID readers connected to Serial1, Serial2 for identifying which train is in an active block

https://youtu.be/WULCDdYoGKw

<h2>2 - MQTT_Client_PS_C1_minimised</h2>
Arduino Mega R3

Controls servos for turnouts located around its position - near side incline, yard exit (AC) and entrance(CW) and the small passing loop (that will become a station) on the nearby incline.

Sensor inputs for nearby block detectors

https://youtu.be/_Db5GG-U-4c?si=iM1FfPi6PhrIINgh

3 RFID readers connected to Serial1, Serial2, Serial3 for identifying which train is in an active block

https://youtu.be/WULCDdYoGKw

<h2>3 - MQTT_Client_PS_Node_3_minimised</h2>
Arduino Mega R3

Used to overheat and cause all kinds of electronic devices so gradually had connected devices removed.

Now just connected to sensors, block detectors.

https://youtu.be/_Db5GG-U-4c?si=iM1FfPi6PhrIINgh

2 RFID readers connected to Serial1, Serial2 for identifying which train is in an active block

https://youtu.be/WULCDdYoGKw

<h2>4 - MQTT_Client_ESP_inclineTop</h2>
ESP32

Originally used for servo control

https://youtu.be/HPEj1L96IVc?si=Ds6YPtx4ZbFY4Yob

But they were very twitchy when connected to an ESP, so they've now been moved elsewhere and this ESP just has a few LDRs for short block detection connected

https://youtu.be/rpdpft-YQa8?si=AEQFfvEUaDozkKNs

Also controls 8 relays on a relay board for electrofrog polarity on the upper incline crossovers. It does this by subscribing to the MQTT topic for the turnouts, and switching the relays on / off based on the state of the turnout.

<h2>5 - g3_RelayShiftOutDriver_c2</h2>
Intel Galileo Gen2

Not many pins (Arduino Uno pinout) so used for serial comms - shift out registers for nearby relays (for electrofrog points) and signal LEDs

https://youtu.be/x6u-Y4YNqIA?si=C3A1f6SgvCUgYEgm

and shift in registers for the IR block detectors under the yard lines

https://youtu.be/wbRwXlxj-rE?si=k8q2qmVYuoK_Six-

<h2>6 - g3_RelayShiftOutDriver_c3</h2>
Intel Galileo Gen2

Installed to take over servo control of nearby turnouts after first the Arduino then an ESP didn't handle them particularly well (twiching and spikes)

Also controlling a nearby 8 relay board (all directly connected) and also some shift in / out registers for the same reasons as 5.

<h2>7 - g3_RelayShiftOutDriver_c1</h2>
Intel Galileo Gen2

The latest board to be set up, controlling the servos and relays for turnouts and electrofrog on the station exit on the upper deck.

https://youtu.be/tPeac9mDtp8?si=ZxWgiHt2byoZG-_f

<h2>8 - MQTT_Client_ESP_C1_Station</h2>
ESP32
Has a couple of LDRs connected for tracking occupancy of the short blocks (mostly double slips) around the station on the upper deck.
