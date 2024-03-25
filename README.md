# JMRI-Nodes
All of my current, live, JMRI nodes. 

All are written in the Arduino IDE but are intended to run on different devices - Arduino Mega, ESP32 and Intel Galileo. The intended hardware is indicated in the filename, if none is stated it's likely to be for an Arduino Mega.

These sketches handle subscriptions to an MQTT server, through which they receive update messages from JMRI. 

Messages from JMRI usually indicate the change of state of a turnout, which means that the sketches need to move servo motors accordingly, as well as set electrofrog polarity. Additionally they may receive updates to signal LED status which leads to updates to LEDs through shift out registers.

They also monitor locally connected sensors - current detectors (usually directly connected to the device), Infrared sensors (usually connected through a shift-in register), LDRs (connected to an analog pin), reporting state updates to JMRI (through MQTT).

Some of the sensor classes include debounce code which handles 'flapping' and prevents spamming of the MQTT server (and therefore JMRI) with flapping sensor states.

DISCLAIMER - these sketches are written very much for my requirements and will very probably not directly transfer to another layout. They're here more as a reference, how I'm doing things, and hopefully will provide something useful to someone.

Map of where nodes are situated on the layout

https://github.com/Vintage80sModelRailway/Nodes-for-JMRI/blob/main/nodes%20map.jpg

<h2>1 - MQTT_Client_PS_Node_2</h2>

<h2>2 - MQTT_Client_PS_C1_minimised</h2>

<h2>3 - MQTT_Client_PS_Node_3_minimised</h2>

<h2>4 - MQTT_Client_ESP_inclineTop</h2>

<h2>5 - g3_RelayShiftOutDriver_c2</h2>

<h2>6 - g3_RelayShiftOutDriver_c3</h2>

<h2>7 - g3_RelayShiftOutDriver_c1</h2>
