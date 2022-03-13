# Room Alignment

Subscribes to MQTT topics to recieve room scan updates from Hololens and Google Tango. Then tries to align them using IPC and publishes the transformation matrix.

# MQTT Recorder

Utility to create MQTT replays for debugging and testing purposes. 

MQTT communication can be recorded with timestamps in replay files.
These replays can then be played to simulate devices going through the tracking setup, to conduct tests with equal sensor data.

# Dependencies

* PCL 1.8.1
* Mosquittopp (x64 build provided)
