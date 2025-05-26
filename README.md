# People-Counter---FarYam
Arduino-based occupancy counter with dual ultrasonics, DHT11/MQ-135 environmental sensing, DS3231 RTC timestamping, and SD-card logging. Built for real-time people count monitoring in Manning Hall (Chapel Hill)
##How it works 
Single-Entry Doorway Entries count entering and exiting 

We use two ultrasonic sensors—**Sensor 1** and **Sensor 2**—to detect people moving through a doorway:

- **Entry**  
  1. Sensor 1 is triggered first  
  2. Sensor 2 is triggered next (within the valid time window)  
  3. → Increment the occupant count

- **Exit**  
  1. Sensor 2 is triggered first  
  2. Sensor 1 is triggered next (within the valid time window)  
  3. → Decrement the occupant count

- **No Count**  
  - If only one sensor fires and the other does not follow in time, no entry or exit is recorded.

- **Error Condition**  
  - If both sensors remain blocked (e.g. someone stands between them) or timing is invalid, an error is logged.

The DHT11 Temp/Humidity and MQ-135 Gas Sensors are constantly reading the environment 

When a person enters/exits, the temperature, humidity, date, time, room occupancy is logged in the SD Card.
The same happens every 5 minutes if there are no occupancy updates.

###Install Libraries 
DHT Sensor Library 
MQ135
RTClib
SD (built-in)
EEPROM (built-in)

### Hardware 
Arduino UNO R4
MQ-135 Gas Sensor
DHT11 - Temperature and Humidity Sensor 
DS3231 Real Time Clock Module 
SD Card Module
Four DoubleA Battery Pack Holder 
HC-SR04 Ultrasonic Sensors 
Bambu 3D Printer


### Software
Arduino IDE
TinkerCAD
Excel
Python 
