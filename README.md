# pulsar
Pulsar is an Arduino-Based Radio Transmission System for Measurements and Digital Values
It aims to be a powerful and versatile RF analog/digital medium-fast data transmission system.
The first throw runs with NRF24L01 modules, the next one will be LoRa.
  It providees a 1 Sec data transmission pace
  -optimised for long range: only 16 Bytes payLoad data and reduced transmission speed
  -safe transmission: echo verification, retry 125ms later and again 500mS later if NACK
  -real time scheduler:  data acquisition occurs with constant, reliable 125mS slices
  -transmits 4 analog measures 0-1V with parametrizable range begin, range end, value for voltage dividers...
  -Preprocessing of analog data:
   a) averages 8 measures @ 125mS into one second value
   b) provides attack and decay filters to filter out irrelevant spikes
  -option to transmit [A0, A0 min, A0 max, A2] instead of [A1,A2,A3,A4]
  -reports digital data with 8 bits per input: one bit for every 125mS slice, so you can get a 125mS resolution on digital values even at a    slower one second pace.  (at the cost of an average latency of ~500mSec)
  -reports measured own Vin/Raw (battery load status) voltage on one byte (200mV resolution, with 5V offset e.g. from 5V to 10V) WIP
  -reports evaluation of calculated own Vcc voltage on one byte  (200mV resolution) WIP
  -reports evaluation of calculated internal temperature on one byte
  -reports number of 1st, 2nd transmission retries and failed transmission per running hour WIP
  -debug mode report on serial monitor
  -plot mode
  -can easily be tweaked to send every minute and sample every second.
