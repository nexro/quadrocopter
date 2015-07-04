# Arduino Mega Quadrocopter
This software is written for an Arduino Mega to control a quadrocopter.
It includes a PID controller to controll the stability of the vehicle, some functions to receive commands from a rc sender (in progress) and a small visual function to flash a red ligt on the bottom of the quadrocopter.

## Necessary hardware
- To run the software you need at least an Arduino Mega 2560. Please note that the software does definitely not work on an Arduino Uno. I didn't test other Arduinos.
- You need 4 brushless servo-controlled motors with 4 rotors (2* CW, 2* CCW)
  You can purchase these parts here: http://flyduino.net/Suppo-Motoren and here: http://flyduino.net/Propeller-Rotoren-carbon-Karbon-Fiberglas
- You need 4 ESCs. Theses are necessary for the Arduino to controll the rotation speeds for the motors-
  You can purchase them here: http://flyduino.net/ESC-Regler-brushless
- You need some kind of a frame where you can mount your parts on. You can also purchase such a frame in a shop or you can build one on your own. It's up to you
- To stabilize the quadrocopter when it's flying you need an accelerometer and a gyroscope. MPU6050 combines both on one piece of hardware
  You can purchase it here: http://www.amazon.com/Kootek-MPU-6050-MPU6050-sensors-Accelerometer/dp/B008BOPN40/ref=sr_1_1?ie=UTF8&qid=1435510180&sr=8-1
  Generally you could also use any other gyroscope/accelerometer hardware, but this code is just compatible with a MPU6050
- You also need a power source. I use a 11,1V LIPO
- Last but not least you need a rc sender and a receiver. It's up to you which one you use. I bought this one:
   http://www.multicopter-shop.de/fernsteuerungen/fernsteuerungsanlagen/wfly-wft07-pcms-7ch-24ghz-dsss-empfanger.html
   It's cheap compared to some other rc hardware, but it's really sufficient and it combines both, the sender and the receiver
   
## Wiring
I just want to give you a short summary of how to wire the components. For details, e.g. what is I2C or how to use it with an Arduino you should use google. Google is much better in explaining things than me ;)
- At first you have to connect the MPU6050 to the I2C bus of the Arduino. (details can be found here: http://diyhacking.com/arduino-mpu-6050-imu-sensor-tutorial/ )
- Next connect the ESCs with your power source(e.g. Lipo). Attention: NEVER USE THE ARDUINO AS POWER SOURCE FOR THE ESCs! If you're lucky, the ESCs just don't work. If you're not, the Arduino gets damaged.
- Now connect the data-wires from your ESCs with your Arduino. Generally you can use every digital pin on your arduino board, but this code wants this wiring:
  ESC for right front: pin 8 on Arduino
  ESC for left front: pin1 0 on Arduino
  ESC for left back: pin 5 on Arduino
  ESC for right back: pin 7 on Arduino
- Now power your Arduino. For that you should use the regulated power source from one of your ESCs. You shouldn't connect the Arduino directly to the LIPO
- At last connect the rc receiver with an adequate power source. Then connect the following pins:
  Throttle pin(mostly channel 3) to A8 on the Arduino
  A pin for a flip switch (my rc uses channel 7 for that) to A9 on the Arduino
  
That's it! You are now ready to flash the code with the Arduino IDE to your Arduino Mega and give it a try!

## Special functions
- This software uses a PID controller to controll the stabilization of the quadrocopter. A PID controller needs some tunings, because every quadrocopter is built different and so every quadrocopter needs different tunings. (see this page for PID details: http://blog.oscarliang.net/understanding-pid-for-quadcopter-rc-flight/ )
  To change the tunings you can either change the values directly withing the code (PITCH_P_VAL, PITCH_I_VAL, ...) or you can use the serial interface from the Arduino and a Computer to directly set new tunings. Note that when you set the tunings over the serial interface, the values get saved in the EEPROM and get loaded after you restart your Arduino. So you don't have to set the correct tunings everytime you power your Arduino.
  To set the tunings over the serial port, you have to send the following 8 bytes from a computer to your Arduino: 7   -   0   -   Pitch_P   -   Pitch_I   -   Pitch_D   -   Roll_P   -   Roll_I   -   Roll_D
- You can connect a LED to pin 47. This LED turns on, when the quadrocopter is calibrated and ready to fly

## Annotation
The work on this code is still in progress. I'm sure it is not perfect and there are still important features missing. Feel free to contribute :)
