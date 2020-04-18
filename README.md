 # BME280-Driver
Interfacing STM NUCLEO-L073RZ  with Bosch BME280 sensor 

## Hardware & Software
* STM NUCLEO-L073RZ 
* BME280 MODULE 
* STM32CubeMx IDE 

## Pin Connection
BME280 Sensor interfaced on *I2C* bus over Nucleo Board.  
<pre>
BME280  ->  Nucleo Board(Arduino Header)  
VIN     ->  5 Volt
GND     ->  GND
SCL     ->  PB9(D15) 
SDA     ->  PB8(D14)
INT     ->  PA6(D12)
</pre>   
Note: While Doing Connection keep the Solder Bridges of Nucleo Board in mind. 

## Resources 
Application code can be found under *Core/src/main.c*.   
For Adding Serial Printf function , I followed Shawn Hymel's [Tutorial](https://shawnhymel.com/1873/how-to-use-printf-on-stm32/).  
Original Version of the Library which is used in this project can be found [here](https://github.com/BoschSensortec/BME280_driver).
