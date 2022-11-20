# **Solar-Power-Management-Using-STM32**

This project is a replica of the [Solar Power Measurement](https://nevonprojects.com/solar-power-measurement-system-using-arm-cortex/) project.
Completely written from scratch for NUCLEO-F401RE (STM32-F401RE) but since they are HAL CODES, they can be ported easily to other STM32 boards.

The STM32 board reads the ADC values in multi-channel and outputs to the DMA and at a modest frequency set by the the TIMER_2. The TIMER_2 acts as an EXTERNAL INTERRUPT as we are not using the Continuous Conversion Mode.

The LCD uses the I2C pins on the board so make sure to set them up as a peripheral. 

As the STM32 ADC is using a 12bit resolution, the number "4095" is the maximum it can count upto. Those moving in from programming for arduido are familiar with the 10bit ADC of max value "1023". This higher resolution is recommended because when reading the shunt voltage, the voltage difference across it is very less and the extra resolution helps get an accurate reading for calculating the current. 

Components for this project include:
* NUCLEO-F401RE
* 16X2 I2C LCD
* LDR
* LM35 Linear Temperature Sensor
* Solar Pannel
* Bunch of reisistors for voltage dividers

## **Selecting Resistors:**
We will be measuring the voltage from the solar pannel using a voltage divider and reading the current across a load resistor using a series shunt resistor.

The LED added to the circuit has a current limiting resistor of value 100 Ohms.

### **Load Resistor:**
There is a internal load resistor of value 3.3 Killo Ohms in series with the shunt resistor. You can select any value for this. A value of 1K-10K is recommended.

### **Voltage Divider:**
The STM32 board has a MAX ADC input limit of 5V. To read voltage values more than that, we need to implement a voltage divider. For selecting the appropriate values for the volltage divider, implement the given formula.

**_Vout = (Vs*R2)/(R1+R2)_**

Where _Vout_ should be 5V and _Vs_ should be the Maximum value of voltage you plan to input for measurement.
Therefore, for our resistors -
R1 = 30K Ohm, R2 = 7.5K Ohm and Vs = 25V.

5 = (25*30000)/(30000+7500)

And the equation checks out well for the voltage divider.

### **Shunt Resistor:**

The requirements for a shunt resistor are fairly strightforward.
1. The resistance of the resistor should be small enough to add negligible additional load  on the circuit.
1. The voltage difference across the shunt reistance should be measureable by your ADC. A better resolution of ADC or a larger resistance helps.
1. The resistor should be of a higher wattage because the current of the entire circuit will be flowing through it.

Since we didnt have a heavy enough load for the circuit, we decided to go with a 100 Ohm 1/4 Watt reistor with a 5% tolerance.

## **Changes to the code:**

After selecting the resistors and measuring them with a multimeter for absolute values. Make changes to the code on the _main.c_ file of the project. Under lines _74_ -

```
float R1=30000.0; // R1 of the voltage divider
float R2=7500.0; // R2 of the voltage divider
float RShunt=100.0; // Value of the shunt resistor
```
Change the address for the I2C LCD accordingly under line _83_-
```
#define SLAVE_ADDRESS_LCD 0x3F<<1
```

The LM35 used in our circuit had a +10° difference with the actual value of the ambient temperature so feel free modify the equation accordingly on line _242_-
```
temp = ((tempADC * (5000 / 4095.0))/10)-10;
```
To calibrate the LDR for calculating LUX, make changes to the code on line _244_-
```
lux  = (luxADC/4095.0)*100;
```
The code above is an aproximation for lux values and is calibrated against a simple phone app. Could definately use better calibration equations.

If you want to change when the High Temperature Alert LED triggers, make changes to the if condition on line _259_-
```
if(temp>60){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,GPIO_PIN_SET);
	}
```

## Schematic:
The schematic below is what we have used for the project and the code is configured for the appropriate resistor values.

![schematic](https://github.com/rupava/Solar-Power-Management-Using-STM32/blob/main/Schematic_SPMS_2_2022-11-20.png)
