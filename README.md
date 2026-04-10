# EBC meter
## 430nm EBC measurements 
### Initial testing
Originally set up with 
* a Microchip PIC 
* OSRAM BPW21 Photo Diode,
* a transimpedance amplifier (TIA),
* 11 bit ADC,
* 428nm 5mm LED (380mcd, 20mA).

Although this worked in principle, I first needed a better understanding and a higher resolution ADC, so took a simpler approach using a Pico, Micropython and an APDS-9250 sensor. The APDS sensor gives reasonable results for low EBC values less than about 30EBC. It also serves to tests the light properties of the cuvette holder. The surface area of the APDS and that sensor is general is not intended for this sort of application. The BPW-21 with a considerably larger photodiode area is a better fit. The Hamamatsu S9195 is potentially orders of magnitude better in terms of 430nm response, but I have not been able to source that yet.

### Current setup
Switching back to BPW-21 sensor and 3W star base LED modules I am in the process if building a solution. This branch will hold the development stages.

At present LED current seems optimal around 750mA to get as bright a light as possible that does not max out the sensor with a clear sample. Design work will aim for:
- 800mA max current
- through a red (660nm) and a blue (428nm) LED.
- A sample period of <20ms
- and no more than 1Hz sample frequency with a single LED.
- Ultimtely intended to run from a rechargeable Lipo and/or 5V USB supply

Stages are:
- MCU, unspecified using RP2040 for now; SPI & 2 (red & blue sample) output pins
- opamp constant current controller (TSZ121)
- Lipo to 5V supply
- 5V LC filtering & 3V3 high PSRR LDO regulaotr
- 3v3 analogue side
    - variable gain TIA (TSZ121) 10kΩ & 1MΩ & associated capacitors
    - analogue switch 
    - ADC SPI interface, delta-sigma type

