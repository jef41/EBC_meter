# EBC_meter
## 430nm EBC measurements 
### Initial testing
Originally set up with 
* a Microchip PIC 
* OSRAM BPW21 Photo Diode,
* a transimpedance amplifier (TIA),
* 11 bit ADC,
* 428nm 5mm LED (380mcd, 20mA).

Although this worked in principle, I first needed a better understanding and a higher resolution ADC, so took a simpler approach using a Pico, Micropython and an APDS-9250 sensor.

### Current setup
The included code correctly measures values for a neutral density filter, to within about 0.5 EBC units. However, when used with a beer sample light scattering is too much of an issue and an approximately 25 EBC beer (Amber to Copper) is detected as about 70 EBC (Stout). 
## To Do
Next steps will be to filter the beer (which appears bright to the eye, but cloudy under 430nm), early indications show that filteration improves the accuracy, but only bringing the reading to about 50 EBC. Further testing is required on this point.

Subsequent to that, source a 700nm LED to measure turbidity, and subtract the turbidity EBC units from the 430nm EBC units. This requires 2 LEDs and 2 sensors, which could be vertically stacked.

### Hardware
Current setup is using 
* a Raspberry Pi Pico to switch
* a NPN transistor (2N3053).
* This transistor provides power to the LED (direct from a 3.7V Lithium cell).
* the LED is a generic 3W 430nm peak wavelength
* a 430nm optical bandpass filter (does seem to provide more accurate results when tested with the neutral density filter)
* an APDS-9250 IR&RGB sensor, housed in a MIKROE COLOR 9 CLICK PCB and using an I2C interface to the Pico
* held together in a seal-able 3D printed enclosure - base of which is shown in an image below

### Procedure
This is proof of concept rather than a robust working device. 
* Insert a cuvette filled with water,
* run the code
* when it has a stable result the code will prompt for the cuvette to be swapped for the beer sample
* insert the beer sample
* an absorbance and EBC value are returned

With the addition of sample filtration and/or some compensation for turbidity this looks promising. The ?? sensor that I started with will have much greater sensitivity, and would allow for better TIA gain and ADC, but at a considerably higher component count. The APDS-9250 might remain the preferred approach.

<img src="https://github.com/jef41/EBC_meter/assets/6393750/a3368ae0-cfd2-4016-bc10-9a0ef1f00d6d" width="40%" alt="Photograph of the components in a solderless prototyping baord and held together in a 3D printed housing">

