[![DOI](https://zenodo.org/badge/139644223.svg)](https://zenodo.org/badge/latestdoi/139644223)

# Margay Logger Library
This library is designed to facilitate the simple and modular usage of the Margay data logger

## Citation

If you use this library in a publication, please cite (temporary citation until a dedicated Margay paper is published):

**Wickert, A. D., K. R. Barnhart, W. H. Armstrong, M. Romero, B. Schulz, G.-H. C. Ng, C. T. Sandell, J. La Frenierre, S. B. Penprase, M. Van Wyk de Vries, and K. R. MacGregor (2024), [Automated ablation stakes to constrain temperature-index melt models](https://doi.org/10.1017/aog.2024.21), *Ann. Glaciol.*, *64*(92), 425–438, doi:10.1017/aog.2024.21.**

## Installation:

## Examples:

## Operation:

### Logging Start:
Using this library, logging begins automatically once power is applied. If error conditions are found, they will be indicated by status lights, but the logger will attempt to continue if possible. The following should represent the light sequence.

- On power application
	- `AUX` light will illuminate **green**, will stay on while testing
- After testing is complete
	- `AUX` light will turn **off**
	- `STAT` light will illuminate with various colors depending on the status of the logger 
		- **Green** -> All systems check out OK, logging will proceed 
		- **Orange** -> A sensor system is not registering properly, some sensor data may be missing or incorrect 
		- **Cyan** -> Clock time is incorrect, but otherwise working correctly 
		- **Pink** -> SD card is not inserted 
		- **Red** -> Critical on board component ([Jesus Nut](https://en.wikipedia.org/wiki/Jesus_nut)) is not functioning correctly, such as SD card or clock, logging will likely not be able to proceed
		- **Yellow, Fast Blinking** -> Battery capacity (assuming 3 series alkaline cells) is less than 50% of operational range. Device will still function fine, but operational time is less than ideal.
		- **Red, Fast Blinking** -> Battery voltage is less than 3.3v. Functionality of hardware no longer guaranteed beyond this point.  
- After display of status code(s)
	- `STAT` light will blink blue to indicate logging (or attempted logging) has begun

### Troubleshooting:
If an error code is received try the following steps:

- **General**
	- Disconnect and reconnect power, both USB and battery ([Turn it off and back on again](https://i.imgur.com/Yj6dB3W.gif))
	- Verify the quality of all screw terminal connections by gently tugging on the wires and making sure they stay in place, if not, remove and re-tighten the connection 
	- Ensure sensors and/or cables are not damaged, this can result in shorts or other problems
	- Make sure batteries have sufficient voltage to run the logger, when the battery voltage drops below *3.3v*, malfunctions can occur 
- **Orange**
	- Verify correct polarity of sensor connection
	- Ensure the right sensor is connected
	- Verify the screw terminals are well connected to the wires (a loose connection can cause a failure)
	- Make sure battery power is applied, some sensors can fail otherwise
- **Cyan**
	- Connect the logger to a computer and reset the clock using the [Northern Widget Time Set GUI](https://github.com/NorthernWidget/SetTime_GUI)
- **Pink**
	- Insert the SD card, or make sure card is fully seated 
- **Red**
	- Attempt power cycle 
	- Try different SD card
	- Disconnect all sensors
	- If none of the previous steps remove the red light, contact [Northern Widget](http://www.northernwidget.com/contact/) for further support 
- **Yellow, Fast Blinking** 
	- Replace batteries
- **Red, Fast Blinking** 
	- *If* this error occurs while also connected over USB, check proper connection of batteries
	- Replace batteries
