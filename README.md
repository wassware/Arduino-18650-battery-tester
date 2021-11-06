# Arduino based Lithium battery charger and tester
I was given an Arduino a few years ago and beyond flashing lights I did not do much with it at first.
With more serious projects the Arduino soon ran out of memory so moved onto the ESP32.
I have accumulated a large number of 18650 cells for use in projects like electric bikes, lawnmowers and boat engines. These came from discarded packs and equipment.
Charging, testing and restoring to a store charge level takes time so decided to automate the process.
As the Arduino has a 5 volt interface to the world and as I had some Arduino Unos spare the challenge was to make a 6 channel version and use all the IO pins of the Arduino and make it fit in the 2k of working memory available. ESP32 IO levels are 3.3V so would need more components to make this work.
Lithium batteries can be dagnerous as we all know so used a charger+protection module to handle the charge process for each cell. A simple MOSFET+resistor to handles the discharge. So each cell needed an analogue voltage read and 2 digital IO to control the charge control and discharge MOSFET. So 6 channels uses up all the 
UNO IO pins.
