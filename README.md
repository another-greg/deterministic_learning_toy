# Overview
Source code and board design schematics for the deterministic reasoning toy. 

The toy works by detecting the presence of magnets via 5 hall effect sensors. Presently, only the presence/absence of magnets at the sensors are detected.
In the future, the polarity of the magnets could be used to expand the number of unique block identifiers. This is probably unnecessary.

## How to run the code
Download this repo, open the "deterministic reasoning toy.ino" file using the Arduino IDE. Select your board and upload the code.

## How to order more circuit boards
The board was designed using EasyEDA/JLCPCB. Upload the "hall_effect_breakout_2025-11-21.epro" file to EasyEDA. You should then be able to immediately
export the design for purchase + assembly from JLCPCB.
