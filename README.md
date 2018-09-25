Software and hardware to fix volume problem on audi concert1/chorus1 units made by blaupunkt.

everything is working, maybe minnor issues in code/logic:

after restart of car, looks like loudness is not set properly, but turning volume 1 click will fix this, probably bug in automatic loudness settings

Schematics:

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/9e00c9a2b6ebaae502b4e7f6c2973995be0e9a08/HW/standalone/audi_concert1_chorus1_volume_fix/audi_concert1_chorus1_volume_fix.png">

pdf: https://github.com/tomaskovacik/audi_concert1_chorus1_volume_fix/blob/master/HW/standalone/audi_concert1_chorus1_volume_fix/audi_concert1_chorus1_volume_fix.pdf

Pictures of finished product:

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180906_180733.jpg">

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180906_180740.jpg">

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180906_180745.jpg">

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180906_180752.jpg">

Bottom side, with PLCC52 socket, inside part of pastic need to be cut:

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180906_180757.jpg">

on lower left side: USB, upper lef: I2C connection with fixed data to TDA
on lower right connector for controling panel, if this is implemented in code (not yet), middle right fixed pullup resistor on D+ line of USB
upper right, ST-Link interface

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180920_154234.jpg">
