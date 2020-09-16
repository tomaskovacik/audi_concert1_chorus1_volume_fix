Software and hardware to fix volume problem on audi concert1/chorus1 units made by blaupunkt. More about this problem in [this](https://github.com/tomaskovacik/audi_concert1_chorus1_volume_fix/wiki/Problem-of-Some-AUDI-Chorus-and-AUDI-Concert-Autoradio-Models,-or-%22Delayed-Action-Mine%22-from-Blaupunkt-Company) wiki page which is copy from [here](https://web.archive.org/web/20071017195907/http://erta.ru/review/chorus-problem_eng.shtml).

#### Compatible radios:

These are 100% compatible, manufactured by Blaupunkt:
```
Audi Chorus - 7 646 243 380 (4B0 035 152)
Audi Chorus A8 - 7 647 243 380
Audi Concert - 7 646 248 380 (4B0 035 186)
Audi Concert A8 - 7 647 248 380 (4D0 035 186A)
Audi Concert Navi - 7 647 249 380 (4B0 035 186B)
AUDI	CONCERT TT AUZ1Z3	7 648 244 380 (4B0 035 186D)
```
Maybe for these, drop me a message with pictures of radio from outside and inside, and I can check if your radio is compatible with this module:
```
AUDI	CHORUS A3	7 640 251 380
AUDI	CHORUS A8	7 648 243 380
AUDI	CHORUS AUZ1Z2	7 647 245 380   4B0 035 152A
AUDI	CHORUS M4 AUZ1Z2	7 648 245 380   4B0 035 152B
AUDI	CHORUS M4 AUZ1Z2	7 648 245 380   4B0 035 152B
AUDI	CONCERT A8	7 648 248 380
AUDI	CONCERT M4 AUZ1Z3	7 648 247 380   4B0 035 186C
```
#### Not compatible radios

Audio Concert made by Panasonic/Matsushita (8L0 035 186A, possible other codes ).This radio from this manufacturer did not have problems with volume (if it has, the problem is in volume rotary encoder)

There are other radios manufactured by Blaupunkt called Audi Concert Plus, which are not compatible:
```
AUDI	CONCERT PLUS AUZ1Z3	7 649 246 380   4B0 035 186E
AUDI	CONCERT PLUS A8 AUZ1Z3	7 649 249 380   4D0 035 186F
```
These use a different design and so are not compatible with this module. I never read that these radios did have a volume problem. If there is one then it is most likely a problem with volume rotary encoder.


#### Issues
Known issues <a href="https://github.com/tomaskovacik/audi_concert1_chorus1_volume_fix/issues">are here</a>

#### Video
Module in action: <a href="https://youtu.be/YbFa_UYPMRQ"></a>

#### Schematics:

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/HW/audi_concert1_chorus1_volume_fix/audi_concert1_chorus1_volume_fix.png">


Schematics in <a href="https://github.com/tomaskovacik/audi_concert1_chorus1_volume_fix/blob/master/HW/audi_concert1_chorus1_volume_fix/audi_concert1_chorus1_volume_fix.pdf">pdf</a>.

Assemble plan for latest v4 is <a href="https://tomaskovacik.github.io/audi_concert1_chorus1_volume_fix/HW/audi_concert1_chorus1_volume_fix/bom_v4_for_v1_code/ibom.html">here<a/>. Components with value prefi NP_ are not required (not populated). These are there for future features. 1k_GALA resistor has to be populated if SW with <a href="https://github.com/tomaskovacik/audi_concert1_chorus1_volume_fix/blob/gala/SW/audi_volume_fix_stm32/audi_volume_fix_stm32.ino">gala</a> support is used (SW from gala branch). AP for old v1 hw is <a href="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/HW/audi_concert1_chorus1_volume_fix/assemble_plan1_0.png">here</a>.


Pictures of finished product:

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180906_180733.jpg">

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180906_180740.jpg">

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180906_180745.jpg">

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180906_180752.jpg">

Bottom side, with PLCC52 socket, inside part of pastic need to be cut:

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180906_180757.jpg">

Upper side, with comments to sockets

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20180920_154234_comments.jpg">

How to wire it (currently without intefacing with front pannel)

<img src="https://raw.githubusercontent.com/tomaskovacik/audi_concert1_chorus1_volume_fix/master/Pics/20181003_170247.jpg">

# COPYING

Everything is under GPLv3

# CONTRIBUTION

Contributions are welcome

# PCBs

PCBs are on sale also as whole product, kitt (PCB, soldered crystals, unsoldered preprogramed IC, pasives,PLCC52 socket)

<a href="https://www.tindie.com/products/tomaskovacik/volume-fix-for-audi-concert1chorus1/"><img src="https://d2ss6ovg47m0r5.cloudfront.net/badges/tindie-larges.png" alt="I sell on Tindie" width="200" height="104"></a>
