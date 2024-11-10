# RP2040-GPSDO

A raspberry pi pico 2040 with some hardware to make a GPS Disciplined Oscillator.


I needed a good stable reference oscillator. Buying one is not an option for me. 
So i have been looking around for a good solution. Found more different types of solutions.


The idea iam following is based on:

 http://roland.cordesses.free.fr/GPS_Si2cor.html

This solution is based on an arduino nano device and sketch.
Using an SI5351 and the PPS from a GPS receiver to make a GPSDO.

I wanted to use the PIO of the RP2040 and found the following impressive project:

https://rjk.codes/post/building-a-frequency-counter/

The code can be found here:
https://github.com/richardjkendall/rp2040-freq-counter


### SI5351 hardware modification

The standard SI5351 crystal is replaced by a 0.5ppm TCXO.

I found the following instruction video.

[Instruction video](https://www.youtube.com/watch?v=Rv4Swh5Gmck&t=4s)


### Software mods

The arduino uses a 2.5MHZ clock from the SI5351 to count for 40 seconds.
After counting the deviation is used to change the clock in the SI5351.

I did change the fix time of 40 seconds by a more flexible process, starting to measure 
every second ; if there are 10 subsequent measurements with a zero deviation the process
continues by increasing the measurement time.

Furthermore iam using 10MHz as reference clock; so 4 times better! 


### Build 

<div align="center"><img src="docs/images/build-outside.jpg" width="480px"></div>
![RP2040-GPSDO](docs/images/front.JPG)

<div align="center"><img src="docs/images/build-inside.jpg" width="480px"></div>

### Result

Applying a filter on a 10 Mhz output.

<div align="center"><img src="docs/images/10Mhz out.jpg" width="480px"></div>

### Further thoughts

I found the work of Erik he additional uses a Phase detector to improve the process.

https://github.com/erikkaashoek/Arduino_SI5351_GPSDO

This is something i like to test....