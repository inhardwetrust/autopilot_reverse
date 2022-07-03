<h1>Demo stm32 with mpu6050 gyroscope. Flight controller NBOne - reverse engineering</h1>

<p>NBOne is a cheap flight controller made with stm32 and mpu6050 on board. 4 input ports, 4 output ports.</p>
It was a try to make own firmware for flight controller. Learning to work with gyroscope, servos, and input pwm.</p>

How the board is working: Getting input data from mpu6050 and Servo Tester (PWM). All data are merged and controls servos for Ailerons. Changing Roll and Pitch.</p> <p>Angles as a string are trinsmitted through the serial port to PC </p>

Youtube demo: https://www.youtube.com/watch?v=2DHh55xmrMc

CubeIDE (STM32)</p>

<ul>
<li>
Board circuit in KiCad
https://github.com/inhardwetrust/autopilot_reverse/tree/master/Electroics/NBOne_PCB-to-Circuits
</li>
<li>
Unity Project with Demo plane - Coming soon-
</li>
</ul>

Important notice.
It is imposible to debug this board with CubeIDE debuging due to the not original controller used (STM32F103CBT6R).
So you need to build it as binary and upload via STM32-ST-Link Utility.

Links:
I liked to use CoolTerm and SerialPlot apps for testing.
https://freeware.the-meiers.org/
https://hackaday.io/project/5334/logs

There is an quite interesting Lib fro mpu6050<br>
https://github.com/jrowberg/i2cdevlib

Thanks to authors.

Next steps:
It's very interesting aproach is run mpu6050 in a DMP mode(without MotionApps)
This guy managed to work it on AVR<br>
https://github.com/bzerk/MPU6050_DMP_6_axis_demo_/blob/master/MPU6050_DMP_6_axis_demo_.pde




