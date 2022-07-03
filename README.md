<h1>Demo stm32 with mpu6050 gyroscope. Flight controller NBOne - reverse engineering</h1>

<p>NBOne is a cheap flight controller made with stm32 and mpu6050 on board. 4 input ports, 4 output ports.</p>
It was a try to make own firmware for flight controller. Learning to work with gyroscope, servos, and input pwm.</p>

How the board is working: Getting input data from mpu6050 and Servo Tester (PWM). All data is mixed and controls servos for Ailerons. Changing Roll and Pitch.</p> <p>Angles as a string are trinsmitted through the serial port to PC </p>

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




