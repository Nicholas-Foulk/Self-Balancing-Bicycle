# Self-Balancing-Bicycle
PWM and GPIO circuit for motor control box

The pulse width modulation and general purpose input/output circuit goes with our proxy motor control box circuit, which is used for
switching between manual and automatic control of the bicycle. The program is run on LPC Expresso IDE and requires a few pins on the 
LPC 1769 board. The c file, "test_pwm.c" was used for this portion.


Website for phone connection using wifi

There will be a router mounted onto the bicycle for wifi connection. Having a router will give us a kill switch to the bicycle as an ultimate safety feature. The website code is also inside this repository in multiple html files.



Accelerometer 

Our project has an accelerometer, LSM303D, in our project for collecting data off of the front the bicycle and it is setup
on the LPC 1769 board using the SPI connections. The c file used for programming our connections is "LSM303D_SPI.c" 
and is in this repository. 

Control Algorithm

Our control algorithm uses data gathered from the accelerometer and controls the stepper motor for our seesaw balancing system. The control algorithm was first implemented on LPC_1769 with no problems under the files CMSIS_CORE_17xx and ControlAlgorithm folders in this project. The control was later implemented in freeRTOS to introduce a round robin task scheme for including more processes into our project. FreeRTOS allows us to use our Bluetooth/UART program and control algorithm at the same time. Our freeRTOS implementation is requires the folders CMSISv1p30_LPC17xx, FreeRTOS_Library, and ControlAlgorithmfreeRTOS. The IDE used for the control algorithm is LPC Xpresso IDE.
