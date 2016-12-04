# Self-Balancing-Bicycle
Dependencies
In order to run the project code, you need to install LPCXpresso IDE. You also need to import the CMSISv1p30_LPC17xx, 
freeRTOS_Library, and ControlAlgorithmfreeRTOS projects provided in the ZIP file. This source code will only run on a 
FreeRTOS-capable ARM microprocessor like the one provided on the LPC1769 microcontroller. The freeRTOS instruction manual 
comes with links for downloading the necessary project folders for freeRTOS. Links to download the necessary software as 
well as reference material are provided below.


Running the Project
To run the code, it must be uploaded to the LPC 1769 through the LPCXpresso IDE. The code can either be run off of a computer
 as a debug instance or flashed onto the board.


Manual Control (Bluetooth) 
The phone application is called Bluetooth Serial Controller and can be downloaded for free on Google Play for Android devices.
 A config file for the controller is provided below. To enable the bicycle motor, press GO. To increase or decrease speed tap
 or hold down SPD UP or SPD DWN, respectively. To stop the motor, press STOP. This also disables control of the motor, so GO 
 must be pressed again to re-enable control. Press LEFT and RIGHT to turn the steering motor left or right.


Useful Links:


http://www.nxp.com/products/software-and-tools/software-development-tools/software-tools/lpc-microcontroller-utilities/lpcxpresso-ide-v8.2.2:LPCXPRESSO


http://www.nxp.com/wcm_documents/techzones/microcontrollers-techzone/LPCLibrary-Books/Books-pdf/using.freertos.lpc17xx.summary.pdf 


https://www.lpcware.com/system/files/LPCXpresso_IDE_User_Guide.pdf


http://www.FreeRTOS.org/Documentation/code


Repository(also includes controller config):

https://github.com/Nicholas-Foulk/Self-Balancing-Bicycle.git 













PWM and GPIO circuit for motor control box

The pulse width modulation and general purpose input/output circuit goes with our proxy motor control box circuit, which is used for
switching between manual and automatic control of the bicycle. The program is run on LPC Expresso IDE and requires a few pins on the 
LPC 1769 board. The c file, "test_pwm.c" was used for this portion.


Accelerometer 

Our project has an accelerometer, LSM303D, in our project for collecting data off of the front the bicycle and it is setup
on the LPC 1769 board using the SPI connections. The c file used for programming our connections is "LSM303D_SPI.c" 
and is in this repository. 

Control Algorithm

Our control algorithm uses data gathered from the accelerometer and controls the stepper motor for our seesaw balancing system. 
The control algorithm was first implemented on LPC_1769 with no problems under the files CMSIS_CORE_17xx and ControlAlgorithm folders 
in this project. The control was later implemented in freeRTOS to introduce a round robin task scheme for including more processes into 
our project. FreeRTOS allows us to use our Bluetooth/UART program and control algorithm at the same time. Our freeRTOS implementation is 
requires the folders CMSISv1p30_LPC17xx, FreeRTOS_Library, and ControlAlgorithmfreeRTOS. The IDE used for the control algorithm is LPC Xpresso IDE.
