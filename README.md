# FW-ContikiOS

In this repository, you will find a working version of ContikiOS 3.x for the FeuerWhere (FW) mote developed by the [IHP](https://www.ihp-microelectronics.com/en/start.html).

## Getting started

Here is presented the procedure to flash the ContikiOS into the FW mote and how to run a simple example for communicating two nodes using the reliable unicast module available in the RIME stack using the Linux console.  

## Flashing ContikiOS into FW motes

In order to flash the ContikiOS into the FW mote using the **TI MSP-FET Flash Emulation tools (debugger/Programmer)**, follows the steps below: 

1. Get the raw ContikiOS ported to FW-node (.zip/.tar) (the ContikiOS folder available in this repo has all the modifications mentioned below). 

2. Install all packets required to program the FW mote using the TI MSP-FET Flash Emulation tools (debugger/Programmer).

   a. Download the MSP430Flasher from the TI Web site (http://www.ti.com/tool/MSP430-FLASHER).  
   b. Run the installation file by double-clicking the exe file (Perhaps, it is needed to change some privileges before: (1) ```sudo chmod +x some-app.run```; (2) ```sudo ./some-app.run```) 
   c. Copy the generated folder into the Contiki folder.
   d. Set the PATH to the libmsp430.so.
  
3. Connect the debugger to the FW1’s JTAG. **Warning:** If the power supply of FW1 is USB, please verify that the knob is settled as USB not BAT. 
  
4.  Flash the ContikiOS by going into the folder ```contiki_FW/examples/FW1``` and writing the command: ```make TARGET=feuerwhere example-broadcast.upload ```

5. Done. You have a running ContikiOS in the FW mote.

## Communication between two FW motes

Here, we present a simple example for communicating two FW-nodes using the reliable unicast module available in the RIME stack of ContikiOS. Hence, follow the step below: 

1. A simple reliable unicast example is available at ```../examples/rime/```. To flash this example into a FW-node using the TI MSP-FET Flash Emulation tools, follow the step below:
   a. Clean-up the targe: ```(sudo) make TARGET=feuerwhere clean``` 
   b. Compile and upload the example into the FW mote:```sudo make TARGET=feuerwhere example-runicast.upload```    
      I. Some problems may arise during the testing process. 
         1. Both FW motes have the same ```Node_Id``` and ```Rime address```. To change, follow these steps: 
            - ```(sudo) make TARGET=feuerwhere clean```  (Do every time you flash a file into the FW mote)
            - ```(sudo)  make TARGET=feuerwhere nodeid=1 example-runicast.upload``` 
         2. The FW motes are not able to communicated with each other. Add the following lines 323 to 325 in ```/platform/feuerwhere/contiki-feuerwhere-main.c``` to enable RIME Stack for FW motes: 
      ``` 
      NETSTACK_RDC.init();
      NETSTACK_MAC.init(); 
      NETSTACK_NETWORK.init();
      ``` 
      Comment the following code line: 
      ```//netstack_init(); ``` 
      
2. The same example can be flashed into the FW motes using the SMC programmer and the python script developed by the IHP as follows:  
   a. ```sudo make TARGET=feuerwhere clean```
   b. ```sudo make TARGET=feuerwhere nodeid=1 example-runicast.hex```
   c. ```sudo python mb_program.py -p /dev/ttyUSB0 -f example-runicast.hex``` 

## Essential commands
### Flashing the FW mote with the MSP430 programmer and the Linux console (terminal)
1. Go into the folder contiki_FW/examples/FW1
2. ```(sudo) make TARGET=feuerwhere clean```
3. ```(sudo) make TARGET=feuerwhere nodeid=1 example-broadcast.upload ```

### Flashing the FW mote the SCM adapter programmer and the Linux console (terminal)
1. Go into the folder contiki_FW/examples/FW1
2. ```(sudo) make TARGET=feuerwhere clean```
3. ```(sudo) make TARGET=feuerwhere nodeid=1 example-broadcast.hex```
4. ```(sudo) python mb_program.py -p /dev/ttyUSB0 -f example-broadcast.hex```

## Additional packages 

**Source: https://anrg.usc.edu/contiki/index.php/Installation

You can run the following command to install all packages for multiple platforms:

``` sudo apt-get install build-essential binutils-msp430 gcc-msp430 msp430-libc msp430mcu mspdebug gcc-arm-none-eabi gdb-arm-none-eabi openjdk-8-jdk openjdk-8-jre ant libncurses5-dev```

When working with a 64-bit virtual machine, you may run into problems with the `serialdump-linux` executable because it may have been compiled for 32-bit machines. Install the following package to fix this issue.

 ```sudo apt-get install lib32ncurses5```
