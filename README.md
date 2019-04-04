# FW-ContikiOS

In this repository, you will find a working version of ContikiOS 3.x for the FeuerWhere (FW) mote developed by the [IHP](https://www.ihp-microelectronics.com/en/start.html).

## Getting started

Here is presented the procedure to flash the ContikiOS into the FW mote and how to run a simple example for communicating two nodes using the reliable unicast module available in the RIME stack using the Linux console.  

## Flashing ContikiOS into FW motes

In order to flash the ContikiOS into the FW mote using the **TI MSP-FET Flash Emulation tools (debugger/Programmer)**, follows the steps below: 

1. Get the ContikiOS ported to FW-node (.zip/.tar) by Postdam University-IHP1. 
2. Install all packets required to program FW mote using the TI MSP-FET Flash Emulation tools (debugger/Programmer). 
  a. Download the MSP430Flasher from the TI Web site (http://www.ti.com/tool/MSP430-FLASHER).  
  b. Run the installation file by double-clicking the exe file (Perhaps, it is needed to change some privileges before: (1) sudo chmod +x some-app.run; (2) sudo ./some-app.run )  
  c. Copy the generated folder into the Contiki folder. 
3. Connect the debugger to the FW1’s JTAG. 
  - Warning: If the power supply of FW1 is USB, please verify that the knob is settled as USB not BAT. 
4.  Flash the ContikiOS by going into the folder ```contiki_FW/examples/FW1``` and writing the command: ```make TARGET=feuerwhere example-broadcast.upload ```
5. Done. You have a running ContikiOS in the FW mote.

## Communication between two FW motes

 Here, we present a simple example for communicating two FW-nodes using the reliable unicast module available in the RIME stack of ContikiOS. Hence, follow the step below: 

1. A simple reliable unicast example is available at ```../examples/rime/```. To flash this example into a FW-node using the TI MSP-FET Flash Emulation tools, follow the step below: 
  a. ```(sudo) make TARGET=feuerwhere clean``` 
  b. Compile and upload the example with ```sudo make TARGET=feuerwhere example-runicast.upload``` 
    I. Some problems may arise during the testing process: 
      1. Both FW have the same Node_Id and Rime address. To change, follow these steps: 
        - ```(sudo) make TARGET=feuerwhere clean```  (Do every time you flash a file into the FW mote)
        - ```(sudo)  make TARGET=feuerwhere nodeid=1 example-runicast.upload``` 
    II. FW are not able to communicated with each other. 
      1. Add the following lines 323 to 325 in ```/platform/feuerwhere/contiki-feuerwhere-main.c``` to enable RIME Stack for FW-nodes: 
      ``` NETSTACK_RDC.init(); 
          NETSTACK_MAC.init(); 
          NETSTACK_NETWORK.init(); 
      ```
       2.Comment the following code line: 
       ```//netstack_init(); ``` 
2. The same example can be flashed into the FW-nodes using the SMC programmer and the python script developed by the IHP:  
  - ```sudo make TARGET=feuerwhere clean ```

  - ```sudo make TARGET=feuerwhere nodeid=1 example-runicast.hex```  

  - ```sudo python mb_program.py -p /dev/ttyUSB4 -f example-runicast.hex``` 


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
