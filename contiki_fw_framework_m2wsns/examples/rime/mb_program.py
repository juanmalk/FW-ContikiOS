#!/usr/bin/env python

import serial
from exceptions import IOError
import sys, time, string, cStringIO, struct
import getopt, sys



class BSLException(Exception):
    pass

class VerifyException(Exception):
    pass

class Segment:
    """store a string with memory contents along with its startaddress"""
    def __init__(self, startaddress = 0, data=None):
        if data is None:
            self.data = ''
        else:
            self.data = data
        self.startaddress = startaddress

    def __getitem__(self, index):
        return self.data[index]

    def __len__(self):
        return len(self.data)

    def __repr__(self):
        return "Segment(startaddress = 0x%04x, data=%r)" % (self.startaddress, self.data)
    
class BSL:
    "lowlevel communication"
    #Constants
    MODE_BSL                = 0x01
    

    BSL_SYNC                = 0x80
    BSL_TXPASS             = 0x11
    BSL_TXBLK               = 0x10 #Transmit block to boot loader
    BSL_RXBLK               = 0x18 #Receive  block from boot loader
#    BSL_ERASE               = 0x12 #Erase one segment
    BSL_MASS_ERASE          = 0x15 #Erase complete FLASH memory
    BSL_CHANGEBAUD          = 0x52 #Change baudrate
#    BSL_LOADPC              = 0x17 #Load PC and start execution
    BSL_TXVERSION           = 0x19 #Get BSL version


    #Header Definitions
    DATA_ACK                = 0x00
    
    # BSL Respondes
    BSL_REPLY_DATA          = 0x3A
    BSL_REPLY_MSG           = 0x3B
    
    BSL_MSG_SUCCESS         = 0x00
    BSL_MSG_PASS_ERROR      = 0x05
    
    BSL_EXPECTED_VERSION    = [0x3a, 0,5,4,3]
    
    #BSL_BAUD_RATE_57600     = 0x06
    BSL_BAUD_RATE_115200     = 0x06
    
    ACTION_PROGRAM          = 0x01 #Mask: program data
    ACTION_VERIFY           = 0x02 #Mask: verify data
    ACTION_ERASE_CHECK      = 0x04 #Mask: erase check

    #Max. bytes sent within one frame if parsing a TI TXT file.
    #( >= 16 and == n*16 and <= MAX_DATA_BYTES!)
    MAXDATA                 = 240-16

    def __init__(self):
        self.timeout = 1    #timeout of the serial port
        self.tx_timeout = 0.0012    # timeout between sending chars
        self.segments = []
        self.byteCtr        = 0
        
    def openPort(self, port, baudrate):
        # Startup-Baudrate: 9600,8,N,1, 1s timeout
        self.serialport = serial.Serial(
            port,
            baudrate,
            parity = serial.PARITY_EVEN,
            timeout = self.timeout,
            stopbits = serial.STOPBITS_ONE
        )
        
        self.serialport.flushInput()
        self.serialport.flushOutput()
        
        
    def closePort(self):
        self.serialport.close()     
        
    def write(self, frame):
        # write characters one by one, and wait after sending
        # the next one
        for b in frame:
            wr_len = self.serialport.write(b)
            if (wr_len != 1):
                raise IOError("TX Error")
            #self.txWait()
            
    # expect cmd as string
    def writeCmd(self, cmd, verifyAck=True):    
        
        # write SYNC character at the beginning of each frame
        frame = list()
        frame += "%c" % self.BSL_SYNC
        
        # next 2 bytes in the length of the command 
        frame += "%c" % (len(cmd) & 0xff)
        frame += "%c" % ((len(cmd) >> 8) & 0xff)
        
        # variable-length field includes the command
        frame += cmd 
            
        # next 2 bytes is the CRC of the cmd
        cmd = list(cmd)
        checksum = self.calcChecksum(cmd, len(cmd))
        frame += "%c" % (checksum & 0xff)
        frame += "%c" % ((checksum >> 8) & 0xff)
        
        # send frame
        self.write(frame)
        
        # read ACK, and check if verification not disabled
        s = bsl.read(1)
        if verifyAck:
            if len(s) < 1:
                raise IOError("ACK timeout for cmd " + str(cmd))
            elif ord(s) != self.DATA_ACK:
                raise IOError("Wrong ACK when tx cmd, %d" % ord(s))
            
    def readCmd(self):        
        
        # read BSL_SYNC
        c = self.read(1)
        if len(c) < 1:
            raise IOError("Timeout when reading frame (BSL_SYNC")
        elif ord(c) != self.BSL_SYNC:
            raise IOError("Wrong BSL_SYNC char, %d" % ord(c)) 
        
        # read command len (2 bytes)
        str_len = self.read(2)
        if len(str_len) < 2:
            raise IOError("Read only %d out of 2 len bytes" % len(str_len))
        
        frame_len = ord(str_len[0]) 
        frame_len += ord(str_len[1]) << 8
       
        cmd = self.read(frame_len)
        if len(cmd) < frame_len:
            raise IOError("Rx cmd too short, %d < %d" % len(cmd), frame_len)
        
        # read CRC
        rx_crc = self.read(2)
        if len(rx_crc) < 2:
            raise IOError("CRC too short %d " % len(rx_crc))
        
        crc = ord(rx_crc[0]) 
        crc += ord(rx_crc[1]) << 8
        
        checksum =  self.calcChecksum(cmd, len(cmd))
        
        if crc != checksum:
            raise IOError("Wrong CRC of Rx cmd")
        
        return cmd
    
    
    def read(self, max_chars):    
        return self.serialport.read(max_chars)
    
    def txWait(self):
        time.sleep(self.tx_timeout)
            
    def calcChecksum(self, data, length):
        """Calculates a checksum of "data"."""
        checksum = 0xffff
        for i in range(length):
            x = ( (checksum >> 8) ^ (ord(data[i])) ) & 0xff
            x = x ^ (x >> 4)    
            checksum = (checksum << 8) ^ (x << 12) ^ (x << 5) ^ x
        return 0xffff & checksum 
    
    def massErase(self):
        cmd = "%c" % self.BSL_MASS_ERASE
        bsl.writeCmd(cmd)
        
        # verify BSL reply
        self.rxSuccessMsg("Mass Erase")
        print "Mass Erase completed"
            
    def getBSLVersion(self, verifyVersion = False):         
        cmd = "%c" % self.BSL_TXVERSION
        bsl.writeCmd(cmd)
        
        # verify BSL reply
        reply = self.readCmd()
        version_list = [ord(c) for c in reply] 
        if version_list != self.BSL_EXPECTED_VERSION:
            print "Version: " + reply 
            raise IOError("Not expected BSL version " + str(version_list))
        
        return reply 
    
    def changeBaud57600(self):
        cmd = "%c" % self.BSL_CHANGEBAUD;
        cmd += "%c" % self.BSL_BAUD_RATE_57600;
        bsl.writeCmd(cmd)
         
    def changeBaud115200(self):
        cmd = "%c" % self.BSL_CHANGEBAUD;
        cmd += "%c" % self.BSL_BAUD_RATE_115200;
        bsl.writeCmd(cmd)
  
    def print_bsl_cmd(self, string):    
        for c in string:
            print "%x " % ord(c),
  
    # serial ports RTS/DTR are I2C inputs to the switch, which
    # gets 8-bits data over I2C and sets the switch outputs accordingly
    def SetSCL(self, level):
        self.serialport.setRTS(not level)
#
    def SetSDA(self, level):
        self.serialport.setDTR(not level)
#
    def I2CStart(self):
        self.SetSDA(1)
        self.SetSCL(1)
        self.SetSDA(0)
#
    def I2CStop(self):
        self.SetSDA(0)
        self.SetSCL(1)
        self.SetSDA(1)
#
    def I2CWriteBit(self, bit):
        self.SetSCL(0)
        self.SetSDA(bit)
        time.sleep(2e-6)
        self.SetSCL(1)
        time.sleep(1e-6)
        self.SetSCL(0)
#
    def I2CWriteByte(self, byte):
        self.I2CWriteBit( byte & 0x80 );
        self.I2CWriteBit( byte & 0x40 );
        self.I2CWriteBit( byte & 0x20 );
        self.I2CWriteBit( byte & 0x10 );
        self.I2CWriteBit( byte & 0x08 );
        self.I2CWriteBit( byte & 0x04 );
        self.I2CWriteBit( byte & 0x02 );
        self.I2CWriteBit( byte & 0x01 );
        self.I2CWriteBit( 0 );  # "acknowledge"
#
    def I2CWriteCmd(self, addr, cmdbyte):
        self.I2CStart()
        self.I2CWriteByte( 0x90 | (addr << 1) )
        self.I2CWriteByte( cmdbyte )
        self.I2CStop()
        time.sleep(0.020) 
        
    def enterBSL(self):    
        self.serialport.flushInput()  #clear buffers
        self.serialport.flushOutput()  #clear buffers
       
        # assure that "the workaround MSP" is not waiting for Reset sequence
        self.exitBSL()
        self.exitBSL()
        self.I2CWriteCmd(0,0)
        time.sleep(0.005)
        self.I2CWriteCmd(0,0x2)
        self.I2CWriteCmd(0,0)
        time.sleep(0.250)
        self.serialport.flushInput()  #clear buffers
        self.serialport.flushOutput()  #clear buffers
        
    # Although these sequence resets the node, the UART interface does not
    # work properly afterwards, it sends wrong data (still don't know why)
    # Therefore, the best way to reset a node is to program it again
    def resetNode(self):    
        self.I2CWriteCmd(0,0)
        #time.sleep(0.250)
        self.I2CWriteCmd(0,1)
        #time.sleep(0.250)
        self.I2CWriteCmd(0,0)
        #time.sleep(0.250)
        self.serialport.flushInput()  #clear buffers
        self.serialport.flushOutput()  #clear buffers
        
        
    def exitBSL(self):    
        self.I2CWriteCmd(0,0)
        self.I2CWriteCmd(0,1)
        self.I2CWriteCmd(0,0)
        
    def rxSuccessMsg(self, fail_msg = "", alternative_replies = []):     
        reply = self.readCmd()
        reply = [ord(c) for c in reply] 
        
        # check if msg starts with REPLY_MSG char
        if len(reply) < 2:
            raise IOError("too short reply on %s" % fail_msg)
            
        if reply[0] != self.BSL_REPLY_MSG:
            raise IOError("Did not receive reply msg on %s" % fail_msg)
            
        if reply[1] == self.BSL_MSG_SUCCESS:
            return reply[1] 
        
        # check alternative replies
        if reply[1] in alternative_replies:
            return reply[1]
             
        #expected_cmd = [self.BSL_REPLY_MSG, self.BSL_MSG_SUCCESS] 
        #if reply != expected_cmd:
        raise IOError("Wrong reply (%d) on %s" % (ord(reply[1]), fail_msg) )
 
    def txPasswd(self, passwd=None):
        if passwd is None:
            passwd = chr(0xff)*32
        else:
            #sanity check of password
            if len(passwd) != 32:
                raise ValueError, "password has wrong length (%d)\n" % len(passwd)
            
        cmd = "%c" % self.BSL_TXPASS
        cmd += passwd 
        self.writeCmd(cmd, verifyAck = False)
        
        # expect msg with an ACK
        res = self.rxSuccessMsg("Tx Passwd", 
                          alternative_replies = [self.BSL_MSG_PASS_ERROR])
        
        if res == self.BSL_MSG_SUCCESS: 
            print "TxPasswd completed" 
        elif res == self.BSL_MSG_PASS_ERROR:
            raise IOError("Wrong password, performing Mass Erase")
            
        else:
            raise IOError("Wrong reply on TxPasswd")    
            
    def loadIHex(self, file):
        """load data from a (opened) file in Intel-HEX format"""
        segmentdata = []
        currentAddr = 0
        startAddr   = 0
        lines = file.readlines()
        for l in lines:
            if l[0] != ':': raise BSLException("File Format Error\n")
            l = l.strip()       #fix CR-LF issues...
            length  = int(l[1:3],16)
            address = int(l[3:7],16)
            type    = int(l[7:9],16)
            check   = int(l[-2:],16)
            if type == 0x00:
                if currentAddr != address:
                    if segmentdata:
                        self.segments.append( Segment(startAddr, string.join(segmentdata,'')) )
                    startAddr = currentAddr = address
                    segmentdata = []
                for i in range(length):
                    segmentdata.append( chr(int(l[9+2*i:11+2*i],16)) )
                currentAddr = length + currentAddr
            elif type in (0x01, 0x02, 0x03, 0x04, 0x05):
                pass
            else:
                sys.stderr.write("Ignored unknown field (type 0x%02x) in ihex file.\n" % type)
        if segmentdata:
            self.segments.append( Segment(startAddr, string.join(segmentdata,'')) )
            
    def createCmd(self, command, address = 0, dataOut=None, length = 0):        
        cmd = "%c" % command 
        
        if address: 
            cmd += "%c%c%c" % (address & 0xff, (address >> 8) & 0xff, (address >> 16) & 0xff)
                
        if dataOut:        
            for c in dataOut:
                cmd += "%c" % c
                
        elif length:
            cmd += "%c%c" % (length & 0xff, (length >> 8) & 0xff)
            
        return cmd    

    def verifyBlk(self, addr, blkout, action):
        """verify memory against data or 0xff"""

        cmd = self.createCmd(self.BSL_RXBLK, addr, length=len(blkout))
        self.writeCmd(cmd)
        reply = self.readCmd() 
        
        if (len(reply) < 1):
            raise VerifyException("empty reply on VerifyBlk")
        
        elif ord(reply[0]) != self.BSL_REPLY_DATA:
            raise VerifyException("Wrong reply cmd on Verify, %d != %d" 
                          % (ord(reply[0]), self.BSL_REPLY_DATA))
            
        blkin = reply[1:]

        for i in range(len(blkout)):
            if action & self.ACTION_VERIFY:
                #Compare data in blkout and blkin
                if blkin[i] != blkout[i]:
                    sys.stderr.write("Verification failed at 0x%04x "
                                     "(0x%02x, 0x%02x)\n" % 
                                     (addr+i, ord(blkin[i]), ord(blkout[i])))
                    sys.stderr.flush()
                    raise VerifyException("Verification (programming) failed")
                continue
            
            elif action & self.ACTION_ERASE_CHECK:
                #Compare data in blkin with erase pattern
                if blkin[i] != chr(0xff):
                    sys.stderr.write("Erase Check failed at 0x%04x (0x%02x)\n" 
                                     % (addr+i, ord(blkin[i])))
                    sys.stderr.flush()
                    raise BSLException("Erase Check failed") 
                continue
            
                
    def programBlk(self, addr, blkout, action):
        """programm a memory block"""

        #Check, if specified range is erased
        self.verifyBlk(addr, blkout, self.ACTION_ERASE_CHECK)

        if action & self.ACTION_PROGRAM:
            cmd = self.createCmd(self.BSL_TXBLK, address = addr, 
                                 dataOut = blkout)
            self.writeCmd(cmd, verifyAck = False)
            self.rxSuccessMsg("Programming")

        #Verify block
        self.verifyBlk(addr,  blkout, self.ACTION_VERIFY)        
        
    #segments:
    #list of tuples or lists:
    #segements = [ (addr1, [d0,d1,d2,...]), (addr2, [e0,e1,e2,...])]
    def programData(self): 
       
        segments = self.segments
        action = self.ACTION_PROGRAM
       
        """programm or verify data"""
        print "Programming"
        for seg in segments:
            currentAddr = seg.startaddress
            pstart = 0
            while pstart<len(seg.data):
                length = self.MAXDATA
                if pstart+length > len(seg.data):
                    length = len(seg.data) - pstart
                self.programBlk(currentAddr, seg.data[pstart:pstart+length], action)
                pstart = pstart + length
                currentAddr = currentAddr + length
                self.byteCtr = self.byteCtr + length #total sum
        print "Programming finished"
        time.sleep(0.1) 
     
    def finish(self):
        # finish BSL
        print "BSL finished"
        bsl.exitBSL()
        bsl.closePort()
               
def usage():
    sys.stderr.write("""
USAGE: %s [options] 

General options:
  -f FILE,              Specify image file (ihex format) to program the node 
  -e                    Perform MassErase
  -p PORT               Port the node is connected to 
  -r                    Reset node
 """ % (sys.argv[0]))  
    
    
try:
    opts, args = getopt.getopt(sys.argv[1:], "f:ep:r", [])
except getopt.GetoptError, err:
    # print help information and exit:
    print str(err) # will print something like "option -a not recognized"
    usage()
    sys.exit(2)
    
bsl = BSL()
    
program_node = False    
mass_erase = False
port_specified = False
reset_node = False
unique_option_cnt = 0
    
for o, a in opts:
    if o == "-f":
        filename = a
        program_node = True
        try:
            file = open(filename, "rb")
        except IOError:
            print "Cannot open file: " + filename
            sys.exit(1)
        
        bsl.loadIHex(file)    
        unique_option_cnt += 1
         
    elif o == "-e":
        mass_erase = True;
        
    elif o == "-p":
        port_specified = True;
        port = a
        
    elif o == "-r":
        reset_node = True
        unique_option_cnt += 1
        
    else:
        print "unhandled option"
        usage();
        sys.exit(2)
        
if not port_specified:
    print "No serial port specified"    
    usage()
    sys.exit(2)
    
if (not mass_erase) and (not program_node) and (not reset_node):
    print "Neither programming/mass erase/reset selected"
    usage()    
    sys.exit(2)
    
if unique_option_cnt > 1:
    print "Only one unique command (reset/programming) allowed"
    usage()    
    sys.exit(2)
    
    
# when resetting use 115200 baudrate
# there were problems when opening port in 9600 mode - the node was resetted
# but serial stopped working (MSP430 kept sending data, but nothing was
# forwarded to the PC over FTDI)
if reset_node:
    print "Resetting node"
    bsl.openPort(port, 115200)
    bsl.finish()
    sys.exit(0)
 
# open port with a baudrate of 9600 (default setup of FWnode BSL)
try:
    bsl.openPort(port, 9600)
    
except:
    print "Count not open port " + port
    sys.exit(1)
        
   

# enter BSL enter sequence; after that, FWnode is ready to get BSL commands
bsl.enterBSL()

# always erase memory before programming to assure that password is
# 0xff 32 times; otherwise, password equals to interrupt vectors 
print "Mass Erase"
bsl.massErase()
    
# send the default password to BSL and unlock most BSL commands     
bsl.txPasswd()

#for i in range(100):
#    bsl.getBSLVersion(verifyVersion = True)


# change baudrate to 115200
print "Changing baudrate to 115200"
bsl.changeBaud115200()
bsl.closePort()
bsl.openPort(port, 115200)
    
if program_node:
    # program the node with image file
    bsl.programData()

bsl.finish()
