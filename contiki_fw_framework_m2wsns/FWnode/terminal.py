#!/usr/bin/python

import argparse
import sys
import serial
import time
import os
from math import floor

USB_DEVICES_DIR = "/sys/bus/usb/devices"
IMAGE_GET_ID="./read_id.hex"

FWNODE_PORT_SPEED = "115200"

parser = argparse.ArgumentParser("Read data from FWnode over USB")
parser.add_argument("-p", "--port", required=True,
                    help="port the FWnode is connected to")
parser.add_argument("-o", "--out", required=False,
                    help="output is written to this file")
parser.add_argument("-t", required=False, action='store_true',
                    help="timestamp as an absolute time")
parser.add_argument("-a", required=False, action='store_true',
                    help="append data to the file ")
args = vars(parser.parse_args(sys.argv[1:]))

port = args["port"]
outfilename = args["out"]

timestamp_absolute = args["t"]
file_append = args["a"]
#open file for writing if "out" argument specified
if outfilename:

    if file_append:
        open_args = "a"
    else:
        open_args = "w"

    outfile = open(outfilename, open_args)

    if not outfile:
        print "ERROR: could not open file " + outfilename
        sys.exit(1)

else:
	outfile = None


SEC_IN_MIN = 60
SEC_IN_HOUR = 3600
HOUR_IN_DAY = 24
SEC_IN_DAY = SEC_IN_HOUR * HOUR_IN_DAY

def sec_to_string(sec):

	days = floor(sec / (SEC_IN_HOUR * HOUR_IN_DAY))
	sec -= days * SEC_IN_DAY

	hours = floor(sec / (SEC_IN_HOUR))
	sec -= hours * SEC_IN_HOUR

	mins = floor(sec / SEC_IN_MIN)
	sec -= mins * SEC_IN_MIN

	return "%02d %02d:%02d:%06.3f" % (days, hours, mins, sec)

# open serial port and read id
ser = serial.Serial(port, FWNODE_PORT_SPEED)
start_time = time.time()
while 1:
    line = ser.readline()

    # print timestamp either as an absolute time or a relative time from
    # the start
    if timestamp_absolute:
        #str_time = time.strftime("%d-%m-%Y,%H:%M.%S", time.gmtime())
        str_time = time.strftime("%d-%m-%Y,%H:%M:%S.%f", time.gmtime()
    else:
        cur_time = time.time()
        elapsed = cur_time - start_time
        str_time = sec_to_string(elapsed)

    full_line = str_time + "  " + line
    #print '  %.3f ' % elapsed,
    print full_line,

	# write line to the output file, if defined
    if outfile:
       outfile.write(full_line)
       outfile.flush()
       os.fsync(outfile.fileno())

else:
    # did not read port
    pass
