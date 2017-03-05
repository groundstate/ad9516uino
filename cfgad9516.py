#!/usr/bin/env python

#
# cfgad9516.py - a Python script for configuring an AD9516 via an Arduino 
#
# The MIT License (MIT)
#
# Copyright (c)  2017  Michael J. Wouters
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import re
import serial
import argparse

# Globals

VERSION = "0.1"
AUTHORS = "Michael Wouters"

debug = False

# ------------------------------------------
def ShowVersion():
	print "cfgad9516",VERSION
	print "Written by",AUTHORS
	print "This ain't no stinkin' Perl script!"
	return

# ------------------------------------------
def Debug(msg):
	if (debug):
		print msg
	return

# ------------------------------------------

parser = argparse.ArgumentParser(description='Configure an AD9516')
parser.add_argument('--debug','-d',help='debug',action='store_true')
parser.add_argument('--port','-p',help='specify the serial port',default='/dev/ttyACM0')
parser.add_argument('--read','-r',help='read register REG',metavar=('REG'))
parser.add_argument('--reset',help='reset',action='store_true')
parser.add_argument('--show','-s',help='show all registers',action='store_true')
parser.add_argument('--write','-w',help='write VAL to  register REG',nargs=2,metavar=('REG','VAL'))
parser.add_argument('--version','-v',help='show version and exit',action='store_true')
args = parser.parse_args()

debug = args.debug

if (args.version):
	ShowVersion()
	exit()

Debug('Opening ' + args.port)
try:
	port = serial.Serial(args.port,9600,timeout=1)
except :
	print 'Error opening device',args.port
	exit()
	
# Test for an appropriately programmed Arduino
port.write('*IDN?\n')
idn = port.readline()[:-2] # remove CRLF 
if (re.match(r'^AD9516uino',idn) == None):
	print 'Found ' + idn + ',not AD516uino'
	exit()

if (args.read):
	try:
		ival = int(args.read,16)
	except ValueError:
		print 'Bad input value', args.read
		exit()
	
	hval = hex(ival)[2:] # strip 0x
	port.write(':REG#H'+hval+'?\n')
	
	regval = port.readline()[:-2]
	if (len(regval)==0):
		print "No response"
		exit()
		
	regval = regval.upper()
	m=re.match(r"\s*([a-fA-F0-9]+),\s*([a-fA-F0-9]+)",regval)
	if (m):
		print 'reg=0x'+m.group(1)+",val=0x"+m.group(2)
	else:
		print "Bad response - got",regval
	
	exit()

if (args.reset):
	port.write('*RST')
	exit()
	
if (args.show):
	print 'Register configuration (reg,value,default value - all in hex)'
	# The only way to determine when output is finished is to look for the 
	# the last register, 0x232
	gotLast=False
	port.write(':REG?\n')
	while (not gotLast):
		regval = port.readline()[:-2] # blocks with 1 s timeout
		if (re.match(r"\s*232\s*,",regval)):
				gotLast=True
		print regval
	exit()

if (args.write):
	try:
		ival = int(args.write[0],16)
	except ValueError:
		print 'Bad input value', args.write[0]
		exit()
	try:
		ival2 = int(args.write[1],16)
	except ValueError:
		print 'Bad input value', args.write[1]
		exit()
	
	hval = hex(ival)[2:] # strip 0x
	hval2 = hex(ival2)[2:] # strip 0x
	port.write(':REG#H'+hval+',#H'+hval2+'\n')
	port.write(':REG#H232,#H1\n')
	exit()
	
# Default is to confirm there's something there
print 'An Arduino+AD9516 is connected'

port.close()

