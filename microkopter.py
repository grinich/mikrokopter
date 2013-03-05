#! /usr/bin/env python

# Mikrokopter communication in python
# All code (c) 2013 Michael Grinich unless otherwise noted.


import os
import glob
import serial
import time
import traceback
import io
import sys
import struct


########################################################################
##    Serial communication stack
########################################################################

class MkComm:
    ADDRESS_ALL    = 0
    ADDRESS_FC     = 1
    ADDRESS_NC     = 2
    ADDRESS_MK3MAG = 3
    
    def __init__(self, printDebugMsg=False):
        #self.logfile = open('mklog.txt', "rbU")

        self.serPort = None
        self.printDebugMsg = printDebugMsg

        msg = MkMsg(address=0, cmd='d', data=[500])
        self.getDebugMsgLn = msg.generateUartLine()
    
    def dprint(self, to_print):
        if self.printDebugMsg:
            print '[Debug]    ' + str(to_print)

    def open(self, comPort, timeout = 0.5):
        self.serPort = serial.Serial(comPort, 57600, timeout=0.5)
        if not self.serPort.isOpen():
            raise IOError("Failed to open serial port")

    def close(self):
        self.serPort.close()
        
    def isOpen(self):
        return self.serPort != None

    def sendLn(self, ln):
        # print 'Sending UART:', repr(ln)
        print 'Sending UART:', ln.encode('hex_codec')
        self.serPort.write(ln)

    def waitForLn(self):

        if not self.serPort.isOpen(): 
            print 'Serial port is not open! WTF?'

        # TODO: Add a timeout here in case it doesn't read anything
        r = []
        while 1:
            c = self.serPort.read(1)
            if c is '':
                self.dprint('<waitForLn> No bytes to read')
                break
            elif c == '\r':
                r.append(c)
                # sys.stdout.write(''.join(r))
                # r = []
                break
            else:
                r.append(c)

        # self.dprint('<<Received '+str(len(r))+' bytes>> ' + str(r) )
        return ''.join(r)

    def waitForMsg(self, cmd2wait4, timeout=0.5):
        msg = None
        done = False

        self.dprint("Waiting for message %s timeout=%.1fs" % (cmd2wait4, timeout) )

        startTime = time.clock()
        while (not done):

            line = self.waitForLn()

            try:
                msg = MkMsg.createFromData(line)
                # msg = MkMsg(msg=line)
                self.dprint( "Received msg: " + str(msg) )
                if (msg.cmd == cmd2wait4):
                    done = True
            except InvalidMsg:
                if self.printDebugMsg: print "no valid msg"
                
            if ((time.clock()-startTime) > timeout):
                raise NoResponse(cmd2wait4)
        self.dprint( "Done in: %.4fs" % (time.clock()-startTime) )
        self.dprint('')
        return msg
    
    def sendMsg(self, msg):
        self.dprint('Sending: ' + str(msg))
        self.sendLn(msg.generateUartLine())


# Past this are specific commands that probably can be abstracted much better.
# They're actually probably broken.

    def sendNCRedirectUartFromFC(self):
        self.serPort.flushInput()
        msg = MkMsg(address=MkComm.ADDRESS_NC, cmd='u', data=[0])
        self.sendMsg(msg)
        time.sleep(.5)
        # No reply expected...   
        
    def sendSettings(self, settings):
        msg = MkMsg(address=MkComm.ADDRESS_FC, cmd='s', data=settings)
        self.sendMsg(msg)
        #time.sleep(1)
        msg = self.waitForMsg('S', timeout=2)
        
    def getDebugMsg(self):
        self.serPort.flushInput()
        self.sendLn(self.getDebugMsgLn)
        msg = self.waitForMsg('D')
        msg = DebugDataMsg(msg)
        return msg
    
    def getSettingsMsg(self, index=0xFF):
        self.serPort.flushInput()
        msg = MkMsg(address=MkComm.ADDRESS_FC, cmd='q', data=[index])
        self.sendMsg(msg)
        msg = self.waitForMsg('Q')
        msg = SettingsMsg(msg)
        return msg


    def setMotorTest(self, motorSpeeds):
        msg = MkMsg(address=MkComm.ADDRESS_FC, cmd='t', data=motorSpeeds)
        self.sendMsg(msg)
        

    # TODO: Here be dragons. Remove motor test code.
    # Probably really dangerous to allow motor control from serial interface
    # Those blades are literally inches from my face right now.

    def doVibrationTest(self, nbSamples, channel):
        data = []
        for i in range(0,(min(nbSamples,1000)/50)):
          msg = MkMsg(address=MkComm.ADDRESS_FC, cmd='f', data=[channel, i])
          self.sendMsg(msg)
          msg = self.waitForMsg('F')
          msg = VibrationDataMsg(msg)
          data += msg.getData()
           
        # FIXE: should be fixed in the FC code
        data[0]=data[1]
        return data
    
    def recordDbgMsg(self, samplePeriod, nbSamples):
        result = []
        self.serPort.flushInput()
        msg = MkMsg(address=0, cmd='d', data=[int(samplePeriod*100)])
        self.sendMsg(msg)
        for i in range(nbSamples):
            print i
            msg = self.waitForMsg('D', timeout=samplePeriod+1)
            msg = DebugDataMsg(msg)
            result.append(msg)
        return result 




########################################################################
##    Exceptions -- throw them, please.
########################################################################


class MkException(Exception):
    pass

class InvalidMsg(MkException):  
    def __str__(self):
      return "Invalid message"

class CrcError(MkException):
    def __str__(self):
      return "CRC error"

class InvalidArguments(MkException):
    def __str__(self):
      return "Invalid Arguments"

class InvalidMsgType(MkException):
    def __str__(self):
      return "Invalid Message type" 

class InvalidMsgBeginning(MkException):
    def __str__(self):
      return "Invalid Message. Must beging with '#'." 

class InvalidMsgLen(MkException):
    def __str__(self):
      return "Invalid Message Length. Must be at least 6 characters. (apparently)" 

class InvalidMsgEnding(MkException):
    def __str__(self):
      return "Invalid Message. Must end with '\r'." 

class NoResponse(MkException):
    def __init__(self, cmd):
      self.cmd = cmd
      
    def __str__(self):
      return "No Reponse. Waiting for \"%s\" message" % self.cmd
      
    pass


####################################
##    Base message class
####################################


class MkMsg(object):

    @staticmethod
    def createFromData(data):
        commandToClass = \
        {DebugDataRequestMsg.command : DebugDataRequestMsg, 
         DebugDataMsg.command : DebugDataMsg, 
         VersionRequestMsg.command : VersionRequestMsg,
         VersionMsg.command : VersionMsg,
         SettingsRequestMsg.command : SettingsRequestMsg,
         SettingsMsg.command : SettingsMsg,
         OSDRequestMsg.command : OSDRequestMsg,
         OSDMsg.command : OSDMsg,   
         DisplayLineRequestMsg.command : DisplayLineRequestMsg,
         DisplayLineMsg.command : DisplayLineMsg,
         ThreeDDataRequestMsg.command : ThreeDDataRequestMsg,
         ThreeDDataMsg.command : ThreeDDataMsg,
         AnalogLabelRequestMsg.command : AnalogLabelRequestMsg,
         AnalogLabelMsg.command : AnalogLabelMsg,
         BLCtrlRequestMsg.command : BLCtrlRequestMsg,
         BLCtrlMsg.command : BLCtrlMsg}

        if (data == None): 
            raise InvalidArguments

        (address, command, data) = MkMsg.parseUartLine(data)

        if (command in commandToClass.keys()):
            return commandToClass[command](address=address, cmd=command, data=data)
        else:
            return MkMsg(address=address, cmd=command, data=data)


    def __init__(self, address=None, cmd=None, data=None):
        # if (msg != None):
        #     # Create instance based on received message
        #     self.parseUartLine(msg)
        if (address != None and cmd != None and data != None):
            # Create instance based on address, command and data
            self.address = address
            self.cmd = cmd
            self.data = data
        else:
            print 'Cannot create instance. Use MkMsg.createFromData(data) to parse raw msgs.'  
            raise InvalidArguments

    # Not sure what the message type is? Just print it.
    def __str__(self):
        return "<" + self.__class__.__name__ + "> addr = " + str(self.address) + ", " + \
                "cmd: " + self.cmd + ", " + \
                "data: " + str(self.data)


    # generateUartLine and parseUartLine are how we 
    # serialize and deserialize these objects to send to MK
    # These functions are based on the 'VibrationTest' code 
    # by FredericG in the MikroKopter SVN repo under 'Projects'

    def generateUartLine(self):
        msg = ""

        # make header
        msg += '#'
        msg += chr(self.address+ord('a'))
        msg += self.cmd

        # add data
        done = False
        i = 0
        while (i<len(self.data)) and not done:
            a = 0
            b = 0
            c = 0
            try:
                a = self.data[i]
                b = self.data[i+1]
                c = self.data[i+2]
                i = i + 3
            except IndexError:
                done = True
            msg += chr(ord('=') + (a >> 2))
            msg += chr(ord('=') + (((a & 0x03) << 4) | ((b & 0xf0) >> 4)))
            msg += chr(ord('=') + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6)))
            msg += chr(ord('=') + ( c & 0x3f))

        # add crc and  NL
        crc1, crc2 = MkMsg._calcCrcBytes(msg)
        msg += crc1 + crc2
        msg += '\r'
        return msg

    @staticmethod
    def parseUartLine(msg):
        if len(msg)<6:
            raise InvalidMsgLen()
        if (msg[0] != '#'):
            raise InvalidMsgBeginning()
        if (msg[-1] != '\r'):
            raise InvalidMsgEnding()

        _address = ord(msg[1])
        _cmd = msg[2]

        data64 = map(ord, msg[3:-3])    # last 3 bytes are CRC and \n

        done = False
        i = 0
        _data = []
        while (i<len(data64)) and not done:
            a = 0
            b = 0
            c = 0
            d = 0
            try:
                a = data64[i] - ord('=')
                b = data64[i+1] - ord('=')
                c = data64[i+2] - ord('=')
                d = data64[i+3] - ord('=')
                i = i + 4
            except IndexError:
                done = True

            _data.append((a << 2)&0xFF | (b >> 4))
            _data.append(((b & 0x0f) << 4)&0xFF | (c >> 2));
            _data.append(((c & 0x03) << 6)&0xFF | d);

        crc1,crc2 = MkMsg._calcCrcBytes(msg[:-3])
        if (crc1 != msg[-3] or crc2 != msg[-2]):
            #print msg
            raise CrcError
        
        return (_address, _cmd, _data)

        #print "crc= %x %x %x %x" % ( crc1, crc2, (ord(msg[-3])-ord('=')), (ord(msg[-2])-ord('=')))


    def data2SignedInt(self, index):
        int = self.data[index]+self.data[index+1]*256
        if (int > 0xFFFF/2):
            int -= 0xFFFF
        return int

    @staticmethod
    def _calcCrcBytes(str):
        crc = 0
        for c in str:
            crc += ord(c)
        crc &= 0xfff
        return (chr(crc/64+ord('=')), chr(crc%64+ord('=')))




####################################
##    Settings
####################################



class SettingsRequestMsg(MkMsg):
    """Send this message to request settings"""

    address = MkComm.ADDRESS_FC 
    command = 'q'

       # TODO : use the actual settingsIndex here
    def __init__(self, settingsIndex = None, msg=None):
        # For data
        # u8 Settings Index ( 1..5 READ or 0xff for actual setting) 
        # u8 Settings Index (11..15 RESET setting to default (channel mapping will not be changed))
        # u8 Settings Index (21..25 RESET setting to default (complete reset including channel settings))
        data = []
        super(SettingsRequestMsg, self).__init__(address=SettingsRequestMsg.address, 
                                                 cmd=SettingsRequestMsg.command, 
                                                 data=data)


class SettingsMsg(MkMsg):

    command = 'Q'

    def __init__(self, address=None, cmd=None, data=None):
        if (cmd != SettingsMsg.command):
            raise InvalidMsgType
        if len(data) != 105:
            print 'WARNING Settings msg is %u long. Somehow supposed to be 105.' % len(data)
        super(SettingsMsg, self).__init__(address=address, cmd=SettingsMsg.command, data=data)


    # Indicies for settings variables
    DATENREVISION   = 80
    IDX_INDEX       = 0
    IDX_STICK_P     = 1 + 19
    IDX_STICK_D     = 1 + 20
    IDX_NAME        = 1 + 90

    def __str__(self):
        return "<SettingsMsg>" + \
            '\n\tgetSettings: ' + str(self.getSettings() ) + \
            '\n\tgetIndex: ' + str(self.getIndex() ) + \
            '\n\tgetName: ' + str(self.getName() ) 

    def getSettings(self):
        return self.data
        
    def getIndex(self):
        return self.getSetting(SettingsMsg.IDX_INDEX)
    
    def getSetting(self, settingIndex):
        return self.data[settingIndex]
    
    def getName(self):
        name = ""
        for i in self.data[SettingsMsg.IDX_NAME:]:
            if i==0:
                break
            name += chr(i)
        return name
    
    def setSetting(self, settingIndex, value):
        self.data[settingIndex] = value



########################################################################
##    Debug request & response
########################################################################


class DebugDataRequestMsg(MkMsg):
    """Send this message to request a DebugDataMsg.
       Set autoSendInterval for callbacks. Unit is seconds.
       This must be renewed every 4 seconds, says the docs.
       """
    command = 'd'
    address = MkComm.ADDRESS_FC # any

    def __init__(self, address=None, cmd=None, data=None, autoSendInterval = 0):
        if (address != None and cmd != None and data != None):
            super(DebugDataRequestMsg, self).__init__(address=address, cmd=cmd, data=data)
        else:
            # multiplied by 10 in receiver and then used as milliseconds. 
            data = [int(autoSendInterval*100)]
            super(DebugDataRequestMsg, self).__init__(address=DebugDataRequestMsg.address, 
                                                      cmd=DebugDataRequestMsg.command, 
                                                      data=data)

    def __str__(self):
        return "<" + self.__class__.__name__ + ">" + \
                "\n\tSend interval: " + str(self.data[0] * 10) + "ms"



class DebugDataMsg(MkMsg):

    command = 'D'
    def __init__(self, address=None, cmd=None, data=None):
        if (cmd != DebugDataMsg.command):
            raise InvalidMsgType
        super(DebugDataMsg, self).__init__(address=address, cmd=cmd, data=data)


    # Analog labels for debug data
        # "AngleNick       ", //0
        # "AngleRoll       ",
        # "AccNick         ",
        # "AccRoll         ",
        # "OperatingRadius ",
        # "FC-Flags        ", //5
        # "NC-Flags        ",
        # "NickServo       ",
        # "RollServo       ",
        # "GPS Data        ",
        # "CompassHeading  ", //10
        # "GyroHeading     ",
        # "SPI Error       ",
        # "SPI Okay        ",
        # "I2C Error       ",
        # "I2C Okay        ", //15
        # "16              ",
        # "17              ",
        # "18              ",
        # "19              ", // SD-Card-time 
        # "EarthMagnet [%] ", //20
        # "Z_Speed         ",
        # "N_Speed         ",
        # "E_Speed         ",
        # "Magnet X        ",
        # "Magnet Y        ", //25
        # "Magnet Z        ",
        # "Distance N      ",
        # "Distance E      ",
        # "GPS_Nick        ",
        # "GPS_Roll        ", //30
        # "Used_Sats       "


    IDX_ANALOG_ANGLENICK=   2+2*0
    IDX_ANALOG_ANGLEROLL=   2+2*1
    IDX_ANALOG_ACCNICK  =   2+2*2
    IDX_ANALOG_ACCROLL  =   2+2*3
    IDX_ANALOG_COMPASS  =   2+2*3
    IDX_ANALOG_VOLTAGE  =   2+2*9




    def __str__(self):
        return "<"+ self.__class__.__name__ +">" + \
            '\n\tAngleNick: ' + str(self.getAngleNick()) + \
            '\n\tAngleRoll: ' + str(self.getAngleRoll()) + \
            '\n\tAcclNick: ' + str(self.getAccNick()) + \
            '\n\tAccRoll: ' + str(self.getAccRoll()) + \
            '\n\tCompassHeading: ' + str(self.getCompassHeading()) + \
            '\n\tVoltage: ' + str(self.getVoltage()) 


    def getAngleNick(self):
        return self.data2SignedInt(DebugDataMsg.IDX_ANALOG_ANGLENICK)

    def getAngleRoll(self):
        return self.data2SignedInt(DebugDataMsg.IDX_ANALOG_ANGLEROLL)

    def getAccNick(self):
        return self.data2SignedInt(DebugDataMsg.IDX_ANALOG_ACCNICK)

    def getAccRoll(self):
        return self.data2SignedInt(DebugDataMsg.IDX_ANALOG_ACCROLL)

    def getCompassHeading(self):
        return self.data2SignedInt(DebugDataMsg.IDX_ANALOG_COMPASS)

    def getVoltage(self):
        return float(self.data2SignedInt(DebugDataMsg.IDX_ANALOG_VOLTAGE))/10

########################################################################
##    Debug request & response
########################################################################


class VibrationDataMsg:
    command = 'F'
    def __init__(self, msg):
        if (msg.cmd != command):
            raise InvalidMsgType
        self.msg = msg
        
    def getData(self):
        data = []
        for i in range(0,50):
          data.append(self.msg.data2SignedInt(2*i))
        return data


########################################################################
##    Version request & response
########################################################################


class VersionRequestMsg(MkMsg):
    """Send to request version"""
    address = 0 # any
    command = 'v'

    def __init__(self, address=None, cmd=None, data=None):
        if (address != None and cmd != None and data != None):
            super(VersionRequestMsg, self).__init__(address=address, cmd=cmd, data=data)
        else:
            data = []
            super(VersionRequestMsg, self).__init__(address=VersionRequestMsg.address, 
                                                    cmd=VersionRequestMsg.command, 
                                                    data=data)

class VersionMsg(MkMsg):
    """docstring for VersionMsg"""
    command = 'V'
    address = 0 #any
    def __init__(self, address=None, cmd=None, data=None):
        if (cmd != VersionMsg.command):
            raise InvalidMsgType
        if len(data) != 105:
            print 'Settings msg is %u long' % len(data)
        super(VersionMsg, self).__init__(address=address, cmd=cmd, data=data)


    def getVersion(self):
        return (self.data[0], self.data[1])


########################################################################
##    OSD message commands
########################################################################


class OSDRequestMsg(MkMsg):
    """Request OSD data. autoSendInterval is """
    command = 'o'
    address = MkComm.ADDRESS_NC

    def __init__(self, address=None, cmd=None, data=None, autoSendInterval = 0):
        if (address != None and cmd != None and data != None):
            super(OSDRequestMsg, self).__init__(address=address, cmd=cmd, data=data)
        else:
            # multiplied by 10 in receiver and then used as milliseconds. 
            data = [int(autoSendInterval*100)]
            super(OSDRequestMsg, self).__init__(address=OSDRequestMsg.address, 
                                                cmd=OSDRequestMsg.command, 
                                                data=data)

    def __str__(self):
        return "<" + self.__class__.__name__ + ">" + \
                "\n\tInterval: " + str(self.data[0] * 10) + "ms"



class OSDMsg(MkMsg):
    """docstring for OSDMsg"""
    command = 'O'
    address = MkComm.ADDRESS_NC

    def __init__(self, address=None, cmd=None, data=None):
        if (cmd != OSDMsg.command):
            raise InvalidMsgType
        super(OSDMsg, self).__init__(address=address, cmd=cmd, data=data)


        gpsPositionStructFormat = "iiiB"
        # Longitude in 1E-7 degrees (s32)
        # Latitude in 1E-7 degrees (s32)
        # Altitude (s32)
        # Status (u8)

        gpsDeviationStructFormat = "Hh"
        # Distance in dm (u16)
        # Bearing in degrees(s16) 


        ### NaviData struct packing

        # Version (u8)
        # CurrentPosition (GPS_Pos_t struct)
        # TargetPosition (GPS_Pos_t struct)
        # TargetPositionDeviation (GPS_PosDev_t struct)

        # HomePosition (GPS_Pos_t struct)
        # HomePositionDeviation (GPS_PosDev_t struct)
        # WaypointIndex (u8)
        # WaypointNumber (u8)

        # SatsInUse (u8)
        # Barometric Altimeter (s16)                              // hight according to air pressure
        # Variometer (s16) -- climb(+) and sink(-) rate
        # Flying time seconds (u16)

        # Battery voltage in 0.1 Volts (u8)
        # 2D Ground speed in cm/s (u16)
        # Heading in degress as angle to north (s16)
        # Compass value in degrees (s16)

        # AngleNick degrees (s8)
        # AngleRoll degrees (s8)
        # RC_Quality (u8)
        # Flags from FC (u8)

        # Flags from NC (u8)
        # Error code, 0 is OK (u8)
        # Operating Radius around home position in meters (u8)
        # Velocity in vertical direction in cm/sec (s16)

        # Target Hold time -- seconds to stay at the given target, counts down to 0 if target has been reached (u8)
        # More flags from FC since version 5 (u8)
        # Setpoint for altitude (s16)
        # Gas (u8) -- for future use

        # Actual current in 0.1A steps (u16)
        # Used capacity in mAh (u16)

        # begin format string with '=' because __attribute__((packed)) in c++ removes padding
        format_string = '=' + 'B' + gpsPositionStructFormat + gpsPositionStructFormat + gpsDeviationStructFormat + \
                        gpsPositionStructFormat + gpsDeviationStructFormat + 'B' + 'B' + \
                        'B' + 'h' + 'h' + 'H' + \
                        'B' + 'H' + 'h' + 'h' + \
                        'b' + 'b' + 'B' + 'B' + \
                        'B' + 'B' + 'B' + 'h' + \
                        'B' + 'B' + 'h' + 'B' + \
                        'h' + 'h'

        (self.version, 
         self.currentPositionLongitude,
         self.currentPositionLatitude, 
         self.currentPositionAltitude, 
         self.currentPositionStatus,   
         self.targetPositionLongitude,
         self.targetPositionLatitude, 
         self.targetPositionAltitude, 
         self.targetPositionStatus,
         self.targetPositionDistance,
         self.targetPositionBearing,
         self.homePositionLongitude,
         self.homePositionLatitude, 
         self.homePositionAltitude, 
         self.homePositionStatus,   
         self.homePositionDistance,
         self.homePositionBearing,
         self.waypointIndex,
         self.waypointNumber,
         self.satsInUse,
         self.altitude,
         self.variometer,
         self.flyingTime,
         self.batteryVoltage,
         self.groundSpeed,
         self.heading,
         self.compass,
         self.angleNick,
         self.angleRoll,
         self.RCQuality,
         self.FC_flags,
         self.NC_FLAGS,
         self.errorCode,
         self.operatingRadius,
         self.verticalVelocity,
         self.targetHoldTime,
         self.FC_flags2,
         self.altitudeSetpoint,
         self.gas,
         self.current,
         self.capacity) = struct.unpack(format_string, 
                                "".join(map(chr, self.data[:82])) )

    def __str__(self):
        return "<" + self.__class__.__name__ + ">" + \
                "\n\tLatitude: " + str(float(self.currentPositionLatitude) / 10**7) + \
                "\n\tLongitude: " + str(float(self.currentPositionLongitude) / 10**7)



########################################################################
##    Display Line messages
########################################################################


class DisplayLineRequestMsg(MkMsg):
    """Request Display Line. Submit index of menuItem"""
    command = 'h'
    address = 0 # any

    def __init__(self, address=None, cmd=None, data=None, menuItem = 0):
        if (address != None and cmd != None and data != None):
            super(DisplayLineRequestMsg, self).__init__(address=address, cmd=cmd, data=data)
        else:
            data = [int(menuItem)]
            super(DisplayLineRequestMsg, self).__init__(address=DisplayLineRequestMsg.address, 
                                                        cmd=DisplayLineRequestMsg.command, 
                                                        data=data)

    def __str__(self):
        return "<" + self.__class__.__name__ + ">" + \
                "\n\tRemote Key:" + str(self.data[0]) + \
                "\n\tAutoSendInterval: " + str(self.data[1]) 


class DisplayLineMsg(MkMsg):
    """docstring for DisplayLineMsg"""
    command = 'H'

    def __init__(self, address=None, cmd=None, data=None):
        if (cmd != DisplayLineMsg.command):
            raise InvalidMsgType
        super(DisplayLineMsg, self).__init__(address=address, cmd=cmd, data=data)

    def __str__(self):
        return "<" + self.__class__.__name__ + ">" + \
                "\n\tMenu item: " + str(self.data[0]) + \
                "\n\tMax menu item: " + str(self.data[1]) + \
                "\n\tDisplay text: " + "".join([ chr(c) for c in self.data[2:] ])



########################################################################
##    3D Data Interval message commands
########################################################################


class ThreeDDataRequestMsg(MkMsg):
    """3D Data Request Message. autoSendInterval in seconds"""
    command = 'c'
    address = 0 # any

    def __init__(self, address=None, cmd=None, data=None, autoSendInterval = 0):
        if (address != None and cmd != None and data != None):
            super(ThreeDDataRequestMsg, self).__init__(address=address, cmd=cmd, data=data)
        else:
            # multiplied by 10 in receiver and then used as milliseconds. 
            data = [int(autoSendInterval*100)]
            super(ThreeDDataRequestMsg, self).__init__(address=ThreeDDataRequestMsg.address, 
                                                   cmd=ThreeDDataRequestMsg.command, 
                                                   data=data)


class ThreeDDataMsg(MkMsg):
    command = 'C'
    def __init__(self, address=None, cmd=None, data=None):
        if (cmd != ThreeDDataMsg.command):
            raise InvalidMsgType
        super(ThreeDDataMsg, self).__init__(address=address, cmd=cmd, data=data)


    # TOFIX : not sure if this is working with u8s properly 

    # Data3D_t -- See uart1.h in NaviCtrl code
    IDX_ANGLENICK=   0 #s16
    IDX_ANGLEROLL=   2 #s16
    IDX_HEADING  =   4 #s16
    IDX_STICKNICK  = 5 #u8
    IDX_STICKROLL  = 6 #u8
    IDX_STICKYAW  =  7 #u8
    IDX_STICKGAS  =  8 #u8
    IDX_RESERVE  =   9  # 4 wide

    # in 0.1 deg
    def getAngleNick(self):
        return self.data2SignedInt(ThreeDDataMsg.IDX_ANGLENICK)

    # in 0.1 deg
    def getAngleRoll(self):
        return self.data2SignedInt(ThreeDDataMsg.IDX_ANGLEROLL)

    # in 0.1 deg
    def getHeading(self):
        return self.data2SignedInt(ThreeDDataMsg.IDX_HEADING)

    def getStickNick(self):
        return self.data[ThreeDDataMsg.IDX_STICKNICK]

    def getStickRoll(self):
        return self.data[ThreeDDataMsg.IDX_STICKROLL]

    def getStickYaw(self):
        return self.data[ThreeDDataMsg.IDX_STICKYAW]

    def getStickGas(self):
        return self.data[ThreeDDataMsg.IDX_STICKGAS]

    def __str__(self):
        return "<" + self.__class__.__name__ + ">" + \
                "\n\taddr: " + str(self.address) + ", " + \
                "\n\tcmd: " + self.cmd + \
                "\n\tAngle Nick: " + str(self.getAngleNick()) + \
                "\n\tAngle Roll: " + str(self.getAngleRoll()) + \
                "\n\tHeading: " + str(self.getHeading()) + \
                "\n\tStick Nick: " + str(self.getStickNick()) + \
                "\n\tStick Roll: " + str(self.getStickRoll()) + \
                "\n\tStick Yaw: " + str(self.getStickYaw()) + \
                "\n\tStick Gas: " + str(self.getStickGas())


########################################################################
##    Compass heading message commands
########################################################################


class AnalogLabelRequestMsg(MkMsg):
    """Request the label of the given labelIndex for analog values"""
    command = 'a'
    address = 0 # any

    def __init__(self, address=None, cmd=None, data=None, labelIndex = None):
        if (address != None and cmd != None and data != None):
            super(AnalogLabelRequestMsg, self).__init__(address=address, cmd=cmd, data=data)
        elif (labelIndex == None):
            raise InvalidMsgType
        else:
            data = [labelIndex]
            super(AnalogLabelRequestMsg, self).__init__(address=AnalogLabelRequestMsg.address, 
                                                   cmd=AnalogLabelRequestMsg.command, 
                                                   data=data)

    def __str__(self):
        return "<" + self.__class__.__name__ + ">" + \
                "\n\tLabel index: " + str(self.data[0])


class AnalogLabelMsg(MkMsg):
    command = 'A'
    def __init__(self, address=None, cmd=None, data=None):
        if (cmd != AnalogLabelMsg.command):
            raise InvalidMsgType
        super(AnalogLabelMsg, self).__init__(address=address, cmd=cmd, data=data)

    def __str__(self):
        return "<" + self.__class__.__name__ + ">" + \
                "\n\tLabel index: " + str(self.data[0]) + \
                "\n\tLabel Text: " + "".join([ chr(c) for c in self.data[1:] ])






########################################################################
##    Motor control message commands
########################################################################


class BLCtrlRequestMsg(MkMsg):
    """autoSendInterval in seconds"""
    command = 'k'
    address = 0 # any

    def __init__(self, address=None, cmd=None, data=None, autoSendInterval = 0):
        if (address != None and cmd != None and data != None):
            super(BLCtrlRequestMsg, self).__init__(address=address, cmd=cmd, data=data)
        else:
            # multiplied by 10 in receiver and then used as milliseconds. 
            data = [int(autoSendInterval*100)]
            super(BLCtrlRequestMsg, self).__init__(address=BLCtrlRequestMsg.address, 
                                                   cmd=BLCtrlRequestMsg.command, 
                                                   data=data)
    def __str__(self):
        return "<" + self.__class__.__name__ + ">" + \
                "\n\tInterval: " + str(self.data[0] * 10) + "ms"




class BLCtrlMsg(MkMsg):
    command = 'K'
    def __init__(self, address=None, cmd=None, data=None):
        if (cmd != BLCtrlMsg.command):
            raise InvalidMsgType
        super(BLCtrlMsg, self).__init__(address=address, cmd=cmd, data=data)


    def __str__(self):
        return "<" + self.__class__.__name__ + ">" + \
                "\n\tIndex: " + str(self.data[0]) + \
                "\n\tCurrent: " + str(self.data[1]) + \
                "\n\tTemperature: " + str(self.data[2]) + " C" \
                "\n\tMax PWM: " + str(self.data[3]) + \
                "\n\tStatus: " + str(self.data[4])



