#! /usr/bin/env python


from microkopter import *
from serial.tools import list_ports


serial_usb_port = [port[0] for port in list_ports.grep('usbserial')]
if len(serial_usb_port) == 0:
    print "Couldn't find the serial interface. Is it plugged in?"
    sys.exit()

# TODO : add this later to handle exceptions
comPort = serial_usb_port[0]

comPort = '/dev/tty.usbserial-A40078OI'

comm = MkComm()
comm.printDebugMsg = True
comm.open(comPort = comPort, timeout = 0.5)

print "Sending FC->NC forwarding"
comm.sendNCRedirectUartFromFC()


# Version message request
# msg = VersionRequestMsg()
# comm.sendMsg(msg)
# returned_msg = comm.waitForMsg(VersionMsg.command)
# print returned_msg
# print


msg = ThreeDDataRequestMsg()
comm.sendMsg(msg)
returned_msg = comm.waitForMsg(ThreeDDataMsg.command)
print returned_msg
print




# Read Pulse Position Modulation (PPM) channels
# Returns s16 PPM-Array[11]

# print 'Reading PPM channels'
# msg = MkMsg(address=MkComm.ADDRESS_FC, cmd='p', data=[])
# comm.sendMsg(msg)
# returned_msg = comm.waitForMsg('P')
# print returned_msg



# Trying to get GPS data
# samplePeriod = 0.05
# nbSamples = 20
# result = []
# comm.serPort.flushInput()
# msg = MkMsg(address=MkComm.ADDRESS_FC, cmd='o', data=[int(samplePeriod*100)])
# comm.sendMsg(msg)
# returned_msg = comm.waitForMsg('O')
# print returned_msg



# samplePeriod = 0.05
# msgs = []
# for k in range(50):
#     comm.serPort.flushInput()
#     msg = MkMsg(address=MkComm.ADDRESS_FC, cmd='a', data=[k])
#     comm.sendMsg(msg)
#     returned_msg = comm.waitForMsg('A')
#     msgs.append(returned_msg)

# for msg in msgs:
#     print msg.data[0], ''.join([chr(c) for c in msg.data[1:]])


# print 'Read waypoints'
# comm.serPort.flushInput()
# msg = MkMsg(address=MkComm.ADDRESS_FC, cmd='x', data=[0])
# comm.sendMsg(msg)
# returned_msg = comm.waitForMsg('X')
# print returned_msg





# All on screen options
# msgs = []
# for k in range(32):
#     msg = MkMsg(address=MkComm.ADDRESS_NC, cmd='l', data=[k])
#     comm.sendMsg(msg)
#     returned_msg = comm.waitForMsg('L')
#     msgs.append(returned_msg)
# for msg in msgs:
#     for c in msg.data:
#         sys.stdout.write(chr(c))
#     sys.stdout.write('\n')


# READING A BUNCH OF PARAMS (NC-Parameter)
# msgs = []
# for k in range(256):
#     msg = MkMsg(address=MkComm.ADDRESS_NC, cmd='j', data=[0, k])
#     comm.sendMsg(msg)
#     returned_msg = comm.waitForMsg('J')
#     msgs.append(returned_msg)

# for msg in msgs:
#     print msg





# msg = MkMsg(address=MkComm.ADDRESS_NC, cmd='x', data=[0])
# comm.sendMsg(msg)
# returned_msg = comm.waitForMsg('X')
# print returned_msg



# samplePeriod = 0.05
# comm.serPort.flushInput()
# msg = MkMsg(address=MkComm.ADDRESS_NC, cmd='l', data=[16])
# comm.sendMsg(msg)
# returned_msg = comm.waitForMsg('L')

# print ''.join([chr(c) for c in returned_msg.data])




# print 'Reading OSD data'
# msg = MkMsg(address=MkComm.ADDRESS_NC, cmd='w', data=[])
# comm.sendMsg(msg)
# returned_msg = comm.waitForMsg('W')
# print returned_msg




# print 'Collecting 1 second of samples'
# samplePeriod = 0.05
# nbSamples = 20
# result = []
# comm.serPort.flushInput()
# msg = MkMsg(address=0, cmd='d', data=[int(samplePeriod*100)])
# comm.sendMsg(msg)

# for i in range(nbSamples):
#     returned_msg = comm.waitForMsg('D')
#     debug_msg = DebugDataMsg(returned_msg)
#     print debug_msg
#     result.append(debug_msg)




# print 'Getting settings messages'
# msg = comm.getSettingsMsg()
# print msg



# print 'Requesting BL stuff'
# ret = []
# k = range(256)
# for i in k:
#     msg = MkMsg(address=MkComm.ADDRESS_FC, cmd='u', data=[i])
#     comm.serPort.flushInput()
#     comm.sendMsg(msg)
#     returned_msg = comm.waitForMsg('U')
#     ret.append( returned_msg )
# for r in ret:
#     print r




# msg.setSetting(SettingsMsg.IDX_STICK_P, msg.getSetting(SettingsMsg.IDX_STICK_P)+1)
# comm.sendSettings(msg.getSettings())

# messages = comm.recordDbgMsg(0.05, 20)
# for msg in messages:
#     print msg.getAngleRoll()



# except Exception,e:
#     print
#     print "An error occured: ", e
#     print
#     traceback.print_exc()
#     print
#     raw_input("Press ENTER, the application will close")
