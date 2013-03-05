#!/usr/bin/python

# Adapted by Michael Grinich to grab messages from MicroKopter
# serial easedropper.  

# Jim Paris <jim@jtan.com>
# http://jdonnal.scripts.mit.edu/home/?p=135

# Simple terminal program for serial devices.  Supports setting
# baudrates and simple LF->CRLF mapping on input, but does not
# configure flow control, carrier detection, etc.

# ^C quits.  There is no escaping, so you can't currently send this
# character to the remote host.  Piping input or output should work.

# Supports multiple serial devices simultaneously.  When using more
# than one, each device's output is in a different color.  Input
# is directed to the first device, or can be sent to all devices
# with --all.

import sys
import os
import serial
import threading
import traceback
import time
import signal

from microkopter import *


# Need OS-specific method for getting keyboard input.
if os.name == 'nt':
    import msvcrt
    class Console:
        def __init__(self):
            pass
        def cleanup(self):
            pass
        def getkey(self):
            while 1:
                z = msvcrt.getch()
                if z == '\0' or z == '\xe0': # function keys
                    msvcrt.getch()
                else:
                    if z == '\r':
                        return '\n'
                    return z
elif os.name == 'posix':
    import termios, select
    class Console:
        def __init__(self):
            self.fd = sys.stdin.fileno()
            try:
                self.old = termios.tcgetattr(self.fd)
                tc = termios.tcgetattr(self.fd)
                tc[3] = tc[3] & ~termios.ICANON & ~termios.ECHO & ~termios.ISIG
                tc[6][termios.VMIN] = 1
                tc[6][termios.VTIME] = 0
                termios.tcsetattr(self.fd, termios.TCSANOW, tc)
            except termios.error:
                # ignore errors, so we can pipe stuff to this script
                pass
        def cleanup(self):
            try:
                termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old)
            except:
                # ignore errors, so we can pipe stuff to this script
                pass
        def getkey(self):
            # Return -1 if we don't get input in 0.1 seconds, so that
            # the main code can check the "alive" flag and respond to SIGINT.
            [r, w, x] = select.select([self.fd], [], [self.fd], 0.1)
            if r:
                return os.read(self.fd, 1)
            elif x:
                return ''
            else:
                return -1
else:
    raise ("Sorry, no terminal implementation for your platform (%s) "
           "available." % sys.platform)

class JimtermColor(object):
    def __init__(self):
        self.setup(1)
    def setup(self, total):
        if total > 1:
            self.codes = [
                "\x1b[1;36m", # cyan
                "\x1b[1;33m", # yellow
                "\x1b[1;35m", # magenta
                "\x1b[1;31m", # red
                "\x1b[1;32m", # green
                "\x1b[1;34m", # blue
                "\x1b[1;37m", # white
                ]
            self.reset = "\x1b[0m"
        else:
            self.codes = [""]
            self.reset = ""
    def code(self, n):
        return self.codes[n % len(self.codes)]

class Jimterm:
    """Normal interactive terminal"""

    def __init__(self,
                 serials,
                 suppress_write_bytes = None,
                 suppress_read_firstnull = True,
                 transmit_all = False,
                 add_cr = False,
                 raw = False,
                 color = True):

        self.color = JimtermColor()
        if color:
            self.color.setup(len(serials))

        self.serials = serials
        self.suppress_write_bytes = suppress_write_bytes or ""
        self.suppress_read_firstnull = suppress_read_firstnull
        self.last_color = ""
        self.threads = []
        self.transmit_all = transmit_all
        self.add_cr = add_cr
        self.raw = raw

    def print_header(self, nodes, bauds):
        for (n, (node, baud)) in enumerate(zip(nodes, bauds)):
            print (self.color.code(n)
                   + node + ", " + str(baud) + " baud"
                   + self.color.reset)
        print "^C to exit"
        print "----------"

    def start(self):
        self.alive = True

        # serial->console, all devices
        for (n, serial) in enumerate(self.serials):
            self.threads.append(threading.Thread(
                target = self.reader,
                args = (serial, self.color.code(n))
                ))

        # console->serial
        self.console = Console()
        self.threads.append(threading.Thread(target = self.writer))


        # start all threads
        for thread in self.threads:
            thread.daemon = True
            thread.start()

    def stop(self):
        self.alive = False

    def join(self):
        for thread in self.threads:
            while thread.isAlive():
                thread.join(0.1)

    def reader(self, serial, color):
        """loop and copy serial->console"""
        first = True
        try:

            line = []

            while self.alive:
                data = serial.read(1)
                if not data:
                    continue

                # don't print a NULL if it's the first character we
                # read.  This hides startup/port-opening glitches with
                # some serial devices.
                if self.suppress_read_firstnull and first and data == '\0':
                    first = False
                    continue
                first = False

                if color != self.last_color:
                    self.last_color = color
                    sys.stdout.write(color)

                line.append(data)
                if data == '\r':
                    # if len(data) > 1: 

                    if (line != ['\r'] ):
                        try:
                            msg = MkMsg.createFromData(line)
                            # if ((msg.command == 'o') or (msg.command == 'O')):
                            print time.strftime('%X') + str(msg)
                        except Exception, e:

                            print e, line
                    line = []
                    # sys.stdout.write("".join(line))
                    # sys.stdout.write("\n")


                # if (self.raw or
                #     (ord(data) >= 32 and ord(data) < 128) or
                #     data == '\r' or data == '\n' or data == '\t'):
                #     if self.add_cr and data == '\n':
                #         sys.stdout.write('\r' + data)
                #     else:
                #         sys.stdout.write(data)
                # else:
                #     sys.stdout.write('\\x'+("0"+hex(ord(data))[2:])[-2:])

                sys.stdout.flush()

        except Exception as e:
            sys.stdout.write(color)
            sys.stdout.flush()
            traceback.print_exc()
            sys.stdout.write(self.color.reset)
            sys.stdout.flush()
            self.console.cleanup()
            os._exit(1)

    def writer(self):
        """loop and copy console->serial until ^C"""
        try:
            while self.alive:
                try:
                    c = self.console.getkey()
                except KeyboardInterrupt:
                    c = '\x03'
                if c == '\x03':
                    self.stop()
                    return
                elif c == -1:
                    # No input, try again
                    continue
                elif c == '':
                    # EOF on input.  Wait a tiny bit so we can
                    # flush the remaining input, then stop.
                    time.sleep(0.25)
                    self.stop()
                    return
                elif c in self.suppress_write_bytes:
                    # Don't send these bytes
                    continue
                else:
                    # send character
                    if self.transmit_all:
                        for serial in self.serials:
                            serial.write(c)
                    else:
                        self.serials[0].write(c)
        except Exception as e:
            sys.stdout.write(self.color.reset)
            sys.stdout.flush()
            traceback.print_exc()
            self.console.cleanup()
            os._exit(1)

    def run(self):
        # Set all serial port timeouts to 0.1 sec
        saved_timeouts = []
        for (n, serial) in enumerate(self.serials):
            saved_timeouts.append(serial.timeout)
            serial.timeout = 0.1

        # Handle SIGINT gracefully
        signal.signal(signal.SIGINT, lambda *args: self.stop())

        # Go
        self.start()
        self.join()

        # Restore serial port timeouts
        for (n, serial) in enumerate(self.serials):
            serial.timeout = saved_timeouts[n]

        # Cleanup
        print self.color.reset # and a newline
        self.console.cleanup()

if __name__ == "__main__":
    import argparse
    import re

    formatter = argparse.ArgumentDefaultsHelpFormatter
    description = ("Simple serial terminal that supports multiple devices.  "
                   "If more than one device is specified, device output is "
                   "shown in varying colors.  All input goes to the "
                   "first device.")
    parser = argparse.ArgumentParser(description = description,
                                     formatter_class = formatter)

    parser.add_argument("device", metavar="DEVICE", nargs="+",
                        help="Serial device.  Specify DEVICE@BAUD for "
                        "per-device baudrates.")

    parser.add_argument("--quiet", "-q", action="store_true",
                        help="Less verbose output (omit header)")
    parser.add_argument("--baudrate", "-b", metavar="BAUD", type=int,
                        help="Default baudrate for all devices", default=115200)
    parser.add_argument("--crlf", "-c", action="store_true",
                        help="Add CR before incoming LF")
    parser.add_argument("--all", "-a", action="store_true",
                        help="Send keystrokes to all devices, not just "
                        "the first one")
    parser.add_argument("--mono", "-m", action="store_true",
                        help="Don't use colors in output")
    parser.add_argument("--raw", "-r", action="store_true",
                        help="Don't escape unprintable characters")

    args = parser.parse_args()

    devs = []
    nodes = []
    bauds = []
    for (n, device) in enumerate(args.device):
        m = re.search(r"^(.*)@([1-9][0-9]*)$", device)
        if m is not None:
            node = m.group(1)
            baud = m.group(2)
        else:
            node = device
            baud = args.baudrate
        if node in nodes:
            sys.stderr.write("error: %s specified more than once\n" % node)
            raise SystemExit(1)
        try:
            dev = serial.Serial(node, baud)
        except serial.serialutil.SerialException:
            sys.stderr.write("error opening %s\n" % node)
            raise SystemExit(1)
        nodes.append(node)
        bauds.append(baud)
        devs.append(dev)

    term = Jimterm(devs,
                   transmit_all = args.all,
                   add_cr = args.crlf,
                   raw = args.raw,
                   color = (os.name == "posix" and not args.mono))
    if not args.quiet:
        term.print_header(nodes, bauds)
    term.run()
