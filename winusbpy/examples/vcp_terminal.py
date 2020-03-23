#!/usr/bin/env python
"""
VIRTUAL COM PORT TERMINAL
- implements a read/write terminal for communicating
with pyusb devices

SERIAL STATE notifications (2 bytes, interrupt endpoint)
    15..7 - reserved
    6   bOverRun    Received data has been discarded due to a device overrun
    5   bParity     A parity error has occurred
    4   bFraming    A framing error has occurred
    3   bRingSignal State of the ring indicator (RI)
    2   bBreak      Break state
    1   bTxCarrier  State of the data set ready (DSR)
    0   bRxCarrier  State of carrier detect (CD)

Line Coding Data Field (7 bytes, control endpoint)
    offset field       (bytes) Description
    --------------------------------------------------------------------
    0      dwDTERate   4       bit rate (bits per second)
    4      bCharFormat 1       stop bits (0 : 1bit, 1, 1.5bits, 2, 2bits)
    5      bParityType 1       0:None, 1:Odd, 2:Even, 3:Mark, 4:Space
    6      bDataBits   1       5, 6, 7, 8, 16

Control Line State Field (2 bytes, control endpoint)
    wValueBit  Description     (2 bytes data)
    ---------------------------
    bit 1 = 0  RTS : de-assert (negative voltage)
    bit 1 = 1  RTS : assert    (positive voltage)
    bit 0 = 0  DTR : de-assert (negative voltage)
    bit 0 = 1  DTR : assert    (positive voltage)

This tool can be 'compiled' into a windows binary using the following commands
    >>> from distutils.core import setup
    >>> import py2exe
    >>> setup(console=['vcp_terminal.py'])
"""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from __future__ import unicode_literals

import os
import sys
import time
# import usb.core as usb
import threading
import logging
import argparse

from winusbpy import *

try:
    import Queue as queue
except ImportError:
    import queue

CDC_CMDS = {
    "SEND_ENCAPSULATED_COMMAND": 0x00,
    "GET_ENCAPSULATED_RESPONSE": 0x01,
    "SET_COMM_FEATURE": 0x02,
    "GET_COMM_FEATURE": 0x03,
    "CLEAR_COMM_FEATURE": 0x04,
    "SET_LINE_CODING": 0x20,
    "GET_LINE_CODING": 0x21,
    "SET_CONTROL_LINE_STATE": 0x22,
    "SEND_BREAK": 0x23,   # wValue is break time
}


class _Getch(object):
    """ Gets a single character from standard input.
            Does not echo to the screen.
    """

    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self):
        return self.impl()


class _GetchUnix(object):
    def __call__(self):
        import sys
        import tty
        import termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows(object):
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


class ComPort(object):

    def __init__(self, usb_device: WinUsbPy, start=True):
        api = usb_device
        self.device = usb_device
        self._rxinterval = 0.005            # sec
        self._rxqueue = queue.Queue()
        self._rxthread = None
        self._rxactive = False
        self.baudrate = 9600
        self.parity = 0
        self.stopbits = 1
        self.databits = 8
        self.maximum_packet_size = 0

        # Control interface
        interface_descriptor = api.query_interface_settings(0)
        # print("bLength: " + str(interface_descriptor.b_length))
        # print("bDescriptorType: " + str(interface_descriptor.b_descriptor_type))
        # print("bInterfaceNumber: " + str(interface_descriptor.b_interface_number))
        # print("bAlternateSetting: " + str(interface_descriptor.b_alternate_setting))
        # print("bNumEndpoints " + str(interface_descriptor.b_num_endpoints))
        # print("bInterfaceClass " + str(interface_descriptor.b_interface_class))
        # print("bInterfaceSubClass: " + str(interface_descriptor.b_interface_sub_class))
        # print("bInterfaceProtocol: " + str(interface_descriptor.b_interface_protocol))
        # print("iInterface: " + str(interface_descriptor.i_interface))

        pipe_info_list = map(api.query_pipe, range(interface_descriptor.b_num_endpoints))
        # for item in pipe_info_list:
        #     print("PipeType: " + str(item.pipe_type))
        #     print("PipeId: " + str(item.pipe_id))
        #     print("MaximumPacketSize: " + str(item.maximum_packet_size))
        #     print("Interval: " + str(item.interval))

        # Data Interface
        api.change_interface(0)
        interface2_descriptor = api.query_interface_settings(0)
        # print("bLength: " + str(interface2_descriptor.b_length))
        # print("bDescriptorType: " + str(interface2_descriptor.b_descriptor_type))
        # print("bInterfaceNumber: " + str(interface2_descriptor.b_interface_number))
        # print("bAlternateSetting: " + str(interface2_descriptor.b_alternate_setting))
        # print("bNumEndpoints " + str(interface2_descriptor.b_num_endpoints))
        # print("bInterfaceClass " + str(interface2_descriptor.b_interface_class))
        # print("bInterfaceSubClass: " + str(interface2_descriptor.b_interface_sub_class))
        # print("bInterfaceProtocol: " + str(interface2_descriptor.b_interface_protocol))
        # print("iInterface: " + str(interface2_descriptor.i_interface))

        pipe_info_list = map(api.query_pipe, range(interface2_descriptor.b_num_endpoints))
        for item in pipe_info_list:
            # print("PipeType: " + str(item.pipe_type))
            # print("PipeId: " + str(item.pipe_id))
            # print("MaximumPacketSize: " + str(item.maximum_packet_size))
            # print("Interval: " + str(item.interval))
            if item.pipe_id & 0x80:
                self._ep_in = item.pipe_id
            else:
                self._ep_out = item.pipe_id
            self.maximum_packet_size = min(item.maximum_packet_size, self.maximum_packet_size) or item.maximum_packet_size

        # self._ep_in = usb.util.find_descriptor(
        #     data_itf, custom_match=lambda e: (
        #         e.bEndpointAddress & 0x80))
        # self._ep_out = usb.util.find_descriptor(
        #     data_itf, custom_match=lambda e: not (
        #         e.bEndpointAddress & 0x80))


        # data_itfs = list(
        #     usb.util.find_descriptor(
        #         cfg,
        #         find_all=True,
        #         custom_match=lambda e: (
        #             e.bInterfaceClass == 0xA)))
        # if not data_itfs:
        #     print("Unable to connect.  No data interfaces on device")
        #     exit()
        #
        # data_itf = data_itfs[0]
        # cmd_itfs = list(
        #     usb.util.find_descriptor(
        #         cfg,
        #         find_all=True,
        #         custom_match=lambda e: (
        #             e.bInterfaceClass == 0x2)))
        # itf_num = cmd_itfs[0].bInterfaceNumber
        # if len(cmd_itfs) != len(data_itfs):
        #     log.debug("COM port data / command interface mismatch")
        #
        # ports = len(data_itfs)
        # log.debug("found {0} COM port\n".format(ports))

        # try:
        #     self.device.detach_kernel_driver(itf_num)
        # except usb.USBError:
        #     pass
        # except NotImplementedError:
        #     pass

        # self._ep_in = usb.util.find_descriptor(
        #     data_itf, custom_match=lambda e: (
        #         e.bEndpointAddress & 0x80))
        # self._ep_out = usb.util.find_descriptor(
        #     data_itf, custom_match=lambda e: not (
        #         e.bEndpointAddress & 0x80))

        if start:
            self._startRx()

    def _startRx(self):
        if self._rxthread is not None and (
                self._rxactive or self._rxthread.isAlive()):
            return
        self._rxactive = True
        self._rxthread = threading.Thread(target=self._read)
        self._rxthread.daemon = True
        self._rxthread.start()

    def _endRx(self):
        self._rxactive = False

    def _read(self):
        """ check ep for data, add it to queue and sleep for interval """
        while self._rxactive:
            try:
                # rv = self._ep_in.read(self._ep_in.wMaxPacketSize)
                rv = self.device.read(self._ep_in, self.maximum_packet_size)
                for rvi in rv:
                    self._rxqueue.put(rvi)
            except Exception as e:  # usb.USBError as e:
                log.warn("USB Error on _read {}".format(e))
                return
            
            time.sleep(self._rxinterval)

    def _getRxLen(self):
        return self._rxqueue.qsize()
    rxlen = property(fget=_getRxLen)

    def readBytes(self):
        rx = []
        while not self._rxqueue.empty():
            rx.append(self._rxqueue.get())
        return rx

    def readText(self):
        return "".join(chr(c) for c in self.readBytes())

    def write(self, data):
        try:
            ret = self.device.write(self._ep_out, data)
        except Exception as e:
            log.warn("USB Error on write {}".format(e))
            return

        if len(data) != ret:
            log.error(
                "Bytes written mismatch {0} vs {1}".format(
                    len(data), ret))
        else:
            log.debug("{} bytes written to ep".format(ret))

    def setControlLineState(self, RTS=None, DTR=None):
        ctrlstate = (2 if RTS else 0) + (1 if DTR else 0)

        txdir = 0           # 0:OUT, 1:IN
        req_type = 1     # 0:std, 1:class, 2:vendor
        # 0:device, 1:interface, 2:endpoint, 3:other
        recipient = 1
        req_type = (txdir << 7) + (req_type << 5) + recipient

        pkt = UsbSetupPacket(
            request_type=req_type,
            request=CDC_CMDS["SET_CONTROL_LINE_STATE"],
            value=ctrlstate,
            index=0x00,
            length=0x00
        )
        # buff = [0xc0, 0x12, 0x00, 0x00, 0x00, 0x00, 0x08]
        buff = None

        wlen = self.device.control_transfer(pkt, buff)
        log.debug("Linecoding set, {}b sent".format(wlen))

    def setLineCoding(self, baudrate=None, parity=None,
                      databits=None, stopbits=None):
        sbits = {1: 0, 1.5: 1, 2: 2}
        dbits = {5, 6, 7, 8, 16}
        pmodes = {0, 1, 2, 3, 4}
        brates = {300, 600, 1200, 2400, 4800, 9600, 14400,
                  19200, 28800, 38400, 57600, 115200, 230400}

        if stopbits is not None:
            if stopbits not in sbits.keys():
                valid = ", ".join(str(k) for k in sorted(sbits.keys()))
                raise ValueError("Valid stopbits are " + valid)
            self.stopbits = stopbits

        if databits is not None:
            if databits not in dbits:
                valid = ", ".join(str(d) for d in sorted(dbits))
                raise ValueError("Valid databits are " + valid)
            self.databits = databits

        if parity is not None:
            if parity not in pmodes:
                valid = ", ".join(str(pm) for pm in sorted(pmodes))
                raise ValueError("Valid parity modes are " + valid)
            self.parity = parity

        if baudrate is not None:
            if baudrate not in brates.keys():
                brs = sorted(brates.keys())
                dif = [abs(br - baudrate) for br in brs]
                best = brs[dif.index(min(dif))]
                raise ValueError(
                    "Invalid baudrates, nearest valid is {}".format(best))
            self.baudrate = baudrate

        linecode = [
            self.baudrate & 0xff,
            (self.baudrate >> 8) & 0xff,
            (self.baudrate >> 16) & 0xff,
            (self.baudrate >> 24) & 0xff,
            sbits[self.stopbits],
            self.parity,
            self.databits]

        txdir = 0           # 0:OUT, 1:IN
        req_type = 1        # 0:std, 1:class, 2:vendor
        recipient = 1       # 0:device, 1:interface, 2:endpoint, 3:other
        req_type = (txdir << 7) + (req_type << 5) + recipient

        pkt = UsbSetupPacket(
            request_type=req_type,
            request=CDC_CMDS["SET_LINE_CODING"],
            value=0x0000,
            index=0x00,
            length=len(linecode)
        )
        # buff = [0xc0, 0x12, 0x00, 0x00, 0x00, 0x00, 0x08]
        buff = linecode


        wlen = self.device.control_transfer(pkt, buff)
            # req_type, CDC_CMDS["SET_LINE_CODING"],
            # data_or_wLength=linecode)
        log.debug("Linecoding set, {}b sent".format(wlen))

    def getLineCoding(self):
        txdir = 1           # 0:OUT, 1:IN
        req_type = 1        # 0:std, 1:class, 2:vendor
        recipient = 1       # 0:device, 1:interface, 2:endpoint, 3:other
        req_type = (txdir << 7) + (req_type << 5) + recipient

        buf = self.device.ctrl_transfer(bmRequestType=req_type,
                                        bRequest=CDC_CMDS["GET_LINE_CODING"],
                                        wValue=0,
                                        wIndex=0,
                                        data_or_wLength=255,
                                        )
        self.baudrate = buf[0] + (buf[1] << 8) + \
            (buf[2] << 16) + (buf[3] << 24)
        self.stopbits = 1 + (buf[4] / 2.0)
        self.parity = buf[5]
        self.databits = buf[6]
        print("LINE CODING:")
        print("  {0} baud, parity mode {1}".format(self.baudrate, self.parity))
        print(
            "  {0} data bits, {1} stop bits".format(
                self.databits,
                self.stopbits))

    def disconnect(self):
        self._endRx()
        while self._rxthread is not None and self._rxthread.isAlive():
            pass
        usb.util.dispose_resources(self.device)
        if self._rxthread is None:
            log.debug("Rx thread never existed")
        else:
            log.debug("Rx thread is {}".format(
                "alive" if self._rxthread.isAlive() else "dead"))
        attempt = 1
        while attempt < 10:
            try:
                self.device.attach_kernel_driver(0)
                log.debug(
                    "Attach kernal driver on attempt {0}".format(attempt))
                break
            except usb.USBError:
                attempt += 1
                time.sleep(0.1)     # sleep seconds
        if attempt == 10:
            log.error("Could not attach kernal driver")


def configLog():
    log = logging.getLogger("vcp_terminal")
    log.setLevel(logging.DEBUG)
    if "PYTERMINAL_DEBUG" in os.environ:
        fileHandler = logging.FileHandler("terminal.log")
        log_fmt = logging.Formatter(
            "%(levelname)s %(name)s %(threadName)-10s " +
            "%(funcName)s() %(message)s")
        fileHandler.setFormatter(log_fmt)
        log.addHandler(fileHandler)
    return log


def selectDevice(vid, pid):
    api = WinUsbPy()
    devices = api.list_usb_devices(deviceinterface=True, present=True)

    if not devices:
        print("No devices detected")
        return None
    #
    # selection = -1
    # selected = False
    #
    # print("PyUSB VCP Terminal: use ctrl+c or ctrl+d to exit")
    # while not selected:
    #     for i, d in enumerate(devices):
    #         try:
    #             manufacturer = d.manufacturer + " "
    #         except:
    #             manufacturer = "Unknown"
    #         try:
    #             prod = d.product + " "
    #         except:
    #             prod = ""
    #         try:
    #             serial = d.serial_number + " "
    #         except:
    #             serial = ""
    #
    #         print(
    #             "%d: %04x:%04x on Bus %03d %03d %s%s%s" %
    #             (i,
    #              d.idVendor,
    #              d.idProduct,
    #              d.bus,
    #              d.address,
    #              manufacturer,
    #              prod,
    #              serial))
    #
    #     selection = input("Enter device: ")
    #
    #     try:
    #         selection = int(selection)
    #         if selection < 0 or selection >= len(devices):
    #             raise Exception()
    #         selected = True
    #     except:
    #         print(
    #             "Please enter number between 0 and {}".format(
    #                 len(devices) - 1))
    #
    # d = devices[selection]
    api.init_winusb_device(vid, pid)

    return api


def configInputQueue():
    """ configure a queue for accepting characters and return the queue
    """
    def captureInput(iqueue):
        while True:
            c = getch()
            if c == '\x03' or c == '\x04':    # end on ctrl+c / ctrl+d
                log.debug("Break received (\\x{0:02X})".format(ord(c)))
                iqueue.put(c)
                break
            log.debug(
                "Input Char '{}' received".format(
                    c if c != '\r' else '\\r'))
            iqueue.put(c)

    input_queue = queue.Queue()
    input_thread = threading.Thread(target=lambda: captureInput(input_queue))
    input_thread.daemon = True
    input_thread.start()
    return input_queue, input_thread


PRINTABLE_CHAR = set(list(range(ord(' '), ord('~') + 1)) + [ord('\r'), ord('\n')])


def fmt_text(text):
    """ convert characters that aren't printable to hex format
    """
    newtext = ("\\x%02x" % c if c not in PRINTABLE_CHAR else chr(c) for c in text)
    textlines = "\r\n".join(l.strip('\r') for l in "".join(newtext).split('\n'))
    return textlines


def runTerminal(d: WinUsbPy):
    log.info("Beginning a terminal run")
    p = ComPort(d)
    p.setControlLineState(True, True)
    p.setLineCoding()
    
    q, t = configInputQueue()

    while True:
        if p.rxlen:
            print(fmt_text(p.readBytes()), end="")

        if not q.empty():
            c = q.get()
            if c == '\x03' or c == '\x04':    # end on ctrl+c / ctrl+d
                print()
                p.disconnect()
                break
            try:
                p.write(c)
            except Exception as e:
                log.warn("USB Error on write {}".format(e))
                return

def ftdi_to_clkbits(baudrate):  # from libftdi
    """
    10,27 => divisor = 10000, rate = 300
    88,13 => divisor = 5000, rate = 600
    C4,09 => divisor = 2500, rate = 1200
    E2,04 => divisor = 1250, rate = 2,400
    71,02 => divisor = 625, rate = 4,800
    38,41 => divisor = 312.5, rate = 9,600
    D0,80 => divisor = 208.25, rate = 14406
    9C,80 => divisor = 156, rate = 19,230
    4E,C0 => divisor = 78, rate = 38,461
    34,00 => divisor = 52, rate = 57,692
    1A,00 => divisor = 26, rate = 115,384
    0D,00 => divisor = 13, rate = 230,769
    """
    clk = 48000000
    clk_div = 16
    frac_code = [0, 3, 2, 4, 1, 5, 6, 7]
    actual_baud = 0
    if baudrate >= clk / clk_div:
        encoded_divisor = 0
        actual_baud = (clk // clk_div)
    elif baudrate >= clk / (clk_div + clk_div / 2):
        encoded_divisor = 1
        actual_baud = clk // (clk_div + clk_div // 2)
    elif baudrate >= clk / (2 * clk_div):
        encoded_divisor = 2
        actual_baud = clk // (2 * clk_div)
    else:
        # We divide by 16 to have 3 fractional bits and one bit for rounding
        divisor = clk * 16 // clk_div // baudrate
        best_divisor = (divisor + 1) // 2
        if best_divisor > 0x20000:
            best_divisor = 0x1ffff
        actual_baud = clk * 16 // clk_div // best_divisor
        actual_baud = (actual_baud + 1) // 2
        encoded_divisor = ((best_divisor >> 3) +
                           (frac_code[best_divisor & 0x7] << 14))

    value = encoded_divisor & 0xFFFF
    index = encoded_divisor >> 16
    return actual_baud, value, index


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='WinUSB Com Port')
    parser.add_argument('--vid', help='USB Vendor ID')
    parser.add_argument('--pid', help='USB Product ID')

    args = parser.parse_args()

    log = configLog()

    getch = _Getch()            # init an instance

    d = selectDevice(args.vid, args.pid)
    if d is None:
        exit()

    if sys.argv[-1] == '-d':
        """ use ...pyusb$ ipython3 -i toots/terminal.py -- -d """
        p = ComPort(d, start=False)
        print("Debug mode: p is ComPort, d is pyusb Device")
    else:
        runTerminal(d)

    def show_fti_baud():
        brates = [300, 600, 1200, 2400, 4800, 9600, 14400,
                  19200, 28800, 38400, 57600, 115200, 230400]

        for br in brates:
            br_ftdi, value, index = ftdi_to_clkbits(br)
            print("{:7} | {:7} | {:5.1%} | wVal {:4X}h | wIndex {:4X}h".format(
                br, br_ftdi, (br - br_ftdi) * 1.0 / br, value, index))

