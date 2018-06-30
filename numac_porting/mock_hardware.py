"""
Module contains mocks

Functionality we want to mock
from stm_uart_port import UART_Port
- read_byte()


from bus import Bus#, BusError
- syncwrite(ids_list, register_index, bytearray_list)


"write-only" hardware we mock with mock.Mock() elsewhere:
- pyb.Pin
- MotorDriver.MotorDriver

"""

from collections import deque
import logging  # prevents different threads' output from mixing
try:
    from Queue import Queue
except ImportError:
    from queue import Queue


class MockUART_Port_from_COM():
    # Example use case: Take COM port on Windows and feed its bytes into what
    # would normally be a xbee UART port

    def __init__(self, serial=None, baud=None):
        self.ser = serial # assumed to be open
        # TODO possibly set a deque size limit and warn if it gets full?
        #self.byte_list = deque()

    def read_byte(self):
        #TODO clarify
        # We get e.g. b'\xff' and taking the index gives us 255 aka 0xff
        raw_byte = self.ser.read()
        byte = raw_byte[0]
        return byte
        ## Pass a byte from our 
        #if self.byte_list:
        #    #retbyte = self.byte_list[0]
        #    #self.byte_list.drop(0)
        #    retbyte = self.byte_list.popleft()
        #    return retbyte
        ## Otherwise load available bytes into byte_list
        #print("Getting cmdr bytesssssssssssssss")
        #while self.ser.inWaiting(): #TODO :
        #    byte = self.ser.read() # these are bytes not strings!
        #    if byte != None and byte != "":
        #        try:
        #            self.byte_list.append(byte)
        #            #self.byte_list.append(int(byte.encode('hex'),16))
        #            print(byte, len(self.byte_list))
        #        except ValueError as e:
        #            logging.debug(( 'error', byte, e,))
        #            raise
        #print("Got bytes from cmdr!:", len(self.byte_list))
        #return 1 # TODO?

    def clear_read_buffer(self):
        self.ser.reset_input_buffer()


class MockBusToQueue():
    def __init__(self):
        self.queue = Queue() # TODO more setup?

    def get_queue(self):
        return self.queue

    #def sync_write(self, ids_list, register_index, bytearray_list):
    def sync_write(self, dev_ids, offset, values):
        # Adapted straight from bus.Bus.sync_write() to support queues instead
        #if self.show & Bus.SHOW_COMMANDS:
        #    ids = ', '.join(['{}'.format(id) for id in dev_ids])
        #    log('Sending SYNC_WRITE to IDs {} offset 0x{:02x} len {}'.format(ids, offset, len(values[0])))
        num_ids = len(dev_ids)
        if num_ids != len(values):
            # TODO let's not trigger this, k?
            raise ValueError('len(dev_ids) = {} must match len(values) = {}'.format(num_ids, len(values)))
        bytes_per_id = len(values[0])
        param_len = num_ids * (bytes_per_id + 1) + 2
        data = [0]*param_len #bytearray(param_len)
        packet = [0] * (param_len + 6) #bytearray(param_len)
        data[0] = offset
        data[1] = bytes_per_id
        data_idx = 2
        for id_idx in range(num_ids):
            if len(values[id_idx]) != bytes_per_id:
                raise ValueError('len(values[{}]) not equal {}'.format(id_idx, bytes_per_id))
            data[data_idx] = dev_ids[id_idx]
            data_idx += 1
            data[data_idx:data_idx + bytes_per_id] = values[id_idx]
            data_idx += bytes_per_id

        #self.fill_and_write_packet(packet.Id.BROADCAST, packet.Command.SYNC_WRITE, data)
        packet[0] = 255
        packet[1] = 255
        packet[2] = 0xfe # ID 255
        packet[3] = len(data) + 4 # Length = bytes + id + len + inst + checksum
        packet[4] = 0x83 # 0x83 == sync write
        packet[5:-2] = data
        packet[-1] = ~sum(packet[2:-1]) & 0xff
        for bt in packet:
            # Could use block=False if we were to run into queue limit
            self.queue.put(bt)


class MockCS_ADC():
    def __init__(self, *args):
        pass
    def read(self):
        return 1

class MockMotorDriver():
    def __init__(self, *args, **kwargs):
        self.cs_adc = MockCS_ADC()
        pass

    def direct_set_speed(self, *args):
        pass

    def run_reversed(self, *args):
        pass

## based on needs in numa.py
#class MockPin():
#    def __init__(self, *args):
#        self.board = MockBoard()
#
#class MockBoard():
#    def __init__(self):
#        pass
