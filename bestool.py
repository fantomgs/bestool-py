#! python3


from enum import Enum
import sys
from typing import List
import serial
import serial.tools.list_ports
from serial.tools import miniterm
import zlib
import click
import time
from datetime import datetime, timedelta
import struct

__author__ = "Ben V. Brown"
BES_BAUD = 921600


class BESMessageTypes(Enum):
    SYNC = 0x50
    CODE_INFO = 0x53
    CODE = 0x54
    RUN = 0x55
    PROGRAMMER_INIT = 0x60
    FLASH_COMMAND = 0x65
    ERASE_BURN_SART = 0x61
    FLASH_BURN_DATA = 0x62


class BESPacket:
    MINIMAL_PACKET_LEN = 5 # minimum packet len is 5 (header, command, sequence, dataLen, checksum)

    magic = 0xBE
    command = 0
    sequence = 0
    data_len = 0
    checksum = 0
    data = []    

    def __init__(self):
        pass
    
    def __init__(self, data):
        self.packet = self.parse_packet(data)
    
    def parse_packet(self, data):
        self.magic = data[0]
        self.command = data[1]
        self.sequence = data[2]
        self.data_len = data[3]
        self.checksum = data[4 + self.data_len]
        self.data = []
        if self.data_len > 0:
            self.data = data[4:4 + self.data_len]
        return self

    
class BESLink:
    """
    Wrapper class for communcations with the BES bootloader thing
    """
    SYNC_MESSAGE    = [0xBE, BESMessageTypes.SYNC.value, 0x00, 0x01, 0x01, 0xEF]
    CODE_MESSAGE    = [0xBE, BESMessageTypes.CODE.value, 0xA2, 0x03, 0x00, 0x00, 0x00, 0x48]
    RUN_MESSAGE     = [0xBE, BESMessageTypes.RUN.value, 0x01, 0x00, 0xEB]

    @classmethod
    def wait_for_sync(cls, serial_port: serial.Serial):
        print(f"Waiting for sync on {serial_port.name}")
        print(f"Send SYNC request")
        sys.stdout.flush()
        serial_port.write(cls.SYNC_MESSAGE)
        exit_time = datetime.now() + timedelta(seconds=30)
        # Sync packet from bootloader is {BE,50,00,03,00,00,01,ED}
        while datetime.now() < exit_time:
            data = cls._read_packet(serial_port)
            packet = BESPacket(data).packet
            if packet.command == BESMessageTypes.SYNC.value:
                sync_code = packet.data[0] # 0 - bootloader just started, 2 - bootloader is already running
                state = "unknown state"
                if sync_code == 0:
                    state = "started"
                if sync_code == 2:
                    state = "running"
                print("Got SYNC reply (code 0x%02x - bootloader is %s)" % (sync_code, state))
                if sync_code == 0:
                    cls.wait_for_sync(serial_port)
                else:
                    if sync_code != 2:
                        raise Exception("Unknown bootloader sync state 0x%02x" % sync_code)                    
                sys.stdout.flush()
                break

    @classmethod
    def load_code_blob(cls, serial_port: serial.Serial):
        """
        Loading in the code blob
        """
        exit_time = datetime.now() + timedelta(seconds=30)
        # code_addr & 0x3 == 0 && code_len > 0 && code_addr >= 0x20001950 && code_len + code_addr < 0x2003F000

        with open("code.bin", "r+b") as f:
            code_payload = f.read()
            f.close()

            # address = 0x20002000
            entry, param, sp, address = struct.unpack("<IIII", code_payload[0:16])
            size = len(code_payload)

            crc = CRC32().compute(code_payload)

            print("Send code %d bytes @0x%08x, crc32 = 0x08%x, entry @ 0x%08x, param 0x%08x, sp @ 0x%08x" % (size, address, crc, entry, param, sp))
            sys.stdout.flush()

            code_info_msg = [
                0xBE,
                BESMessageTypes.CODE_INFO.value,
                0x00,
                0x0C,
                # code address
                (address >> 0) & 0xff,
                (address >> 8) & 0xff,
                (address >> 16) & 0xff,
                (address >> 24) & 0xff,
                # code size
                (size >> 0) & 0xff,
                (size >> 8) & 0xff,
                (size >> 16) & 0xff,
                (size >> 24) & 0xff,
                # code crc32
                (crc >> 0) & 0xff,
                (crc >> 8) & 0xff,
                (crc >> 16) & 0xff,
                (crc >> 24) & 0xff,
                # cksum
                0x00,
            ]
            code_info_msg[-1] = cls._calculate_message_checksum(code_info_msg[0:-1])
            # Send code info message
            print("Send CODE_INFO message")
            serial_port.write(code_info_msg)
            # wait for response
            while datetime.now() < exit_time:
                packet = cls._read_packet(serial_port)
                if packet[1] == BESMessageTypes.CODE_INFO.value:
                    print("Resp OK to start code upload")
                    sys.stdout.flush()
                    break
            print("Send CODE message")
            serial_port.write(cls.CODE_MESSAGE)
            serial_port.write(code_payload)
            # wait for response
            while datetime.now() < exit_time:
                packet = cls._read_packet(serial_port)
                #TODO: catch error: in case of incorrect CODE CRC we get resync message RX [ be,50,01,03,00,00,01,ec ]  8
                if packet[1] == BESMessageTypes.CODE.value:
                    print("Resp OK to loading code")
                    sys.stdout.flush()
                    break
            print("Send RUN message")
            serial_port.write(cls.RUN_MESSAGE)
            while datetime.now() < exit_time:
                packet = cls._read_packet(serial_port)
                if packet[1] == BESMessageTypes.RUN.value:
                    #TODO: catch error be 54 01 01 24 c7 - ERR_CODE_INFO_MISSING
                    print("Resp OK to starting code")
                    sys.stdout.flush()
                    break

    @classmethod
    def read_flash_info(cls, serial_port: serial.Serial):
        """
        Unknown if this _needs_ to be run

        """
        exit_time = datetime.now() + timedelta(seconds=30)
        print("starting reading flash id")
        sys.stdout.flush()
        cmd_get_flash_id = [0xBE, 0x65, 0x02, 0x01, 0x11, 0xC8]
        serial_port.write(cmd_get_flash_id)

        while datetime.now() < exit_time:
            packet = cls._read_packet(serial_port)
            if packet[1] == BESMessageTypes.FLASH_COMMAND.value:
                print(f"Flash info: ID {packet[5:8]}")
                sys.stdout.flush()
                break
        cmd_get_flash_unique_id = [0xBE, 0x65, 0x03, 0x01, 0x12, 0xC6]
        serial_port.write(cmd_get_flash_unique_id)

        while datetime.now() < exit_time:
            packet = cls._read_packet(serial_port)
            if packet[1] == BESMessageTypes.FLASH_COMMAND.value:
                print(f"Flash info: Unique ID {packet[5:]}")
                sys.stdout.flush()
                break

    @classmethod
    def run_get_cfgdata(cls, serial_port: serial.Serial):
        """
        No idea what this is for yet
        """
        exit_time = datetime.now() + timedelta(seconds=30)

        msg_sys_poll_1 = [0xBE, 0x03, 0x05, 0x08, 0x00, 0xE0, 0x0F, 0x3C, 0x00, 0x10, 0x00, 0x00, 0xF6]
        serial_port.write(msg_sys_poll_1)

        time.sleep(0.1)
        serial_port.reset_input_buffer()

        msg_sys_poll_2 = [0xBE, 0x03, 0x06, 0x08, 0x00, 0xF0, 0x0F, 0x3C, 0x00, 0x10, 0x00, 0x00, 0xE5]
        serial_port.write(msg_sys_poll_2)

        time.sleep(0.1)
        serial_port.reset_input_buffer()

    @classmethod
    def program_binary_file(cls, serial_port: serial.Serial, filename: str):
        """
        Load the provided program in at the default locations

        """

        with open(filename, "r+b") as f:
            file_payload = f.read()
        file_payload = file_payload[0:-4]
        file_length_raw = len(file_payload)
        # have to pad up to a multiple of 0x8000
        if file_length_raw % 0x8000 != 0:
            padding_len = 0x8000 - (file_length_raw % 0x8000)
            padding = [0xFF] * padding_len
            packed_file = file_payload + bytes(padding)

        file_length = len(packed_file)
        #
        start_address = 0x3C000000
        burn_start_msg = [
            0xBE,
            0x61,
            0x07,
            0x0C,
            0x00,
            0x00,
            0x00,
            0x3C,
            0x00,
            0x00,
            0x0D,
            0x00,
            0x00,
            0x80,
            0x00,
            0x0,
            0x04,
        ]
        burn_start_msg[4] = (start_address >> 0) & 0xFF
        burn_start_msg[5] = (start_address >> 8) & 0xFF
        burn_start_msg[6] = (start_address >> 16) & 0xFF
        burn_start_msg[7] = (start_address >> 24) & 0xFF
        burn_start_msg[8] = (file_length >> 0) & 0xFF
        burn_start_msg[9] = (file_length >> 8) & 0xFF
        burn_start_msg[10] = (file_length >> 16) & 0xFF
        burn_start_msg[11] = (file_length >> 24) & 0xFF
        # update checksum
        burn_start_msg[-1] = cls._calculate_message_checksum(burn_start_msg[0:-1])
        serial_port.write(burn_start_msg)
        exit_time = datetime.now() + timedelta(seconds=30)

        while datetime.now() < exit_time:
            packet = cls._read_packet(serial_port)
            if packet[1] == BESMessageTypes.ERASE_BURN_SART.value:
                print(f"Flash burn start returned {packet}")
                sys.stdout.flush()
                if packet[3] != 0x01:
                    raise Exception("Possible bad programming start?")
                break
        # Start splitting up the payload and sending it
        total_packets_to_send = len(packed_file) / 0x8000
        packets_waiting_ack = []
        seq = 0
        while len(packed_file) > 0:
            chunk = packed_file[0:0x8000]
            packed_file = packed_file[0x8000:]
            data_to_send = cls._create_burn_data_message(seq, chunk)
            print(f"Sending data chunk {seq}")
            sys.stdout.flush()
            serial_port.write(data_to_send)
            packets_waiting_ack.append(seq)
            if seq < 1:
                time.sleep(0.4)
            seq += 1
            while len(packets_waiting_ack) > 1:
                # Only allow two outstanding ones
                ack_seq = cls._wait_for_programming_ack(serial_port)
                if ack_seq in packets_waiting_ack:
                    packets_waiting_ack.remove(ack_seq)
                else:
                    raise Exception(f"Double ack for {ack_seq}")
        while len(packets_waiting_ack) > 0:
            # Only allow two outstanding ones
            print(f"Waiting for {packets_waiting_ack}")
            sys.stdout.flush()
            ack_seq = cls._wait_for_programming_ack(serial_port)
            if ack_seq in packets_waiting_ack:
                packets_waiting_ack.remove(ack_seq)
            else:
                raise Exception(f"Double ack for {ack_seq}")
        print("Sending done; sending commit")
        sys.stdout.flush()
        # Now send the final commit message
        commit_msg = [
            0xBE,
            0x65,
            0x08,
            0x09,
            0x22,
            0x00,
            0x00,
            0x00,
            0x3C,
            0x1C,
            0xEC,
            0x57,
            0xBE,
            0x50,
        ]
        commit_msg[5] = (start_address >> 0) & 0xFF
        commit_msg[6] = (start_address >> 8) & 0xFF
        commit_msg[7] = (start_address >> 16) & 0xFF
        commit_msg[8] = (start_address >> 24) & 0xFF
        commit_msg[13] = cls._calculate_message_checksum(commit_msg[0:-1])
        serial_port.write(commit_msg)

        exit_time = datetime.now() + timedelta(seconds=30)
        while datetime.now() < exit_time:
            packet = cls._read_packet(serial_port)
            if packet[1] == BESMessageTypes.FLASH_COMMAND.value:
                if packet[2] == 0x08 and packet[3] == 0x01:
                    print("Done")
                    sys.stdout.flush()
                    return
        raise Exception("Timed out finalising")

    @classmethod
    def _wait_for_programming_ack(cls, serial_port: serial.Serial) -> int:
        """
        Wait for an ack for programming
        """
        exit_time = datetime.now() + timedelta(seconds=30)
        while datetime.now() < exit_time:
            packet = cls._read_packet(serial_port)
            if packet[1] == BESMessageTypes.FLASH_BURN_DATA.value:
                sequence1 = packet[2] - 0xC1
                sequence2 = packet[5]

                print(f"Flash confirm {sequence1}/{sequence2}")
                sys.stdout.flush()
                if sequence2 == sequence1:
                    return sequence1
        raise Exception("Timeout waiting for programming ack")

    @classmethod
    def _create_burn_data_message(cls, sequence: int, data_payload: List[bytes]) -> List[bytes]:
        """
        Creates the ready-to-send message to burn this chunk of data
        """
        chunk_size = len(data_payload)
        if chunk_size != 0x8000:
            raise Exception("Size not supported")
        template = [
            0xBE,
            0x62,
            0xC1,
            0x0B,
            0x00,
            0x80,
            0x00,
            0x00,
            0xAB,
            0x77,
            0x7F,
            0xF4,
            0x00,
            0x00,
            0x00,
            0xFE,
        ]
        template[2] = 0xC1 + sequence
        template[4] = chunk_size & 0xFF
        template[5] = (chunk_size >> 8) & 0xFF
        crc32_of_chunk = zlib.crc32(data_payload)
        template[8] = (crc32_of_chunk >> 0) & 0xFF
        template[9] = (crc32_of_chunk >> 8) & 0xFF
        template[10] = (crc32_of_chunk >> 16) & 0xFF
        template[11] = (crc32_of_chunk >> 24) & 0xFF
        template[12] = sequence
        template[15] = cls._calculate_message_checksum(template[0:-1])
        print("Tx H", bytes(template).hex(","))
        sys.stdout.flush()
        template.extend(data_payload)
        return template

    @classmethod
    def _read_packet(cls, port: serial.Serial) -> List[bytes]:
        """
        Try and read a bes packet in the timeout
        """
        packet = []
        remain = 1

        while remain > 0:
            # print("Try read %d bytes" % rd_size)
            data = port.read(size=remain)
            # print("Got %d bytes" % len(data))
            if len(packet) == 0:
                if data[0] == 0xBE:
                    packet.extend(data)
                    remain = BESPacket.MINIMAL_PACKET_LEN - len(data)
                    # print("Got 0xBE, received %d bytes, remain %d bytes" % (len(data), remain))
            else:
                packet.extend(data)
                if len(packet) > 3:
                    remain = BESPacket.MINIMAL_PACKET_LEN + packet[3] - len(packet)
                    # print("Got data len %d, received %d bytes, remain %d bytes" % (packet[3], len(data), remain))
                else:
                    remain -= len(data)
                
        print("RX [", bytes(packet).hex(","), "] ", len(packet))
        sys.stdout.flush()
        # Validate the checksum
        if not cls._validate_message_checksum(packet):
            raise Exception("Invalid message checksum")
        return packet

    @classmethod
    def _lookup_packet_length(cls, packet_id1: bytes, packet_id2: bytes):
        """
        Since they do not encode the length into the packet; we need to look them up manually
        This only stores the expected lengths for the messages coming from the MCU; for outgoing Tx messages that is left up to the sender functions
        """

        if packet_id1 == BESMessageTypes.SYNC.value:
            return 8
        if packet_id1 == BESMessageTypes.CODE_INFO.value:
            return 6
        if packet_id1 == BESMessageTypes.CODE_SEND.value:
            return 6
        if packet_id1 == BESMessageTypes.PROGRAMMER_INIT.value:
            return 11
        if packet_id1 == BESMessageTypes.FLASH_COMMAND.value:
            if packet_id2 == 2:
                return 9
            if packet_id2 == 0x08:
                return 6
            return 22
        if packet_id1 == BESMessageTypes.ERASE_BURN_SART.value:
            return 6
        if packet_id1 == BESMessageTypes.FLASH_BURN_DATA.value:
            return 8

        raise Exception(f"Unhandled packet length request for 0x{packet_id1:02x} / 0x{packet_id2:02x}")

    @classmethod
    def _validate_message_checksum(cls, packet: List[bytes]) -> bool:
        """
        Validate the basic packet sum checksum for a message;
        this is actually just validate that all bytes sum to 0xFF (ignoring overflow)
        """
        chk = cls._calculate_message_checksum(packet[0:-1])
        return chk == packet[-1]

    @classmethod
    def _calculate_message_checksum(cls, packet: List[bytes]) -> bytes:
        """
        Calculates the checksum for this message and returns it
        """
        target = 0xFF
        sum = 0
        for b in packet:
            sum += b
            sum = sum & 0xFF
        return target - (sum)


# Spawn monitor on the port
def monitor(port: str):
    try:
        # Step on the args to stop them being parsed by miniterm
        sys.argv = ["besttool.py"]
        miniterm.main(
            default_port=port,
            default_baudrate=BES_BAUD,
        )
    except Exception as e:
        raise e


@click.group()
def cli():
    pass


@cli.command()
@click.argument("port_name")
def sync(port_name):
    """"""
    print(f"Enter to bootload mode @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    BESLink.wait_for_sync(port)
    port.close()

@cli.command()
@click.argument("port_name")
def code(port_name):
    """"""
    print(f"Querying for info @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    BESLink.wait_for_sync(port)
    BESLink.load_code_blob(port)
    port.close()

@cli.command()
@click.argument("port_name")
def info(port_name):
    """"""
    print(f"Querying for info @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    BESLink.wait_for_sync(port)
    BESLink.load_code_blob(port)
    BESLink.read_flash_info(port)
    port.close()


@cli.command()
@click.argument("filepath")
@click.argument("port_name")
def program(filepath, port_name):
    """"""
    print(f"beginning programming of {filepath} to device @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    BESLink.wait_for_sync(port)
    BESLink.load_code_blob(port)
    BESLink.read_flash_info(port)
    BESLink.run_get_cfgdata(port)
    BESLink.program_binary_file(port, filepath)
    port.close()


@cli.command()
@click.argument("filepath")
@click.argument("port_name")
def program_watch(filepath, port_name):
    """"""
    print(f"beginning programming of {filepath} to device @ {port_name} and then will drop into monitor")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    BESLink.wait_for_sync(port)
    BESLink.load_code_blob(port)
    BESLink.read_flash_info(port)
    BESLink.run_get_cfgdata(port)
    BESLink.program_binary_file(port, filepath)
    port.close()
    monitor(port_name)


@cli.command()
def list_ports():
    """Lists available com ports"""
    print("Detected Ports")
    sys.stdout.flush()
    for port in serial.tools.list_ports.comports():
        print(port)
    sys.stdout.flush()

class CRC32:
    def __init__(self):
        self.table = []
        self.value = None

        for i in range(256):
            v = i
            for j in range(8):
                v = (0xEDB88320 ^ (v >> 1)) if(v & 1) == 1 else (v >> 1)
            self.table.append(v)

    def start(self):
        self.value = 0xffffffff
        return self

    def update(self, buf):
        for c in buf:
            self.value = self.table[(self.value ^ c) & 0xFF] ^ (self.value >> 8)
        return self

    def finalize(self):
        return self.value ^ 0xffffffff

    def compute(self, buf):
        return self.start().update(buf).finalize()


if __name__ == "__main__":
    cli()
