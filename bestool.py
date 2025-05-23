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

# send at msg_type
class BESMessageTypes(Enum):
    SYS                         = 0x00
    READ                        = 0x01
    WRITE                       = 0x02
    BULK_READ                   = 0x03 # bl+, pg+
    SYNC                        = 0x50 # bl+, pg-
    CODE_INFO                   = 0x53 # bl+, pg-
    CODE                        = 0x54
    RUN                         = 0x55
    SECTOR_SIZE                 = 0x60 # ok
    ERASE_BURN_START            = 0x61 # ok
    ERASE_BURN_DATA             = 0x62
    BURN_DATA                   = 0x64
    FLASH_CMD                   = 0x65 # ok, bl-, pg+
    GET_SECTOR_INFO             = 0x66 # ok
    SEC_REG_ERASE_BURN_START    = 0x67 # ok
    SEC_REG_ERASE_BURN_DATA     = 0x68

# send at data[0]
class BESFlashCmdTypes(Enum):
    GET_ID            = 0x11 # ok - CMD_GET_ID (GET_FLASH_ID)
    GET_UNIQUE_ID     = 0x12 # ok - CMD_GET_UNIQUE_ID (GET_FLASH_UNIQUE_ID)
    GET_SIZE          = 0x13 # ok - CMD_GET_SIZE (GET_FLASH_SIZE)
    ERASE_SECTOR      = 0x21 # ok
    BURN_DATA         = 0x22 # ok
    ERASE_CHIP        = 0x31 # ok
    SEC_REG_ERASE     = 0x41 # ok - SEC_ERASE
    SEC_REG_BURN      = 0x42 # ok
    SEC_REG_LOCK      = 0x43 # ok - SEC_LOCK
    SEC_REG_READ      = 0x44 # ok
    ENABLE_REMAP      = 0x51 # ok
    DISABLE_REMAP     = 0x52 # ok


class BESSysCmdTypes(Enum):
    REBOOT          = 0xF1
    SHUTDOWN        = 0xF2
    FLASH_BOOT      = 0xF3
    SET_BOOTMODE    = 0xE1
    CLR_BOOTMODE    = 0xE2
    GET_BOOTMODE    = 0xE3


class BESPacket:
    MINIMAL_PACKET_LEN = 5 # minimum packet len is 5 (header, command, sequence, dataLen, checksum)

    sync = 0xBE
    msg_type = 0
    sequence = 0
    data_len = 0    # max 21 bytes accepted by bootloader
    data = bytearray()
    checksum = 0

    def __init__(self):
        pass
    
    def __init__(self, data):
        self.packet = self.parse_packet(data)
    
    def parse_packet(self, data):
        self.sync = data[0]
        self.msg_type = data[1]
        self.sequence = data[2]
        self.data_len = data[3]
        self.checksum = data[4 + self.data_len]
        self.data = bytearray()
        if self.data_len > 0:
            self.data.extend(data[4:4 + self.data_len])
        return self

    
class BESLink:
    """
    Wrapper class for communcations with the BES bootloader thing
    """
    SYNC_MESSAGE    = [0xBE, BESMessageTypes.SYNC.value, 0x00, 0x01, 0x01, 0xEF]
    CODE_MESSAGE    = [0xBE, BESMessageTypes.CODE.value, 0xA2, 0x03, 0x00, 0x00, 0x00, 0x48]
    RUN_MESSAGE     = [0xBE, BESMessageTypes.RUN.value, 0x01, 0x00, 0xEB]

    serial_port: serial.Serial
    wr_seq = 0
    rd_seq = 0
    programmer_running = False

    @classmethod
    def __init__(cls, serial_port: serial.Serial):
        cls.serial_port = serial_port

    def close_port(cls):
        cls.serial_port.close()

    @classmethod
    def wait_for_sync(cls) -> str:
        print(f"Waiting for sync on {cls.serial_port.name}")
        print("Send SYNC request")
        sys.stdout.flush()
        # cls._write_paket_raw(cls.SYNC_MESSAGE)
        # in programmer mode argument doesn't matters coz programmer doensn't support SYNC message and will reply with error code 0xf
        # in bootrom mode: 1 - for noraml mode, 0x56 - for secure mode
        cls._write_paket_raw_data(BESMessageTypes.SYNC, [ 0x01 ])
        exit_time = datetime.now() + timedelta(seconds=30)
        # Sync packet from bootloader is {BE,50,00,03,00,00,01,ED}
        state = "unknown state"
        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet.msg_type == BESMessageTypes.SYNC.value:
                sync_code = packet.data[0] # 0 - bootloader just started, 2 - bootloader is already running and synced, 0xf - programmer is already running (programmer return ERR_TYPE_INVALID = 0x0F for SYNC msg)
                state = "unknown state"
                if sync_code == 0:
                    state = "bootloader started"
                if sync_code == 2: # 2 in normal mode, 0x57 in secure mode
                    state = "bootloader running"
                if sync_code == 0xf:
                    state = "programmer running"
                print(f"Got SYNC reply (code 0x{sync_code:02x} - {state})")
                if sync_code == 0:
                    cls.wait_for_sync()
                elif sync_code == 0xf: # after 0xf code programmer sends msg 0x60(SECTOR_SIZE) as sign of resync
                    cls.programmer_running = True
                    packet = cls._read_packet()
                    if packet.msg_type == BESMessageTypes.SECTOR_SIZE.value:
                        ver, sector_size = struct.unpack("<HI", packet.data)
                        print(f"Got SECTOR_SIZE reply, ver 0x{ver:04x}, sector_size 0x{sector_size:08x}")
                elif sync_code != 2:
                    raise Exception(f"Unknown sync state 0x{sync_code:02x}")                    
                sys.stdout.flush()
                break
        return state

    @classmethod
    def run_programmer(cls):
        state = BESLink.wait_for_sync()
        if state != "programmer running":
            BESLink.load_code_blob("../romdumper/programmer2001.code.bin")

    @classmethod
    def load_code_blob(cls, payload_file):
        """
        Loading in the code blob
        """
        exit_time = datetime.now() + timedelta(seconds=30)
        # code_addr & 0x3 == 0 && code_len > 0 && code_addr >= 0x20001950 && code_len + code_addr < 0x2003F000

        #TODO: autodetect payload is raw or formatted by checking header
        # with open("../romdumper/main.bin", "r+b") as f:
        with open(payload_file, "r+b") as f:
            code_payload = f.read()
            f.close()

            # address = 0x20002000
            entry, param, sp, address = struct.unpack("<IIII", code_payload[0:16])
            if entry == 0xBE57EC1C:
                raise Exception("Use payload from formatted code file is not supported yet")
            size = len(code_payload)

            crc = zlib.crc32(code_payload)

            print(f"Paylod info: size {size}, load address 0x{address:08x}, crc32 0x{crc:08x}, entry 0x{entry:08x}, param 0x{param:08x}, sp 0x{sp:08x}")
            sys.stdout.flush()

            code_info_msg_data = [
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
            ]
            # Send code info message
            print("Send CODE_INFO message")
            sys.stdout.flush()
            cls._write_paket_raw_data(BESMessageTypes.CODE_INFO, code_info_msg_data)
            # wait for response
            while datetime.now() < exit_time:
                packet = cls._read_packet()
                if packet.msg_type == BESMessageTypes.CODE_INFO.value:
                    if packet.data[0] == 0:
                        print("Resp OK to start code upload")
                        sys.stdout.flush()
                        break
                    else:
                        desc = ""
                        if packet.data[0] == 0xf: # programmer return ERR_TYPE_INVALID = 0x0F for unsupported msg types
                            desc = " - seems like code is already running"
                        raise Exception(f"Resp NOT OK to start code upload, error 0x{packet.data[0]:02x}{desc}")
                # it seems like programmer returns msg_type 0x60 for every unsupported sended msg_type value
                elif packet.msg_type == BESMessageTypes.SECTOR_SIZE.value:
                    cls.programmer_running = True
                    ver, sector_size = struct.unpack("<HI", packet.data)
                    print(f"Resp NOT OK - programmer already running, ver 0x%04x, sector_size 0x%08x" % (ver, sector_size))
                    sys.stdout.flush()
                    raise Exception("Code load failed - programmer already running")
                else:
                    raise Exception(f"Code load failed - unknown msg_type in reply 0x{packet.msg_type:02x}")
            print("Send CODE")
            sys.stdout.flush()
            cls._write_paket_raw(cls.CODE_MESSAGE)
            cls.serial_port.write(code_payload)
            # wait for response
            while datetime.now() < exit_time:
                packet = cls._read_packet()
                #TODO: catch error: in case of incorrect CODE CRC bootloader silently (without sending 0x54 reply with error code) send resync message RX [ be,50,01,03,00,00,01,ec ]  8
                #TODO: catch error: be 54 01 01 24 c7 - ERR_CODE_INFO_MISSING
                if packet.msg_type == BESMessageTypes.CODE.value:
                    if packet.data[0] == 0x20:
                        print("Resp OK to loading code")
                        sys.stdout.flush()
                        break
                    else:
                        raise Exception(f"Load code failed: error 0x{packet.data[0]:02x}")
                else:
                    raise Exception(f"Load code failed: bad reply msg_type 0x{packet.msg_type:02x}")
            print("Send RUN message")
            sys.stdout.flush()
            cls._write_paket_raw(cls.RUN_MESSAGE)
            while datetime.now() < exit_time:
                packet = cls._read_packet()
                # msg 0x55 is returned by bootloader after code running is done
                # normally when programmer payload code is starting first incoming message will be 0x60
                # programmer blob never return (loop forever)? exit only by reboot
                if packet.msg_type == BESMessageTypes.RUN.value:
                    if packet.data[0] == 0:
                        cls.programmer_running = False
                        ret_code = struct.unpack("<I", packet.data[1:5])
                        print(f"Resp OK run code done, ret 0x{ret_code:08x}")
                        sys.stdout.flush()
                        break
                    else:
                        raise Exception(f"Run code exit with error 0x{packet.data[0]:02x}")
                elif packet.msg_type == BESMessageTypes.SECTOR_SIZE.value:
                    cls.programmer_running = True
                    ver, sector_size = struct.unpack("<HI", packet.data)
                    print(f"Resp OK - programmer sucessfully running, ver 0x{ver:04x}, sector_size 0x{sector_size:08x}")
                    sys.stdout.flush()
                    break
                else:
                    raise Exception(f"Run code failed: bad reply msg_type {packet.msg_type:02x}")

    @classmethod
    def read_flash_info(cls):
        exit_time = datetime.now() + timedelta(seconds=30)
        print("Start reading flash id")
        sys.stdout.flush()
        # 0x65:0x11
        cls._write_paket_raw_data(BESMessageTypes.FLASH_CMD, [ BESFlashCmdTypes.GET_ID.value ])
        # cls._write_paket_raw_data(BESMessageTypes.SYNC, [ 0x01 ])

        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet.msg_type == BESMessageTypes.FLASH_CMD.value:
                print(f"Flash info: ID {packet.data[1:4].hex("-")}")
                sys.stdout.flush()
                break
        # 0x65:0x12
        cls._write_paket_raw_data(BESMessageTypes.FLASH_CMD, [ BESFlashCmdTypes.GET_UNIQUE_ID.value ])

        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet.msg_type == BESMessageTypes.FLASH_CMD.value:
                print(f"Flash info: Unique ID {packet.data[5:].hex()}")
                sys.stdout.flush()
                break

    @classmethod
    def run_get_cfgdata(cls):
        """
        No idea what this is for yet
        """
        exit_time = datetime.now() + timedelta(seconds=30)

        msg_sys_poll_1 = [
            0xBE,
            BESMessageTypes.BULK_READ.value,
            0x05,
            0x08,
            0x00, 0xE0, 0x0F, 0x3C, # read from 0x3C0FE000 (flash offset 0xFE000) (why? there is nothing. should be 0xFFE000 in for some data from bes_reserved?)
            0x00, 0x10, 0x00, 0x00, # size 0x1000
            0xF6
        ]
        cls._write_paket_raw(msg_sys_poll_1)

        time.sleep(0.1)
        cls.serial_port.reset_input_buffer()

        msg_sys_poll_2 = [
            0xBE,
            0x03,
            0x06,
            0x08,
            0x00, 0xF0, 0x0F, 0x3C, # read at 0x3C0FF000 (flash offset 0xFF000) (why? factory is located at 0xFFF000)
            0x00, 0x10, 0x00, 0x00,
            0xE5
        ]
        cls._write_paket_raw(msg_sys_poll_2)

        time.sleep(0.1)
        cls.serial_port.reset_input_buffer()

    @classmethod
    def program_binary_file(cls, filename: str):
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
        cls._write_paket_raw(burn_start_msg)
        exit_time = datetime.now() + timedelta(seconds=30)

        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet[1] == BESMessageTypes.ERASE_BURN_START.value:
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
            cls._write_paket_raw(data_to_send)
            packets_waiting_ack.append(seq)
            if seq < 1:
                time.sleep(0.4)
            seq += 1
            while len(packets_waiting_ack) > 1:
                # Only allow two outstanding ones
                ack_seq = cls._wait_for_programming_ack()
                if ack_seq in packets_waiting_ack:
                    packets_waiting_ack.remove(ack_seq)
                else:
                    raise Exception(f"Double ack for {ack_seq}")
        while len(packets_waiting_ack) > 0:
            # Only allow two outstanding ones
            print(f"Waiting for {packets_waiting_ack}")
            sys.stdout.flush()
            ack_seq = cls._wait_for_programming_ack()
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
        cls._write_paket_raw(commit_msg)

        exit_time = datetime.now() + timedelta(seconds=30)
        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet[1] == BESMessageTypes.FLASH_CMD.value:
                if packet[2] == 0x08 and packet[3] == 0x01:
                    print("Done")
                    sys.stdout.flush()
                    return
        raise Exception("Timed out finalising")
    
    @classmethod
    def dump_to_file(cls, s_address, s_size, filename: str):
        """
        Bulk dump data from any device's address to file

        """
        if str.lower(s_address[:2]) == "0x":
            address = int(s_address, 16)
        else:
            address = int(s_address)
        if str.lower(s_size[:2]) == "0x":
            size = int(s_size, 16)
        else:
            size = int(s_size)
        with open(filename, "wb") as f:
            if cls.programmer_running:
                resync_at = datetime.now() + timedelta(seconds=10)
            chunk_size = 0x8000//2
            if chunk_size > size:
                chunk_size = size
            remain = size
            while remain > 0:
                if remain < chunk_size:
                    chunk_size = remain
                data = cls.read_chunk(address, chunk_size)
                f.write(data)
                received = len(data)
                address += received
                remain -= received
                if received != chunk_size:
                    print(f"Expected {chunk_size}, {received} got")
                print(f"Total {(size-remain)} of {size} ({((size-remain)/size*100):.0f}%)")
                # time.sleep(0.01) # give chance to mcu to resets wdt?
                # dump may fails on long ops, so try to reinit programmer state (flush buffers, reset timouts, etc.)
                # seems like this is not needed for bootloader but for programmer only
                if cls.programmer_running and datetime.now() >= resync_at:
                    cls.wait_for_sync()
                    resync_at = datetime.now() + timedelta(seconds=10)
            f.close()

    
    @classmethod
    def read_chunk(cls, address, size) -> bytearray:
        ret = bytearray()
        bulk_read_msg = [
            (address >> 0) & 0xFF,
            (address >> 8) & 0xFF,
            (address >> 16) & 0xFF,
            (address >> 24) & 0xFF,
            (size >> 0) & 0xFF,
            (size >> 8) & 0xFF,
            (size >> 16) & 0xFF,
            (size >> 24) & 0xFF,
        ]
        cls._write_paket_raw_data(BESMessageTypes.BULK_READ, bulk_read_msg)
        exit_time = datetime.now() + timedelta(seconds=10)

        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet.msg_type == BESMessageTypes.BULK_READ.value:
                print(f"Bulk read returned {packet.data.hex(" ")}")
                sys.stdout.flush()
                if packet.data[0] != 0x00:
                    raise Exception(f"Bad return code {packet.data[0]:02x}")
                i = address
                remain = size
                chunk_size = 16
                while remain > 0:
                    data = cls.serial_port.read(size=chunk_size)
                    remain -= len(data)
                    # print(f"{i:08x}: {data.hex(" ")} [{remain}]")
                    i += len(data)
                    ret.extend(data)
                # print("Done")
                break
            else:
                raise Exception("Unexpected msg_type {packet.msg_type:02x} during BULK_READ")
        return ret

    @classmethod
    def reboot(cls, address, size) -> bytearray:
        ret = bytearray()
        bulk_read_msg = [
            (address >> 0) & 0xFF,
            (address >> 8) & 0xFF,
            (address >> 16) & 0xFF,
            (address >> 24) & 0xFF,
            (size >> 0) & 0xFF,
            (size >> 8) & 0xFF,
            (size >> 16) & 0xFF,
            (size >> 24) & 0xFF,
        ]
        cls._write_paket_raw_data(BESMessageTypes.BULK_READ, bulk_read_msg)
        exit_time = datetime.now() + timedelta(seconds=10)

        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet.msg_type == BESMessageTypes.BULK_READ.value:
                print(f"Bulk read returned {packet.data.hex(" ")}")
                sys.stdout.flush()
                if packet.data[0] != 0x00:
                    raise Exception(f"Bad return code {packet.data[0]:02x}")
                i = address
                remain = size
                chunk_size = 16
                while remain > 0:
                    data = cls.serial_port.read(size=chunk_size)
                    remain -= len(data)
                    # print(f"{i:08x}: {data.hex(" ")} [{remain}]")
                    i += len(data)
                    ret.extend(data)
                # print("Done")
                break
            else:
                raise Exception("Unexpected msg_type {packet.msg_type:02x} during BULK_READ")
        return ret

    @classmethod
    def _wait_for_programming_ack(cls) -> int:
        """
        Wait for an ack for programming
        """
        exit_time = datetime.now() + timedelta(seconds=30)
        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet[1] == BESMessageTypes.ERASE_BURN_DATA.value:
                sequence1 = packet[2] - 0xC1
                sequence2 = packet[5]

                print(f"Flash confirm {sequence1}/{sequence2}")
                sys.stdout.flush()
                if sequence2 == sequence1:
                    return sequence1
        raise Exception("Timeout waiting for programming ack")

    @classmethod
    def _create_burn_data_message(cls, sequence: int, data_payload: bytearray) -> bytearray:
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
    def _read_packet_raw(cls) -> bytearray:
        """
        Try and read a bes packet in the timeout
        """
        packet = bytearray()
        remain = 1

        while remain > 0:
            # print("Try read %d bytes" % rd_size)
            data = cls.serial_port.read(size=remain)
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
    def _read_packet(cls) -> BESPacket:
        raw = cls._read_packet_raw()
        return BESPacket(raw).packet

    @classmethod
    def _write_paket_raw(cls, pkt: bytearray):
        pkt[-1] = cls._calculate_message_checksum(pkt[0:-1])
        print("TX [", bytes(pkt).hex(","), "] ", len(pkt))
        cls.serial_port.write(pkt)

    @classmethod
    def _write_paket_raw_data(cls, msg_type: BESMessageTypes, data: bytes):
        cls.wr_seq += 1
        pkt = [
            0xBE,
            msg_type.value,
            0, #cls.wr_seq,
            len(data)
        ]
        pkt.extend(data)
        pkt.append(cls._calculate_message_checksum(pkt))
        print("TX [", bytes(pkt).hex(","), "] ", len(pkt))
        cls.serial_port.write(pkt)

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
        if packet_id1 == BESMessageTypes.SECTOR_SIZE.value:
            return 11
        if packet_id1 == BESMessageTypes.FLASH_CMD.value:
            if packet_id2 == 2:
                return 9
            if packet_id2 == 0x08:
                return 6
            return 22
        if packet_id1 == BESMessageTypes.ERASE_BURN_START.value:
            return 6
        if packet_id1 == BESMessageTypes.ERASE_BURN_DATA.value:
            return 8

        raise Exception(f"Unhandled packet length request for 0x{packet_id1:02x} / 0x{packet_id2:02x}")

    @classmethod
    def _validate_message_checksum(cls, packet: bytearray) -> bool:
        """
        Validate the basic packet sum checksum for a message;
        this is actually just validate that all bytes sum to 0xFF (ignoring overflow)
        """
        chk = cls._calculate_message_checksum(packet[0:-1])
        return chk == packet[-1]

    @classmethod
    def _calculate_message_checksum(cls, packet: bytearray) -> bytes:
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
    bes = BESLink(serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30))
    bes.wait_for_sync()
    bes.close_port()

@cli.command()
@click.argument("port_name")
@click.option("-p", "--payload", default="../romdumper/programmer2001.code.bin")
@click.option("-f", "--force", is_flag=True)
# @click.option("--force", "force")
def code(port_name, payload, force):
    """"""
    print(f"Load code @ {port_name}")
    sys.stdout.flush()
    bes = BESLink(serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30))
    state = bes.wait_for_sync()
    if force or state != "programmer running":
        bes.load_code_blob(payload)
        bes.wait_for_sync()
    bes.close_port()

@cli.command()
@click.argument("port_name")
def info(port_name):
    """"""
    print(f"Querying for info @ {port_name}")
    sys.stdout.flush()
    bes = BESLink(serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=5))
    BESLink.run_programmer()
    BESLink.read_flash_info()
    bes.close_port()


@cli.command()
@click.argument("filepath")
@click.argument("port_name")
def program(filepath, port_name):
    """"""
    print(f"beginning programming of {filepath} to device @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    BESLink.run_programmer(port)
    BESLink.read_flash_info(port)
    # BESLink.run_get_cfgdata(port)
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
    BESLink.run_programmer(port)
    BESLink.read_flash_info(port)
    # BESLink.run_get_cfgdata(port)
    BESLink.program_binary_file(port, filepath)
    port.close()
    monitor(port_name)

@cli.command()
@click.argument("port_name")
@click.option("-o", "-w", "filepath")
@click.option("-a", "address")
@click.option("-s", "size")
@click.option("-p", "--use-programmer", is_flag=True)
def dump(port_name, address, size, filepath, use_programmer):
    """"""
    print(f"Dumping {size} from {address} to {filepath} from device @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    bes = BESLink(port)
    bes.wait_for_sync()
    if use_programmer:
        bes.run_programmer() # doesn't need a programmer to dump smthng
        bes.read_flash_info() # unsupported without programmer
    bes.dump_to_file(address, size, filepath)
    port.close()


@cli.command()
def list_ports():
    """Lists available com ports"""
    print("Detected Ports")
    sys.stdout.flush()
    for port in serial.tools.list_ports.comports():
        print(port)
    sys.stdout.flush()


if __name__ == "__main__":
    cli()
