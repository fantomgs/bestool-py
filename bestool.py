#! /usr/local/bin/python3

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

__author__ = "Ben V. Brown (ext. by Fan Tom G.S.)"
BES_BAUD = 921600
FLASH_START = 0x2C000000
BURN_OFFSET = 0x00080000

# send at msg_type
class BESMessageTypes(Enum):
    SYS                         = 0x00
    READ                        = 0x01
    WRITE                       = 0x02
    NOTIFICATION                = 0x10 # bl?, pg+ - sent by programmer in case of error? i.t. flash open error: data[0] - err returned by hal_norflash_open, data[1:3] - flash_id)
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
    REBOOT          = 0xF1 # args: no.                  support: bl: hw_reset_0x71; pg: bootmode |= BOOTMODE_SKIP_FLASH_BOOT, hw_reset_0x71 (i.e. in bootloader mode - reboot, in programmer mode - reboot to bootloader mode); return: bl+, pg+
    SHUTDOWN        = 0xF2 # args: no.                  support: bl+, pg+; action: bl - PMU_REG_POWER_OFF |= 1 and loop(1), pg - loop(1); return: bl+, pg+
    FLASH_BOOT      = 0xF3 # args: no.
    SET_BOOTMODE    = 0xE1 # args: 4 bytes.
    CLR_BOOTMODE    = 0xE2 # args: 4 bytes.
    GET_BOOTMODE    = 0xE3 # args: no, return 4 bytes.

class BESBootmodeTypes(Enum):
  WATCHDOG              = 1 << 0
  GLOBAL                = 1 << 1
  RTC                   = 1 << 2
  CHARGER               = 1 << 3
  READ_ENABLED          = 1 << 4
  WRITE_ENABLED         = 1 << 5
  JTAG_ENABLED          = 1 << 6
  FORCE_USB_DLD         = 1 << 7
  FORCE_UART_DLD        = 1 << 8
  DLD_TRANS_UART        = 1 << 9
  SKIP_FLASH_BOOT       = 1 << 10
  CHIP_TEST             = 1 << 11
  FACTORY               = 1 << 12
  CALIB                 = 1 << 13
  ROM_RESERVED_14       = 1 << 14
  FLASH_BOOT            = 1 << 15
  REBOOT                = 1 << 16
  ROM_RESERVED_17       = 1 << 17
  FORCE_USB_PLUG_IN     = 1 << 18
  POWER_DOWN_WAKEUP     = 1 << 19
#   TEST_MASK             = 0x700000
  TEST_MODE             = 1 << 20
  TEST_SIGNALINGMODE    = 1 << 21
  TEST_NOSIGNALINGMODE  = 1 << 22
  ENTER_HIDE_BOOT       = 1 << 23
  RESERVED_BIT24        = 1 << 24
  REBOOT_FROM_CRASH     = 1 << 25
  CDC_COMM              = 1 << 28
  REBOOT_BT_ON          = 1 << 29
  REBOOT_ANC_ON         = 1 << 30
  LOCAL_PLAYER          = 1 << 31

def bootmode_to_string(mode) -> str:
    mode = atoi(mode)
    names = []
    for mask in BESBootmodeTypes:
        if mode & mask.value:
            names.append(mask.name)
    return " | ".join(names)


class BESPacket:
    MINIMAL_PACKET_LEN = 5 # minimum packet len is 5 (header, command, sequence, dataLen, checksum)

    sync = 0xBE
    msg_type = None
    sequence = 0
    data_len = 0    # max 21 bytes accepted by bootloader
    data: bytearray
    checksum = 0
    
    raw: bytearray

    def __init__(self):
        pass
    
    def __init__(self, data):
        self.packet = self.parse_packet(data)
        self.raw = data
    
    def parse_packet(self, data):
        self.sync = data[0]
        self.msg_type = BESMessageTypes(data[1])
        self.sequence = data[2]
        self.data_len = data[3]
        self.checksum = data[4 + self.data_len]
        # self.data = bytearray()
        if self.data_len > 0:
            self.data = data[4:4 + self.data_len]
        return self

    
class BESLink:
    """
    Wrapper class for communcations with the BES bootloader thing
    """
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
            if packet.msg_type == BESMessageTypes.SYNC:
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
                    if packet.msg_type == BESMessageTypes.SECTOR_SIZE:
                        ver, sector_size = struct.unpack("<HI", packet.data)
                        print(f"Got SECTOR_SIZE reply, ver 0x{ver:04x}, sector_size 0x{sector_size:08x}")
                elif sync_code != 2:
                    raise Exception(f"Unknown sync state 0x{sync_code:02x}")
                sys.stdout.flush()
                break
        return state

    @classmethod
    def run_programmer(cls, sync=True, force=False):
        state = None
        if sync:
            state = BESLink.wait_for_sync()
        # print(f"state {state}")
        if force or state != "programmer running":
            BESLink.load_code_blob("./payload/programmer2001.bin")

    @classmethod
    def load_code_blob(cls, payload_file):
        """
        Loading in the code blob
        """
        exit_time = datetime.now() + timedelta(seconds=30)
        # code_addr & 0x3 == 0 && code_len > 0 && code_addr >= 0x20001950 && code_len + code_addr < 0x2003F000

        with open(payload_file, "r+b") as f:
            code_payload = f.read()
            f.close()

            entry, = struct.unpack("<I", code_payload[0:4])
            # print(f"entry 0x{entry:08x}")
            if entry == 0xBE57EC1C:
                security, version,_, build_info_addr = struct.unpack("<HHII", code_payload[4:16])
                if version == 0 or version == 2:
                    code_offset = 0x41C
                elif version == 4:
                    code_offset = 0x61C
                else:
                    raise Exception(f"Unsupported programmer file format version {version}")
                dst_addr, = struct.unpack("<I", code_payload[-4:])
                code_payload = code_payload[code_offset:-4]
                build_info = code_payload[build_info_addr - dst_addr:].decode('utf-8')
                print(f"Header info: security {security}, version {version}, build {build_info}, code load address 0x{dst_addr:08x}")
            
            size = len(code_payload)
            entry, param, sp, address = struct.unpack("<IIII", code_payload[0:16])

            if dst_addr is not None and dst_addr != address:
                raise Exception("Code load address from footer 0x{dst_addr:08x} != 0x{address} from header")

            crc = zlib.crc32(code_payload)

            print(f"Payload info: size {size}, load address 0x{address:08x}, crc32 0x{crc:08x}, entry 0x{entry:08x}, param 0x{param:08x}, sp 0x{sp:08x}")
            sys.stdout.flush()

            code_info_msg_data = bytearray()
            code_info_msg_data.extend(struct.pack("<I", address))
            code_info_msg_data.extend(struct.pack("<I", size))
            code_info_msg_data.extend(struct.pack("<I", crc))
            # Send code info message
            print("Send CODE_INFO message")
            sys.stdout.flush()
            cls._write_paket_raw_data(BESMessageTypes.CODE_INFO, code_info_msg_data)
            # wait for response
            while datetime.now() < exit_time:
                packet = cls._read_packet()
                if packet.msg_type == BESMessageTypes.CODE_INFO:
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
                elif packet.msg_type == BESMessageTypes.SECTOR_SIZE:
                    cls.programmer_running = True
                    ver, sector_size = struct.unpack("<HI", packet.data)
                    print(f"Resp NOT OK - programmer already running, ver 0x%04x, sector_size 0x%08x" % (ver, sector_size))
                    sys.stdout.flush()
                    raise Exception("Code load failed - programmer already running")
                else:
                    raise Exception(f"Code load failed - unknown msg_type in reply 0x{packet.msg_type:02x}")
            print("Send CODE")
            sys.stdout.flush()
            cls._write_paket_raw_data(BESMessageTypes.CODE, [0x00, 0x00, 0x00])
            cls.serial_port.write(code_payload)
            # wait for response
            while datetime.now() < exit_time:
                packet = cls._read_packet()
                #TODO: catch error: in case of incorrect CODE CRC bootloader silently (without sending 0x54 reply with error code) send resync message RX [ be,50,01,03,00,00,01,ec ]  8
                #TODO: catch error: be 54 01 01 24 c7 - ERR_CODE_INFO_MISSING
                if packet.msg_type == BESMessageTypes.CODE:
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
            # cls._write_paket_raw(cls.RUN_MESSAGE)
            cls._write_paket_raw_data(BESMessageTypes.RUN, [])
            while datetime.now() < exit_time:
                packet = cls._read_packet()
                # msg 0x55 is returned by bootloader after code running is done
                # normally when programmer payload code is starting first incoming message will be 0x60
                # programmer blob never return (loop forever)? exit only by reboot
                if packet.msg_type == BESMessageTypes.RUN:
                    if packet.data[0] == 0:
                        cls.programmer_running = False
                        ret_code = struct.unpack("<I", packet.data[1:5])
                        print(f"Resp OK run code done, ret 0x{ret_code:08x}")
                        sys.stdout.flush()
                        break
                    else:
                        raise Exception(f"Run code exit with error 0x{packet.data[0]:02x}")
                elif packet.msg_type == BESMessageTypes.SECTOR_SIZE:
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
            if packet.msg_type == BESMessageTypes.FLASH_CMD:
                print(f"Flash info: ID {packet.data[1:4].hex("-")}")
                sys.stdout.flush()
                break
        # 0x65:0x12
        cls._write_paket_raw_data(BESMessageTypes.FLASH_CMD, [ BESFlashCmdTypes.GET_UNIQUE_ID.value ])

        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet.msg_type == BESMessageTypes.FLASH_CMD:
                print(f"Flash info: Unique ID {packet.data[5:].hex()}")
                sys.stdout.flush()
                break

    @classmethod
    def program_binary_file(cls, filename: str):
        """
        Load the provided program in at the default locations

        """
        cls.program_raw_binary_file(BURN_OFFSET, filename)
    
    @classmethod
    def program_raw_binary_file(cls, flash_offset, filename: str):
        """
        Burn the provided binary at flash offset

        """
        # sector_size = 0x8000 # 32KiB
        sector_size = 0x1000 # 4KiB
        flash_offset = atoi(flash_offset)
        if flash_offset < 0 or flash_offset > 16*1024*1024:
            raise Exception(f"Flash offset {flash_offset:0x08x} out of bounds")
        with open(filename, "r+b") as f:
            packed_file = f.read()
        burn_len = len(packed_file)
        # have to pad up to a multiple of sector_size
        if burn_len % sector_size != 0:
            raise Exception(f"File {filename} len is not multiple of 0x{sector_size:04x}")
            padding_len = sector_size - (burn_len % sector_size)
            padding = [0xFF] * padding_len
            packed_file = packed_file + bytes(padding)

        burn_len = len(packed_file)
        burn_addr = FLASH_START + flash_offset
        data = bytearray()
        data.extend(struct.pack("<I", burn_addr))
        data.extend(struct.pack("<I", burn_len))
        data.extend(struct.pack("<I", sector_size))
        cls._write_paket_raw_data(BESMessageTypes.ERASE_BURN_START, data)
        exit_time = datetime.now() + timedelta(seconds=30)

        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet.msg_type == BESMessageTypes.ERASE_BURN_START:
                print(f"Flash burn start returned {packet}")
                sys.stdout.flush()
                if packet.data_len != 0x01 or packet.data[0] != 0x00:
                    raise Exception("Possible bad programming start?")
                break
        # Start splitting up the payload and sending it
        total_packets_to_send = len(packed_file) / sector_size
        packets_waiting_ack = []
        seq = 0
        while len(packed_file) > 0:
            chunk = packed_file[0:sector_size]
            packed_file = packed_file[sector_size:]
            print(f"Sending data chunk {seq}")
            sys.stdout.flush()
            cls._send_burn_data_message(seq, chunk)
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
        # Now send the final commit message - it writes magic value at flash header marking flash as containing valid firmware
        cls.burn_data_short(burn_addr, struct.pack("<I", 0xBE57EC1C))
    
    @classmethod
    def burn_short(cls, burn_addr, burn_data:bytearray):
        """ Use FLASH_CMD->BURN_DATA to burn directly up to 16 bytes """
        if len(burn_data) > 16:
            raise Exception("burn_short: cannot burn more than 16 bytes at once")
        packet_data = bytearray([BESFlashCmdTypes.BURN_DATA.value])
        packet_data.extend(struct.pack("<I", burn_addr))
        packet_data.extend(burn_data)
        cls._write_paket_raw_data(BESMessageTypes.FLASH_CMD, packet_data)

        exit_time = datetime.now() + timedelta(seconds=30)
        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet.msg_type == BESMessageTypes.FLASH_CMD:
                # if packet[2] == 0x08 and packet[3] == 0x01:
                if packet.data[0] == 0x00:
                    print("burn_short: done")
                    sys.stdout.flush()
                    return
                else:
                    raise Exception(f"burn_short: error {packet.data[0]:02x} in FLASH_CMD reply")
            else:
                raise Exception(f"burn_short: unexpected msg_type {packet.msg_type} in FLASH_CMD reply")
        raise Exception("burn_short: burn data timed out")
    
    @classmethod
    def dump_to_file(cls, address, size, filename: str):
        """
        Bulk dump data from any device's address to file

        """
        address = atoi(address)
        size = atoi(size)        
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
                time.sleep(0.01)
                # dump may fails on long ops, so try to reinit programmer state (flush buffers, reset timouts, etc.)
                # seems like this is not needed for bootloader but for programmer only
                if cls.programmer_running and datetime.now() >= resync_at:
                    cls.wait_for_sync()
                    resync_at = datetime.now() + timedelta(seconds=10)
            f.close()

    
    @classmethod
    def read_chunk(cls, address:int, size:int) -> bytearray:
        ret = bytearray()
        cls._write_paket_raw_data(BESMessageTypes.BULK_READ, struct.pack("<II", address, size))
        exit_time = datetime.now() + timedelta(seconds=10)

        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet.msg_type == BESMessageTypes.BULK_READ:
                # print(f"Bulk read returned {packet.data.hex(" ")}")
                sys.stdout.flush()
                if packet.data[0] != 0x00:
                    raise Exception(f"Bad return code {packet.data[0]:02x}")
                i = address
                remain = size
                chunk_size = 16
                while remain > 0:
                    if chunk_size > remain:
                        chunk_size = remain
                    data = cls.serial_port.read(size=chunk_size)
                    remain -= len(data)
                    # print(f"{i:08x}: {data.hex(" ")} [{remain}]")
                    i += len(data)
                    ret.extend(data)
                # print("Done")
                break
            else:
                raise Exception(f"Unexpected msg_type {packet.msg_type} during BULK_READ")
        return ret

    @classmethod
    def reboot(cls):
        cls._write_paket_raw_data(BESMessageTypes.SYS, [ BESSysCmdTypes.REBOOT.value ])
        print("Send REBOOT request...")
        packet = cls._read_packet()
        if packet.msg_type == BESMessageTypes.SYS:
            print(f"REBOOT reply {packet.data.hex(" ")}")
            sys.stdout.flush()
            if packet.data[0] != 0x00:
                raise Exception(f"Bad return code {packet.data[0]:02x}")
        else:
            raise Exception(f"Unexpected msg_type {packet.msg_type} in REBOOT reply")

    @classmethod
    def shutdown(cls):
        cls._write_paket_raw_data(BESMessageTypes.SYS, [ BESSysCmdTypes.SHUTDOWN.value ])
        print("Send SHUTDOWN request...")
        packet = cls._read_packet()
        if packet.msg_type == BESMessageTypes.SYS:
            print(f"SHUTDOWN reply {packet.data.hex(" ")}")
            sys.stdout.flush()
            if packet.data[0] != 0x00:
                raise Exception(f"Bad return code {packet.data[0]:02x}")
        else:
            raise Exception(f"Unexpected msg_type {packet.msg_type} in SHUTDOWN reply")

    @classmethod
    def flash_boot(cls):
        cls._write_paket_raw_data(BESMessageTypes.SYS, [ BESSysCmdTypes.FLASH_BOOT.value ])
        print("Send FLASH_BOOT request...")
        packet = cls._read_packet()
        if packet.msg_type == BESMessageTypes.SYS:
            print(f"Got FLASH_BOOT reply {packet.data.hex(" ")}")
            sys.stdout.flush()
            if packet.data[0] != 0x00:
                raise Exception(f"Bad return code {packet.data[0]:02x}")
        else:
            raise Exception(f"Unexpected msg_type {packet.msg_type} in FLASH_BOOT reply")

    @classmethod
    def get_bootmode(cls) -> int:
        cls._write_paket_raw_data(BESMessageTypes.SYS, [ BESSysCmdTypes.GET_BOOTMODE.value ])
        print("Send GET_BOOTMODE request...")
        packet = cls._read_packet()
        if packet.msg_type == BESMessageTypes.SYS:
            print(f"Got GET_BOOTMODE reply {packet.data.hex(" ")}")
            sys.stdout.flush()
            if packet.data[0] != 0x00:
                raise Exception(f"Bad return code {packet.data[0]:02x}")
            return struct.unpack("<I", packet.data[1:5])[0]
        else:
            raise Exception(f"Unexpected msg_type {packet.msg_type} in GET_BOOTMODE reply")

    @classmethod
    def set_bootmode(cls, mode):
        """ Set BOOTMODE bits except of READ_ENABLED and WRITE_ENABLED """
        mode = atoi(mode)
        data = bytearray( [BESSysCmdTypes.SET_BOOTMODE.value] )
        data.extend(struct.pack("<I", mode))
        cls._write_paket_raw_data(BESMessageTypes.SYS, data)
        print("Send SET_BOOTMODE request...")
        packet = cls._read_packet()
        if packet.msg_type == BESMessageTypes.SYS:
            print(f"Got SET_BOOTMODE reply {packet.data.hex(" ")}")
            sys.stdout.flush()
            if packet.data[0] != 0x00:
                raise Exception(f"Bad return code {packet.data[0]:02x}")
        else:
            raise Exception(f"Unexpected msg_type {packet.msg_type} in SET_BOOTMODE reply")

    @classmethod
    def clear_bootmode(cls, mode):
        """ Clear BOOTMODE bits except of READ_ENABLED and WRITE_ENABLED """
        mode = atoi(mode)
        data = bytearray( [BESSysCmdTypes.CLR_BOOTMODE.value] )
        data.extend(struct.pack("<I", mode))
        cls._write_paket_raw_data(BESMessageTypes.SYS, data)
        print("Send CLR_BOOTMODE request...")
        packet = cls._read_packet()
        if packet.msg_type == BESMessageTypes.SYS:
            print(f"Got CLR_BOOTMODE reply {packet.data.hex(" ")}")
            sys.stdout.flush()
            if packet.data[0] != 0x00:
                raise Exception(f"Bad return code {packet.data[0]:02x}")
        else:
            raise Exception(f"Unexpected msg_type {packet.msg_type} in CLR_BOOTMODE reply")

    @classmethod
    def _wait_for_programming_ack(cls) -> int:
        """
        Wait for an ack for programming
        """
        exit_time = datetime.now() + timedelta(seconds=30)
        while datetime.now() < exit_time:
            packet = cls._read_packet()
            if packet.msg_type == BESMessageTypes.ERASE_BURN_DATA:
                sequence = struct.unpack("<H", packet.data[1:])
                if packet.data[0] != 0x00:
                    raise Exception(f"Got ERASE_BURN_DATA reply error 0x{packet.data[0]:02x} for seq {sequence}")
                print(f"Burn confirmed seq {sequence}")
                sys.stdout.flush()
                return sequence
            else:                
                raise Exception(f"Got bad msg_type {packet.msg_type}({(BESMessageTypes(packet.msg_type).name)}) while waiting for ERASE_BURN_DATA reply")
        raise Exception("Timeout waiting for programming ack")

    @classmethod
    def _send_burn_data_message(cls, sequence: int, data_payload: bytearray) -> bytearray:
        """
        Send message to burn this chunk of data
        """
        chunk_size = len(data_payload)
        if chunk_size != 0x8000 and chunk_size != 0x1000:
            raise Exception("Size is not supported")
        crc32_of_chunk = zlib.crc32(data_payload)
        data = bytearray()
        data.extend(struct.pack("<I", chunk_size))
        data.extend(struct.pack("<I", crc32_of_chunk))
        data.extend(struct.pack("<I", sequence))
        cls._write_paket_raw_data(BESMessageTypes.ERASE_BURN_DATA, data)
        cls.serial_port.write(data_payload)

    @classmethod
    def _read_packet_raw(cls) -> bytearray:
        """
        Try and read a bes packet
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
                
        print(f"RX {len(packet): 2d} [{packet.hex(" ")}]")
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
        print(f"TX {len(pkt): 2d} [{pkt.hex(" ")}]")
        cls.serial_port.write(pkt)

    @classmethod
    def _write_paket_raw_data(cls, msg_type: BESMessageTypes, data: bytes):
        pkt = bytearray([
            0xBE,
            msg_type.value,
            cls.wr_seq,
            len(data)
        ])
        pkt.extend(data)
        pkt.append(cls._calculate_message_checksum(pkt))
        print(f"TX {len(pkt): 2d} [{pkt.hex(" ")}]")
        cls.serial_port.write(pkt)
        cls.wr_seq += 1

    @classmethod
    def _lookup_packet_length(cls, packet_id1: bytes, packet_id2: bytes):
        """
        This only stores the expected lengths for the messages coming from the MCU; for outgoing Tx messages that is left up to the sender functions
        """

        if packet_id1 == BESMessageTypes.SYNC.value:
            return 8
        if packet_id1 == BESMessageTypes.CODE_INFO.value:
            return 6
        if packet_id1 == BESMessageTypes.CODE.value:
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
@click.option("-p", "--payload", default="./payload/programmer2001.bin")
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

# TODO: add address and offset arguments
@cli.command()
@click.argument("filepath")
@click.option("-p", "--port", is_flag=False, default="/dev/ttyS6")
# @click.option("-a", "--address")
# @click.option("-s", "--size")
def burn(filepath, port):
    """"""
    print("NOT IMPLEMENTED")
    return
    print(f"beginning programming of {filepath} to device @ {port}")
    sys.stdout.flush()
    if port == None:
        print("Port is not set, using {}")
        for port in serial.tools.list_ports.comports():
            print(port)
    return
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    BESLink.run_programmer(port)
    BESLink.read_flash_info(port)
    # BESLink.run_get_cfgdata(port)
    BESLink.program_binary_file(port, filepath, address, size)
    port.close()

# TODO: add address and offset arguments
@cli.command()
@click.argument("filepath")
@click.argument("port_name")
def burn_watch(filepath, port_name):
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
        bes.run_programmer() # doesn't need a programmer to dump smthng, but ~x2 slow
        bes.read_flash_info() # unsupported without programmer
    bes.dump_to_file(address, size, filepath)
    bes.close_port()

@cli.command()
@click.argument("port_name")
@click.option("-s", "--sync", is_flag=True)
@click.option("-r", "--resync", is_flag=True)
def reboot(port_name, sync, resync):
    """"""
    print(f"Do reboot device @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    bes = BESLink(port)
    if sync:
        bes.wait_for_sync()
    bes.reboot()
    if resync:
        bes.wait_for_sync()
    bes.close_port()

@cli.command()
@click.argument("port_name")
@click.option("-s", "--sync", is_flag=True)
@click.option("-r", "--resync", is_flag=True)
def shutdown(port_name, sync, resync):
    """"""
    print(f"Do shutdown device @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    bes = BESLink(port)
    if sync:
        bes.wait_for_sync()
    bes.shutdown()
    if resync:
        bes.wait_for_sync()
    bes.close_port()

@cli.command()
@click.argument("port_name")
def flash_boot(port_name, sync):
    """ Exit from bootloader in bootloader mode or call boot_from_flash_reent in programmer mode """
    print(f"Exit bootloader/call boot from flash device @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    bes = BESLink(port)
    bes.flash_boot()
    bes.close_port()

@cli.command()
@click.argument("port_name")
@click.option("-s", "--sync", is_flag=True)
def bootmode(port_name, sync):
    """ Show bootmode bits """
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    bes = BESLink(port)
    if sync:
        bes.wait_for_sync()
    print(f"Get bootmode @ {port_name}")
    sys.stdout.flush()
    bits = bes.get_bootmode()
    print(f"Bootmode = 0x{bits:08x} ({bootmode_to_string(bits)})")
    bes.close_port()

@cli.command()
@click.argument("port_name")
@click.option("-e/-d", "--set/--clear", default=True)
@click.option("-b", "--bits", default=None)
@click.option("-s", "--sync", is_flag=True)
def mod_bootmode(port_name, bits, set, sync):
    """ Modify bootmode bits """
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    bes = BESLink(port)
    if sync:
        bes.wait_for_sync()
    if bits == None:
        print(f"Get bootmode @ {port_name}")
        sys.stdout.flush()
        bits = bes.get_bootmode()
        print(f"Bootmode = 0x{bits:08x} ({bootmode_to_string(bits)})")
    else:
        print(f"{("Set" if set else "Clear")} bootmode bits {bits} ({bootmode_to_string(bits)}) @ {port_name}")
        sys.stdout.flush()
        if set:
            bes.set_bootmode(bits)
        else:
            bes.clear_bootmode(bits)
    bes.close_port()

@cli.command()
@click.argument("port_name")
def en_jtag(port_name):
    print(f"Reboot with enabled jtag @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    bes = BESLink(port)
    bes.wait_for_sync()
    bes.set_bootmode(BESBootmodeTypes.JTAG_ENABLED.value)
    bes.reboot()
    bes.close_port()
    monitor(port_name)

@cli.command()
@click.argument("port_name")
@click.option("-s/ ", "--sync/--no-sync", is_flag=True, default=True)
@click.option("-f", "--force", is_flag=True)
def run_programmer(port_name, sync, force):
    """"""
    print(f"Querying for info @ {port_name}")
    sys.stdout.flush()
    bes = BESLink(serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=5))
    bes.run_programmer(sync, force)
    bes.close_port()

@cli.command()
@click.argument("port_name", default="COM6")
@click.option("-s", "--sync", is_flag=True)
def recovery(port_name, sync):
    """ Reboot in recovery mode """
    print(f"Reboot in recovery @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    bes = BESLink(port)
    if sync:
        bes.wait_for_sync()
    bes.set_bootmode(BESBootmodeTypes.REBOOT_FROM_CRASH.value) # REBOOT_FROM_CRASH - skip booting APP from flash @0x80000
    bes.reboot()
    bes.close_port()
    monitor(port_name)

@cli.command()
@click.argument("port_name", default="COM6")
@click.option("-f", "--force", is_flag=True)
def patch(port_name, force):
    """ Patch Xiaomi L05B/C to skip login DSA signature verification """
    print(f"Run patching @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    bes = BESLink(port)
    bes.run_programmer()
    # patch APP to skip login dsa verifying
    # 0x2C1C0650: 4F F0 FF 30 (MOV.W R0, #0xFFFFFFFF) -> 4F F0 01 00 (MOV.W R0, #1)
    orig = bytes([0x4F, 0xF0, 0xFF, 0x30])
    patch = bytes([0x4F, 0xF0, 0x01, 0x00])
    # patch = bytes([0xFF, 0xFF, 0xFF, 0xFF])
    patch_v2 = bytes([0x01, 0x20, 0x01, 0x20]) # MOVS R0, #1; MOVS R0, #1
    address = 0x2C1C0650
    real = bes.read_chunk(address, 4)
    # print(f"[{real.hex(" ")}] @ 0x{address:08x}")
    if orig != real and not force:
        if real == patch or real == patch_v2:
            print(f"Already patched: [{real.hex(" ")}] @ 0x{address:08x}")
        else:
            print(f"Required pattern [{orig.hex(" ")}] @ 0x{address:08x} not found, current data is [{real.hex(" ")}], skip patching")
    else:
        print(f"Try to patch [{orig.hex()}] -> [{patch.hex()}] @ 0x{address:08x}")
        bes.burn_short(address, patch)
        real = bes.read_chunk(address, 4)
        if real == patch:
            print(f"OK: [{real.hex(" ")}] @ 0x{address:08x}")
        else:
            print(f"Fail: [{real.hex(" ")}] @ 0x{address:08x}")
    bes.close_port()

@cli.command()
@click.argument("address")
@click.argument("size")
@click.argument("port_name", default="COM6")
@click.option("-s", "--sync", is_flag=True)
def md(port_name, address, size, sync):
    """ Memory display """
    print(f"Memory display {size} bytes @ {address} from device @ {port_name}")
    sys.stdout.flush()
    port = serial.Serial(port=port_name, baudrate=BES_BAUD, timeout=30)
    bes = BESLink(port)
    if sync:
        bes.wait_for_sync()
    address = atoi(address)
    size = atoi(size)
    if size > 0x100:
        size = 0x100
    data = bes.read_chunk(address, size)
    print(f"0x{address:08x}: ", data.hex(" "))
    bes.close_port()

@cli.command()
def list_ports():
    """Lists available com ports"""
    print("Detected Ports")
    sys.stdout.flush()
    for port in serial.tools.list_ports.comports():
        print(port)
    sys.stdout.flush()

def atoi(val) -> int:
    if type(val) is str:
            if str.lower(val[:2]) == "0x":
                val = int(val, 16)
            else:
                val = int(val)
    elif type(val) is not int:
        raise Exception(f"atoi: unsupported type {(type(val))}")
    return val

if __name__ == "__main__":
    cli()
