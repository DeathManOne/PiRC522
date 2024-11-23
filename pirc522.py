#!/usr/bin/env python
# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import spidev
import threading
import time

class MFRC522():
    __CMD_IDLE = 0x00
    __CMD_MEM = 0x01
    __CMD_RANDOM_ID = 0x02
    __CMD_CALCULATE_CRC = 0x03
    __CMD_TRANSMIT = 0x04
    __CMD_NO_CHANGE = 0x07
    __CMD_RECEIVE = 0x08
    __CMD_TRANSCEIVE = 0x0C
    __CMD_AUTHENTICATION = 0x0E
    __CMD_SOFT_RESET = 0x0F

    __MF_REQ_A = 0x26
    __MF_WUP_A = 0x52
    __MF_CL_1_ANTICOLLISION = [0x93, 0x20]
    __MF_CL_1_SELECT = [0x93, 0x70]
    __MF_CL_2_ANTICOLLISION = [0x95, 0x20]
    __MF_CL_2_SELECT = [0x95, 0x70]
    __MF_CL_3_ANTICOLLISION = [0x97, 0x20]
    __MF_CL_3_SELECT = [0x97, 0x70]
    __MF_HALT = [0x50, 0x00]
    __MF_AUTH_A = 0x60
    __MF_AUTH_B = 0x61
    __MF_UID_PERSONALIZE = 0x40
    __MF_SET_MOD_TYPE = 0x43
    __MF_READ = 0x30
    __MF_WRITE = 0xA0
    __MF_DECREMENT = 0xC0
    __MF_INCREMENT = 0xC1
    __MF_RESTORE = 0xC2
    __MF_TRANSFER = 0xB0
    __MF_UL_WRITE = 0XA2

    __RC_COMMAND = 0x01
    __RC_COM_I_EN = 0x02
    __RC_DIV_I_EN = 0x03
    __RC_COM_IRQ = 0x04
    __RC_DIV_IRQ = 0x05
    __RC_ERROR = 0x06
    __RC_STATUS_1 = 0x07
    __RC_STATUS_2 = 0x08
    __RC_FIFO_DATA = 0x09
    __RC_FIFO_LEVEL = 0x0A
    __RC_WATER_LEVEL = 0x0B
    __RC_CONTROL = 0x0C
    __RC_BIT_FRAMING = 0x0D
    __RC_COLL = 0x0E

    __RC_MODE = 0x11
    __RC_TX_MODE = 0x12
    __RC_RX_MODE = 0x13
    __RC_TX_CONTROL = 0x14
    __RC_TX_ASK = 0x15
    __RC_TX_SEL = 0x16
    __RC_RX_SEL = 0x17
    __RC_RX_THRESHOLD = 0x18
    __RC_DEMOD = 0x19
    __RC_MF_TX = 0x1C
    __RC_MF_RX = 0x1D
    __RC_SERIAL_SPEED = 0x1F

    __RC_CRC_RESULT_HIGH = 0x21
    __RC_CRC_RESULT_LOW = 0x22
    __RC_MOD_WIDTH = 0x24
    __RC_RFC_FG = 0x26
    __RC_GS_N = 0x27
    __RC_CW_GS_P = 0x28
    __RC_MOD_GS_P = 0x29
    __RC_T_MOD = 0x2A
    __RC_T_PRESCALER = 0x2B
    __RC_T_RELOAD_HIGH = 0x2C
    __RC_T_RELOAD_LOW = 0x2D
    __RC_T_COUNTER_VAL_HIGH = 0x2E
    __RC_T_COUNTER_VAL_LOW = 0x2F

    __RC_TEST_SEL_1 = 0x31
    __RC_TEST_SEL_2 = 0x32
    __RC_TEST_PIN_EN = 0x33
    __RC_TEST_PIN_VALUE = 0x34
    __RC_TEST_BUS = 0x35
    __RC_AUTO_TEST = 0x36
    __RC_VERSION = 0x37
    __RC_ANALOG_TEST = 0x38
    __RC_TEST_DAC_1 = 0x39
    __RC_TEST_DAC_2 = 0x3A
    __RC_TEST_ADC = 0x3B

    def __init__(self, pinIrq:int, pinReset:int, maxSpeedHz:int) -> None:
        self.__PIN_IRQ:int = pinIrq
        self.__IRQ:threading = threading.Event()

        self.__SPI:spidev = spidev.SpiDev()
        self.__SPI.open(0, 0)
        self.__SPI.max_speed_hz = maxSpeedHz

        GPIO.setup(pinReset, GPIO.OUT)
        GPIO.output(pinReset, GPIO.HIGH)
        GPIO.setup(pinIrq, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pinIrq, GPIO.FALLING, callback=self.__irqCallback)

    def __irqCallback(self, notUsed=None):
        self.__IRQ.set()

    def __spiTransfer(self, data:list) -> list:
        try: r:list = self.__SPI.xfer2(data)
        except Exception: r:list = []
        finally: return r

    def __spiRead(self, address:int) -> int:
        return self.__spiTransfer([(((address << 1) & 0x7E) | 0x80), 0])[1]

    def __spiWrite(self, address:int, data:int) -> None:
        self.__spiTransfer([(address << 1) & 0x7E, data])

    def __initialize(self, antennaLevel:int) -> int:
        self.__softReset()
        self.__spiWrite(self.__RC_TX_MODE, 0x00)
        self.__spiWrite(self.__RC_RX_MODE, 0x00)
        self.__spiWrite(self.__RC_MOD_WIDTH, 0x26)
        self.__spiWrite(self.__RC_T_MOD, 0x8D)
        self.__spiWrite(self.__RC_T_PRESCALER, 0x3E)
        self.__spiWrite(self.__RC_T_RELOAD_HIGH, 0x03)
        self.__spiWrite(self.__RC_T_RELOAD_LOW, 0xE8)
        self.__spiWrite(self.__RC_TX_ASK, 0x40)
        self.__spiWrite(self.__RC_MODE, 0x3D)
        self.antennaEnable(True)
        return self.antennaLevel(antennaLevel)

    def __BitmaskSet(self, address:int, mask:int) -> None:
        current:int = self.__spiRead(address)
        self.__spiWrite(address, current | mask)

    def __BitmaskClear(self, address:int, mask:int) -> None:
        current:int = self.__spiRead(address)
        self.__spiWrite(address, current & (~mask))

    def __crcAppend(self, data:list) -> tuple[bool, list]:
        (success, crc) = self.__crcCalculate(data)
        if not success: return (success, data)

        data.append(crc[0])
        data.append(crc[1])
        return (True, data)

    def __crcCalculate(self, data:list) -> tuple[bool, list]:
        self.__spiWrite(self.__RC_COMMAND, self.__CMD_IDLE)
        self.__spiWrite(self.__RC_DIV_IRQ, 0x04)
        self.__BitmaskSet(self.__RC_FIFO_LEVEL, 0x80)

        for d in data:
            self.__spiWrite(self.__RC_FIFO_DATA, d)
        self.__spiWrite(self.__RC_COMMAND, self.__CMD_CALCULATE_CRC)

        for timeout in range(5000, 0, -1):
            irq:int = self.__spiRead(self.__RC_DIV_IRQ)
            if (irq & 0x04) != 0: break
            if timeout <= 1: return (False, [])
        self.__spiWrite(self.__RC_COMMAND, self.__CMD_IDLE)

        crc:list = []
        crc.append(self.__spiRead(self.__RC_CRC_RESULT_LOW))
        crc.append(self.__spiRead(self.__RC_CRC_RESULT_HIGH))

        if len(crc) != 2: return (False, [])
        return (True, crc)

    def __crcCheck(self, data:list) -> bool:
        (success, crc) = self.__crcCalculate(data[:-2])
        if not success: return False
        if data[-2] != crc[0]: return False
        if data[-1] != crc[1]: return False
        return True

    def __piccCommunication(self, command:int, data:list, framingBit:int, checkCrc:bool) -> tuple[str, list, int]:
        if command == self.__CMD_AUTHENTICATION: irqWait:int = 0x10
        elif command == self.__CMD_TRANSCEIVE: irqWait:int = 0x30
        else: irqWait:int = 0x00

        self.__spiWrite(self.__RC_COMMAND, self.__CMD_IDLE)
        self.__spiWrite(self.__RC_COM_IRQ, 0x7F)
        self.__BitmaskSet(self.__RC_FIFO_LEVEL, 0x80)
        self.__spiWrite(self.__RC_BIT_FRAMING, framingBit)

        for d in data:
            self.__spiWrite(self.__RC_FIFO_DATA, d)
        self.__spiWrite(self.__RC_COMMAND, command)

        if command == self.__CMD_TRANSCEIVE:
            self.__BitmaskSet(self.__RC_BIT_FRAMING, 0x80)

        for timeout in range(2000, 0, -1):
            irq:int = self.__spiRead(self.__RC_COM_IRQ)
            if (irq & irqWait) != 0: break
            if (irq & 0x01) != 0: return ("PICC_TIMEOUT", [], 0)
            if timeout <= 1: return ("SOFT_TIMEOUT", [], 0)
        self.__BitmaskClear(self.__RC_BIT_FRAMING, 0x80)

        error:int = self.__spiRead(self.__RC_ERROR)
        if (error & 0x01) != 0: return ("PROTOCOL_ERROR", [], 0)
        if (error & 0x02) != 0: return ("PARITY_ERROR", [], 0)
        if (error & 0x04) != 0: return ("CRC_ERROR", [], 0)
        if (error & 0x10) != 0: return ("BUFFER_OVERFLOW", [], 0)

        newData:list = []
        newDataLength:int = self.__spiRead(self.__RC_FIFO_LEVEL)
        while newDataLength > 0:
            newData.append(self.__spiRead(self.__RC_FIFO_DATA))
            newDataLength -= 1

        validBits:int = self.__spiRead(self.__RC_CONTROL) & 0x07
        if len(newData) > 0 and checkCrc:
            if len(newData) == 1 and validBits == 4: return ("MF_NACK", [], 0)
            if len(newData) < 2 or validBits != 0: return ("CRC_ERROR", [], 0)
            if not self.__crcCheck(newData): return ("CRC_CHECK_ERROR", [], 0)

        if (error & 0x08) != 0: status = "COLLISION"
        else: status = "OK"
        return (status, newData, validBits)

    def __piccHalt(self) -> bool:
        (success, buffer) = self.__crcAppend(self.__MF_HALT.copy())
        if not success: return False

        self.__BitmaskClear(self.__RC_STATUS_2, 0x80)
        (status, data, validBits) = self.__piccCommunication(self.__CMD_TRANSCEIVE, buffer, 0x00, False)
        self.mifareDeauthenticate()

        return status == "PICC_TIMEOUT" or status == "SOFT_TIMEOUT"

    def __piccReset(self) -> None:
        self.__spiWrite(self.__RC_COMMAND, self.__CMD_IDLE)
        self.mifareDeauthenticate()
        self.__BitmaskClear(self.__RC_COLL, 0x80)

    def __piccType(self, sak:int) -> str:
        sak:int = sak & 0x7F
        match sak:
            case 0x00:
                return "MIFARE_UL"
            case 0x01:
                return "TNP3XXX"
            case 0x04:
                return "NOT_COMPLETE"
            case 0x08:
                return "MIFARE_1K"
            case 0x09:
                return "MIFARE_MINI"
            case 0x10 | 0x11:
                return "MIFARE_PLUS"
            case 0x18:
                return "MIFARE_4K"
            case 0x20:
                return "ISO_14443_4"
            case 0x40:
                return "ISO_18092"
            case _:
                return "UNKNOWN"

    def __mifareTransceive(self, data:list, acceptTimeout:bool) -> str:
        (success, data) = self.__crcAppend(data)
        if not success: return "CRC_ERROR"

        (status, newData, validBits) = self.__piccCommunication(self.__CMD_TRANSCEIVE, data, 0x00, False)
        if acceptTimeout and status == "PICC_TIMEOUT": return "OK"
        if acceptTimeout and status == "SOFT_TIMEOUT": return "OK"
        if status != "OK":  return status

        if len(newData) != 1: return "DATA_ERROR"
        if validBits != 4:  return "VALID_BITS_ERROR"
        if newData[0] != 0x0A: return "ACK_ERROR"
        return "OK"

    def __mifareGetValue(self, blockAddress:int) -> tuple[bool, int]:
        (success, data) = self.mifareRead(blockAddress)
        if not success: return (False, 0)
        return (True, (data[3] << 24 ) + (data[2] << 16) + (data[1] << 8) + data[0])

    def __mifareTwoStep(self, command:int, blockAddress:int, value:int) -> str:
        status = self.__mifareTransceive([command, blockAddress], False)
        if status != "OK": return status

        data:list = [value & 0xFF,
                     (value >> 8) & 0xFF,
                     (value >> 16) & 0xFF,
                     (value >> 24) & 0xFF]
        return self.__mifareTransceive(data, True)

    def __mifareAccessBits(self, accessBits:list) -> tuple[bool, list]:
        if len(accessBits) != 4: return (False, [])
        for accessBit in accessBits:
            if accessBit < 0 or accessBit > 7: return (False, [])

        c1:int = (accessBits[3] & 4) << 1 | (accessBits[2] & 4) << 0 | (accessBits[1] & 4) >> 1 | (accessBits[0] & 4) >> 2
        c2:int = (accessBits[3] & 2) << 2 | (accessBits[2] & 2) << 1 | (accessBits[1] & 2) << 0 | (accessBits[0] & 2) >> 1
        c3:int = (accessBits[3] & 1) << 3 | (accessBits[2] & 1) << 2 | (accessBits[1] & 1) << 1 | (accessBits[0] & 1) << 0

        data:list = [(~c2 & 0xF) << 4 | (~c1 & 0xF)]  # byte 6
        data.append(c1 << 4 | (~c3 & 0xF)) # byte 7
        data.append(c3 << 4 | c2) # byte 8

        if len(data) != 3: return (False, [])
        return (True, data)

    def __mifareOpenBackdoor(self, maxTryPerUnbrickCode:int) -> str:
        unbrickCodes:list = [
            [0x01, 0x02, 0x03, 0x04, 0x04, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [0x01, 0x23, 0x45, 0x67, 0x00, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [0x12, 0x34, 0x56, 0x78, 0x08, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ]
        for unbrickCode in unbrickCodes:
            for passing in range(maxTryPerUnbrickCode + 1):
                if not self.__piccHalt(): return "ERROR_HALT"

                status = self.__mifareTransceive([self.__MF_UID_PERSONALIZE], False)
                if status == "OK":  return self.__mifareTransceive([self.__MF_SET_MOD_TYPE], False)
                if passing >= maxTryPerUnbrickCode: break
                if self.mifareWrite(0, unbrickCode): continue
        return "UID_BRICKED"

    def __softReset(self) -> None:
        self.__spiWrite(self.__RC_COMMAND, self.__CMD_SOFT_RESET)
        time.sleep(0.05)
        self.__spiWrite(self.__RC_T_MOD, 0x87)
        self.__spiWrite(self.__RC_T_PRESCALER, 0xFF)
        self.__spiWrite(self.__RC_TX_ASK, 0x40)
        self.__spiWrite(self.__RC_MODE, 0x3D)

    def close(self) -> None:
        self.mifareDeauthenticate()
        self.__SPI.close()
        GPIO.remove_event_detect(self.__PIN_IRQ)
        del self.__PIN_IRQ
        del self.__IRQ
        del self.__SPI

    def antennaLevel(self, level:int) -> int:
        if level < 0: level = 1
        if level > 7: level = 7
        self.__spiWrite(self.__RC_RFC_FG, level << 4)
        return (self.__spiRead(self.__RC_RFC_FG) & 0x70) >> 4

    def antennaEnable(self, enable:bool) -> None:
        if enable: self.__BitmaskSet(self.__RC_TX_CONTROL, 0x03)
        else: self.__BitmaskClear(self.__RC_TX_CONTROL, 0x03)

    def piccWaitTag(self, antennaLevel:int, timeout:int) -> int:
        self.__IRQ.clear()
        startTime:time = time.time()

        while timeout == 0 or ((time.time() - startTime) < timeout):
            self.__initialize(antennaLevel)
            self.__spiWrite(self.__RC_COM_IRQ, 0x00)
            self.__spiWrite(self.__RC_COM_I_EN, 0xA0)
            self.__spiWrite(self.__RC_FIFO_DATA, 0x26)
            self.__spiWrite(self.__RC_COMMAND, 0x0C)
            self.__spiWrite(self.__RC_BIT_FRAMING, 0x87)
            if self.__IRQ.wait(0.1): break

        self.__IRQ.clear()
        return self.__initialize(antennaLevel)

    def piccReestablishCommunication(self, uid:int) -> bool:
        if not self.__piccHalt(): return False
        if not self.piccRequest(True): return False
        try :
            (success, newUid, newSak, newType) = self.piccSelect()
            return success and newUid == uid
        except Exception: return False

    def piccRequest(self, wakeUp:bool) -> bool:
        command:list = [self.__MF_REQ_A]
        if wakeUp: command = [self.__MF_WUP_A]

        self.__piccReset()
        (status, data, validBits) = self.__piccCommunication(self.__CMD_TRANSCEIVE, command, 0x07, False)
        return status == "OK" and validBits == 0

    def piccSelect(self) -> tuple[bool, list, int, str]:
        cascades:list = [self.__MF_CL_1_ANTICOLLISION[0], self.__MF_CL_2_ANTICOLLISION[0], self.__MF_CL_3_ANTICOLLISION[0]]
        uid:list = []
        sak:int = 0x04

        self.__piccReset()
        for cascade in cascades:
            buffer:list = [cascade]
            levelKnownBits:int = 0
            timeout:bool = True

            for r in range(32):
                if levelKnownBits >= 32:
                    buffer = buffer[0:6]
                    bufferDirty:bool = len(buffer) != 6
                    bufferDirty |= any(not isinstance(byte, int) for byte in buffer)
                    if bufferDirty:
                        buffer = [cascade]
                        levelKnownBits = 0
                        continue
                    txLastBits:int = 0

                    buffer[1] = 0x70
                    if len(buffer) < 7: buffer.append(buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5])
                    else: buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5]
                    (success, buffer) = self.__crcAppend(buffer)
                    if not success: return (False, ["ERROR_CRC"], sak, self.__piccType(sak))
                else:
                    txLastBits:int = levelKnownBits % 8
                    uidFullByte:int = levelKnownBits // 8
                    allFullByte:int = 2 + uidFullByte

                    if len(buffer) < 2: buffer.append((allFullByte << 4) + txLastBits)
                    else: buffer[1] = (allFullByte << 4) + txLastBits
                    bufferLength:int = allFullByte + (1 if txLastBits > 0 else 0)
                    buffer = buffer[0:bufferLength + 1]

                framingBit:int = (txLastBits << 4) + txLastBits
                (status, data, validBits) = self.__piccCommunication(self.__CMD_TRANSCEIVE, buffer, framingBit, False)
                if status != "OK" and status != "COLLISION": return (False, [status], sak, self.__piccType(sak))
                if len(data) == 0: return (False, ["DATA_EMPTY"], sak, self.__piccType(sak))

                if levelKnownBits < 32:
                    if txLastBits != 0: buffer[-1] |= data.pop(0)
                    buffer += data

                if status == "COLLISION":
                    collision:int = self.__spiRead(self.__RC_COLL)
                    if (collision & 0x20) != 0: return (False, ["COLLISION_ERROR"], sak, self.__piccType(sak))

                    collisionPosition:int = collision & 0x1F
                    if collisionPosition == 0: collisionPosition = 32
                    if collisionPosition <= levelKnownBits: return (False, ["COLLISION_POSITION_ERROR"], sak, self.__piccType(sak))

                    levelKnownBits = collisionPosition
                    buffer[-1] |= 1 << ((levelKnownBits - 1) % 8)
                else:
                    if levelKnownBits >= 32:
                        timeout = False
                        break
                    levelKnownBits = 32

            if timeout: return (False, ["SOFT_TIMEOUT"], sak, self.__piccType(sak))
            if len(data) != 3 or validBits != 0: return (False, ["SAK_ERROR"], sak, self.__piccType(sak))
            if not self.__crcCheck(data): return (False, ["CRC_CHECK_ERROR"], sak, self.__piccType(sak))

            sak = data[0]
            uid = buffer[2:-2]
            if (sak & 0x04) == 0: break
        return (True, uid, sak, self.__piccType(sak))

    def mifareAuthenticate(self, authKeyType:int, blockAddress:int, key:list, uid:list) -> bool:
        if authKeyType != self.__MF_AUTH_A and authKeyType != self.__MF_AUTH_B: return False
        if len(key) < 6: return False
        if len(uid) < 4 : return False
        if len(key) > 6: key = key[0:6]
        if len(uid) > 4: uid = uid[0:4]

        buffer:list = [authKeyType, blockAddress]
        for k in key: buffer.append(k)
        for id in uid: buffer.append(id)
        (status, data, validBits) = self.__piccCommunication(self.__CMD_AUTHENTICATION, buffer, 0x00, False)

        if status != "OK": return False
        return (self.__spiRead(self.__RC_STATUS_2) & 0x08) != 0

    def mifareDeauthenticate(self) -> None:
        self.__BitmaskClear(self.__RC_STATUS_2, 0x08)

    def mifareSevenByteUidFirstInit(self, typeFn:int) -> bool:
        match typeFn:
            case 0: uidFn:int = 0x00
            case 1: uidFn:int = 0x40
            case 2: uidFn:int = 0x20
            case 3: uidFn:int = 0x60
            case _: return False
        (success, buffer) = self.__crcAppend([self.__MF_UID_PERSONALIZE, uidFn])
        if not success: return False
        return self.__mifareTransceive(buffer, False) == "OK"

    def mifareDecrement(self, blockAddress:int, delta:int) -> bool:
        return self.__mifareTwoStep(self.__MF_DECREMENT, blockAddress, delta) == "OK"

    def mifareIncrement(self, blockAddress:int, delta:int) -> bool:
        return self.__mifareTwoStep(self.__MF_INCREMENT, blockAddress, delta) == "OK"

    def mifareRestore(self, blockAddress:int) -> tuple[bool, bool, int]:
        status = self.__mifareTwoStep(self.__MF_RESTORE, blockAddress, 0)
        if status != "OK":  return (False, 0)

        (success, value) = self.mifareTransfer(blockAddress)
        if not success: return (False, 0)
        return self.__mifareGetValue(blockAddress)

    def mifareTransfer(self, blockAddress:int) -> tuple[bool, int]:
        status = self.__mifareTransceive([self.__MF_TRANSFER, blockAddress], False)
        if status != "OK":  return (False, 0)
        return self.__mifareGetValue(blockAddress)

    def mifareFormatValueBlock(self, blockAddress:int, value:int) -> bool:
        buffer:list = [value & 0xFF]
        buffer.append((value >> 8) & 0xFF)
        buffer.append((value >> 16) & 0xFF)
        buffer.append((value >> 24) & 0xFF)
        buffer.append(~buffer[0])
        buffer.append(~buffer[1])
        buffer.append(~buffer[2])
        buffer.append(~buffer[3])
        buffer.append(buffer[0])
        buffer.append(buffer[1])
        buffer.append(buffer[2])
        buffer.append(buffer[3])
        buffer.append(blockAddress)
        buffer.append(~blockAddress)
        buffer.append(buffer[12])
        buffer.append(buffer[13])
        return self.mifareWrite(blockAddress, buffer)

    def mifareRead(self, blockAddress:int) -> tuple[bool, list]:
        (success, buffer) = self.__crcAppend([self.__MF_READ, blockAddress])
        if not success: return (False, ["CRC_ERROR"])

        (status, data, validBits) = self.__piccCommunication(self.__CMD_TRANSCEIVE, buffer, 0x00, True)
        if status != "OK": return (False, [status])
        return (True, data[:-2])

    def mifareWrite(self, blockAddress:int, data:list) -> bool:
        if len(data) < 16: return False
        if len(data) > 16: data = data[0:16]

        status = self.__mifareTransceive([self.__MF_WRITE, blockAddress], False)
        if status != "OK": return False
        return self.__mifareTransceive(data, False) == "OK"

    def mifareChangeUid(self, data:list, maxTryPerUnbrickCode:int) -> bool:
        if len(data) < 16: return False
        if len(data) > 16: data = data[0:16]
        if data[4] != data[0] ^ data[1] ^ data[2] ^ data[3]: return False

        if self.__mifareOpenBackdoor(maxTryPerUnbrickCode) != "OK": return False
        if not self.mifareWrite(0, data): return False

        if self.__piccHalt():
            self.piccRequest(True)
        return True

    def mifareChangeTrailer(self, sector:int, keys:list, accessBits:list) -> bool:
        if len(keys) != 2: return False
        if len(accessBits) != 4: return False
        for key in keys:
            if len(key) != 6: return False
        for accessBit in accessBits:
            if accessBit < 0 or accessBit > 7: return False

        (success, datas) = self.__mifareAccessBits(accessBits)
        if not success: return False

        buffer:list = []
        for key in keys[0]: buffer.append(key)
        for accessBit in datas: buffer.append(accessBit)
        buffer.append(datas[0] ^ datas[1] ^ datas[2])
        for key in keys[1]: buffer.append(key)

        block:int = (sector * 4) + 3
        return self.mifareWrite(block, buffer)

    def mifareDump(self, authKeyType:int, key:list, sectorCount:int, uid:list) -> list:
        datas:list = []

        for block in range(sectorCount * 4):
            byte:int = block % 4
            sector:int = (block - byte) // 4

            if byte == 0:
                if not self.mifareAuthenticate(authKeyType, block, key, uid): continue

            (success, data) = self.mifareRead(block)
            datas.append(f"{sector:3d}" + "-" + str(byte) + " " + str(data))
        return datas

    def softPowerDown(self) -> None:
        self.__BitmaskSet(self.__RC_COMMAND, 0x10)

    def softPowerUp(self) -> bool:
        self.__BitmaskClear(self.__RC_COMMAND, 0x10)
        startTime:time = time.time()
        timeout:int = 60

        while (time.time() - startTime) < timeout:
            if (self.__spiRead(self.__RC_COMMAND)) & 0x10 != 0x10: return True
            time.sleep(0.1)
        return False
