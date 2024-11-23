
# PiRC522 - complete driver
- For Raspberry Pi.
- You only need to enable **SPI** with **raspi-config**

## Documentation
From
[Miki Balboa ](https://github.com/miguelbalboa/rfid)
and
[Ondřej Ondryáš](https://github.com/ondryaso/pi-rc522)

## Not tested commands
I could not test those commands, because i do not have
- a **magic card**
- a **7 byte UID card**

### Private
```python
  status:str = __mifareOpenBackdoor(maxTryPerUnbrickCode:int)
```

### Public
```python
  success:bool = mifareSevenByteUidFirstInit(typeFn:int) # typeFn between 0 and 3
  success:bool = mifareChangeUid(data:list, maxTryPerUnbrickCode:int)
```
## Commands

### Initialization
```python
  pinReset:int = 4
  pinIrq:int = 5
  speed:int = 1000000
  antennaLevel:int = 3
  timeout:int = 0 # in seconds
  blockAddress:int = 5 # sector 1 block 1

  rfid:MFRC522 = MFRC522(pinIrq, pinReset, speed)
  antennaLevel:int = rfid.piccWaitTag(antennaLevel, timeout)
  success:bool = rfid.piccRequest(True|False) # True = rfid.MF_REQ_A | False = rfid.MF_WUP_A
  (success, uid, sak, piccType) = rfid.piccSelect()
  # success is a bool
  # uid is a list
  # sak is a hex (int)
  # piccType is a string
```

### Dump all data
```python
  sectorCount:int = 16
  key:list = [0xff,0xff,0xff,0xff,0xff,0xff]

  datas:list = rfid.mifareDump(rfid.MF_AUTH_A|B, key, sectorCount, uid)
  for data in datas:
      print (data)
```

### De/Authentication
```python
  key:list = [0xff,0xff,0xff,0xff,0xff,0xff]

  success:bool = rfid.mifareAuthenticate(rfid.MF_AUTH_A|B, blockAddress, key, uid)
  rfid.mifareDeauthenticate()
```

### Read/Write
```python
  data:list = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]

  success:bool = rfid.mifareWrite(blockAddress, data)
  (success, data) = rfid.mifareRead(blockAddress)
  # success is a bool
  # data is a list
```

### Value block
#### Format value block
- Always format the block with **mifareFormatValueBlock** before **increment**, **decrement**, **restore** and **transfer**
- Only need to format once, not more
```python
  initialValue:int = 3
  success:bool = mifareFormatValueBlock(blocAddress, initialValue)
```

#### Increment - Decrement - Restore
```python
  delta = 23
  
  success:bool = rfid.mifareIncrement(blockAddress, delta)
  success:bool = rfid.mifareDecrement(blockAddress, delta)
  (success, value) = rfid.mifareRestore(blockAddress)
  # success is a bool
  # value is an int
  
```

#### Transfer
Always use **mifareTransfer** after
- **increment**
- **decrement**
- but not required after **restore**, it's already did in driver
```python
  (success, value) = rfid.mifareTransfer(blockAddress)
  # success is a bool
  # value is an int
```

### If needed
#### antenna
```python
  gain:int = 4 # Have to be between 0 and 7
  enable:bool = True

  antennaLevel:int = rfid.antennaLevel(gain)
  rfid.antennaEnable(enable)
```

#### Reestablishing the connection
```python
  success:bool = rfid.piccReestablishCommunication(uid)
```

### Soft power
```python
  rfid.softPowerDown()
  success:bool = rfid.softPowerUp()
```

### Change Trailer for a section
```python
  sector:int = 3
  keys:list = [
    [0xff,0xff,0xff,0xff,0xff,0xff], # default for key A
    [0xff,0xff,0xff,0xff,0xff,0xff]  # default for key B
  ]
  accessBits:list = [
    0, # default for block 0
    0, # default for block 1
    0, # default for block 2
    1  # default for block 3 (trailer block)
  ]
  success:bool = mifareChangeTrailer(sector, keys, accessBits)
```

### Close
```python
  rfid.close()
  del rfid
```
