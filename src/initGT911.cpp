#include "initGT911.h"
//#include <Wire.h>

#ifndef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR
#endif


// Interrupt handling
volatile bool initGT911::gt911IRQ = false;

#if defined(ESP8266)
void ICACHE_RAM_ATTR _gt911_irq_handler()
{
  noInterrupts();
  gt911IRQ = true;
  interrupts();
}
#elif defined(ESP32)
void IRAM_ATTR _gt911_irq_handler()
{
  gt911IRQ = true;
}
#else
void initGT911::_gt911_irq_handler()
{
  noInterrupts();
  gt911IRQ = true;
  interrupts();
}
#endif

void initGT911::setInterruptHandler(void (*_isr)(void)) 
{
  if (nullptr == _isr)
    detachInterrupt(_intPin);
  else       
    attachInterrupt(_intPin, _isr, FALLING);
}

// default delay
void initGT911::_gt911_delay(uint32_t ms)
{
  delay(ms);
}

initGT911::initGT911(I2CMaster *twi, uint8_t addr) : _wire(twi ? twi : &Master)
{
  _addr = addr;
}

void initGT911::reset()
{
  delay(1);

  pinMode(_intPin, OUTPUT);
  pinMode(_rstPin, OUTPUT);

  digitalWrite(_intPin, LOW);
  digitalWrite(_rstPin, LOW);

  delay(11);

  digitalWrite(_intPin, _addr == GT911_I2C_ADDR_28);

  delayMicroseconds(110);
  pinMode(_rstPin, INPUT);

  delay(6);
  digitalWrite(_intPin, LOW);

  delay(51);
}

bool initGT911::finish(uint32_t timeout_millis)
{
  elapsedMillis timeout;
  while (timeout < timeout_millis) {
      if (nullptr != async_wait)
        async_wait(context);
      else        
        yield();
      if (_wire->finished()) {
          return false;
      }
  }
  return true;
}

void initGT911::i2cStart(uint16_t reg)
{
  /*
  _wire->beginTransmission(_addr);
  _wire->write(reg >> 8);
  _wire->write(reg & 0xFF);
  */
 uint8_t buf[2]{(uint8_t)(reg>>8),(uint8_t)(reg&0xFF)};
 _wire->write_async(_addr,buf,2,false);
 finish();
}

bool initGT911::write(uint16_t reg, uint8_t data)
{
  i2cStart(reg);
  /*
  _wire->write(data);
  return _wire->endTransmission() == 0;
  */
  _wire->write_async(_wire->NO_RESTART,&data,1,true);
  return endTransmission(1);
}

uint8_t initGT911::read(uint16_t reg)
{
  i2cStart(reg);
  /*
  if (_wire->endTransmission() != 0)
  {
    GT911_Log("I2C read single byte: endTransmission error");
    return 0;
  }

  uint8_t got = _wire->requestFrom((int)_addr, 1);
  if (got == 0 || !_wire->available())
  {
    GT911_Log("I2C read single byte: no data");
    return 0;
  }
  return _wire->read();
  */
  uint8_t result;
  _wire->read_async(_addr,&result,1,true);
  finish();
  return result;
}

bool initGT911::writeBytes(uint16_t reg, uint8_t *data, uint16_t size)
{
  i2cStart(reg);
  /*
  for (uint16_t i = 0; i < size; i++)
  {
    _wire->write(data[i]);
  }
  return _wire->endTransmission() == 0;
  */
  _wire->write_async(_wire->NO_RESTART,data,size,true);
  return endTransmission(size);
}

bool initGT911::readBytes(uint16_t reg, uint8_t *data, uint16_t size)
{
  if (size == 0)
    return true;

  // Start write of register pointer
  i2cStart(reg);
  /*
  if (_wire->endTransmission() != 0)
  {
    GT911_Log("readBytes I2C error: endTransmission");
    return false; // I2C error
  }
  */
  uint16_t index = 0;
  const unsigned long overallTimeout = 20;//00; // ms
  unsigned long startTime = millis();
  int addr = _addr; // first read needs re-start

  while (index < size)
  {
    if ((millis() - startTime) > overallTimeout)
    {
      GT911_Log("readBytes timeout");
      return false;
    }

    uint8_t req = (uint8_t)min<uint16_t>(size - index, I2C_BUFFER_LENGTH);
    bool do_stop = req == (size - index);
    //uint8_t got = _wire->requestFrom((int)_addr, (int)req);
    _wire->read_async(addr,data+index,req,do_stop); // send stop on last request
    addr = _wire->NO_RESTART;
    finish();
    uint8_t got = _wire->get_bytes_transferred();
    GT911_Logf("req %d, got %d bytes; %sstop requested", req, got, do_stop?"":"no ");
    if (got != 0)
      index += got;
    /*
    if (got == 0)
    {
      // small backoff and retry once
      delay(5);
      yield();
      got = _wire->requestFrom((int)_addr, (int)req);
      if (got == 0)
      {
        GT911_Log("I2C read error: no data available after retry");
        return false;
      }
    }

    uint8_t readCount = 0;
    while (readCount < got && index < size)
    {
      if (_wire->available())
      {
        data[index++] = _wire->read();
        readCount++;
      }
      else
      {
        // unexpected early end; break to outer loop to retry remaining
        break;
      }
    }
    */
    // small yield to avoid WDT and give bus time
    delayMicroseconds(50);
    yield(); // TODO: fix this!
  }

  GT911_Logf("readBytes: read %d bytes", index);
  return index == size;
}

uint8_t initGT911::calcChecksum(uint8_t *buf, uint8_t len)
{
  uint8_t ccsum = 0;
  for (uint8_t i = 0; i < len; i++)
  {
    ccsum += buf[i];
  }

  return (~ccsum) + 1;
}

uint8_t initGT911::readChecksum()
{
  return read(GT911_REG_CHECKSUM);
}

int8_t initGT911::readTouches()
{
  uint32_t timeout = millis() + 20;
  GT911_Logf("Reading touches:");
  do
  {
    uint8_t flag = read(GT911_REG_COORD_ADDR);
    if ((flag & 0x80) && ((flag & 0x0F) < GT911_MAX_CONTACTS))
    {
      GT911_Logf("GT911_REG_COORD_ADDR: %02X", flag);
      write(GT911_REG_COORD_ADDR, 0);
      return flag & 0x0F;
    }
    internal_delay(1);
  } while (millis() < timeout);

  return 0;
}

bool initGT911::readTouchPoints()
{
  bool result = readBytes(GT911_REG_COORD_ADDR + 1, (uint8_t *)_points, sizeof(GTPoint) * GT911_MAX_CONTACTS);
  /*
    if (result)
    {
      for (uint8_t i = 0; i < GT911_MAX_CONTACTS; i++)
      {
        if (_rotation == Rotate::_180)
        {
          _points[i].x = _info.xResolution - _points[i].x;
          _points[i].y = _info.yResolution - _points[i].y;
        }
      }
    }
  */
  return result;
}

bool initGT911::begin(int8_t intPin, int8_t rstPin, uint32_t clk)
{
  _intPin = intPin;
  _rstPin = rstPin;

  if (_rstPin > 0)
  {
    delay(300);
    reset();
    delay(200);
  }
  _wire->begin(clk);
  /*
  _wire->setClock(clk);
  _wire->beginTransmission(_addr);
  if (_wire->endTransmission() == 0)
  {
    readInfo(); // Need to get resolution to use rotation
*/
  if (nullptr != readInfo())
  {
    if (intPin > 0)
    {
      pinMode(_intPin, INPUT);
      setInterruptHandler(); // set to default
    }
    return true;
  }
  return false;
}

bool initGT911::productID(uint8_t *buf, uint8_t len)
{
  if (len < 4)
  {
    return false;
  }

  memset(buf, 0, 4);
  return readBytes(GT911_REG_ID, buf, 4);
}

GTConfig *initGT911::readConfig()
{
  readBytes(GT911_REG_CFG, (uint8_t *)&_config, sizeof(_config));

  if (readChecksum() == calcChecksum((uint8_t *)&_config, sizeof(_config)))
  {
    _configLoaded = true;
    return &_config;
  }
  return nullptr;
}

bool initGT911::updateConfig()
{
  uint8_t checksum = calcChecksum((uint8_t *)&_config, sizeof(_config));

  if (_configLoaded && readChecksum() != checksum)
  { // Config is different
    writeBytes(GT911_REG_CFG, (uint8_t *)&_config, sizeof(_config));

    uint8_t buf[2] = {checksum, 1};
    writeBytes(GT911_REG_CHECKSUM, buf, sizeof(buf));
    delay(10); // Wait for config to be applied
    return true;
  }
  return false;
}

GTInfo *initGT911::readInfo()
{
  GTInfo* result = nullptr;
  if (readBytes(GT911_REG_DATA, (uint8_t *)&_info, sizeof(_info)))
    result = &_info;
Serial.printf("Info at %08X\n", (uint32_t) result);    
  return result;
}

uint8_t initGT911::touched(uint8_t mode)
{
  bool irq = false;
  if (mode == GT911_MODE_INTERRUPT)
  {
    irq = gt911IRQ;
    gt911IRQ = false;
  }
  else if (mode == GT911_MODE_POLLING)
  {
    irq = true;
  }

  uint8_t contacts = 0;
  if (irq)
  {
    contacts = readTouches();

    if (contacts > 0)
    {
      readTouchPoints();
    }
  }

  return contacts;
}

GTPoint initGT911::getPoint(uint8_t num)
{
  return _points[num];
}

GTPoint *initGT911::getPoints()
{
  return _points;
}

void initGT911::setupDisplay(uint16_t xRes, uint16_t yRes, rotation_t rotation)
{
  GTConfig *cfg = readConfig();
  if (cfg == nullptr)
  {
    GT911_Log("Config Read Error");
    return;
  }

  cfg->hSpace = (5 | (5 << 4)); // Set horizontal space
  cfg->vSpace = (5 | (5 << 4)); // Set vertical space

  cfg->xResolution = xRes; // Set X resolution
  cfg->yResolution = yRes; // Set Y resolution

  if (rotation == initGT911_ROTATION_0 || rotation == initGT911_ROTATION_180)
  {
    cfg->moduleSwitch1 &= ~(1 << 7); // X-axis set 0
    cfg->moduleSwitch1 &= ~(1 << 6); // Y-axis set 0
  }
  else
  {
    cfg->moduleSwitch1 |= (1 << 7); // X-axis set 1
    cfg->moduleSwitch1 |= (1 << 6); // X-axis set 1
  }
  updateConfig();
}
