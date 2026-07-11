/*
   initGT911 library V1.0.1
   Created by Milad Nikpendar
   Date: 2025-08-18
*/
#ifndef INIT_GT911_H
#define INIT_GT911_H

#include <Arduino.h>
#include <i2c_driver_wire.h>
#include <imx_rt1060/imx_rt1060_i2c_driver.h>

#if !defined(I2C_BUFFER_LENGTH)
#if defined(I2C_DRIVER_WIRE_H)
#define I2C_BUFFER_LENGTH 256 // MAX_MASTER_READ_LENGTH // async I2C library definition
#else
#define I2C_BUFFER_LENGTH BUFFER_LENGTH // hack for Teensyduino
#endif // defined(I2C_DRIVER_WIRE_H)
#endif // !defined(I2C_BUFFER_LENGTH)

#include "initGT911_Structs.h"

#define GT911_Debug_Serial

#ifdef GT911_Debug_Serial
#define GT911_Log(a) Serial.printf(String("[GT911] @ %d: " + String(a) + "\n").c_str(), millis())
#define GT911_Logf(a, ...) Serial.printf(String("[GT911] @ %d: " + String(a) + "\n").c_str(), millis(), ##__VA_ARGS__)
#else
#define GT911_Log(a)
#define GT911_Logf(a, ...)
#endif // GT911_Debug_Serial

// 0x28/0x29 (0x14 7bit)
#define GT911_I2C_ADDR_28 0x14
// 0xBA/0xBB (0x5D 7bit)
#define GT911_I2C_ADDR_BA 0x5D

#define GT911_MAX_CONTACTS 5

#define GT911_REG_CFG 0x8047
#define GT911_REG_CHECKSUM 0x80FF
#define GT911_REG_DATA 0x8140
#define GT911_REG_ID 0x8140
#define GT911_REG_COORD_ADDR 0x814E

enum : uint8_t
{
  GT911_MODE_INTERRUPT,
  GT911_MODE_POLLING
};

typedef enum : uint8_t
{
  initGT911_ROTATION_0 = 0,
  initGT911_ROTATION_90,
  initGT911_ROTATION_180,
  initGT911_ROTATION_270
} rotation_t;

class initGT911
{

private:
  //TwoWire *_wire;
  I2CMaster* _wire;
  int8_t _intPin;
  int8_t _rstPin;
  uint8_t _addr;

  static volatile bool gt911IRQ;
  bool _configLoaded = false;
  GTConfig _config;
  GTInfo _info;
  GTPoint _points[GT911_MAX_CONTACTS];

  void reset();
  void i2cStart(uint16_t reg);
  bool finish(uint32_t timeout_millis = 50);
  bool endTransmission(size_t expected)
  {
    return finish() && expected == _wire->get_bytes_transferred();  
  }
  bool write(uint16_t reg, uint8_t data);
  uint8_t read(uint16_t reg);
  bool writeBytes(uint16_t reg, uint8_t *data, uint16_t size);
  bool readBytes(uint16_t reg, uint8_t *data, uint16_t size);
  uint8_t calcChecksum(uint8_t *buf, uint8_t len);
  uint8_t readChecksum();
  int8_t readTouches();
  bool readTouchPoints();
  void* context{nullptr};
  void (*async_wait)(void* context){nullptr};
  static void _gt911_irq_handler(void);

public:
  initGT911(I2CMaster *twi = &Master, uint8_t addr = GT911_I2C_ADDR_BA);
  bool begin(int8_t intPin = -1, int8_t rstPin = -1, uint32_t clk = 400000);
  bool productID(uint8_t *buf, uint8_t len);
  GTConfig *readConfig();
  bool updateConfig();
  GTInfo *readInfo();
  I2CMaster& getWire(void) { return *_wire; }

  uint8_t touched(uint8_t mode = GT911_MODE_INTERRUPT);
  GTPoint getPoint(uint8_t num);
  GTPoint *getPoints();

  void setupDisplay(uint16_t xRes, uint16_t yRes, rotation_t rotation);

  // Allow user code to override the default ISR, e.g. 
  // if an RTOS task is blocked waiting for it
  void setInterruptHandler(void (*_isr)(void) = _gt911_irq_handler); 
  void setIRQflag(bool b) { gt911IRQ = b; }

  // Normal "wait" just calls yield() repeatedly, but the 
  // user can override this to block an RTOS task (allowing others
  // to run) until the transaction completes
  void setAsyncWait(void (*fn)(void*)) { async_wait = fn; }
  void setContext(void* ctxt) { context = ctxt; }
};

#endif // INIT_GT911_H
