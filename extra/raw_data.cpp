#include <Arduino.h>
#include <SPI.h>

// ESP32 VSPI default pins:
static const int PIN_CS = 5;
static const int PIN_SCK = 18;
static const int PIN_MISO = 19;
static const int PIN_MOSI = 23;

// Registers (ADXL355)
static const uint8_t REG_DEVID_AD  = 0x00; 
static const uint8_t REG_DEVID_MST = 0x01; 
static const uint8_t REG_PARTID    = 0x02; 
static const uint8_t REG_POWER_CTL = 0x2D; 
static const uint8_t REG_RESET     = 0x2F; 
static const uint8_t REG_XDATA3    = 0x08; 

// SPI: Mode 0 (CPOL=0, CPHA=0) 
SPISettings adxlSpi(1000000, MSBFIRST, SPI_MODE0);

static inline void csLow()  { digitalWrite(PIN_CS, LOW); }
static inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

// Address byte: (reg<<1) | RW  (RW=1 read, 0 write)
uint8_t spiRead8(uint8_t reg) {
  uint8_t addr = (reg << 1) | 0x01;
  SPI.beginTransaction(adxlSpi);
  csLow();
  SPI.transfer(addr);
  uint8_t v = SPI.transfer(0x00);
  csHigh();
  SPI.endTransaction();
  return v;
}

void spiWrite8(uint8_t reg, uint8_t val) {
  uint8_t addr = (reg << 1) | 0x00;
  SPI.beginTransaction(adxlSpi);
  csLow();
  SPI.transfer(addr);
  SPI.transfer(val);
  csHigh();
  SPI.endTransaction();
}

void spiReadBurst(uint8_t startReg, uint8_t *buf, size_t n) {
  uint8_t addr = (startReg << 1) | 0x01;
  SPI.beginTransaction(adxlSpi);
  csLow();
  SPI.transfer(addr);
  for (size_t i = 0; i < n; i++) buf[i] = SPI.transfer(0x00);
  csHigh();
  SPI.endTransaction();
}

int32_t unpack20(uint8_t b0, uint8_t b1, uint8_t b2) {
  int32_t raw = ((int32_t)b0 << 12) | ((int32_t)b1 << 4) | ((int32_t)b2 >> 4);
  if (raw & (1 << 19)) raw |= 0xFFF00000;
  return raw;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_CS, OUTPUT);
  csHigh();

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // Optional reset
  spiWrite8(REG_RESET, 0x52);
  delay(50);

  uint8_t id_ad  = spiRead8(REG_DEVID_AD);
  uint8_t id_mst = spiRead8(REG_DEVID_MST);
  uint8_t partid = spiRead8(REG_PARTID);
  Serial.printf("IDs: DEVID_AD=0x%02X, DEVID_MST=0x%02X, PARTID=0x%02X\n", id_ad, id_mst, partid);

  // Put into measurement mode
  spiWrite8(REG_POWER_CTL, 0x00);
  delay(20);
}

void loop() {
  uint8_t b[9];
  spiReadBurst(REG_XDATA3, b, 9);

  int32_t x = unpack20(b[0], b[1], b[2]);
  int32_t y = unpack20(b[3], b[4], b[5]);
  int32_t z = unpack20(b[6], b[7], b[8]);

  Serial.printf("RAW20: X=%ld  Y=%ld  Z=%ld\n", (long)x, (long)y, (long)z);
  delay(50);
}
