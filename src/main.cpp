#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// ===== User options =====
static const bool DO_CALIBRATION = true;   // set false if you want to skip
static const int  CALIB_MS       = 2000;   // calibration duration (ms)

// ESP32 VSPI default pins:
static const int PIN_CS   = 5;
static const int PIN_SCK  = 18;
static const int PIN_MISO = 19;
static const int PIN_MOSI = 23;

// ADXL355 registers
static const uint8_t REG_DEVID_AD  = 0x00;
static const uint8_t REG_DEVID_MST = 0x01;
static const uint8_t REG_PARTID    = 0x02;
static const uint8_t REG_RANGE     = 0x2C;   
static const uint8_t REG_POWER_CTL = 0x2D;   
static const uint8_t REG_RESET     = 0x2F;
static const uint8_t REG_XDATA3    = 0x08;  

SPISettings adxlSpi(1000000, MSBFIRST, SPI_MODE0);

static inline void csLow()  { digitalWrite(PIN_CS, LOW); }
static inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

static uint32_t g_lsbPerG = 256000;   
static float g_offX = 0, g_offY = 0, g_offZ = 0;

// SPI helpers: cmd = (reg<<1) | RW
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

struct AccelRaw { int32_t x, y, z; };

AccelRaw readRawXYZ() {
  uint8_t b[9];
  spiReadBurst(REG_XDATA3, b, 9);
  return { unpack20(b[0], b[1], b[2]),
           unpack20(b[3], b[4], b[5]),
           unpack20(b[6], b[7], b[8]) };
}

uint32_t lsbPerG_fromRangeReg(uint8_t rangeReg) {
  uint8_t r = rangeReg & 0x03;
  // 01=±2g, 10=±4g, 11=±8g
  if (r == 0x01) return 256000;
  if (r == 0x02) return 128000;
  if (r == 0x03) return  64000;
  return 256000; // safe fallback
}

// Calibration: board flat, top side up => X≈0, Y≈0, Z≈+1g
void calibrateFlat() {
  Serial.printf("Calibrating for %d ms (keep sensor flat & still)...\n", CALIB_MS);

  int64_t sx=0, sy=0, sz=0;
  int n = 0;

  uint32_t t0 = millis();
  while (millis() - t0 < (uint32_t)CALIB_MS) {
    AccelRaw a = readRawXYZ();
    sx += a.x; sy += a.y; sz += a.z;
    n++;
    delay(5);
  }

  float mx = (float)sx / n;
  float my = (float)sy / n;
  float mz = (float)sz / n;

  g_offX = mx;
  g_offY = my;
  g_offZ = mz - (float)g_lsbPerG;

  Serial.printf("Offsets saved (raw counts): offX=%.1f offY=%.1f offZ=%.1f (n=%d)\n",
                g_offX, g_offY, g_offZ, n);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_CS, OUTPUT);
  csHigh();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  spiWrite8(REG_RESET, 0x52);
  delay(50);

  Serial.printf("IDs: AD=0x%02X MST=0x%02X PART=0x%02X\n",
                spiRead8(REG_DEVID_AD),
                spiRead8(REG_DEVID_MST),
                spiRead8(REG_PARTID));

  spiWrite8(REG_POWER_CTL, 0x01);
  delay(10);

  uint8_t rangeReg = spiRead8(REG_RANGE);
  g_lsbPerG = lsbPerG_fromRangeReg(rangeReg);
  Serial.printf("RANGE reg 0x2C=0x%02X => lsbPerG=%lu\n", rangeReg, (unsigned long)g_lsbPerG);

  if (DO_CALIBRATION) calibrateFlat();

  // Measurement mode
  spiWrite8(REG_POWER_CTL, 0x00);
  delay(20);
}

void loop() {
  AccelRaw a = readRawXYZ();

  float x_corr = (float)a.x - g_offX;
  float y_corr = (float)a.y - g_offY;
  float z_corr = (float)a.z - g_offZ;

  float x_g = x_corr / (float)g_lsbPerG;
  float y_g = y_corr / (float)g_lsbPerG;
  float z_g = z_corr / (float)g_lsbPerG;

  float mag_g = sqrtf(x_g*x_g + y_g*y_g + z_g*z_g);

  Serial.printf("g: X=%.4f Y=%.4f Z=%.4f | |a|=%.4f g\n", x_g, y_g, z_g, mag_g);
  delay(50);
}
