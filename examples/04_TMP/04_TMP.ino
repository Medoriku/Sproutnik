#include <Wire.h>
#include <SparkFun_TMP117.h>

struct TmpData {
  bool tmp1_ok = false;
  bool tmp2_ok = false;
  float tmp1_c = 0.0f;
  float tmp2_c = 0.0f;
};

TMP117 tmp_sensor1;
TMP117 tmp_sensor2;
TmpData tmp_data;
uint8_t tmp_addr1 = 0x48;
uint8_t tmp_addr2 = 0x4A;

void TMP_begin(TwoWire &wire = Wire, uint8_t addr1 = 0x48, uint8_t addr2 = 0x4A) {
  tmp_addr1 = addr1;
  tmp_addr2 = addr2;
  tmp_data.tmp1_ok = tmp_sensor1.begin(tmp_addr1, wire);
  tmp_data.tmp2_ok = tmp_sensor2.begin(tmp_addr2, wire);
}

void TMP_update() {
  if (tmp_data.tmp1_ok && tmp_sensor1.dataReady()) {
    tmp_data.tmp1_c = tmp_sensor1.readTempC();
  }
  if (tmp_data.tmp2_ok && tmp_sensor2.dataReady()) {
    tmp_data.tmp2_c = tmp_sensor2.readTempC();
  }
}

TmpData TMP_getData() {
  return tmp_data;
}
