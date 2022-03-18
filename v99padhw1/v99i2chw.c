#include "v99i2chw.h"
#include "wait.h"


void v99_init(void){
  i2c_master_init();
}

void v99_write(uint8_t addr,uint8_t data){
  i2c_master_start(DEVICE_ADDRESS<<1);
  i2c_master_write(addr);
  i2c_master_write(data);
  i2c_master_stop();
}

uint8_t v99_read(uint8_t addr){
  uint8_t data = 0;

  i2c_master_start(DEVICE_ADDRESS<<1);
  i2c_master_write(addr);
  i2c_master_stop();
  i2c_master_start((DEVICE_ADDRESS<<1)|1);
  data = i2c_master_read(I2C_NACK);
  i2c_master_stop();

  return data;
}
