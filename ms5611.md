# 气压计MS5611移植与调试
## 开发笔记
- 宏定义  
#define SENSORS_ENABLE_PRESSURE_MS561
- 修改`i2cdevWriteReg8`函数
```
bool i2cdevWriteReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint16_t len, uint8_t *data)
{
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t)5) == pdFALSE) {
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    if(memAddress != I2CDEV_NO_MEM_ADDR)
    {
        i2c_master_write_byte(cmd, memAddress, I2C_MASTER_ACK_EN);
    }
    i2c_master_write(cmd, (uint8_t *)data, len, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t)5);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

#if defined CONFIG_I2CBUS_LOG_READWRITES

    if (!err) {
        char str[length * 5 + 1];

        for (size_t i = 0; i < length; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] Write %d bytes to register 0x%X, data: %s",
                      port, devAddr, length, regAddr, str);
    }

#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else

    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] Failed to write %d bytes to__ register 0x%X, error: 0x%X",
                    port, devAddr, length, regAddr, err);
    }

#endif

    if (err == ESP_OK) {
        return TRUE;
    } else {
        return false;
    }
}
```  
- 修改`i2cdevReadReg8`函数
```  
bool i2cdevReadReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint16_t len, uint8_t *data)
{
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t)5) == pdFALSE) {
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(memAddress != I2CDEV_NO_MEM_ADDR)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
        i2c_master_write_byte(cmd, memAddress, I2C_MASTER_ACK_EN);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t)5);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

#if defined CONFIG_I2CBUS_LOG_READWRITES

    if (!err) {
        char str[length * 5 + 1];

        for (size_t i = 0; i < length; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] Read_ %d bytes from register 0x%X, data: %s", port, devAddr, length, regAddr, str);
    }

#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {}
#else

    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] Failed to read %d bytes from register 0x%X, error: 0x%X",
                    port, devAddr, length, regAddr, err);
    }

#endif

    if (err == ESP_OK) {
        return TRUE;
    } else {
        return false;
    }
}
```  
- 修改`ms5611ReadPROM`函数  
```
bool ms5611ReadPROM()
{
    uint8_t buffer[MS5611_PROM_REG_SIZE];
    uint16_t *pCalRegU16 = (uint16_t *)&calReg;
    int32_t i = 0;
    bool status = false;

    for (i = 0; i < MS5611_PROM_REG_COUNT; i++) {
        // start read sequence
        status = i2cdevWriteByte(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR,
                                 MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE));

        // Read conversion
        if (status) {
            status = i2cdevReadReg8(I2Cx, devAddr,I2CDEV_NO_MEM_ADDR, MS5611_PROM_REG_SIZE, buffer);
            pCalRegU16[i] = ((uint16_t)buffer[0] << 8) | buffer[1];
        }
    }

    return status;
}
```
### 尽量使用已有函数，减少编译错误
- 在`sensorsDeviceInit`函数中添加  
```
mpu6050SetI2CMasterModeEnabled(false);//set MPU6050_USERCTRL_I2C_MST_EN_BIT = 0
```
并在`sensorsSetupSlaveRead`函数中注释此行
```
 //mpu6050SetI2CMasterModeEnabled(false);
```  
### 因为`sensorsDeviceInit`函数之后有读取操作，又MS5611挂载在MPU6050的I2C上，所以把Bypass提前设置好

-  编辑`processBarometerMeasurements`函数
```
#define LPS25H_LSB_PER_MBAR      4096UL
#define LPS25H_LSB_PER_CELSIUS   480UL
#define LPS25H_TEMP_OFFSET        (42.5f)

void processBarometerMeasurements(const uint8_t *buffer)
{
    //TODO: replace it to MS5611
  static uint32_t rawPressure = 0;
  static int16_t rawTemp = 0;

  // Check if there is a new pressure update
  if (buffer[0] & 0x02) {
    rawPressure = ((uint32_t) buffer[3] << 16) | ((uint32_t) buffer[2] << 8) | buffer[1];
  }
  // Check if there is a new temp update
  if (buffer[0] & 0x01) {
    rawTemp = ((int16_t) buffer[5] << 8) | buffer[4];
  }

  sensorData.baro.pressure = (float) rawPressure / LPS25H_LSB_PER_MBAR;
  sensorData.baro.temperature = LPS25H_TEMP_OFFSET + ((float) rawTemp / LPS25H_LSB_PER_CELSIUS);
  sensorData.baro.asl = ms5611PressureToAltitude(&sensorData.baro.pressure);
}
```  
- 至此ESP-DRONE的MS5611移植成功
可以在`ms5611SelfTest`函数中添加debug函数
```
 DEBUG_PRINTI("temperature = %f,pressure = %f\n",temperature,pressure);
```  
作者所处环境温度34.65摄氏度，压力1002.41Pa
![ESP-Drone](./docs/_static/ms5611.png)