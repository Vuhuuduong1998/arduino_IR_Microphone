#include "Arduino.h"
#include <PDM.h>
#include "arduinoFFT.h"
#include <math.h> 

#define SAMPLE_NUM (200)


const uint8_t MODBUS_SLAVE_ID = 10; // Địa chỉ Slave (ID của thiết bị Modbus)
const uint16_t REG_ADDRESS_4003 = 0x0002; // Địa chỉ thanh ghi mà chúng ta sẽ sử dụng
const uint16_t REG_ADDRESS_4004 = 0x0003; 
const uint16_t REG_ADDRESS_4005 = 0x0004; 
uint16_t register_4003 =0;
uint16_t register_4004 =0; 
uint16_t register_4005 =0; // Giá trị thanh ghi khởi tạo

const uint16_t sampleSize = 128; // Kích thước mẫu FFT
const float samplingFrequency = 16000; // Tần số lấy mẫu của PDM (Hz)
const int adcMaxValue = 1023; // Giá trị tối đa của ADC (10-bit ADC)
float vReal[sampleSize];
float vImag[sampleSize];
float amplitudeInDB;
float amplitude;
float frequency;
int16_t samples[sampleSize];
uint8_t sound_index = 0; 
uint8_t data_ready = 0; 
float frequency_buff[SAMPLE_NUM];
float amplitudeInDB_buff[SAMPLE_NUM]; 
// ArduinoFFT FFT = ArduinoFFT(); // Tạo đối tượng FFT
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, sampleSize, samplingFrequency, true);

void test_sound(void);

void setup() {
  Serial.begin(38400);
  while (!Serial) ;
  Serial1.begin(9600);
  while (!Serial1);
  PDM.onReceive(onPDMData); // Thiết lập hàm xử lý sự kiện, Hàm này phải đặt trước hàm PDM.begin()
  PDM.setBufferSize(sampleSize); // Thiết lập kích thước bộ đệm
  // Khởi tạo PDM với 1 kênh (mono) và tần số lấy mẫu
  if (!PDM.begin(1, samplingFrequency)) {
    Serial1.println("Failed to start PDM!");
    while (1);
  }
}

void loop() {

  if (Serial.available() > 0) {
    // Đọc dữ liệu từ Serial
    uint8_t request[8];

    while (Serial.available() < 8);
    for (int i = 0; i < 8; i++) {
      request[i] = Serial.read();
    }
    // Serial1.flush(); 
      uint8_t slaveId = request[0];
      uint8_t functionCode = request[1];
      uint16_t address = (((uint16_t)request[2]) << 8) | request[3];
      uint16_t value = (((uint16_t)request[4]) << 8) | request[5];
      uint16_t crcReceived = (((uint16_t)request[7]) << 8) | request[6];

      uint16_t crcCalculated = calculateCRC16(request, 6); // Tính CRC cho phần dữ liệu

      // Kiểm tra CRC
      if (crcReceived == crcCalculated) 
      {
        if (slaveId == MODBUS_SLAVE_ID)
         {
          if (functionCode == 0x06) 
          { // Write Single Register
            if (address == REG_ADDRESS_4003) 
            {
              register_4003 = value; // Ghi giá trị vào thanh ghi
              uint8_t response[8];
              response[0] = slaveId;
              response[1] = functionCode;
              response[2] = (uint8_t)(REG_ADDRESS_4003 >> 8);
              response[3] = (uint8_t)REG_ADDRESS_4003;
              response[4] = (uint8_t)(register_4003 >> 8); 
              response[5] = (uint8_t)register_4003;     
              uint16_t crc = calculateCRC16(response, 6);
              response[6] = crc & 0xFF;
              response[7] = (crc >> 8) & 0xFF;

              // Gửi phản hồi qua Serial
              Serial.write(response, 8);

            }
          }
           else if (functionCode == 0x03) 
           { // Read Single Register
            if (address == REG_ADDRESS_4003) 
            {
                uint8_t response[7];
                response[0] = slaveId;
                response[1] = functionCode;
                response[2] = 2;
                response[3] = (uint8_t)(register_4003 >> 8);
                response[4] = (uint8_t)register_4003;      
                uint16_t crc = calculateCRC16(response, 5);
                response[5] = crc & 0xFF;
                response[6] = (crc >> 8) & 0xFF;

                // Gửi phản hồi qua Serial
                Serial.write(response, 7);
                // Serial.print("Read value: ");
                // Serial.println(registerValue, HEX);
            }
            else if (address == REG_ADDRESS_4004) 
            {
              uint8_t response[7];
              response[0] = slaveId;
              response[1] = functionCode;
              response[2] = 2;
              response[3] = (uint8_t)(register_4004 >> 8);
              response[4] = (uint8_t)register_4004;      
              uint16_t crc = calculateCRC16(response, 5);
              response[5] = crc & 0xFF;
              response[6] = (crc >> 8) & 0xFF;

              // Gửi phản hồi qua Serial
              Serial.write(response, 7);
              // Serial.print("Read value: ");
              // Serial.println(registerValue, HEX);
            }
            else if (address == REG_ADDRESS_4005) 
            {
              uint8_t response[7];
              response[0] = slaveId;
              response[1] = functionCode;
              response[2] = 2;
              response[3] = (uint8_t)(register_4005 >> 8);
              response[4] = (uint8_t)register_4005;      
              uint16_t crc = calculateCRC16(response, 5);
              
              response[5] = crc & 0xFF;
              response[6] = (crc >> 8) & 0xFF;

              // Gửi phản hồi qua Serial
              Serial.write(response, 7);
              // Serial.print("Read value: ");
              // Serial.println(registerValue, HEX);
            }
          }
        }
      }
    
  }
  if(register_4003 == 1)
  {
    test_sound(); 
    register_4003 = 0; 
  }
}

void onPDMData() {
  // Lấy dữ liệu từ bộ đệm PDM
  PDM.read(samples, sampleSize);
  data_ready = 1; 
}
void sound_process()
{
  // Serial1.println("sound process");
  // delay_custom(15);
  // Chuyển đổi dữ liệu từ dạng PDM thành dạng PCM
  for (int i = 0; i < sampleSize; i++) {
    vReal[i] = (float)samples[i];
    vImag[i] = 0; // Phần ảo là 0
  }
  // Tìm tần số cao nhất
  float peak = 0;
  for (int i = 0; i < (sampleSize / 2); i++) {
    if (vReal[i] > peak) {
      peak = vReal[i];
    }
  }
  // Tính biên độ
  amplitude = peak;
  FFT.windowing(vReal, sampleSize, FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Áp dụng hàm cửa sổ
  FFT.compute(vReal, vImag, sampleSize, FFT_FORWARD); // Thực hiện FFT
  FFT.complexToMagnitude(vReal, vImag, sampleSize); // Tính toán độ lớn của các tần số
    // Tính tần số
  frequency = FFT.majorPeak(vReal, sampleSize, samplingFrequency);
  // // Chuyển biên độ thành dB
  amplitudeInDB = 20.0 * log10(amplitude + 1e-6);
  frequency_buff[sound_index] = frequency; 
  amplitudeInDB_buff[sound_index] = amplitudeInDB;  
  // Serial1.println(frequency_buff[sound_index]);
  // Serial1.println(amplitudeInDB_buff[sound_index]);
  // Serial1.println(sound_index);
  sound_index ++; 
}

void delay_custom(uint32_t delay_time)
{
  uint32_t start_time = millis(); 
  while (millis() - start_time <delay_time); 
}
void test_sound()
{
    sound_index = 0; 
    for(int i= 0; i<SAMPLE_NUM; i++)
    {
      data_ready = 0;
      while(data_ready == 0)
      {
        Serial1.println("Loop");
      }
      sound_process(); 
      // delay(10);
    }
 
    float frequency_avg = 0; 
    uint8_t index_avg = 0; 
    float amplitudeInDB_avg = 0; 
    
    for(uint8_t i = 0; i<SAMPLE_NUM; i++)
    {
      // Serial1.println(frequency_buff[i]);
      if(frequency_buff[i] > 2000)
      {
        frequency_avg +=  frequency_buff[i];
        amplitudeInDB_avg += amplitudeInDB_buff[i]; 
        index_avg ++; 
      }
    }

    frequency_avg /= index_avg; 
    amplitudeInDB_avg /= index_avg; 
    register_4004 = (uint16_t)frequency_avg;
    register_4005 = (uint16_t)amplitudeInDB_avg;
    
    for(uint8_t i=0; i<SAMPLE_NUM; i++)
    {
      frequency_buff[i] = 0;
      amplitudeInDB_buff[i] = 0; 
    }
    Serial1.println("index_avg");
    Serial1.println(index_avg);
    Serial1.println("frequency_avg");
    Serial1.println(register_4004);   // register 4004
    Serial1.println("amplitudeInDB_avg");
    Serial1.println(register_4005); // register 4005
}

// Hàm tính CRC-16-CCITT
uint16_t calculateCRC16(uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    uint8_t i;

    while (length--) {
        crc ^= *data++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001; // Polynomial for Modbus CRC16
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}
