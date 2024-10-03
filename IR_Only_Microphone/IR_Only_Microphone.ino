// #include "IR_transmitter.h"
#include <ModbusRTUSlave_D.h>
#include "Arduino.h"

// #include "PinDefinitionsAndMore.h"  // Define macros for input and output pin etc.
// #include <IRremote.hpp>
#include <PDM.h>
#include "arduinoFFT.h"
#include <math.h> 

// #ifdef SOFTWARE_SERIAL
// #include "SoftwareSerial.h"
// SoftwareSerial mySerial(10, 11);  // RX, TX
// #endif

#define SAMPLE_NUM (200)

ModbusRTUSlave modbus(Serial, 13);  // serial port, driver enable pin for rs-485
bool coils[20];
bool discreteInputs[20];
uint16_t holdingRegisters[20];
uint16_t inputRegisters[20];

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
  Serial1.begin(9600);
  while (!Serial1);

  modbus.configureCoils(coils, 20);                        // bool array of coil values, number of coils
  modbus.configureDiscreteInputs(discreteInputs, 20);      // bool array of discrete input values, number of discrete inputs
  modbus.configureHoldingRegisters(holdingRegisters, 20);  // unsigned 16 bit integer array of holding register values, number of holding registers
  modbus.configureInputRegisters(inputRegisters, 20);      // unsigned 16 bit integer array of input register values, number of input registers
  for (uint8_t i = 0; i < 20; i++) {
    holdingRegisters[i] = 0;
  }
  modbus.begin(10, 38400);

  // IrSender.begin(4, ENABLE_LED_FEEDBACK, ALTERNATIVE_IR_FEEDBACK_LED_PIN); // Specify send pin and enable feedback LED at default feedback LED pin
  // #if defined(IR_SEND_PIN)
  //   IrSender.begin();
  // #endif
  PDM.onReceive(onPDMData); // Thiết lập hàm xử lý sự kiện, Hàm này phải đặt trước hàm PDM.begin()
  PDM.setBufferSize(sampleSize); // Thiết lập kích thước bộ đệm
  // Khởi tạo PDM với 1 kênh (mono) và tần số lấy mẫu
  if (!PDM.begin(1, samplingFrequency)) {
    Serial1.println("Failed to start PDM!");
    while (1);
  }
}

uint32_t cmd = 0xA90;

void loop() {

  modbus.poll();
  if (modbus.already()) {
    // switch (holdingRegisters[0]) {
    //   case 1:
    //     IR_send(IR_CODE_SECURITY_CH1);
    //     break;
    //   case 2:
    //     IR_send(IR_CODE_FULL_MAIN_LIGHT_CH1);
    //     break;
    //   case 3:
    //     IR_send(IR_CODE_INCREASE_CH1);
    //     break;
    //   case 4:
    //     IR_send(IR_CODE_DECREASE_CH1);
    //     break;
    //   case 5:
    //     IR_send(IR_CODE_ON_OFF_CH1);
    //     break;
    //   case 6:
    //     IR_send(IR_CODE_SLEEP_LIGHT_CH1);
    //     break;
    //   case 7:
    //     IR_send(IR_CODE_TIMER_CH1);
    //     break;
    //   case 8:
    //     IR_send(IR_CODE_ON_OLD_CH1);
    //     break;
    //   case 9:
    //     IR_send(IR_CODE_OFF_OLD_CH1);
    //     break;
    //   case 10:
    //     for(uint8_t i =0; i<4; i++)
    //     {
    //       IR_send(IR_CODE_INCREASE_CH1);
    //       delay(500); 
    //     }
    //   break; 
    //   default:
    //     break;
    // }
    // delay_custom(300);
    // switch (holdingRegisters[1]) {
    //   case 1:
    //     IR_send(IR_CODE_SECURITY_CH2);
    //     break;
    //   case 2:
    //     IR_send(IR_CODE_FULL_MAIN_LIGHT_CH2);
    //     break;
    //   case 3:
    //     IR_send(IR_CODE_INCREASE_CH2);
    //     break;
    //   case 4:
    //     IR_send(IR_CODE_DECREASE_CH2);
    //     break;
    //   case 5:
    //     IR_send(IR_CODE_ON_OFF_CH2);
    //     Serial1.println("Run5");
    //     break;
    //   case 6:
    //     IR_send(IR_CODE_SLEEP_LIGHT_CH2);
    //     break;
    //   case 7:
    //     IR_send(IR_CODE_TIMER_CH2);
    //     break;
    //   case 8:
    //     IR_send(IR_CODE_ON_OLD_CH2);
    //     break;
    //   case 9:
    //     IR_send(IR_CODE_OFF_OLD_CH2);
    //     break;       
    //   default:
    //     break;
    // }
    switch (holdingRegisters[2])
    {
      case 1: 
        test_sound();
      break; 
      default:
      break; 
    }
    // holdingRegisters[0] = 0;
    // holdingRegisters[1] = 0;
    // holdingRegisters[2] = 0;
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
      if((frequency_buff[i] > 2000) && (amplitudeInDB_buff[i] > 37))
      {
        frequency_avg +=  frequency_buff[i];
        amplitudeInDB_avg += amplitudeInDB_buff[i]; 
        index_avg ++; 
      }
    }

    frequency_avg /= index_avg; 
    amplitudeInDB_avg /= index_avg; 
    holdingRegisters[3] =(uint16_t) frequency_avg; 
    holdingRegisters[4] =(uint16_t) amplitudeInDB_avg;
    holdingRegisters[2] = 0;
    Serial1.println("Print Buff: \n\r");
    for(uint8_t i=0; i<SAMPLE_NUM; i++)
    {
      Serial1.println("frequency: ");
      Serial1.println(frequency_buff[i]);
      Serial1.println("Amplitude: ");
      Serial1.println(amplitudeInDB_buff[i]); 
    }
    for(uint8_t i=0; i<SAMPLE_NUM; i++)
    {
      frequency_buff[i] = 0;
      amplitudeInDB_buff[i] = 0; 
    }
    Serial1.println("index_avg");
    Serial1.println(index_avg);
    Serial1.println("frequency_avg");
    Serial1.println(holdingRegisters[3]);   // register 4004
    Serial1.println("amplitudeInDB_avg");
    Serial1.println(holdingRegisters[4]); // register 4005
}

uint32_t invert_0_and_1(uint32_t data) {
  uint32_t test = 0;
  for (uint8_t i = 0; i < 32; i++) {
    if ((data & (1 << i)) != 0) {
      test &= ~(1 << i);
    } else {
      test |= 1 << i;
    }
  }
  return test;
}

uint32_t CMD_Inverse(uint32_t data) {
  uint32_t result = 0;
  data ^= 0xffffffff;
  for (int i = 0; i < 32; i++) {
    if ((data >> i) & 0x0001) {
      result = result << 1;
      result |= 0x0001;
    } else {
      result = result << 1;
      result &= ~0x0001;
    }
  }
  return result;
}

// void IR_send(uint32_t data) {
//   uint32_t cmd;
//   cmd = invert_0_and_1((uint32_t)data);
//   // Serial.print("send --> ");Serial.println(cmd,HEX);
//   IrSender.sendNECRaw(cmd, 0);
//   // delay(350);
// }