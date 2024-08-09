#include <PDM.h>
#include "arduinoFFT.h"
#include <math.h> 
// #define LED_PIN 13

const uint16_t sampleSize = 256; // Kích thước mẫu FFT
const float samplingFrequency = 16000; // Tần số lấy mẫu của PDM (Hz)
const int adcMaxValue = 1023; // Giá trị tối đa của ADC (10-bit ADC)
float vReal[sampleSize];
float vImag[sampleSize];
float amplitudeInDB;
float amplitude;
float frequency;
volatile int samplesRead;
int16_t samples[sampleSize];
// ArduinoFFT FFT = ArduinoFFT(); // Tạo đối tượng FFT
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, sampleSize, samplingFrequency, true);

void setup() {
  Serial1.begin(9600);
  while (!Serial1);
  PDM.onReceive(onPDMData); // Thiết lập hàm xử lý sự kiện, Hàm này phải đặt trước hàm PDM.begin()
  PDM.setBufferSize(sampleSize); // Thiết lập kích thước bộ đệm
  // Khởi tạo PDM với 1 kênh (mono) và tần số lấy mẫu
  if (!PDM.begin(1, samplingFrequency)) {
    Serial1.println("Failed to start PDM!");
    while (1);
  }
  // pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  //  if(frequency > 2000)
   {
      // Đợi dữ liệu từ hàm xử lý sự kiện
  // Serial1.print("Amplitude: ");
  // Serial1.println(amplitude);   
    // Serial1.print("Frequency: ");
    Serial1.println(frequency);
    // // Serial1.print("Amplitude (dB): ");
    Serial1.println(amplitudeInDB);
    delay(100);
   }
  
  // for (int i = 0; i < 128; i++) {
  //   Serial.println(samples[i]);
  // }
// if (samplesRead) {

//     // Print samples to the serial monitor or plotter
//     
//       // if(channels == 2) {
//       //   Serial.print("L:");
//       //   Serial.print(sampleBuffer[i]);
//       //   Serial.print(" R:");
//       //   i++;
//       // }
//      
      
//     }

//     // Clear the read count
//     samplesRead = 0;
//   }
  // delay(100);
}

void onPDMData() {
  // Lấy dữ liệu từ bộ đệm PDM
  // int16_t samples[sampleSize];
  PDM.read(samples, sampleSize);
  // Serial.println("Run");
  // Chuyển đổi dữ liệu từ dạng PDM thành dạng PCM
  for (int i = 0; i < sampleSize; i++) {
    vReal[i] = (float)samples[i];
    vImag[i] = 0; // Phần ảo là 0
  }
  // Tìm tần số cao nhất
  float peak = 0;
  int peakIndex = 0;
  for (int i = 0; i < (sampleSize / 2); i++) {
    if (vReal[i] > peak) {
      peak = vReal[i];
      peakIndex = i;
    }
  }
  // Tính biên độ
   amplitude = peak;
  // // Chuyển biên độ thành dB
  amplitudeInDB = 20.0 * log10(amplitude + 1e-6);

  FFT.windowing(vReal, sampleSize, FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Áp dụng hàm cửa sổ
  FFT.compute(vReal, vImag, sampleSize, FFT_FORWARD); // Thực hiện FFT
  FFT.complexToMagnitude(vReal, vImag, sampleSize); // Tính toán độ lớn của các tần số
    // Tính tần số
  frequency = FFT.majorPeak(vReal, sampleSize, samplingFrequency);
}