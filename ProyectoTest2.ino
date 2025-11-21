// ESP32_measure_with_dac_read.ino
#include <Arduino.h>

const int dacPin = 25;       // DAC1 (salida)
const int adcPin = 34;       // mide Vout (entre Rlimit y phantom)
const int dacReadPin = 32;   // ADC pin donde conectas un jumper desde DAC (para medir Vin real)

const int sampleRate = 20000; // 20 kHz
const int freq = 1000;
const int samples = 2000;    // aumentar para mejor SNR

uint16_t rawADC_vout[samples];
uint16_t rawADC_vin[samples];
float VinArray[samples];

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);              // 0-4095
  analogSetAttenuation(ADC_11db);        // medir hasta ~3.3V
  delay(200);
}

void loop() {
  // Generar senoide en DAC y leer Vout + Vin real
  for (int i = 0; i < samples; i++) {
    float t = (float)i / sampleRate;
    float sineValue = (sin(2 * PI * freq * t) + 1.0) * 127.5;  // 0-255
    dacWrite(dacPin, (int)sineValue);

    // esperar siguiente sample
    delayMicroseconds(1000000 / sampleRate);

    rawADC_vout[i] = analogRead(adcPin);     // vout crudo 0-4095
    rawADC_vin[i]  = analogRead(dacReadPin); // lectura del pin DAC conectado aquí
    VinArray[i] = (sineValue / 255.0) * 3.3; // valor teórico (opcional)
  }

  // Enviar bloque
  Serial.println("START");
  for (int i = 0; i < samples; i++) {
    // enviar: Vin_teorico, rawVinADC, rawVoutADC
    Serial.print(VinArray[i], 4); Serial.print(",");
    Serial.print(rawADC_vin[i]); Serial.print(",");
    Serial.println(rawADC_vout[i]);
  }
  Serial.println("END");

  delay(1000);
}
