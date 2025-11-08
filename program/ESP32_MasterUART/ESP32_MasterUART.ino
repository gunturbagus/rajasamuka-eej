#include <HardwareSerial.h>

HardwareSerial SerialPort(1); // UART1 (gunakan UART selain yang ke USB)

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("ESP32 siap kirim perintah ke STM32");
}

void loop() {
  Serial.println("Kirim perintah 1 (nyalakan lampu)");
  SerialPort.write('1');   // kirim karakter '1'
  delay(2000);

  Serial.println("Kirim perintah 0 (matikan lampu)");
  SerialPort.write('0');   // kirim karakter '0'
  delay(2000);
}
