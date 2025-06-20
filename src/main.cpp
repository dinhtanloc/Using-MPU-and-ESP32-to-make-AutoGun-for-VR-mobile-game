#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <BleMouse.h>

#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "model.h"

const char* ssid = "Cá mập xanh";
const char* password = "cubideptrai";

WiFiUDP udp;
const unsigned int localPort = 8080;
IPAddress receiverIP(192, 168, 137, 69); // máy nhận (laptop)
unsigned int receiverPort = 8080;
bool clientConnected = false;

MPU6050 mpu(0x68);
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa, aaReal, aaWorld;
VectorFloat gravity;

#define THRESHOLD 7000
int count_no = 0;
float euler[3];
int outputLed =14;

// TensorFlow Lite
const tflite::Model* tflModel;
tflite::ErrorReporter* tflErrorReporter;
constexpr int tensorArenaSize = 70 * 1024;
uint8_t tensorArena[tensorArenaSize];
TfLiteTensor* tflInputTensor;
TfLiteTensor* tflOutputTensor;
tflite::MicroInterpreter* tflInterpreter;

IPAddress local_IP(192, 168, 137, 188);
IPAddress gateway(192, 168, 137, 69);
IPAddress subnet(255, 255, 255, 0);
float raw_gx = 0, raw_gy = 0, raw_gz = 0;
int16_t gx = 0, gy = 0, gz = 0;
void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  pinMode(outputLed, OUTPUT);

  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("IP tĩnh cấu hình thất bại");
  }

  WiFi.begin(ssid, password);
  Serial.print("WiFi: Đang kết nối");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi: Kết nối thành công");
  Serial.print("Địa chỉ IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.println("UDP: Server đã bắt đầu");

  // MPU6050
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  calibrateMPU();


  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("MPU6050: Khởi tạo thành công");
  } else {
    Serial.println("MPU6050: Khởi tạo thất bại.");
  }

  // TensorFlow Lite
  static tflite::MicroErrorReporter micro_error_reporter;
  tflErrorReporter = &micro_error_reporter;

  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("TFLite: Schema version không khớp!");
    return;
  }

  static tflite::MicroMutableOpResolver<2> resolver;

  resolver.AddFullyConnected(); // FULLY_CONNECTED
  resolver.AddSoftmax();        // SOFTMAX

  static tflite::MicroInterpreter static_interpreter(
    tflModel, resolver, tensorArena, tensorArenaSize, tflErrorReporter);
  tflInterpreter = &static_interpreter;

  if (tflInterpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("TFLite: AllocateTensors() thất bại");
  }

  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);
}

void loop() {
  if (!dmpReady) return;

  if (!clientConnected) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      char incoming[64];
      udp.read(incoming, sizeof(incoming));
      Serial.print("UDP: Nhận gói tin: ");
      Serial.println(incoming);
      clientConnected = true;
      Serial.println(clientConnected);
      digitalWrite(outputLed, HIGH);
      //sáng đèn
    }
    return;
  }

  int mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    detect_motion_and_send_udp();
  }
}

void detect_motion_and_send_udp() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  int sum = abs(aaWorld.x) + abs(aaWorld.y) + abs(aaWorld.z);
  if (sum > THRESHOLD) {
    run_inference_udp();
  }
}

void run_inference_udp() {
  count_no = 0;
  while (count_no < 40) {
  // while (true) {
    int mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      collect_input_tensor();
      delay(10);
    }
  }

  if (tflInterpreter->Invoke() != kTfLiteOk) {
    Serial.println("TFLite: Invoke thất bại");
    return;
  }

  float out = *tflOutputTensor->data.f;
  Serial.print("Kết quả mô hình: ");
  Serial.println(out);

  char result[16];
  if (out >= 0.7) {
    snprintf(result, sizeof(result), "1");
    Serial.println("📡 Gửi dữ liệu: 1 qua UDP");
  } else {
    snprintf(result, sizeof(result), "0");
    Serial.println("📡 Gửi dữ liệu: 0 qua UDP");
  }

  udp.beginPacket(receiverIP, receiverPort);
  udp.write((uint8_t*)result, strlen(result));
  udp.endPacket();
}

void collect_input_tensor() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  mpu.getRotation(&gx, &gy, &gz);

  tflInputTensor->data.f[count_no * 3 + 0] = (float)aaWorld.x / 16384.0;
  tflInputTensor->data.f[count_no * 3 + 1] = (float)aaWorld.y / 16384.0;
  tflInputTensor->data.f[count_no * 3 + 2] = (float)aaWorld.z / 16384.0;
  float adjustedY = q.y;
  float adjustedZ = q.z;

  // Gửi quaternion để debug
  char data[64];
  snprintf(data, sizeof(data), "r,%.4f,%.4f,%.4f,%.4f", q.w, q.x, q.y, q.z);
  // snprintf(data, sizeof(data), "r,%.4f,%.4f,%.4f,%.4f", q.w, q.x, adjustedZ, adjustedY);
  // snprintf(data, sizeof(data), "r,%.4f,%.4f,%.4f,%.4f", q.w, q.x, adjustedZ, adjustedY, );
  // snprintf(data, sizeof(data), "r,%d,%d,%d", gx, gy, gz); // Định dạng mới: "g,gx,gy,gz"

  udp.beginPacket(receiverIP, receiverPort);
  udp.write((const uint8_t*)data, strlen(data));
  udp.endPacket();
  Serial.print("📡 Gửi quaternion UDP: ");
  Serial.println(data);


  count_no++;
}
void calibrateMPU()
{
  Serial.println("⏳ Đang hiệu chuẩn MPU6050... Giữ yên cảm biến!");
  delay(2000);

  const int numReadings = 1000;

  for (int i = 0; i < numReadings; i++)
  {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.getRotation(&gx, &gy, &gz);

      raw_gx += gx;
      raw_gy += gy;
      raw_gz += gz;

      delay(1);
    }
  }

  raw_gx /= numReadings;
  raw_gy /= numReadings;
  raw_gz /= numReadings;

  Serial.println("Hoàn tất hiệu chuẩn. Offset đề xuất:");
  Serial.print("X: "); Serial.print(raw_gx); Serial.print(" | ");
  Serial.print("Y: "); Serial.print(raw_gy); Serial.print(" | ");
  Serial.print("Z: "); Serial.println(raw_gz);

  mpu.setXGyroOffset((int16_t)raw_gx);
  mpu.setYGyroOffset((int16_t)raw_gy);
  mpu.setZGyroOffset((int16_t)raw_gz);
}

 