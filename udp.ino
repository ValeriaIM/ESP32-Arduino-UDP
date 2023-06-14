#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"
#include "FS.h"                // SD Card ESP32
#include <SPI.h>
#include <SD.h>
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"

const char *ssid = "";
const char *password = "";

AsyncUDP udp;                      // Создать объект UDP

int buttonState = 0;
int btnHold = 0;

// Motion Sensor
bool motionDetected = false;

//#define KEY_PIN       15
#define CAMERA_MODEL_AI_THINKER 

#define FLASH_LED_PIN 13
bool flashState = LOW;

#define BUILTIN_LED_PIN 33      // встроеный светодиод
int PIRDelay = 30000;           // время (мс) в течении которого PIR не опрашивается
long lastTimePIR;               // не прошло ли время для включения датчика

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void startCameraServer();

static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,
  
  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,
  
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  
  .pixel_format = PIXFORMAT_JPEG,
  .frame_size = FRAMESIZE_XGA,
  .jpeg_quality = 12,
  .fb_count = 1,
};

esp_err_t camera_init() {
  //initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  
  if (err != ESP_OK) {
    Serial.print("Camera Init Failed");
    return err;
  }
  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV2640_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_contrast(s, 1);
  }

  s->set_framesize(s, FRAMESIZE_QVGA);

  #if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  #endif
          
  Serial.print("Camera Init OK");
  return ESP_OK;
}

void wifi_init(void){
  WiFi.mode(WIFI_STA);
  
  // WiFi.setSleep (false); // Отключаем wifi sleep в режиме STA для повышения скорости отклика
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }    
  
  Serial.println("Connected");
  Serial.print("IP Address:");    
  
  startCameraServer(); //чтобы можно было посмотреть, что камера работает

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  if(udp.connect(IPAddress(0,0,0,0), 8888))
  {
    Serial.println("UDP connected");
    udp.print("Hello Server!"); 
  }    
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);    
  delay(1000);
  
  camera_init(); // Инициализация камеры
  wifi_init(); // инициализация WiFi

  //pinMode(KEY_PIN, INPUT_PULLUP); // Устанавливаем режим ввода кнопочного штифта

  pinMode(FLASH_LED_PIN, INPUT);
  
  Serial.println("sys is running!");
}

void sendPhoto(){
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.print( "Camera capture failed");
  }
  else
  { 
    udp.write(fb->buf, fb->len); // Копируем данные в буфер отправки
    
    Serial.println("succes to send image for UDP"); 
    udp.print("Hi Server! I send you photo"); 
    //return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
  }
}

void loop() 
{
  //Serial.println(digitalRead(FLASH_LED_PIN));
  if (digitalRead(FLASH_LED_PIN) == HIGH) {
    //lastTimePIR = millis();
    Serial.println("Send Motion Detected, Preparing send photo");
    sendPhoto();
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(PIRDelay);    
  }
}

