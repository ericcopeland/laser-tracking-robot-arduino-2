#include "src/OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"
#include "home_wifi_multi.h"

OV2640 cam;

WebServer server(80);

#include "ArduinoJson.h"
#include "shared_data.h"

long LAST_REQUEST_TIMESTAMP = 0;
const long MAX_DELAY_MS = 500;
int CURRENT_DISTANCE = 100;
int LAST_DISTANCE = 101;

const int MPWM_LEFT_CHANNEL = 3;
const int MPWM_RIGHT_CHANNEL = 2;

const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);

void handle_jpg_stream(void)
{
  char buf[32];
  int s;

  WiFiClient client = server.client();

  client.write(HEADER, hdrLen);
  client.write(BOUNDARY, bdrLen);

  while (true)
  {
    if (!client.connected()) break;
    cam.run();
    s = cam.getSize();
    client.write(CTNTTYPE, cntLen);
    sprintf( buf, "%d\r\n\r\n", s );
    client.write(buf, strlen(buf));
    client.write((char *)cam.getfb(), s);
    client.write(BOUNDARY, bdrLen);
  }
}

const char JHEADER[] = "HTTP/1.1 200 OK\r\n" \
                       "Content-disposition: inline; filename=capture.jpg\r\n" \
                       "Content-type: image/jpeg\r\n\r\n";
const int jhdLen = strlen(JHEADER);

void handle_jpg(void)
{
  WiFiClient client = server.client();

  cam.run();
  if (!client.connected()) return;

  client.write(JHEADER, jhdLen);
  client.write((char *)cam.getfb(), cam.getSize());
}

void handle_control(void) {
    LAST_REQUEST_TIMESTAMP = millis();
    
    DynamicJsonDocument doc(512);
    deserializeJson(doc, server.arg("plain"));

    control_data post_data = {
      .frame_width = doc["frame"]["width"],
      .frame_height = doc["frame"]["height"],
      .laser_left = doc["laser"]["position"]["left"],
      .laser_top = doc["laser"]["position"]["top"],
      .center_width = doc["center"]["width"],
      .center_left_line = doc["center"]["left_line"],
      .center_right_line = doc["center"]["right_line"],
      .landmine_left = doc["nearest_landmine"]["position"]["left"],
      .landmine_top = doc["nearest_landmine"]["position"]["top"],
      .landmine_stop_distance = doc["nearest_landmine"]["stop_distance"]
    };

    int max_speed = 220;
    int min_speed = 180;
    int speed_diff = max_speed - min_speed;
    int turn_speed = 0;
    double error = 0;

    if (CURRENT_DISTANCE <= 15) {
      ledcWrite(MPWM_LEFT_CHANNEL, 0);
      ledcWrite(MPWM_RIGHT_CHANNEL, 0);
    }
    else if (post_data.landmine_top >= post_data.landmine_stop_distance) {
      ledcWrite(MPWM_LEFT_CHANNEL, 0);
      ledcWrite(MPWM_RIGHT_CHANNEL, 0);
    }
    else if (post_data.laser_left == 0 && post_data.laser_top == 0) {
      ledcWrite(MPWM_LEFT_CHANNEL, 0);
      ledcWrite(MPWM_RIGHT_CHANNEL, 0);
    }
    else if (post_data.laser_left < post_data.center_left_line) {
      digitalWrite(MDIR_LEFT, LOW);
      digitalWrite(MDIR_RIGHT, HIGH);
      error = abs(post_data.center_left_line - post_data.laser_left);
      turn_speed = (error / post_data.center_left_line) * speed_diff;
      ledcWrite(MPWM_LEFT_CHANNEL, min_speed + turn_speed);
      ledcWrite(MPWM_RIGHT_CHANNEL, max_speed);
    }
    else if (post_data.laser_left > post_data.center_right_line) {
      digitalWrite(MDIR_LEFT, HIGH);
      digitalWrite(MDIR_RIGHT, LOW);
      error = abs(post_data.center_right_line - post_data.laser_left);
      turn_speed = (error / post_data.center_left_line) * speed_diff;
      ledcWrite(MPWM_LEFT_CHANNEL, max_speed);
      ledcWrite(MPWM_RIGHT_CHANNEL, min_speed + turn_speed);
    }
    else {
      digitalWrite(MDIR_LEFT, HIGH);
      digitalWrite(MDIR_RIGHT, HIGH);
      ledcWrite(MPWM_LEFT_CHANNEL, max_speed);
      ledcWrite(MPWM_RIGHT_CHANNEL, max_speed);
    }
    
    if (post_data.landmine_left != 0 && post_data.landmine_top != 0) {
      digitalWrite(LANDMINE, HIGH);
    } else {
      digitalWrite(LANDMINE, LOW);
    }
    
    DynamicJsonDocument return_doc(512);
    return_doc["status"] = "OK";
    return_doc["time_ms"] = millis();
    return_doc["current_distance"] = CURRENT_DISTANCE;
    return_doc["last_distance"] = LAST_DISTANCE;
    return_doc["turn_speed"] = turn_speed;
    return_doc["error"] = error;
    return_doc["center_left_line"] = post_data.center_left_line;
    return_doc["center_right_line"] = post_data.center_right_line;
    return_doc["laser_left"] = post_data.laser_left;
    return_doc["landmine_left"] = post_data.landmine_left;
    return_doc["landmine_top"] = post_data.landmine_top;
    String buf;
    serializeJson(return_doc, buf);
    server.send(201, F("application/json"), buf);
}

void handleNotFound()
{
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text / plain", message);
}

void setup()
{
  Serial1.begin(9600, SERIAL_8N1, 3, 1);
  Serial.begin(115200);

  pinMode(MDIR_LEFT, OUTPUT);
  pinMode(MDIR_RIGHT, OUTPUT);
  pinMode(LANDMINE, OUTPUT);

  ledcSetup(MPWM_LEFT_CHANNEL, 5000, 8);
  ledcAttachPin(MPWM_LEFT, MPWM_LEFT_CHANNEL);
  ledcSetup(MPWM_RIGHT_CHANNEL, 5000, 8);
  ledcAttachPin(MPWM_RIGHT, MPWM_RIGHT_CHANNEL);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  //config.xclk_freq_hz = 20000000;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame parameters
  //  config.frame_size = FRAMESIZE_UXGA;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  cam.init(config);

  IPAddress ip;

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID1, PWD1);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println("");
  Serial.println(ip);
  Serial.print("Stream Link: http://");
  Serial.print(ip);
  Serial.println("/mjpeg/1");
  server.on("/mjpeg/1", HTTP_GET, handle_jpg_stream);
  server.on("/jpg", HTTP_GET, handle_jpg);
  server.on("/control", HTTP_POST, handle_control);
  server.onNotFound(handleNotFound);
  server.begin();
}

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '{';
    char endMarker = '\n';
    char rc;
 
    while (Serial1.available() > 0) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                LAST_DISTANCE = CURRENT_DISTANCE;
                CURRENT_DISTANCE = atoi(receivedChars);
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void loop()
{
  recvWithStartEndMarkers();
  server.handleClient();
  
  if (millis() > LAST_REQUEST_TIMESTAMP + MAX_DELAY_MS) {
      ledcWrite(MPWM_LEFT_CHANNEL, 0);
      ledcWrite(MPWM_RIGHT_CHANNEL, 0);
      digitalWrite(LANDMINE, LOW);
  }
}
