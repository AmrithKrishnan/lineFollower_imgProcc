
#if !defined ESP32
#error Wrong board selected
#endif

#define CAMERA_MODEL_AI_THINKER

#include "esp_camera.h"      
#include "driver/ledc.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include<WiFi.h>
#include<WebServer.h>

const char* ssid = "Amrith";
const char* password = "amrikrish";
WebServer server(80);

bool start = false;

unsigned long startTime = 0;
unsigned long elapsedTime = 0;
bool isRunning = false;
unsigned long int prevMillis = 0;



void webpage(){
  String html = 
    "<html><body>"
    "<h1>Line-follower Bot Control</h1>"
    "<p>Click the buttons to control the start/stop of the bot</p>"
    "<form action='/start' method='POST'><input type='submit' style='background-color: green;' value='START'></form>"
    "<form action='/stop' method='POST'><input type='submit' style='background-color: red;' value='STOP'></form>"
    "<h2>Timer</h2>"
    "<h3>Elapsed Time: " +
    String(elapsedTime / 1000) +
    " seconds</h3>"
    "</body></html>";
  server.send(200, "text/html", html);
}

void startButton() {
  if (!isRunning) {
    elapsedTime = 0;
    startTime = millis();
    isRunning = true;
  }
  start = true;
  String html =
      "<html><body>"
      "<h1>Line-follower Bot Control</h1>"
      "<p>Click the buttons to control the start/stop of the bot</p>"
      "<form action='/start' method='POST'><input type='submit' style='background-color: green;' value='START'></form>"
      "<form action='/stop' method='POST'><input type='submit' style='background-color: red;' value='STOP'></form>"
      "<p>Click the buttons to control the start/stop of the bot</p>"
      "<h2>Timer</h2>"
      "<h3 id='timer'>Elapsed Time: " +
      String(elapsedTime / 1000) +
      " seconds</h3>"
      "<script>"
      "setInterval(updateTimer, 1000);"
      "function updateTimer() {"
      "   var timer = document.getElementById('timer');"
      "   var elapsed = parseInt(timer.innerText.split(':')[1].trim());"
      "   elapsed += 1;"
      "   timer.innerText = 'Elapsed Time: ' + elapsed + ' seconds';"
      "}"
      "</script>"
      "</body></html>";
  server.send(200, "text/html", html);
}

void stopButton(){
  if (isRunning) {
    elapsedTime += millis() - startTime;
    isRunning = false;
  }
  start = false;
  String stopHtml = 
    "<html><body>"
    "<h1>Line-follower Bot Control</h1>"
    "<p>Click the buttons to control the start/stop of the bot</p>"
    "<form action='/start' method='POST'><input type='submit' style='background-color: green;' value='START'></form>"
    "<form action='/stop' method='POST'><input type='submit' style='background-color: red;' value='STOP'></form>"
    "<p>Line-follower Bot Stopped !!</p>"
    "<h2>Timer</h2>"
    "<h3>Elapsed Time: " +
    String(elapsedTime / 1000) +
    " seconds</h3>"
    "</body></html>";
  server.send(200, "text/html", stopHtml);
}


//! Image resolution:
/*!
    default = "const framesize_t FRAME_SIZE_IMAGE = FRAMESIZE_VGA"
    Other available Frame Sizes:
    160x120 (QQVGA), 128x160 (QQVGA2), 176x144 (QCIF), 240x176 (HQVGA),
    320x240 (QVGA), 400x296 (CIF), 640x480 (VGA, default), 800x600 (SVGA),
    1024x768 (XGA), 1280x1024 (SXGA), 1600x1200 (UXGA)
*/
const framesize_t FRAME_SIZE_IMAGE = FRAMESIZE_240X240;
int Wsize = 240, Hsize = 240;
//! Image Format
/*!
    Other Available formats:
    YUV422, GRAYSCALE, RGB565, JPEG, RGB888
*/
#define PIXFORMAT PIXFORMAT_GRAYSCALE

//! Camera exposure
/*!
    Range: (0 - 1200)
    If gain and exposure both set to zero then auto adjust is enabled
*/
int cameraImageExposure = 0;

//! Image gain
/*!
    Range: (0 - 30)
    If gain and exposure both set to zero then auto adjust is enabled
*/
int cameraImageGain = 0;

const uint8_t ledPin = 4;                  ///< onboard Illumination/flash LED pin (4)
unsigned int ledBrightness = 0;            ///< Initial brightness (0 - 255)
const int pwmFrequency = 50000;            ///< PWM settings for ESP32
const uint8_t ledChannel = LEDC_CHANNEL_0; ///< Camera timer0
const uint8_t pwmResolution = 8;           ///< resolution (8 = from 0 to 255)

const int serialSpeed = 115200;            ///< Serial data speed to use

unsigned long int prevmillis1 = 0, prevmillis2 = 0;

const int rightMotor1 = 12;                 
const int rightMotor2 = 13;
const int leftMotor1 = 15;
const int leftMotor2 = 14;

const int lineThreshold = 100;   ///< Line detection threshold (adjust as needed)
uint8_t ROI_width = 200, ROI_height = 50; ///< only even numbers
uint8_t height1, height2, width1, width2;


//! Camera setting
/*!
    Camera settings for CAMERA_MODEL_AI_THINKER OV2640
    Based on CameraWebServer sample code by ESP32 Arduino
*/
#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#endif

/**************************************************************************/
/**
  Set ROI values
*/
/**************************************************************************/
void ROI_calculation(){
  height1 = Hsize/2 - ROI_height/2 ;
  height2 = Hsize/2 + ROI_height/2 ;
  width1 = Wsize/2 - ROI_width/2 ;
  width2 = Wsize/2 + ROI_width/2 ;
}
/**************************************************************************/
/**
  Camera Image Settings
  Set Image parameters
  Based on CameraWebServer sample code by ESP32 Arduino
  \return true: successful, false: failed
*/
/**************************************************************************/
bool cameraImageSettings()
{

  sensor_t *s = esp_camera_sensor_get();
  if (s == nullptr)
  {
    Serial.println("Error: problem reading camera sensor settings");
    return false;
  }

  // if both set to zero enable auto adjust
  if (cameraImageExposure == 0 && cameraImageGain == 0)
  {
    // enable auto adjust
    s->set_gain_ctrl(s, 1);     // auto gain on
    s->set_exposure_ctrl(s, 1); // auto exposure on
    s->set_awb_gain(s, 1);      // Auto White Balance enable (0 or 1)
    s->set_hmirror(s, 1);
    s->set_vflip(s, 1);
  }
  else
  {
    // Apply manual settings
    s->set_gain_ctrl(s, 0);                   // auto gain off
    s->set_awb_gain(s, 1);                    // Auto White Balance enable (0 or 1)
    s->set_exposure_ctrl(s, 0);               // auto exposure off
    s->set_agc_gain(s, cameraImageGain);      // set gain manually (0 - 30)
    s->set_aec_value(s, cameraImageExposure); // set exposure manually  (0-1200)
  }

  return true;
}

/**************************************************************************/
/**
  Initialise Camera
  Set camera parameters
  Based on CameraWebServer sample code by ESP32 Arduino
  \return true: successful, false: failed
*/
/**************************************************************************/
bool initialiseCamera()
{
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
  config.xclk_freq_hz = 20000000; // 20000000 ori //new 1000000
  config.pixel_format = PIXFORMAT;
  config.frame_size = FRAME_SIZE_IMAGE;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // Check the esp32cam board has a PSRAM chip installed (extra memory used for storing captured images)
  // Note: if not using "AI thinker esp32 cam" in the Arduino IDE, PSRAM must be enabled
  if (!psramFound())
  {
    Serial.println("Warning: No PSRam found so defaulting to image size 'CIF'");
    config.frame_size = FRAMESIZE_CIF;
  }

  esp_err_t camera = esp_camera_init(&config); // initialise the camera
  if (camera != ESP_OK)
  {
    Serial.printf("ERROR: Camera init failed with error 0x%x", camera);
  }

  cameraImageSettings(); // Apply custom camera settings

  return (camera == ESP_OK); // Return boolean result of camera initialisation
}

/**************************************************************************/
/*!
  \brief  Threshold the image
*/
/**************************************************************************/
bool threshold(camera_fb_t* img_var)
{
  if (img_var->len == 0)
    return false; // error if image length is 0

  // Iterate over the pixels within the specified boundaries
  for (int y = height1; y < height2; y++) {
    for (int x = width1; x < width2; x++) {
      int pixelIndex = y * img_var->width + x;
      if (img_var->buf[pixelIndex] > lineThreshold) {
        img_var->buf[pixelIndex] = 255; // if brighter than threshold, make it full bright
      } else {
        img_var->buf[pixelIndex] = 0;
      }
    }
  }

  return true;
}

/**************************************************************************/
/*!
  \brief  detectPoint function
  function to detect the centroid of the ROI of the path
*/
/**************************************************************************/

int detectLine(camera_fb_t *fb) {
  int linePosition = 0;
  int lineSum = 0;
  int lineCount = 0;
//------with ROI-------------
  for (int y = height1 ; y <= height2; y++) {
    for (int x = width1 ; x <= width2; x++) {
      uint8_t pixel = fb->buf[y * fb->width + x];
      if (pixel == 0) {
        lineSum += x;
        lineCount++;
      }
    }
  }

  if (lineCount > 0) {
    linePosition = lineSum / lineCount;
  }

  if (lineSum == 0){
    return 0; // for end of black path
  }

  return linePosition;
}
/**************************************************************************/
/*!
  \brief  motor control function
  function to control moors as per pos
*/
/**************************************************************************/
void motorLeftOrRight(int pos){
  Serial.println(pos);

  if(pos == 0){
    // left and right off
    Serial.println("both motors stop");
    //-----left stop-----
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);

    //-----right stop-----
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
  }
  else if(pos >= Wsize/2 - 15 && pos <= Wsize/2 + 15){
    // drive straight
    Serial.println("both motors forward");
    //-----left straight-----
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);

    //-----right straight-----
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
  }
  else if(pos > Wsize/2 + 15){
    // small left turn
    Serial.println("small left turn -- left stop, right forward");
    //-----left stop-----
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);

    //-----right straight-----
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
  }
  else if(pos < Wsize/2 - 15){
    // small right turn
    Serial.println("small right turn -- left forward, right stop");
    //-----left straight-----
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);

    //-----right stop-----
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
  }
}
/**************************************************************************/
/*!
  \brief  main function to do all the processing
*/
/**************************************************************************/
void mainFunctionForProcessing()
{
  camera_fb_t *fb = nullptr; ///< Pointer to camera buffer
  bool status_OK = false; ///< Boolean indicating if the picture has been taken correctly

    Serial.println("Capturing image...");

    fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Camera capture failed");
        return;
    }

    if(threshold(fb))
      Serial.println("Thresholding successful"); ///< Threshold the path   

    motorLeftOrRight(detectLine(fb));

    // Return camera buffer for future capture
    esp_camera_fb_return(fb);
}

/**************************************************************************/
/*!
  \brief  Setup function
  Initialization for following:
    disable Brownout detection
    camera
*/
/**************************************************************************/
void setup()
{
  Serial.begin(serialSpeed); ///< Initialize serial communication
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..."); 
  }
  Serial.println("Connected to WiFi");

  server.on("/", webpage);
  server.on("/start", startButton);
  server.on("/stop", stopButton);

  server.begin();
  Serial.println("Web server started");

  // Print the IP address
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); ///< Disable 'brownout detector'

  Serial.print("\nInitialising camera: "); ///< Camera check
  if (initialiseCamera())
  {
    Serial.println("OK");
  }
  else
  {
    Serial.println("Error!");
    return;
  }
  ROI_calculation();
}

/**************************************************************************/
/*!
  \brief  Loop function
  Capture image when button pressed on webpage.
*/
/**************************************************************************/
void loop()
{  

server.handleClient();

if(start){
  //-----the mainFunctionForProcessing() will give next values for the motors------
  delay(50);
  mainFunctionForProcessing();

  delay(100); // the motors will be on this long
  //---------switch off both the motors now----------
  //-----left stop-----
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);

  //-----right stop-----
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
}
//--------- to next cycle now-----------
} 