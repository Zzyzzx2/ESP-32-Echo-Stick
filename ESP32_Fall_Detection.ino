#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFiClientSecure.h>


// WiFi credentials
const char* ssid = "Rahan";
const char* password = "rahanshankar";


// Certificate of server
const char* test_root_ca= \
  "-----BEGIN CERTIFICATE-----\n" \
  "MIIDzTCCArWgAwIBAgIQCjeHZF5ftIwiTv0b7RQMPDANBgkqhkiG9w0BAQsFADBa\n" \
  "MQswCQYDVQQGEwJJRTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJl\n" \
  "clRydXN0MSIwIAYDVQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTIw\n" \
  "MDEyNzEyNDgwOFoXDTI0MTIzMTIzNTk1OVowSjELMAkGA1UEBhMCVVMxGTAXBgNV\n" \
  "BAoTEENsb3VkZmxhcmUsIEluYy4xIDAeBgNVBAMTF0Nsb3VkZmxhcmUgSW5jIEVD\n" \
  "QyBDQS0zMFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEua1NZpkUC0bsH4HRKlAe\n" \
  "nQMVLzQSfS2WuIg4m4Vfj7+7Te9hRsTJc9QkT+DuHM5ss1FxL2ruTAUJd9NyYqSb\n" \
  "16OCAWgwggFkMB0GA1UdDgQWBBSlzjfq67B1DpRniLRF+tkkEIeWHzAfBgNVHSME\n" \
  "GDAWgBTlnVkwgkdYzKz6CFQ2hns6tQRN8DAOBgNVHQ8BAf8EBAMCAYYwHQYDVR0l\n" \
  "BBYwFAYIKwYBBQUHAwEGCCsGAQUFBwMCMBIGA1UdEwEB/wQIMAYBAf8CAQAwNAYI\n" \
  "KwYBBQUHAQEEKDAmMCQGCCsGAQUFBzABhhhodHRwOi8vb2NzcC5kaWdpY2VydC5j\n" \
  "b20wOgYDVR0fBDMwMTAvoC2gK4YpaHR0cDovL2NybDMuZGlnaWNlcnQuY29tL09t\n" \
  "bmlyb290MjAyNS5jcmwwbQYDVR0gBGYwZDA3BglghkgBhv1sAQEwKjAoBggrBgEF\n" \
  "BQcCARYcaHR0cHM6Ly93d3cuZGlnaWNlcnQuY29tL0NQUzALBglghkgBhv1sAQIw\n" \
  "CAYGZ4EMAQIBMAgGBmeBDAECAjAIBgZngQwBAgMwDQYJKoZIhvcNAQELBQADggEB\n" \
  "AAUkHd0bsCrrmNaF4zlNXmtXnYJX/OvoMaJXkGUFvhZEOFp3ArnPEELG4ZKk40Un\n" \
  "+ABHLGioVplTVI+tnkDB0A+21w0LOEhsUCxJkAZbZB2LzEgwLt4I4ptJIsCSDBFe\n" \
  "lpKU1fwg3FZs5ZKTv3ocwDfjhUkV+ivhdDkYD7fa86JXWGBPzI6UAPxGezQxPk1H\n" \
  "goE6y/SJXQ7vTQ1unBuCJN0yJV0ReFEQPaA1IwQvZW+cwdFD19Ae8zFnWSfda9J1\n" \
  "CZMRJCQUzym+5iPDuI9yP+kHyCREU3qzuWFloUwOxkgAyXVjBYdwRVKD05WdRerw\n" \
  "6DEdfgkfCv4+3ao8XnTSrLE=\n" \
  "-----END CERTIFICATE-----\n";

// Define the MPU6050 object
Adafruit_MPU6050 mpu;

// Define the threshold values for fall detection
#define FALL_ACCELERATION_THRESHOLD 15 // Threshold for accelerometer values (in m/s^2)
#define UPRIGHT_ORIENTATION_THRESHOLD 3 // Threshold for upright orientation (in m/s^2)

// Variables to store the sensor data
float ax, ay, az;
float gx, gy, gz;

// Flag to indicate if a fall has been detected
bool fallDetected = false;

// Timestamp to track the last SMS sent
unsigned long lastSMSSentTimestamp = -600000;
#define SMS_SEND_INTERVAL 600000 // 10 minutes in milliseconds

void setup() {
  Serial.begin(115200);
  Wire.begin();
  connectToWiFi();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
}

void loop() {
  // Read the sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Extract the acceleration and rotation data
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;

  // Check for rapid acceleration
  if (sqrt(ax * ax + ay * ay + az * az) > FALL_ACCELERATION_THRESHOLD) {
    Serial.println("Acceleration Threshold Reached!");

    // Check the orientation of the sensor
    if (fabs(ay - 9.8) < UPRIGHT_ORIENTATION_THRESHOLD) {
      // The person is upright
      fallDetected = false;
    } else if (fabs(ax) > fabs(ay) && fabs(ax) > fabs(az)) {
      // The person has fallen sideways
      fallDetected = true;
    } else if (fabs(az) > fabs(ay) && fabs(az) > fabs(ax)) {
      // The person has fallen forwards or backwards
      fallDetected = true;
    } else {
      // The orientation is not clear, do not report a fall
      fallDetected = false;
    }

    if (fallDetected) {
      Serial.println("Fall detected!");
      unsigned long currentTimestamp = millis();
      if (currentTimestamp - lastSMSSentTimestamp >= SMS_SEND_INTERVAL) {
        sendTwilioSMS();
        lastSMSSentTimestamp = currentTimestamp;
      }
    }
  } else {
    fallDetected = false;
  }

  // Print the sensor data
  Serial.print("Acceleration (m/s^2): ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.println(az);

  Serial.print("Rotation (deg/s): ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.println(gz);

  delay(100);
}

void sendTwilioSMS() {
  while (true) {
    if (WiFi.status() == WL_CONNECTED) { // Check if the device is connected to WiFi
      WiFiClientSecure client;
      client.setTimeout(30000);
      HTTPClient http;
      client.setCACert(test_root_ca);
      Serial.println("Sending Message...");

      // Prepare the HTTPS request
      http.begin(client, "https://smartwatercan-server.onrender.com/get-sms");

      // Send the HTTPS GET request
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0) {
        Serial.print("Connected to Server and trying to send... ");
        String response = http.getString();
        Serial.println("HTTP Response code: " + String(httpResponseCode));
        Serial.println("HTTP Response body: " + response);
        break; // Break out of the while loop if the request was successful
      } else {
        Serial.print("Error on sending GET: ");
        Serial.println(http.errorToString(httpResponseCode).c_str());
        delay(1000); // Wait for 1 second before trying again
      }

      // Clean up
      http.end();
    } else {
      Serial.println("WiFi Disconnected");
      delay(1000); // Wait for 1 second before checking the WiFi status again
    }
  }
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}