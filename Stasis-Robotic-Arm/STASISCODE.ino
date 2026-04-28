#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
#include "config.h" 
#include "stepper_ctrl.h"
 
static Servo gripper;
static int   servoAngle = 90;
 
static char   cmdBuf[SERIAL_BUF_SIZE];
static size_t cmdIdx = 0;
 
static bool wifiConnected = false;
 
void wifi_init() {
    Serial.printf("[WIFI] Connecting to %s", MY_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.begin(MY_SSID, MY_PASS);
 
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 10000) {
        delay(250);
        Serial.print(".");
    }
 
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.printf("\n[WIFI] Connected: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WIFI] Not connected — OTA unavailable, USB only");
    }
}
 
void ota_init() {
    if (!wifiConnected) return;
 
    ArduinoOTA.setHostname(OTA_HOSTNAME);
 
    ArduinoOTA.onStart([]() {
        stepper_emergency_stop();
        Serial.println("[OTA] Update starting...");
    });
 
    ArduinoOTA.onEnd([]() {
        Serial.println("[OTA] Update complete. Rebooting...");
    });
 
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] %u%%\r", (progress * 100) / total);
    });
 
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error %u: ", error);
        switch (error) {
            case OTA_AUTH_ERROR:    Serial.println("Auth failed");    break;
            case OTA_BEGIN_ERROR:   Serial.println("Begin failed");   break;
            case OTA_CONNECT_ERROR: Serial.println("Connect failed"); break;
            case OTA_RECEIVE_ERROR: Serial.println("Receive failed"); break;
            case OTA_END_ERROR:     Serial.println("End failed");     break;
        }
    });
 
    ArduinoOTA.begin();
    Serial.printf("[OTA] Ready at %s.local\n", OTA_HOSTNAME);
}
 
void print_help() {
    Serial.println("=== Stasis Robotic Arm ===");
    Serial.println("M <joint> <steps> <speed>  Move joint (neg steps=reverse)");
    Serial.println("H <joint>                  Home joint via StallGuard");
    Serial.println("S <angle>                  Set servo (0-180)");
    Serial.println("C <joint> <mA>             Set motor current");
    Serial.println("P                          Print all positions");
    Serial.println("E                          Emergency stop");
    Serial.println("R                          Re-enable motors");
    Serial.println("?                          This help");
}
 
void process_command(const char* cmd) {
    if (cmd[0] == '\0') return;
 
    char type = cmd[0];
    int joint, steps, currentMA, angle;
    float speed;
 
    switch (type) {
        case 'M':
        case 'm': {
            int parsed = sscanf(cmd + 1, "%d %d %f", &joint, &steps, &speed);
            if (parsed == 3 && joint >= 1 && joint <= NUM_JOINTS) {
                stepper_move(joint - 1, steps, speed);
            } else if (parsed == 2 && joint >= 1 && joint <= NUM_JOINTS) {
                stepper_move(joint - 1, steps, DEFAULT_SPEED);
            } else {
                Serial.println("[ERR] Usage: M <joint 1-5> <steps> [speed]");
            }
            break;
        }
 
        case 'H':
        case 'h': {
            if (sscanf(cmd + 1, "%d", &joint) == 1 &&
                joint >= 1 && joint <= NUM_JOINTS) {
                stepper_home(joint - 1);
            } else {
                Serial.println("[ERR] Usage: H <joint 1-5>");
            }
            break;
        }
 
        case 'S':
        case 's': {
            if (sscanf(cmd + 1, "%d", &angle) == 1) {
                angle = constrain(angle, 0, 180);
                gripper.write(angle);
                servoAngle = angle;
                Serial.printf("[SERVO] Angle set to %d\n", angle);
            } else {
                Serial.println("[ERR] Usage: S <angle 0-180>");
            }
            break;
        }
 
        case 'C':
        case 'c': {
            if (sscanf(cmd + 1, "%d %d", &joint, &currentMA) == 2 &&
                joint >= 1 && joint <= NUM_JOINTS) {
                stepper_set_current(joint - 1, (uint16_t)currentMA);
            } else {
                Serial.println("[ERR] Usage: C <joint 1-5> <mA>");
            }
            break;
        }
 
        case 'P':
        case 'p': {
            for (int i = 0; i < NUM_JOINTS; i++) {
                Serial.printf("Joint %d: pos=%d homed=%s moving=%s\n",
                    i + 1,
                    joints[i].position,
                    joints[i].homed ? "yes" : "no",
                    joints[i].moving ? "yes" : "no");
            }
            Serial.printf("Servo: %d deg\n", servoAngle);
            break;
        }
 
        case 'E':
        case 'e':
            stepper_emergency_stop();
            break;
 
        case 'R':
        case 'r':
            stepper_enable_all();
            break;
 
        case '?':
            print_help();
            break;
 
        default:
            Serial.printf("[ERR] Unknown command: %c\n", type);
            break;
    }
}
 
void setup() {
    Serial.begin(115200);
    delay(2000);
 
    Serial.println();
    Serial.println("Stasis 6-DOF Robotic Arm v0.1");
 
    steppers_init();
 
    gripper.setPeriodHertz(50);
    gripper.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    gripper.write(servoAngle);
 
    wifi_init();
    ota_init();
    print_help();
 
    Serial.println("[READY]");
}
 
void loop() {
    if (wifiConnected) {
        ArduinoOTA.handle();
    }
 
    steppers_update();
 
    while (Serial.available()) {
        char c = Serial.read();
 
        if (c == '\n' || c == '\r') {
            cmdBuf[cmdIdx] = '\0';
            if (cmdIdx > 0) {
                process_command(cmdBuf);
            }
            cmdIdx = 0;
        } else if (cmdIdx < SERIAL_BUF_SIZE - 1) {
            cmdBuf[cmdIdx++] = c;
        }
    }
}
 






