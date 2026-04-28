#include "stepper_ctrl.h"
 
static HardwareSerial TMCBus1(1);
static HardwareSerial TMCBus2(2);
 
static TMC2209Stepper drv1(&TMCBus1, R_SENSE, 0);
static TMC2209Stepper drv2(&TMCBus1, R_SENSE, 1);
static TMC2209Stepper drv3(&TMCBus1, R_SENSE, 2);
static TMC2209Stepper drv4(&TMCBus1, R_SENSE, 3);
static TMC2209Stepper drv5(&TMCBus2, R_SENSE, 0);
 
Joint joints[NUM_JOINTS];
 
static void configure_driver(TMC2209Stepper& drv, uint16_t currentMA) {
    drv.begin();
    drv.toff(4);
    drv.rms_current(currentMA);
    drv.microsteps(DEFAULT_MICROSTEPS);
    drv.en_spreadCycle(true);
    drv.pwm_autoscale(true);
    drv.TCOOLTHRS(0xFFFFF);
    drv.SGTHRS(50);
    drv.shaft(false);
}
 
static void init_joint(uint8_t idx, TMC2209Stepper* drv,
                       uint8_t step, uint8_t dir, uint8_t diag) {
    joints[idx].driver       = drv;
    joints[idx].stepPin      = step;
    joints[idx].dirPin       = dir;
    joints[idx].diagPin      = diag;
    joints[idx].position     = 0;
    joints[idx].target       = 0;
    joints[idx].speed        = DEFAULT_SPEED;
    joints[idx].accel        = DEFAULT_ACCEL;
    joints[idx].homed        = false;
    joints[idx].moving       = false;
    joints[idx].lastStepTime = 0;
    joints[idx].stepInterval = 0;
 
    pinMode(step, OUTPUT);
    pinMode(dir,  OUTPUT);
    pinMode(diag, INPUT);
    digitalWrite(step, LOW);
    digitalWrite(dir,  LOW);
}
 
void steppers_init() {
    TMCBus1.begin(TMC_BAUD, SERIAL_8N1, TMC_BUS1_RX, TMC_BUS1_TX);
    TMCBus2.begin(TMC_BAUD, SERIAL_8N1, TMC_BUS2_RX, TMC_BUS2_TX);
    delay(100);
 
    configure_driver(drv1, DEFAULT_CURRENT_MA);
    configure_driver(drv2, DEFAULT_CURRENT_MA);
    configure_driver(drv3, DEFAULT_CURRENT_MA);
    configure_driver(drv4, DEFAULT_CURRENT_MA);
    configure_driver(drv5, DEFAULT_CURRENT_MA);
 
    init_joint(0, &drv1, STEP_1, DIR_1, DIAG_1);
    init_joint(1, &drv2, STEP_2, DIR_2, DIAG_2);
    init_joint(2, &drv3, STEP_3, DIR_3, DIAG_3);
    init_joint(3, &drv4, STEP_4, DIR_4, DIAG_4);
    init_joint(4, &drv5, STEP_5, DIR_5, DIAG_5);
 
    Serial.println("[STEPPER] All 5 TMC2209 drivers initialised");
 
    for (int i = 0; i < NUM_JOINTS; i++) {
        uint8_t ver = joints[i].driver->version();
        Serial.printf("[STEPPER] Joint %d TMC2209 version: 0x%02X %s\n",
                      i + 1, ver, (ver == 0x21) ? "(OK)" : "(FAIL)");
    }
}
 
void stepper_move(uint8_t joint, int32_t steps, float speed) {
    if (joint >= NUM_JOINTS) return;
 
    Joint& j = joints[joint];
 
    if (steps < 0) {
        digitalWrite(j.dirPin, HIGH);
        steps = -steps;
    } else {
        digitalWrite(j.dirPin, LOW);
    }
 
    j.target       = j.position + ((digitalRead(j.dirPin) == HIGH) ? -steps : steps);
    j.speed        = speed;
    j.stepInterval = (uint32_t)(1000000.0f / speed);
    j.lastStepTime = micros();
    j.moving       = true;
 
    Serial.printf("[MOVE] Joint %d: %d steps at %.0f steps/s\n",
                  joint + 1, steps, speed);
}
 
void stepper_home(uint8_t joint) {
    if (joint >= NUM_JOINTS) return;
 
    Joint& j = joints[joint];
    Serial.printf("[HOME] Homing joint %d...\n", joint + 1);
 
    uint32_t homeInterval = (uint32_t)(1000000.0f / 100.0f);
    digitalWrite(j.dirPin, HIGH);
    j.driver->SGTHRS(30);
 
    bool stalled = false;
    uint32_t timeout = millis() + 30000;
 
    while (!stalled && millis() < timeout) {
        digitalWrite(j.stepPin, HIGH);
        delayMicroseconds(2);
        digitalWrite(j.stepPin, LOW);
        delayMicroseconds(homeInterval);
 
        if (digitalRead(j.diagPin) == HIGH) {
            stalled = true;
        }
    }
 
    if (stalled) {
        j.position = 0;
        j.target   = 0;
        j.homed    = true;
        Serial.printf("[HOME] Joint %d homed successfully\n", joint + 1);
    } else {
        Serial.printf("[HOME] Joint %d homing TIMEOUT\n", joint + 1);
    }
 
    j.driver->SGTHRS(50);
    j.moving = false;
}
 
void stepper_set_current(uint8_t joint, uint16_t mA) {
    if (joint >= NUM_JOINTS) return;
    joints[joint].driver->rms_current(mA);
    Serial.printf("[CURRENT] Joint %d set to %d mA\n", joint + 1, mA);
}
 
void stepper_emergency_stop() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        joints[i].moving = false;
        joints[i].driver->toff(0);
    }
    Serial.println("[E-STOP] All motors disabled");
}
 
void stepper_enable_all() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        joints[i].driver->toff(4);
    }
    Serial.println("[ENABLE] All motors re-enabled");
}
 
void steppers_update() {
    uint32_t now = micros();
 
    for (int i = 0; i < NUM_JOINTS; i++) {
        Joint& j = joints[i];
        if (!j.moving) continue;
 
        if (j.position == j.target) {
            j.moving = false;
            Serial.printf("[DONE] Joint %d reached position %d\n",
                          i + 1, j.position);
            continue;
        }
 
        if ((now - j.lastStepTime) >= j.stepInterval) {
            digitalWrite(j.stepPin, HIGH);
            delayMicroseconds(2);
            digitalWrite(j.stepPin, LOW);
 
            if (digitalRead(j.dirPin) == LOW) {
                j.position++;
            } else {
                j.position--;
            }
 
            j.lastStepTime = now;
 
            if (digitalRead(j.diagPin) == HIGH) {
                j.moving = false;
                Serial.printf("[STALL] Joint %d stalled at position %d\n",
                              i + 1, j.position);
            }
        }
    }
}