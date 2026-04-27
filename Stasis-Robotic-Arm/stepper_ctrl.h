#ifndef STEPPER_CTRL_H
#define STEPPER_CTRL_H
 
#include <TMCStepper.h>
#include "config.h"
 
struct Joint {
    TMC2209Stepper* driver;
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t diagPin;
    int32_t position;
    int32_t target;
    float   speed;
    float   accel;
    bool    homed;
    bool    moving;
    uint32_t lastStepTime;
    uint32_t stepInterval;
};
 
extern Joint joints[NUM_JOINTS];
 
void steppers_init();
void stepper_move(uint8_t joint, int32_t steps, float speed);
void stepper_home(uint8_t joint);
void stepper_set_current(uint8_t joint, uint16_t mA);
void stepper_emergency_stop();
void stepper_enable_all();
void steppers_update();
 
#endif