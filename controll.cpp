
#include "mbed.h"

/**
 *  Sonar class for the HC-SR04
 */
class Sonar {
    DigitalOut   trigger;
    InterruptIn  echo;     // calls a callback when a pin changes
    Timer        timer;
    Timeout      timeout;  // calls a callback once when a timeout expires
    Ticker       ticker;   // calls a callback repeatedly with a timeout
    int32_t      begin;
    int32_t      end;
    float        distance;

public:
    /**
     *  Sonar constructor
     *  Creates a sonar object on a set of provided pins
     *  @param trigger_pin  Pin used to trigger reads from the sonar device
     *  @param echo_pin     Pin used to receive the sonar's distance measurement
     */
    Sonar(PinName trigger_pin, PinName echo_pin) : trigger(trigger_pin), echo(echo_pin) {
        trigger = 0;
        distance = -1;

        echo.rise(callback(this, &Sonar::echo_in));    // Attach handler to the rising interruptIn edge
        echo.fall(callback(this, &Sonar::echo_fall));  // Attach handler to the falling interruptIn edge
    }

    /**
     *  Start the background task to trigger sonar reads every 200ms
     */
    void start(void) {
        ticker.attach(callback(this, &Sonar::background_read), 0.02f);
    }

    /**
     *  Stop the background task that triggers sonar reads
     */
    void stop(void) {
        ticker.detach();
    }

    /**
     *  Interrupt pin rising edge interrupt handler. Reset and start timer
     */
    void echo_in(void) {
        timer.reset();
        timer.start();
        begin = timer.read_us();
    }

    /**
     *  Interrupt pin falling edge interrupt handler. Read and disengage timer.
     *  Calculate raw echo pulse length
     */
    void echo_fall(void) {
        end = timer.read_us();
        timer.stop();
        distance = end - begin;
    }

    /**
     *  Wrapper function to set the trigger pin low. Callbacks cannot take in both object and argument pointers.
     *  See use of this function in background_read().
     */
    void trigger_toggle(void) {
        trigger = 0;
    }

    /**
     *  Background callback task attached to the periodic ticker that kicks off sonar reads
     */
    void background_read(void) {
        trigger = 1;
        timeout.attach(callback(this, &Sonar::trigger_toggle), 10.0e-6);
    }

    /**
     *  Public read function that returns the scaled distance result in cm
     */
    float read(void) {
        return distance / 58.0f;
    }
};

/******* Pin configuration *******/
// right hand side motor
PwmOut aIn1(PC_8);
PwmOut aIn2(PC_9);

// left hand side motor
PwmOut bIn1(PB_8);
PwmOut bIn2(PB_9);
DigitalOut nSleep(PC_5);

Sonar sonarFront(A0, A1);
Sonar sonarLeft(A4, A5);
Sonar sonarRight(D7, D12);

// return parameter for debug
Serial pc(SERIAL_TX, SERIAL_RX);
/******* Pin configuration *******/

/*** Motor control function ***/
void forward(float, float);
void backward(float, float);
void stop();
void turnLeft(float, float);
void turnRight(float, float);
/*** Motor control function ***/
float l_duty_cycle = 1.0f;
float r_duty_cycle = 1.0f;

int main() {
    // Motor (a: right, b: left)
    nSleep = 1;
    aIn1.period_us(20);
    aIn2.period_us(20);
    bIn1.period_us(20);
    bIn2.period_us(20);
    
    // Sonar
    sonarFront.start();
    wait(0.066f);
    sonarLeft.start();
    wait(0.066f);
    sonarRight.start();
    
    forward(1.0f, 1.0f); // default
    while(1) {
        
        float frontDist = sonarFront.read();
        float leftDist = sonarLeft.read();
        float rightDist = sonarRight.read();
        
        // Periodically print results from sonar object
        pc.printf("Front: %f ", frontDist);
        pc.printf("Left: %f ", leftDist);
        pc.printf("Right: %f\r\n", rightDist);
        
        // multi level turning
        if(frontDist < 35.0f) { 
            backward(0.9f, 0.9f);
        }
        else {
            if(leftDist-rightDist > 5.0f) {
                if(rightDist < 15.0f) {
                    turnLeft(1.0f, 1.0f);
                }
                else {
                    turnLeft(0.7f, 1.0f);    
                }    
            }
            else if(rightDist-leftDist > 5.0f) {
                if(leftDist < 15.0f) {
                    turnRight(1.0f, 1.0f);
                }
                else {
                    turnRight(1.0f, 0.7f);
                }    
            }
            else {
                forward(1.0f, 1.0f);    
            }
        }

        pc.printf("aIn1 set to %.2f %%\n\r", aIn1.read() * 100);
        pc.printf("aIn2 set to %.2f %%\n\r", aIn2.read() * 100);
        pc.printf("bIn1 set to %.2f %%\n\r", bIn1.read() * 100);
        pc.printf("bIn2 set to %.2f %%\n\r", bIn2.read() * 100);
    }
}

/**
 *  moving forward
 */
void forward(float l_duty_cycle, float r_duty_cycle) {
    aIn1.pulsewidth_us(int(20*r_duty_cycle));
    aIn2.pulsewidth_us(0);
    bIn1.pulsewidth_us(int(20*l_duty_cycle));
    bIn2.pulsewidth_us(0);
    pc.printf("Forward\n\r");
}

/**
 *  moving backward
 */
void backward(float l_duty_cycle, float r_duty_cycle) {
    aIn1.pulsewidth_us(0);
    aIn2.pulsewidth_us(int(20*r_duty_cycle));
    bIn1.pulsewidth_us(0);
    bIn2.pulsewidth_us(int(20*l_duty_cycle));
    pc.printf("Forward\n\r");
    //wait(0.3f);
}

/**
 *  stop
 */
void stop() {
    aIn1.pulsewidth_us(0);
    aIn2.pulsewidth_us(0);
    bIn1.pulsewidth_us(0);
    bIn2.pulsewidth_us(0);
    pc.printf("Stop\n\r");
}

/**
 *  turning right side with modifiable PWM
 */
void turnRight(float l_duty_cycle, float r_duty_cycle) {
    aIn1.pulsewidth_us(0);
    aIn2.pulsewidth_us(int(20*r_duty_cycle));
    bIn1.pulsewidth_us(int(20*l_duty_cycle));
    bIn2.pulsewidth_us(0);
    pc.printf("Right\n\r");
}

/**
 *  turning left side with modifiable PWM
 */
void turnLeft(float l_duty_cycle, float r_duty_cycle) {
    aIn1.pulsewidth_us(int(20*r_duty_cycle));
    aIn2.pulsewidth_us(0);
    bIn1.pulsewidth_us(0);
    bIn2.pulsewidth_us(int(20*l_duty_cycle));
    pc.printf("Left\n\r");
}
