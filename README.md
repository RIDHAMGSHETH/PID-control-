# PID Position Control System for Encoder

## Overview

This project implements a PID controller to maintain precise position control using an encoder. The system utilizes an AVR microcontroller to read encoder signals, calculate errors, and adjust motor position accordingly. The main components of the system are the PID control algorithm, encoder signal handling via interrupts, and motor direction control.

## Features

- **PID Control**: The system uses Proportional (P) and Derivative (D) control to minimize position errors.
- **Encoder Signal Handling**: Interrupt Service Routines (ISRs) handle encoder signals for accurate position tracking.
- **Serial Communication**: The desired position can be set via a serial interface, allowing dynamic control and monitoring.
- **Motor Control**: Motor direction and speed are adjusted based on the calculated PID output.

## Components

- **AVR Microcontroller**: The core component that executes the PID control algorithm and handles encoder inputs.
- **Encoder**: Provides feedback on the current position of the motor.
- **Motor**: The actuator that moves to the desired position.
- **Serial Interface**: Allows communication with the microcontroller to set the desired position.

## Functional Description

### Initialization
```c
void init() {
  // Initialize pins for encoder inputs and motor outputs
  DDRE &= ~(1 << PE4) | (1 << PE5);
  PORTE |= (1 << PE4) | (1 << PE5);
  
  // Configure external interrupts for encoder signals
  EICRB |= (1 << ISC40) | (1 << ISC50);
  EIMSK |= (1 << INT4) | (1 << INT5);
  
  // Initialize Timer/Counter for PWM control of the motor
  DDRH |= (1 << PH6) | (1 << PH5);
  TCCR4A = 0xAA;
  TCCR4B = 0x19;
  ICR4 = 1000;
  
  // Begin serial communication for input and monitoring
  Serial.begin(9600);
}
```

### Main Loop
```c
int main() {
  init();
  
  while (1) {
    // Check for new position inputs from serial interface
    if (Serial.available()) {
      n = Serial.parseFloat();
    }

    // Calculate desired position and execute PID control
    a = ((n * 1300.00) / 360.00);
    pid();
    
    // Adjust motor direction and speed based on PID output
    dir(error);
    if ((error == 0) || (error == -1) || (error == 1) || (error == -2) || (error == 2)) {
      OCR4C = 0;
    }
    else {
      OCR4C = s;
    }

    // Output current status via serial interface
    Serial.print(a);
    Serial.print("  ");
    Serial.println(count);
  }
}
```

### PID Control
```c
void pid() {
  // Calculate error
  error = a - count;
  etdt = (error - pe);
  
  // Compute proportional and derivative terms
  p = kp * error;
  d = kd * etdt;
  
  // Update motor speed using PID output
  s = 1000 + p + d;
  pe = error;
}
```

### Encoder Interrupts
```c
ISR(INT5_vect) {
  // Read encoder signals
  csa = ~PINE & 0x10;
  csb = ~PINE & 0x20;
  
  // Update position count based on signal transitions
  // Handle changes in encoder signals
  // Determine direction of movement based on signal transitions
}

ISR(INT4_vect) {
  // Read encoder signals
  csa = ~PINE & 0x10;
  csb = ~PINE & 0x20;
  
  // Update position count based on signal transitions
  // Handle changes in encoder signals
  // Determine direction of movement based on signal transitions
}
```

## Usage

1. **Setup**: Connect the encoder and motor to the appropriate pins on the AVR microcontroller.
2. **Power Up**: Upload the code to the microcontroller and power up the system.
3. **Set Position**: Use a serial terminal to input the desired position in degrees.
4. **Monitor**: The system will adjust the motor to reach and maintain the set position, outputting current status via the serial interface.

## Conclusion

This project provides a robust solution for precise position control using a PID algorithm and encoder feedback. It demonstrates key concepts in control systems, embedded programming, and real-time signal handling.
