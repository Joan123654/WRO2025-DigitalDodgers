This repository contains **documentation, source code, team photos, vehicle photos, and 3D models**, all related to the complete engineering process for the **WRO Future Engineers 2025** competition.

## Table of Contents  

- [Future Engineers – WRO 2025](#future-engineers--wro-2025)
- [CODE](#code)
  - [Sensors Module](#sensors-module)
    - [Functions & Pins](#functions--pins)
    - [Its Function](#its-function)
    - [Points to Take in Consider](#points-to-take-in-consider)
  - [Motor Control Module](#motor-control-module)
    - [Functions & Pins](#functions--pins-1)
    - [Its Function](#its-function-1)
    - [Points to Take in Consider](#points-to-take-in-consider-1)

# Future Engineers – WRO 2025

The Future Engineers category of the WRO 2025 focuses on developing real-world engineering skills through hands-on robotics challenges. This competition encourages creativity, problem-solving, and teamwork while giving participants the opportunity to design, build, and program advanced robotic systems.

Each year, the challenge changes, requiring teams not only to solve the current problem but also to adapt quickly to new scenarios and requirements. This helps participants learn how to think like engineers: iterating on their designs, testing solutions, and improving their approach as they go.

As part of the competition, teams are required to document their entire project process in a GitHub repository. This includes sharing their design decisions, code, and testing results, as well as reflecting on what worked, what didn’t, and how their solution evolved over time. The documentation is an important part of the evaluation, as it demonstrates each team’s ability to communicate their engineering process clearly and professionally.

# CODE

The code integrates three main types of sensors and multiple actuators to create an autonomous robot that can sense obstacles with ultrasonic sensors, steer with a servo, drive forward using a DC motor driver, and detect and follow visual targets using a Pixy2 camera. A push button toggles the robot between active and inactive states. The code is organized in functional blocks (modules) that handle sensing, actuation, decision-making and user control.

Below each module is expanded with implementation details, rationale, pitfalls, and practical tips for tuning and testing.

## Sensors module

### Functions & pins

`````
// Front
const int triggerPinFront = 7;  
const int echoPinFront = 8;
// Left
const int triggerPinLeft = 24;  
const int echoPinLeft = 22;
// Right
const int triggerPinRight = 40; 
const int echoPinRight = 42;

long readDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW); 

  float time = pulseIn(echoPin, HIGH);
  float distance = (time/2)/29.1;
  return distance;  
}
`````
### Its function

Each ultrasonic sensor (HC-SR04-style) measures distance to the nearest obstacle in its direction.

readDistance(triggerPin, echoPin) emits an ultrasonic pulse (trigger), measures the echo time with pulseIn(), and converts the round-trip microsecond time into centimeters using the formula:

distance_cm = (pulseTime_us / 2) / 29.1

(pulseIn returns microseconds; dividing by 2 accounts for the two-way trip; 29.1 μs/cm is the approximate time it takes sound to travel 1 cm at room temperature).

### Points to take in consider

**Range & placement:** Sensor blind zones (very near objects) and wide-beam angles affect readings.

**Multiple Readings:** If all sensors triggers simultaneously, echoes can be mixed up.

## Motor control module

### Functions & pins

`````
const int IN1 = 28;   
const int IN2 = 30;   
const int ENA = 3;

void motorEncendido(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  analogWrite(ENA, speed); 
}
`````

### Its function

motorEncendido(int speed) sets the direction pins and uses analogWrite(ENA, speed) to control motor speed via PWM.

In the code digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); is used for forward direction only.

### Points to take in consider

**Power supply:** Motor motors can draw large currents and cause voltage dips.


