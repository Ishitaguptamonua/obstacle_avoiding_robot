#include <AFMotor.h>
#include <Servo.h>

// Define constants
#define F FORWARD
#define B BACKWARD
#define R RELEASE
#define MAX_SPEED 255
#define TURN_SPEED 180
#define TRIG 9
#define ECHO 3
#define THRESHOLD 18

// Define error codes
#define ERROR_NO_ECHO 0
#define ERROR_TIMEOUT -1

//  Variable declarations
AF_DCMotor m3(3);  // Motor 3 & 2
AF_DCMotor m4(4);  // Motor 4 & 1
Servo servoMotor;  // Servo motor attached with ultrasonic sensor
int distance = 0;
char dir = '0';

void setMotors(uint8_t speed) {
  // Set the speed of geared DC motors
  m3.setSpeed(speed);
  m4.setSpeed(speed);
}

void move(int state) {
  // Define the movement state of geared DC motors
  m3.run(state);
  m4.run(state);
}

void turnLeft() {
  // To turn the car left
  m3.run(F);
  m4.run(B);
  delay(500);
  move(R);
}

void turnRight() {
  // To turn the car right
  m3.run(B);
  m4.run(F);
  delay(500);
  move(R);
}

inline void rotateServo(uint8_t angle) {
  servoMotor.write(angle);  // Turn servo to angle
}

int getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);  // Transmit Ultrasonic waves
  delayMicroseconds(10);     // For 10 microsec
  digitalWrite(TRIG, LOW);   // Stop Transmitting Ultrasonic waves

  unsigned long pulseTime = pulseIn(ECHO, HIGH, 20000);  // Get the pulse time in microseconds
  // Check if the pulseTime is 0 or exceeds the timeout (indicating no echo received)
  if (pulseTime == 0 || pulseTime >= 20000) {
    // Return appropriate error code
    return (pulseTime == 0) ? ERROR_NO_ECHO : ERROR_TIMEOUT;
  }
  int distance = (float)pulseTime * 0.0343 / 2;  // Calculate distance using the pulse time

  return distance;  // Return distance
}

inline bool isValidDistance(int distance) {
  return (distance > 0) ? true : false;
}

char getDirection() {
  int dist[2] = { 0, 0 };
  char dir = '0';
  rotateServo(0);
  delay(500);
Right:
  dist[0] = getDistance();
  if (!isValidDistance(dist[0]))  // Check if distance is valid
    goto Right;
  delay(500);  // Wait for 500 msec

  rotateServo(180);
  delay(500);
Left:
  dist[1] = getDistance();
  if (!isValidDistance(dist[1]))  // Check if distance is valid
    goto Left;
  delay(500);  // Wait for 500 msec

  rotateServo(90);
  delay(500);

  if ((dist[0] <= THRESHOLD) && (dist[1] <= THRESHOLD))
    dir = '0';
  else if ((dist[0] > THRESHOLD) || (dist[1] > THRESHOLD)) {
    if (dist[0] > dist[1])
      dir = 'R';
    else if (dist[0] < dist[1])
      dir = 'L';
    else if (dist[0] == dist[1])
      dir = 'L';
  }

  return dir;
}

void setup() {
  pinMode(TRIG, OUTPUT);    // Set TRIG for output
  pinMode(ECHO, INPUT);     // Set ECHO for input
  digitalWrite(TRIG, LOW);  // Set default voltage of TRIG
  setMotors(MAX_SPEED);     // Set the speed of the geared DC motors
  servoMotor.attach(10);    // Attach servo to pin no 10
  rotateServo(90);          // Set servo at 90 degrees
  delay(500);
}

void loop() {
  delay(200);
Start:
  distance = getDistance();
  if (!isValidDistance(distance))  // Check if distance is valid
    goto Start;
  if (distance >= THRESHOLD) {
    move(F);
  }

  while (distance >= THRESHOLD) {
    // Measure the distance after every 50 msec
    delay(50);
    distance = getDistance();
  }
  move(R);  // Stop when distance is less than the threshold

  dir = getDirection();  // Get the direction
  setMotors(TURN_SPEED);

  switch (dir) {
    // Turn to the direction recieved earlier
    case '0': move(B); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
  }
  setMotors(MAX_SPEED);
}
