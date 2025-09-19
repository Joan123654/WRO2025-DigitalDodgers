// ------------------------Sensores-------------------------

const int triggerPinFront = 7;  const int echoPinFront = 8;
const int triggerPinLeft = 24;  const int echoPinLeft = 22;
const int triggerPinRight = 40; const int echoPinRight = 42;

// -------------------------Motor---------------------------

const int IN1 = 28;   
const int IN2 = 30;   
const int ENA = 3;

// -----------------------Constantes------------------------

const int umbralFrontal = 50;
const int umbralDerecho = 40;
const int umbralIzquierdo = 40; 
int velocidad = 165;             
int turnTime = 750;    

unsigned long tiempoPrevio = 0;
unsigned long tiempoActual;

// --------------------------Pixy----------------------------
#include <PIDLoop.h>
#include <Pixy2.h>
Pixy2 pixy;
float deadZone = 0.15;

// -----------------------Servomotor-------------------------

#include <Servo.h>
Servo miServo;
const int pinServo = 19;
int servoCenter = 85;
int servoLeft = 55;
int servoRight = 110;

int contadorCurvas = 0;
int deadband = 4;

//--------------------------Botón-----------------------------

const int buttonPin = A0;
int buttonOld = 1;
int buttonNew;
int codeState = 0;

//----------------------Control de giros----------------------

int giroDireccion = 0; // 1 = derecha, 2 = izquierda

//------------------------------------------------(Funciones)-------------------------------------------------------

void startWorking() {
  Serial.println("Starting...");
  miServo.write(servoCenter);
  motorEncendido(velocidad);
  pixy.setLamp(1, 0); 
}

void stopWorking() {
  Serial.println("Stopping...");
  motorEncendido(0);
  miServo.write(servoCenter);
  pixy.setLamp(0, 0); 
}

void detectarBoton() {
  buttonNew = digitalRead(buttonPin);
  if (buttonNew == LOW && buttonOld == HIGH) {
    codeState = !codeState;
    if (codeState) {startWorking(); Serial.println("Inicializando...");}
    else {stopWorking(); Serial.println("Deteniendo...");}
  }
  buttonOld = buttonNew;
}

void printDistances(long L, long F, long R) {
  Serial.print("Izq: "); Serial.print(L); Serial.print(" cm | ");
  Serial.print("Front: "); Serial.print(F); Serial.print(" cm | ");
  Serial.print("Der: "); Serial.println(R);
}

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
 
void motorEncendido(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  analogWrite(ENA, speed); 
}


void giroDerecha() {
  miServo.write(servoRight);
  Serial.println("Iniciando giro derecha...");
  giroDireccion = 1;
  contadorCurvas++;
}

void giroIzquierda() {
  miServo.write(servoLeft);
  Serial.println("Iniciando giro izquierda...");
  giroDireccion = 2;
  contadorCurvas++;
}

void mantenerCentro(long distL, long distR) {
  if (distL - distR > deadband) {
    miServo.write(75);   
  } 
  else if (distR - distL > deadband) {
    miServo.write(105);  
  } 
  else {
    miServo.write(90); 
  }
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void followObject() {
  if (pixy.ccc.numBlocks > 0) {
    float width, height, x, y, cx, cy;
    width = pixy.ccc.blocks[0].m_width;     
    height = pixy.ccc.blocks[0].m_height;
    x = pixy.ccc.blocks[0].m_x;             
    y = pixy.ccc.blocks[0].m_y;
    cx = (x + (width / 2));                 
    cy = (y + (height / 2));
    cx = mapfloat(cx, 0, 320, -1, 1);       
    cy = mapfloat(cy, 0, 200, 1, -1);

    if (cx > -deadZone && cx < deadZone) {
      cx = 0;
    }

    // Lógica de dirección
    if (cx < 0) { 
      miServo.write(servoLeft);  // girar izquierda
      Serial.println("Giro izquierda ");
    } 
    else if (cx > 0) { 
      miServo.write(servoRight);  // girar derecha
      Serial.println("Giro derecha ");
    } 
    else {
      miServo.write(servoCenter);  // recto
      Serial.println("Recto ");
    }
  }
}

void setup() {
  Serial.begin(9600);

  pixy.init();

  pinMode(triggerPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);

  pinMode(triggerPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);  

  pinMode(triggerPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT); 

  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);  
  pinMode(ENA, OUTPUT);

  miServo.attach(pinServo);

  pinMode(buttonPin, INPUT_PULLUP);

  miServo.write(servoCenter);
}

// ----------------------------------------------------------------------(loop)------------------------------------------------------------------------
void loop() {

  detectarBoton(); 

  if (codeState == 1){
    
    long distF = readDistance(triggerPinFront, echoPinFront);     
    long distL = readDistance(triggerPinLeft, echoPinLeft);  
    long distR = readDistance(triggerPinRight, echoPinRight);  
    printDistances(distL, distF, distR);

      int accion = 0;  // 0 = centro, 1 = derecha, 2 = izquierda

    if ((distL >= umbralIzquierdo) && (distR >= umbralDerecho)) {
      accion = 0; 
    } else if (distL < distR) {
      accion = 1; 
    } else {
      accion = 2;
    }

    switch (accion) {
      case 0:
        miServo.write(servoCenter);
        break;
      case 1:
        giroDerecha();
        break;
      case 2:
        giroIzquierda();
        break;
    }
  }
}