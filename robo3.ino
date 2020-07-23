#include <Servo.h>
#include <NewPing.h>

#define pinServo 3
#define TRIGGER_PIN  A0  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A1  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

Servo servo1;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

float DistanciaemCM = 0;

#define MotorLadoEsquerdo1 6
#define MotorLadoEsquerdo2 7

#define MotorLadoDireito1 8
#define MotorLadoDireito2 10

#define VelocidadeMotorLadoEsquerdo 9
#define VelocidadeMotorLadoDireito 11

// velocidades do motores
int ValorVelocidadeMotorLadoEsquerdo = 110;
int ValorVelocidadeMotorLadoDireito = 110;


// valores da distancia dos lados
int ValorDistanciaLadoEsquerdo;
int ValorDistanciaLadoDireito;
int ValorDistanciaFrente;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  servo1.attach(pinServo);

  pinMode(VelocidadeMotorLadoEsquerdo, OUTPUT);
  pinMode(VelocidadeMotorLadoDireito, OUTPUT);
  pinMode(MotorLadoEsquerdo1, OUTPUT);
  pinMode(MotorLadoEsquerdo2, OUTPUT);
  pinMode(MotorLadoDireito1, OUTPUT);
  pinMode(MotorLadoDireito2, OUTPUT);   

  delay(3000);
}

void loop() {
  //Serial.print("Posição do servo: ");
  //Serial.println(servo1.read());
  servo1.write(90); // coloca o servo na posição inicial frontal
  delay(50);
  DistanciaemCM = sonar.ping_cm();
  
  //Serial.print(DistanciaemCM);
  //Serial.println(" cm");

  if(DistanciaemCM <= 20)
  {
      parar();
      paratras();
      delay(300);
      parar();
      servo1.write(180);
      delay(1000);
      ValorDistanciaLadoEsquerdo = sonar.ping_cm();
      delay(500);
      servo1.write(0);
      delay(500);
      ValorDistanciaLadoDireito = sonar.ping_cm();
      delay(500);
      servo1.write(90);
      delay(500);

      if(ValorDistanciaLadoEsquerdo > ValorDistanciaLadoDireito)
      {
          esquerda();
          delay(500);
      }else if(ValorDistanciaLadoEsquerdo < ValorDistanciaLadoDireito){
          direita();
          delay(500);        
      }else{ 
          paratras();
          delay(500);
      }      
  }else{ // se não, ou seja, se a distância for maior que 40 centimetros
      frente();
  }
}

void parar()
{
      // Motor lado esquerdo para frente
      digitalWrite(MotorLadoEsquerdo1, LOW);
      digitalWrite(MotorLadoEsquerdo2, LOW);

      // Motor lado direito para frente
      digitalWrite(MotorLadoDireito1, LOW);
      digitalWrite(MotorLadoDireito2, LOW);
}

void frente()
{
      // velocidade motor lado esquerdo
      analogWrite(VelocidadeMotorLadoEsquerdo, ValorVelocidadeMotorLadoEsquerdo);

      // velocidade motor lado direito
      analogWrite(VelocidadeMotorLadoDireito, ValorVelocidadeMotorLadoDireito);

      // Motor lado esquerdo para frente
      digitalWrite(MotorLadoEsquerdo1, LOW);
      digitalWrite(MotorLadoEsquerdo2, HIGH);

      // Motor lado direito para frente
      digitalWrite(MotorLadoDireito1, LOW);
      digitalWrite(MotorLadoDireito2, HIGH); 
}

void paratras()
{
      // velocidade motor lado esquerdo
      //analogWrite(VelocidadeMotorLadoEsquerdo, ValorVelocidadeMotorLadoEsquerdo);

      // velocidade motor lado direito
      //analogWrite(VelocidadeMotorLadoDireito, ValorVelocidadeMotorLadoDireito);

      // Motor lado esquerdo para tras
      digitalWrite(MotorLadoEsquerdo1, HIGH);
      digitalWrite(MotorLadoEsquerdo2, LOW);

      // Motor lado direito para tras
      digitalWrite(MotorLadoDireito1, HIGH);
      digitalWrite(MotorLadoDireito2, LOW);      
}

void esquerda()
{
      // Motor lado esquerdo para frente
      digitalWrite(MotorLadoEsquerdo1, HIGH);
      digitalWrite(MotorLadoEsquerdo2, LOW);

      // Motor lado direito para frente
      digitalWrite(MotorLadoDireito1, LOW);
      digitalWrite(MotorLadoDireito2, HIGH);
}


void direita()
{
      // Motor lado esquerdo para frente
      digitalWrite(MotorLadoEsquerdo1, LOW);
      digitalWrite(MotorLadoEsquerdo2, HIGH);

      // Motor lado direito para frente
      digitalWrite(MotorLadoDireito1, HIGH);
      digitalWrite(MotorLadoDireito2, LOW);
}
