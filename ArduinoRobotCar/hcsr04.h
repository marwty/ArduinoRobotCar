#ifndef HCSR04_H
#define HCSR04_H

// HC-SR04 sensor pins
#define echo 24
#define trig 22

const int SENSOR_MAX_RANGE = 3000;  // mm


int getDistance() {

  double distance = 0;
  double time = 0;

  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  noInterrupts();
  digitalWrite(trig, HIGH);  //Trigger Impuls 10 us
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  time = pulseIn(echo, HIGH);  // Echo-Zeit messen
  interrupts();

  time = (time / 2);       // Zeit halbieren
  distance = time / 2.91;  // Zeit in Millimeter umrechnen

  return distance;
}

void printDistance() {
  int distance = getDistance();

  Serial.write("Entfernung: ");
  if (distance == -1) {
    Serial.write("out of range\n");
  }

  else {
    Serial.print(getDistance(), DEC);
    Serial.write(" mm\n");
  }
}

#endif