//www.elegoo.com
//2016.12.09

// Arduino pin numbers
const int SW_pin = 2; // digital pin connected to switch output
const int X_pin = A0; // analog pin connected to X output
const int Y_pin = A1; // analog pin connected to Y output

void setup() {
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  Serial.begin(9600);
}

void loop() {

  int xVal = analogRead(X_pin);
  int yVal = analogRead(Y_pin);
  int swVal = digitalRead(SW_pin);
  Serial.print("X-axis: ");
  Serial.println(xVal);

  Serial.print("Y-axis: ");
  Serial.println(yVal);

  Serial.print("Switch:  ");
  Serial.println(swVal);
  delay(100);
}
