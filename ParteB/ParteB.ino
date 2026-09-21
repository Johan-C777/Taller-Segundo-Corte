#define JOY1_X 32
#define JOY1_Y 33
#define JOY2_Z 34
#define BTN_PINZA 27

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Rango de 0 a 4095
  pinMode(BTN_PINZA, INPUT_PULLUP);
}

void loop() {
  int x = analogRead(JOY1_X);
  int y = analogRead(JOY1_Y);
  int z = analogRead(JOY2_Z);
  int btn = digitalRead(BTN_PINZA); // 0 presionado, 1 suelto

  // Enviar trama: X,Y,Z,Boton
  Serial.print(x); Serial.print(",");
  Serial.print(y); Serial.print(",");
  Serial.print(z); Serial.print(",");
  Serial.println(btn);

  delay(30); // Tasa de refresco rápida para movimiento fluido
}