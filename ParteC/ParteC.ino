const int PIN_JOY1_X = 34; // Joystick 1 - X (Giro Base)
const int PIN_JOY1_Y = 35; // Joystick 1 - Y (Caminar Adelante/Atrás)
const int PIN_JOY2_X = 33; // Joystick 2 - X (Giro Cintura)
const int PIN_JOY2_Y = 32; // Joystick 2 - Y (Cuello Arriba/Abajo)
const int PIN_BOTON  = 25; // Joystick 2 - Botón (Salto)

const unsigned long INTERVALO_ENVIO_MS = 20;
const int MUESTRAS_PROMEDIO = 4;
unsigned long ultimoEnvio = 0;

int leerEje(int pin) {
  long suma = 0;
  for (int i = 0; i < MUESTRAS_PROMEDIO; i++) {
    suma += analogRead(pin);
  }
  return suma / MUESTRAS_PROMEDIO;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  analogReadResolution(12);
}

void loop() {
  unsigned long ahora = millis();
  if (ahora - ultimoEnvio < INTERVALO_ENVIO_MS) return;
  ultimoEnvio = ahora;

  int val_x1 = leerEje(PIN_JOY1_X);
  int val_y1 = leerEje(PIN_JOY1_Y);
  int val_x2 = leerEje(PIN_JOY2_X);
  int val_y2 = leerEje(PIN_JOY2_Y);
  int btn    = digitalRead(PIN_BOTON);

  Serial.print(val_x1); Serial.print(',');
  Serial.print(val_y1); Serial.print(',');
  Serial.print(val_x2); Serial.print(',');
  Serial.print(val_y2); Serial.print(',');
  Serial.println(btn);
}