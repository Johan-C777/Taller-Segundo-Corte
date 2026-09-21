#define BTN_A 12
#define BTN_B 14
#define BTN_C 27

void setup() {
  Serial.begin(115200);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);
}

void loop() {
  // Cuando se presiona un botón, enviamos un caracter simple por UART
  if (digitalRead(BTN_A) == LOW) { 
    Serial.println("A"); 
    delay(300); // Antirrebote simple
  }
  if (digitalRead(BTN_B) == LOW) { 
    Serial.println("B"); 
    delay(300); 
  }
  if (digitalRead(BTN_C) == LOW) { 
    Serial.println("C"); 
    delay(300); 
  }
}