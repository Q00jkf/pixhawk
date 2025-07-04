<<<<<<< HEAD
// Basic communication test
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("=== Arduino Nano Test ===");
  Serial.println("System starting...");
}

void loop() {
  static unsigned long lastTime = 0;
  static int counter = 0;
  
  if (millis() - lastTime > 1000) {
    lastTime = millis();
    counter++;
    Serial.print("Heartbeat #");
    Serial.print(counter);
    Serial.print(" - Time: ");
    Serial.println(millis());
  }
  
  // Echo any received data
  if (Serial.available()) {
    String received = Serial.readString();
    Serial.print("Received: ");
    Serial.println(received);
  }
=======
// Basic communication test
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("=== Arduino Nano Test ===");
  Serial.println("System starting...");
}

void loop() {
  static unsigned long lastTime = 0;
  static int counter = 0;
  
  if (millis() - lastTime > 1000) {
    lastTime = millis();
    counter++;
    Serial.print("Heartbeat #");
    Serial.print(counter);
    Serial.print(" - Time: ");
    Serial.println(millis());
  }
  
  // Echo any received data
  if (Serial.available()) {
    String received = Serial.readString();
    Serial.print("Received: ");
    Serial.println(received);
  }
>>>>>>> a53aa67a6dee201668c571b3e5861cfe6dcbfc84
}