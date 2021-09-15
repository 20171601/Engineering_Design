#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0);
  count = toggle = 0;
}

void loop() {
  delay(1000);
  while(count < 10) {
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100);
    count += 1;
    Serial.println(count);
  }
  digitalWrite(PIN_LED, 1);
  while(1){}
}

int toggle_state(int toggle) {
  if (toggle == 1) {
    return 0;
  }else {
    return 1;
  }
}
