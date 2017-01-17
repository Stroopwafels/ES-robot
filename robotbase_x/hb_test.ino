/*
  hb_test.ino
  Kan gebruikt worden om de LCHB-100 H-brug te testen
*/
// H-bridge pins
#define HB_1REV   7
#define HB_1EN    24
#define HB_1FWD   6
#define HB_2REV   3
#define HB_2EN    25
#define HB_2FWD   2
// Yellow LED
#define LED      13

void setup() {
  pinMode(HB_1REV, OUTPUT);
  pinMode(HB_1EN, OUTPUT);
  pinMode(HB_1FWD, OUTPUT);
  pinMode(HB_2REV, OUTPUT);
  pinMode(HB_2EN, OUTPUT);
  pinMode(HB_2FWD, OUTPUT);
  pinMode(LED, OUTPUT);

  Serial.begin(9600);

}

void loop() {
  // Blink de LED 1 sec voor de loop begint
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);

  //Doe de testcyclus per motor. Blink LED 0.1s tussen elke case 
  Serial.println("Motor1\n");
  Serial.println("\tEN: LOW\n");
  digitalWrite(HB_1EN, LOW);
  
  digitalWrite(HB_1FWD, LOW);
  digitalWrite(HB_1REV, LOW);
  digitalWrite(LED, HIGH);
  delay(400);
  digitalWrite(LED, LOW);
  delay(100);

  digitalWrite(HB_1FWD, HIGH);
  digitalWrite(HB_1REV, LOW);
  digitalWrite(LED, HIGH);
  delay(400);
  digitalWrite(LED, LOW);
  delay(100);

  digitalWrite(HB_1FWD, LOW);
  digitalWrite(HB_1REV, HIGH);
  digitalWrite(LED, HIGH);
  delay(400);
  digitalWrite(LED, LOW);
  delay(100);

  digitalWrite(HB_1FWD, HIGH);
  digitalWrite(HB_1REV, HIGH);
  digitalWrite(LED, HIGH);
  delay(400);
  digitalWrite(LED, LOW);
  delay(100);

  Serial.println("\tEN: HIGH\n");
  digitalWrite(HB_1EN, HIGH);
  
  digitalWrite(HB_1FWD, LOW);
  digitalWrite(HB_1REV, LOW);
  digitalWrite(LED, HIGH);
  delay(400);
  digitalWrite(LED, LOW);
  delay(100);

  digitalWrite(HB_1FWD, HIGH);
  digitalWrite(HB_1REV, LOW);
  digitalWrite(LED, HIGH);
  delay(400);
  digitalWrite(LED, LOW);
  delay(100);
  
  digitalWrite(HB_1FWD, LOW);
  digitalWrite(HB_1REV, HIGH);
  digitalWrite(LED, HIGH);
  delay(400);
  digitalWrite(LED, LOW);
  delay(100);

  digitalWrite(HB_1FWD, HIGH);
  digitalWrite(HB_1REV, HIGH);
  digitalWrite(LED, HIGH);
  delay(400);
  digitalWrite(LED, LOW);
  delay(100);
}