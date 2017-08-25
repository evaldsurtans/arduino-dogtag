#include <PinChangeInt.h> 

#define PIN_BUTTON 1
#define PIN_BUZZER 0

bool _is_button_down = false;

void on_button_down() {
   _is_button_down = true;
}

void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);

  attachPinChangeInterrupt(PIN_BUTTON, on_button_down, FALLING);

  Bean.setLed(0, 0, 0);
  Bean.enableWakeOnConnect(true);

  Serial.begin(57600);

  //todo accel interrupts
}

void loop() {
  bool is_connected = Bean.getConnectionState();

  if (Serial.available() > 0) {
    String query = Serial.readString();
    Serial.print(String("Received:") + query + String("\n"));
  }

  
  if(_is_button_down) {

    Serial.print(String("TEST TEST TEST TEST:") + String(digitalRead(PIN_BUTTON), DEC) + String("\n"));
    //Serial.println(digitalRead(PIN_BUTTON));
   
    Bean.setLed(255, 0, 0);
    Bean.sleep(1000);
    Bean.setLed(0, 0, 0);

    //Bean.disconnect();
    
    _is_button_down = false;
  }
  
  if(is_connected) {
    Bean.setLed(0, 255, 0);
    //sleep
  } else {
    Bean.sleep(0xFFFFFFFF);
  }

}

