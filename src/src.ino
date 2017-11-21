#include <PinChangeInt.h>
#include <math.h>
#include <stdlib.h>
#include "Vector3D.h"

//#define DEBUG //Serial info for debugging - not usable together with ionic app
#define DEBUG_CONNECTION //LED when connected

// pinouts
#define PIN_BUTTON 1
#define PIN_BUZZER 0

// active functions
#define MODE_NONE 0
#define MODE_CALIBRATE_STRAIGHT 1
#define MODE_CALIBRATE_SLOUCH 2
#define MODE_TRACKING 3
#define MODE_BACKGROUND 4

#define TIMEOUT_CALIBRATE_STRAIGHT 7000
#define TIMEOUT_CALIBRATE_SLOUCH 3000
#define TIMEOUT_VIBRATION 500
#define TIMEOUT_VIBRATION_PAUSE 300
#define TIMEOUT_BETWEEN_DETECTION 2000

// Automatically disconnect after transmission if not disconnected
// Prevent staying in connected state for prolonged period of time
#define TIMEOUT_DISCONNECT 5000

#define IS_BACGROUND_MODE_ENABLED 1
#define SLOUCH_COEF 0.5f

#define MAX_16_BIT 32767.0f

// active tracking mode
int _mode = MODE_NONE;
int _timer_disconnect = 0;
int _timer_calibrate = 0;
int _timer_slouch_detected = 0;
Vector3D _vec_slouch;
Vector3D _vec_straight;
float _angle_slouch = 0.0f;

bool _is_button_down = false;
char _temp_cast_string_buf[20];

bool _is_serial_open = false;

// Background tracking mode
// 400 bytes max total usable memory
// buffer for MAX_DATA_POSTURE secs - 3min interval
#define MAX_DATA_POSTURE 200 //items
#define TIMEOUT_POSTURE_DATAPOINT 1000 //ms

int8_t* _data_posture_y = new int8_t[MAX_DATA_POSTURE];
int8_t* _data_posture_z = new int8_t[MAX_DATA_POSTURE];
uint16_t _count_data_posture = 0;
int _timeout_background = 0;

void on_button_down() {
   _is_button_down = true;
}

void setup() {

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);

  Bean.setAccelerationRange(VALUE_2G);

  //https://github.com/PunchThrough/bean-arduino-core/blob/c4f0420496a87f59a1dc95d7c340c66f600195ee/hardware/bean/avr/cores/bean/bma250.h
  //https://www.evernote.com/l/ACmo0m6iWNtBD7UKXTM_vPoU6_nWvTlWFps
  Bean.setAccelerometerPowerMode(VALUE_LOW_POWER_100MS);

  // Unfortunately, Unusable for step tracking
  //Bean.enableMotionEvent(ANY_MOTION_EVENT);
  Bean.disableMotionEvents();

  attachPinChangeInterrupt(PIN_BUTTON, on_button_down, FALLING);

  Bean.setLed(0, 0, 0);
  Bean.enableWakeOnConnect(true);

  if(IS_BACGROUND_MODE_ENABLED) {
    _mode = MODE_BACKGROUND;
  }
}

// Communication function
String float_to_string(float value)
{
  // width, precision
  dtostrf((double)value, 10, 4, _temp_cast_string_buf);
  String result = String(_temp_cast_string_buf);
  result.trim();
  return result;
}

// Communication function
void write_json_attr(String key, String value, bool is_string)
{
  String formatedValue = value;
  if(is_string)
  {
    formatedValue = String("\"") + value + String("\": ");
  }

  Serial.print(String("\"") + key + String("\": ") + formatedValue);
}

// Communication function
void write_json_vec(Vector3D vec)
{
  Serial.print("{");
  write_json_attr(String("x"), float_to_string(vec.x), false);
  Serial.print(",");
  write_json_attr(String("y"), float_to_string(vec.y), false);
  Serial.print(",");
  write_json_attr(String("z"), float_to_string(vec.z), false);
  Serial.print("}");
}

// Signal to user
// Device function
void vibrate(int count) {
  for(int i = 0; i < count; i++)
  {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(TIMEOUT_VIBRATION);
    digitalWrite(PIN_BUZZER, LOW);
    if(i < count - 1) {
      //pause in between
      delay(TIMEOUT_VIBRATION_PAUSE);
    }
  }
}

void loop() {
  bool is_connected = Bean.getConnectionState();

  if(is_connected)
  {
    if(!_is_serial_open)
    {
      _is_serial_open = true;
      Serial.begin(57600);
    }
  }
  else
  {
    _timer_disconnect = 0;
    if(_is_serial_open)
    {
      _is_serial_open = false;
      Serial.end();
    }
  }

  Vector3D vec_current(
    Bean.getAccelerationX()/MAX_16_BIT,
    Bean.getAccelerationY()/MAX_16_BIT,
    Bean.getAccelerationZ()/MAX_16_BIT);

  Vector3D vec_current_norm = vec_current.copy();
  vec_current_norm.norm();

  // X axis not needed
  Vector3D vec_posture(
    0.0f,
    Bean.getAccelerationY()/MAX_16_BIT,
    Bean.getAccelerationZ()/MAX_16_BIT);

  Vector3D vec_posture_norm = vec_posture.copy();
  vec_posture_norm.norm();

  int temperature = Bean.getTemperature();
  int battery_level = Bean.getBatteryLevel();

#ifdef DEBUG
  if(_is_serial_open)
  {
    Serial.println("interrupt");
  }
#endif

  if (_is_serial_open && Serial.available() > 0) {
    String query = Serial.readString();

    String msg = query;
    String param = "";

  	int pos = msg.indexOf(" ");
  	if (pos >= 0)
  	{
  		msg = query.substring(0, pos);
  		param = query.substring(pos);
  	}
    //Response as JSON
    Serial.print("{");

    if(msg == "flush")
    {
        write_json_attr("bat", String(battery_level, DEC), false);
        Serial.print(",");
        write_json_attr("temp", String(temperature, DEC), false);
        Serial.print(",");
        write_json_attr("vec", "", false);
        write_json_vec(vec_current);
        Serial.print(",");
        write_json_attr("norm", "", false);
        write_json_vec(vec_current_norm);
        Serial.print(",");

        Serial.print("\"data\":[");
        for(int i = 0; i < _count_data_posture; i++)
        {
          Serial.print("[");
          Serial.print(_data_posture_y[i]);
          Serial.print(",");
          Serial.print(_data_posture_z[i]);
          Serial.print("]");
          if(i < _count_data_posture - 1)
          {
            Serial.print(",");
          }
        }
        Serial.print("]");
        _count_data_posture = 0;
    }

    Serial.print("}\n");

    //Reset disconnect timer
    _timer_disconnect = 0;
  }

  // Active tracking mode
  if(_mode == MODE_CALIBRATE_STRAIGHT) {
    //First straight posture
    if(_timer_calibrate >= TIMEOUT_CALIBRATE_STRAIGHT) {
      _timer_calibrate = 0;
      _mode = MODE_CALIBRATE_SLOUCH;

      _vec_straight = vec_posture_norm.copy();
      vibrate(1);
    }
  }
  else if(_mode == MODE_CALIBRATE_SLOUCH) {
    //Second slouch posture
    if(_timer_calibrate >= TIMEOUT_CALIBRATE_SLOUCH) {
      _timer_calibrate = 0;
      _mode = MODE_TRACKING;

      _vec_slouch = vec_posture_norm.copy();
      _angle_slouch = _vec_straight.angle(&_vec_slouch);
      _timer_slouch_detected = TIMEOUT_BETWEEN_DETECTION;
      vibrate(2);
    }
  }
  else if(_mode == MODE_TRACKING) {
    float angle = _vec_straight.angle(&vec_posture_norm);
    if(angle >= _angle_slouch * SLOUCH_COEF && _timer_slouch_detected == 0) {
      //slouch detected
      vibrate(1);

      //timeout to next detection
      _timer_slouch_detected = TIMEOUT_BETWEEN_DETECTION;
    }
  }
  else if(_mode == MODE_BACKGROUND) {
    if(_count_data_posture < MAX_DATA_POSTURE && _timeout_background >= TIMEOUT_POSTURE_DATAPOINT)
    {
      // convert to 8bit
      int8_t y = (int8_t)(vec_current_norm.y * 127.0f);
      int8_t z = (int8_t)(vec_current_norm.z * 127.0f);
      _data_posture_y[_count_data_posture] = y;
      _data_posture_z[_count_data_posture] = z;
      _count_data_posture++;
      _timeout_background = 0;
    }
  }

  // digitalRead(PIN_BUTTON) == false
  if(_is_button_down) {

    if(_mode == MODE_BACKGROUND || _mode == MODE_NONE) {
      _mode = MODE_CALIBRATE_STRAIGHT;
      _timer_calibrate = 0;
      // Enable active tracking
      vibrate(1);
    }
    else {
      // Disable active tracking
      if(IS_BACGROUND_MODE_ENABLED) {
        _mode = MODE_BACKGROUND;
      } else {
        _mode = MODE_NONE;
      }
      vibrate(2);
    }

#ifdef DEBUG
    if(_is_serial_open)
    {
      Serial.println("button down");
    }
    // LED indicator for button press
    Bean.setLed(255, 0, 0);
    Bean.sleep(1000);
    Bean.setLed(0, 0, 0);
#endif

    _is_button_down = false;
  }

#ifdef DEBUG_CONNECTION
  if(is_connected) {
    Bean.setLed(0, 255, 0);
  } else {
    Bean.setLed(0, 0, 0);
  }
#endif

  uint32_t timeout = 0xFFFFFFFF;

  if(_mode != MODE_NONE)
  {
      if(_mode == MODE_BACKGROUND)
      {
        // if no space for tracking data then sleep forever until data flush
        if(_count_data_posture < MAX_DATA_POSTURE)
        {
          timeout = 1000;
          _timeout_background += timeout;
        }
      }
      else
      {
        //Active tracking
        timeout = 1000;
      }
  }

  if(_mode == MODE_CALIBRATE_SLOUCH || _mode == MODE_CALIBRATE_STRAIGHT) {
    _timer_calibrate += timeout;
  }
  if(_timer_slouch_detected > 0)
  {
    _timer_slouch_detected -= timeout;
    _timer_slouch_detected = max(_timer_slouch_detected, 0);
  }

  if(is_connected)
  {
    _timer_disconnect += timeout;
    if(_timer_disconnect > TIMEOUT_DISCONNECT)
    {
      Bean.disconnect();
    }
  }

  Bean.sleep(timeout);

}
