#include <Servo.h>

#define SNOW_MOBILE 1
#define RC_BENCHY 2
#define NOTANK 3

#define TX_IBUS 1
#define TX_CRSF 2

#define RC_MODEL NOTANK
#define SERIAL_PROTOCOL TX_CRSF

#if SERIAL_PROTOCOL == TX_IBUS
#include <IBusBM.h>
#elif SERIAL_PROTOCOL == TX_CRSF
#include <AlfredoCRSF.h>
#endif

/*
 The further you go down the defines the less likely you should
 want to change something.

 Channel numbers are C indexes, i.e., one less than your transmitter
 shows you.

 PWM vamotoruuulues are based on the servo library, ranging from 0 to 180 - not
 the native 0-255 of Arduino PWM (which is a wrong frequency)..
*/

// this is the slowest supported motor speed - everything lower and we'll
// disable the motor to avoid them burning out
#define PWM_CUTOFF 10

// the digital pin the motor is attached to
#if RC_MODEL == NOTANK
#define MOTOR_PIN_LEFT 3
#define MOTOR_PIN_RIGHT 4
#elif RC_MODEL == RC_BENCHY
#define MOTOR_PIN 3
#define SERVO_PIN 10
#elif RC_MODEL == SNOW_MOBILE
#define MOTOR_PIN 3
#define SERVO_PIN 10
#endif

// arduino can tolerate about 20µA per pin, with a total of 200µA
// a bright 5mm LED draws roughly 20µA - so three sets of two LEDs
// should be safe. If you need more connect them to power directly,
// and trigger them via transistor

// the power LEDs also serve as back lights
#define POWER_LED 13
#define POWER_LEDS {2,13}
#if RC_MODEL == NOTANK
// TODO, this works right now because we're not using LEDs at all - but once we start
// using them the LED setter needs to be fixed to deal with variable length arrays
#define FRONT_LEDS {}
#define EFFECT_LEDS {}
#else
#define FRONT_LEDS {11,12}
#define EFFECT_LEDS {4,5}
#endif

// this should be a three state switch, and needs to be setup as aux channel
// comment this definition if you don't want that feature
#define CONTROL_CHANNEL 7

// a two state switch, preventing throttle to engage unless switched on
// this also reduces the risk of frying your Arduino here, and generally
// makes things safer, so this is no longer optional
#define IGNITION_CHANNEL 6

#define EFFECT_CHANNEL 5

/*
  This configures an input (default: VRA on channel 5) to adjust the speed
  curve.

  The speed curve itself needs adjustment below and recompilation to change.
  Default settings have roughly half the maximum speed with VRA on zero, going
  up to full speed with VRA on max.

  Comment the SPEED_CHANNEL definition if you don't want that.
*/
#define SPEED_CHANNEL 4
#define SPEED_MIN 90
#define SPEED_MAX 180

// time LED should remain in a specific status. This is per loop - so
// take LOOP_DELAY into account as well.
#define LED_DELAY 10
// slightly delays the loop to allow time for mechanical elements to
// move, at cost of slight input delay.
#define LOOP_DELAY 20

#define PWM_MIN 0
#define PWM_MID 90
#define PWM_MAX 180

#define PWM_STOP 90

#define STEER_MIN 0
#define STEER_MID 90
#define STEER_MAX 180

// with trim the values sent by the transmitter might be over/under this range,
// but this is good enough for now
#define RX_MIN 1000
#define RX_MID 1500
#define RX_MAX 2000

#define RX_TOLERANCE 30

#if SERIAL_PROTOCOL == TX_IBUS
IBusBM IBus;
#elif SERIAL_PROTOCOL == TX_CRSF
AlfredoCRSF crsf;
#endif

#if RC_MODEL == NOTANK
Servo motor_left;
Servo motor_right;
#elif RC_MODEL == RC_BENCHY
Servo motor;
Servo steering;
#elif RC_MODEL == SNOW_MOBILE
Servo motor;
Servo steering;
#endif
int ignition=RX_MIN;
int led_state=0;
int throttle_channel;
int steering_channel;
int failsafe_channel;

int power_leds[]=POWER_LEDS;
int front_leds[]=FRONT_LEDS;
int effect_leds[]=EFFECT_LEDS;

enum led_effects{
  led_on,
  led_off,
  led_cycle
};

void setup_controls(){
#ifdef CONTROL_CHANNEL
#if SERIAL_PROTOCOL == TX_IBUS
  int controls = IBus.readChannel(CONTROL_CHANNEL);
#elif SERIAL_PROTOCOL == TX_CRSF
  int controls = crsf.getChannel(CONTROL_CHANNEL);
#endif
#else
  int controls = RX_MIN;
#endif

#if SERIAL_PROTOCOL == TX_IBUS
#if RC_MODEL == NOTANK
  // throttle is only supported right due to support for reversing
  if (controls == RX_MID){
    // throttle right, steering left
    throttle_channel=1;
    steering_channel=3;
    failsafe_channel=0;
  } else {
    // throttle and steering right
    throttle_channel=1;
    steering_channel=0;
    failsafe_channel=3;
  }
#else
  // TODO, this needs a bit of work - right now the test here is with a pistol grip one,
  // where the channels are definitely correct
  if (controls == RX_MID){
    // throttle right, steering left
    throttle_channel=1;
    steering_channel=3;
  } else if (controls == RX_MAX) {
    // throttle and steering right
    throttle_channel=1;
    steering_channel=0;
  } else {
    // default, throttle on the left, steering right
    throttle_channel=2;
    steering_channel=0;
  }
#endif
#else
  // channels start at 1
  steering_channel=1;
  throttle_channel=2;
#endif
}

void set_led(int leds[], int cycle, int effect){
  switch(effect){
    case led_on:
      digitalWrite(leds[0], HIGH);
      digitalWrite(leds[1], HIGH);
      break;
    case led_off:
      digitalWrite(leds[0], LOW);
      digitalWrite(leds[1], LOW);
      break;
    case led_cycle:
      if (cycle < 0){
        digitalWrite(leds[0], LOW);
        digitalWrite(leds[1], LOW);
      } else {
        digitalWrite(leds[0], HIGH);
        digitalWrite(leds[1], HIGH);
      }
      break;
  }
}

void setup() {
  pinMode(POWER_LED, OUTPUT);

#ifdef ARDUINO_AVR_NANO_EVERY
#define debug_print Serial.print
#define debug_println Serial.println
  Serial.begin(115200);
  // on the Every the internal timer doesn't work - so disable it here,
  // and manually call loop() in the loop
#if SERIAL_PROTOCOL == TX_IBUS
  IBus.begin(Serial1,IBUSBM_NOTIMER);
#elif SERIAL_PROTOCOL == TX_CRSF
  Serial1.begin(420000);
  crsf.begin(Serial1);
  debug_println("Setting up CRSF on port 1");
#endif
#elif ARDUINO_AVR_NANO
#if SERIAL_PROTOCOL == TX_IBUS
  IBus.begin(Serial,IBUSBM_NOTIMER);
#elif SERIAL_PROTOCOL == TX_CRSF
  Serial.begin(115200);
  crsf.begin(Serial);
#endif
  #warning Building for Nano disables debug output
#define debug_print(...) ((void)0)
#define debug_println(...) ((void)0)
#else
  #error Build running on unsupported board
#endif

  // the calls here calibrate the neutral position for the ESC
#if RC_MODEL == NOTANK
  motor_left.attach(MOTOR_PIN_LEFT, RX_MIN, RX_MAX);
  motor_left.write(PWM_STOP);
  motor_right.attach(MOTOR_PIN_RIGHT, RX_MIN, RX_MAX);
  motor_right.write(PWM_STOP);

#elif RC_MODEL == RC_BENCHY
  motor.attach(MOTOR_PIN, RX_MIN, RX_MAX);
  motor.write(PWM_STOP);

  steering.attach(SERVO_PIN);
  steering.write(STEER_MAX);
  delay(1);
  steering.write(STEER_MIN);
  delay(1);
  steering.write(STEER_MID);
#elif RC_MODEL == SNOW_MOBILE
  motor.attach(MOTOR_PIN, RX_MIN, RX_MAX);
  motor.write(PWM_STOP);
  steering.attach(SERVO_PIN);
  steering.write(STEER_MAX);
  delay(1);
  steering.write(STEER_MIN);
  delay(1);
  steering.write(STEER_MID);
#endif
  setup_controls();
}

#if RC_MODEL == NOTANK
void controls(int pwm_adjusted, int steer_pwm){
  int pwm_left, pwm_right;

  pwm_left = pwm_adjusted;
  pwm_right = pwm_adjusted;

  // TODO:
  //       - optimise steering - we can use reverse here to avoid slowing down
  //         one side.
  if (pwm_adjusted == PWM_STOP) {
    // special handling for turning at zero speed
    if (steer_pwm < 90) { //left
      steer_pwm = map(steer_pwm, 0, 90, 90, 0);
      pwm_left = PWM_STOP - steer_pwm/2;
      pwm_right = PWM_STOP + steer_pwm/2;
    } else {
      steer_pwm = steer_pwm - 90;
      pwm_left = PWM_STOP + steer_pwm/2;
      pwm_right = PWM_STOP - steer_pwm/2;
    }
  } else if (pwm_adjusted < PWM_STOP) {
    // reversing
    if (steer_pwm < 90) {
      // steer left
      steer_pwm = map(steer_pwm, 0, 90, 90, 0);
      if (pwm_adjusted - steer_pwm < 0)
        pwm_left = pwm_adjusted + steer_pwm;
      else
        pwm_right = pwm_adjusted - steer_pwm;
    } else if (steer_pwm > 90) {
      // right
      steer_pwm = steer_pwm - 90;
      if (pwm_adjusted - steer_pwm < 0)
        pwm_right = pwm_adjusted + steer_pwm;
      else
        pwm_left = pwm_adjusted - steer_pwm;
    }
  } else {
    // generally going forward
    if (steer_pwm < 90) {
      // steer left
      steer_pwm = map(steer_pwm, 0, 90, 90, 0);
      if (pwm_adjusted + steer_pwm > 180)
        pwm_left = pwm_adjusted - steer_pwm;
      else
        pwm_right = pwm_adjusted + steer_pwm;
    } else if (steer_pwm > 90) {
      // right
      steer_pwm = steer_pwm - 90;
      if (pwm_adjusted + steer_pwm > 180)
        pwm_right = pwm_adjusted - steer_pwm;
      else
        pwm_left = pwm_adjusted + steer_pwm;
    }
  }

  debug_print(" adjusted left ");
  debug_print(pwm_left);
  debug_print(" right ");
  debug_print(pwm_right);

  // this is mainly a failsafe for steering modification - for pure throttle
  // above throttle curve adjustment should be enough to prevent the motors
  // running too slow.
  if (pwm_left >= PWM_STOP - PWM_CUTOFF && pwm_left <= PWM_STOP + PWM_CUTOFF)
    pwm_left = PWM_STOP;
  if (pwm_right >= PWM_STOP - PWM_CUTOFF && pwm_right <= PWM_STOP + PWM_CUTOFF)
    pwm_right = PWM_STOP;

  motor_left.write(pwm_left);
  motor_right.write(pwm_right);
}
#elif RC_MODEL == RC_BENCHY
void controls(int pwm_adjusted, int steer_pwm){
  // this should reverse steering when going forward; for reversing steering already should be OK
  if (pwm_adjusted >= PWM_STOP)
    steer_pwm = map(steer_pwm, 0, 180, 180, 0);

  if (steer_pwm >= STEER_MIN and steer_pwm <= STEER_MAX)
    steering.write(steer_pwm);
  else
    steering.write(STEER_MID);

  // we shouldn't need a low speed cutoff as for the tank - if I'm wrong that should
  // go here.
  motor.write(pwm_adjusted);
}
#elif RC_MODEL == SNOW_MOBILE
// this should work like that, but needs testing
void controls(int pwm_adjusted, int steer_pwm){
  if (steer_pwm >= STEER_MIN and steer_pwm <= STEER_MAX)
    steering.write(steer_pwm);
  else
    steering.write(STEER_MID);

  motor.write(pwm_adjusted);
}
#endif

#if SERIAL_PROTOCOL == TX_CRSF
void print_channels()
{
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++)
  {
    debug_print(ChannelNum);
    debug_print(":");
    debug_print(crsf.getChannel(ChannelNum));
    debug_print(" ");
  }
}
#endif

// make sure value is inside of RX limits
//
// debug log shows values just being a bit over/under; if there are overflow issues
// we might end up with large jumps - in which case we'd need to limit this check to just
// max/min +- 20 or so
int rx_adjust(int value, int fallback){
  if (value <= RX_MIN && value >= RX_MIN - RX_TOLERANCE)
    value = RX_MIN;
  else if (value >= RX_MAX && value <= RX_MAX + RX_TOLERANCE)
    value = RX_MAX;
  else if (value <= RX_MIN - RX_TOLERANCE - 1 || value >= RX_MAX + RX_TOLERANCE + 1)
    value = fallback;

  return value;
}

void loop() {
#if SERIAL_PROTOCOL == TX_IBUS
  IBus.loop();
#elif SERIAL_PROTOCOL == TX_CRSF
  crsf.update();
#endif
  if (led_state >= LED_DELAY)
    led_state = LED_DELAY*-1;
  led_state++;

  setup_controls();

#if SERIAL_PROTOCOL == TX_IBUS
  ignition = IBus.readChannel(IGNITION_CHANNEL);
  debug_print(" Ignition: ");
  debug_print(ignition);
#elif SERIAL_PROTOCOL == TX_CRSF
  //ignition = crsf.getChannel(IGNITION_CHANNEL);
  // TODO, figure out how to do ignition here as well
  ignition = RX_MAX;
#endif


  if (ignition != RX_MAX){
#if RC_MODEL == NOTANK
    motor_left.write(PWM_STOP);
    motor_right.write(PWM_STOP);
#elif RC_MODEL == RC_BENCHY
    motor.write(PWM_STOP);
#endif

    set_led(power_leds, led_state, led_cycle);
  } else {
    int steer;
#if SERIAL_PROTOCOL == TX_IBUS
    steer = IBus.readChannel(steering_channel);
#elif SERIAL_PROTOCOL == TX_CRSF
    steer = crsf.getChannel(steering_channel);
#endif
    debug_print("Steer: ");
    debug_print(steer);
    int steer_pwm=0;

    steer = rx_adjust(steer, RX_MID);

    if (steer >= RX_MIN and steer <= RX_MAX){
      steer_pwm = map(steer, RX_MIN, RX_MAX, STEER_MIN, STEER_MAX);
      debug_print(" pwm ");
      debug_print(steer_pwm);
    }

    set_led(power_leds, led_state, led_on);
    int throttle;
#if SERIAL_PROTOCOL == TX_IBUS
    throttle = IBus.readChannel(throttle_channel);
#elif SERIAL_PROTOCOL == TX_CRSF
    throttle = crsf.getChannel(throttle_channel);

    // TODO, for the snow mobile we probably want a different fallback
    throttle = rx_adjust(throttle, RX_MID);

#endif
    debug_print(" Throttle: ");
    debug_print(throttle);

    // TODO: max speed is currently ignored, as that'd only apply to maximum
    //       forward speed. Should be applied to both forward/reverse. But
    //       also might be dropped completely - motor seems to be correctly
    //       sized here, so might not be needed at all.
#if SERIAL_PROTOCOL == TX_IBUS
    int pwm_max_raw = IBus.readChannel(SPEED_CHANNEL);
#elif SERIAL_PROTOCOL == TX_CRSF
    int pwm_max_raw = crsf.getChannel(SPEED_CHANNEL);
#endif
    int pwm_max = map(pwm_max_raw, RX_MIN, RX_MAX, SPEED_MIN, SPEED_MAX);

    int pwm_adjusted = 0;

    // to compensate for wonky remote midpoint adjust +- 10
    if (throttle >= RX_MID+10)
      pwm_adjusted = map(throttle, RX_MID, RX_MAX, PWM_MID+PWM_CUTOFF, PWM_MAX);
    else if (throttle <= RX_MID-10)
      pwm_adjusted = map(throttle, RX_MID, RX_MIN, PWM_MID-PWM_CUTOFF, PWM_MIN);
    else
      pwm_adjusted = PWM_STOP;

    debug_print(" ");
    debug_print(pwm_adjusted);

    controls(pwm_adjusted, steer_pwm);
  }

#if SERIAL_PROTOCOL == TX_IBUS
  int failsafe = IBus.readChannel(failsafe_channel);
#elif SERIAL_PROTOCOL == TX_CRSF
  int failsafe = crsf.getChannel(failsafe_channel);
#endif
//  debug_print(" failsafe ");
//  debug_print(failsafe);

#if SERIAL_PROTOCOL == TX_CRSF
  debug_print(" | ");
  print_channels();
#endif

  debug_println();

  delay(LOOP_DELAY);
}
