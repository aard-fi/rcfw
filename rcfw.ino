#include <IBusBM.h>
#include <Servo.h>

#define SNOW_MOBILE 1
#define RC_BENCHY 2
#define NOTANK 3

#define RC_MODEL NOTANK

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
#endif

// arduino can tolerate about 20µA per pin, with a total of 200µA
// a bright 5mm LED draws roughly 20µA - so three sets of two LEDs
// should be safe. If you need more connect them to power directly,
// and trigger them via transistor

// the power LEDs also serve as back lights
#define POWER_LED 13
#define POWER_LEDS {2,13}

// this should be a three state switch, and needs to be setup as aux channel
// comment this definition if you don't want that feature
#define CONTROL_CHANNEL 7

// a two state switch, preventing throttle to engage unless switched on
// this also reduces the risk of frying your Arduino here, and generally
// makes things safer, so this is no longer optional
#define IGNITION_CHANNEL 6

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

IBusBM IBus;
#if RC_MODEL == NOTANK
Servo motor_left;
Servo motor_right;
#elif RC_MODEL == RC_BENCHY
Servo motor;
Servo steering;
#endif
int ignition=1000;
int led_state=0;
int throttle_channel;
int steering_channel;
int failsafe_channel;

int power_leds[]=POWER_LEDS;

enum led_effects{
  led_on,
  led_off,
  led_cycle
};

void setup_controls(){
#ifdef CONTROL_CHANNEL
  int controls = IBus.readChannel(CONTROL_CHANNEL);
#else
  int controls = 1000;
#endif

  // throttle is only supported right due to support for reversing
  if (controls == 1500){
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
  Serial.begin(115200);
  // on the Every the internal timer doesn't work - so disable it here,
  // and manually call loop() in the loop
  IBus.begin(Serial1,IBUSBM_NOTIMER);
#define debug_print Serial.print
#define debug_println Serial.println
#elif ARDUINO_AVR_NANO
  IBus.begin(Serial,IBUSBM_NOTIMER);
  #warning Building for Nano disables debug output
#define debug_print(...) ((void)0)
#define debug_println(...) ((void)0)
#else
  #error Build running on unsupported board
#endif

  // the calls here calibrate the neutral position for the ESC
#if RC_MODEL == NOTANK
  motor_left.attach(MOTOR_PIN_LEFT, 1000, 2000);
  motor_left.write(PWM_STOP);
  motor_right.attach(MOTOR_PIN_RIGHT, 1000, 2000);
  motor_right.write(PWM_STOP);

#elif RC_MODEL == RC_BENCHY
  motor.attach(MOTOR_PIN, 1000, 2000);
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
void controls(int pwm_adjusted, int steer_pwm, int steer){
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
void controls(int pwm_adjusted, int steer_pwm, int steer){
  if (steer >= 1000 and steer <= 2000)
    steering.write(steer_pwm);
  else
    steering.write(STEER_MID);

  // we shouldn't need a low speed cutoff as for the tank - if I'm wrong that should
  // go here.
  motor.write(pwm_adjusted);
}
#endif

void loop() {
  IBus.loop();
  if (led_state >= LED_DELAY)
    led_state = LED_DELAY*-1;
  led_state++;

  setup_controls();

  ignition = IBus.readChannel(IGNITION_CHANNEL);
  debug_print(" Ignition: ");
  debug_print(ignition);

  if (ignition != 2000){
#if RC_MODEL == NOTANK
    motor_left.write(PWM_STOP);
    motor_right.write(PWM_STOP);
#elif RC_MODEL == RC_BENCHY
    motor.write(PWM_STOP);
#endif

    set_led(power_leds, led_state, led_cycle);
  } else {
    int steer;
    steer = IBus.readChannel(steering_channel);
    debug_print("Steer: ");
    debug_print(steer);
    int steer_pwm=0;
    if (steer >= 1000 and steer <= 2000){
      steer_pwm = map(steer, RX_MIN, RX_MAX, STEER_MIN, STEER_MAX);
      debug_print(" pwm ");
      debug_print(steer_pwm);
    }

    set_led(power_leds, led_state, led_on);
    int throttle;
    throttle = IBus.readChannel(throttle_channel);
    debug_print(" Throttle: ");
    debug_print(throttle);

    // TODO: max speed is currently ignored, as that'd only apply to maximum
    //       forward speed. Should be applied to both forward/reverse. But
    //       also might be dropped completely - motor seems to be correctly
    //       sized here, so might not be needed at all.
    int pwm_max_raw = IBus.readChannel(SPEED_CHANNEL);
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

    controls(pwm_adjusted, steer_pwm, steer);
  }

  int failsafe = IBus.readChannel(failsafe_channel);
  debug_print(" failsafe ");
  debug_print(failsafe);
  debug_println();

  delay(LOOP_DELAY);
}
