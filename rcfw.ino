#include <Servo.h>
#include <avr/wdt.h>

#define SNOW_MOBILE 1
#define RC_BENCHY 2
#define NOTANK 3

// set thins to the model you're using from above list
#ifndef RC_MODEL
#define RC_MODEL NOTANK
#endif

struct channel_mapping {
    int control_channel;
    int ignition_channel;
    int effect_channel;
    int speed_channel;
};

// if we have no idea what the remote control is just disable
// all the fancy extra channels
const channel_mapping default_mapping = {
  .control_channel = -1,
  .ignition_channel = -1,
  .effect_channel = -1,
  .speed_channel = -1
};

const channel_mapping flysky_st_mapping = {
  .control_channel = 8,
  .ignition_channel = 7,
  .effect_channel = 6,
  .speed_channel = 5
};

const channel_mapping radiomaster_st_mapping = {
  .control_channel = 7,
  .ignition_channel = 5,
  .effect_channel = -1,
  .speed_channel = 10
};

const channel_mapping radiomaster_pg_mapping = {
  .control_channel = -1,
  .ignition_channel = 5,
  .effect_channel = -1,
  .speed_channel = 6
};

// I do not have a pistol grip FlySky transmitter - the first entry is a placeholder in case someone
// wants to contribute that.
//
// The generic manufacturer types are used for type auto detection. For the Radiomaster ones we can
// use channel 4 to check the type: PG has it at 1000 as default, while stick has it at 1500.
#define FLYSKY_PG 1
#define FLYSKY_ST 2
#define FLYSKY 3
#define RADIOMASTER_PG 4
#define RADIOMASTER_ST 5
#define RADIOMASTER 6

// select the type of transmitter from above list. For pistol grip style transmitters
// the first two channels should be the same, no matter the manufacturer. For stick style
// transmitters the first four channels should be the same - so basic functionality should
// work by just selecting a matching type.
//
// for extra functionality involving the switches and potis receiver specific configuration
// may be required
#ifndef TX_TYPE
#define TX_TYPE RADIOMASTER
#endif

// this must match the configured speed in the receiver
// if the connection does not work double check that the receiver
// is configured correctly
#define CRSF_SPEED 420000

// set this for logging complete channel status table (CRSF only)
//#define DEBUG_CHANNELS 1

// remove this to skip all debug messages. Note that the Nano already
// doesn't log due to the single serial port.
#define DEBUG_MESSAGES 1

/*
 The further you go down the defines the less likely you should
 want to change something.

 PWM motor ranges are based on the servo library, ranging from 0 to 180 - not
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

// LED definitions must end with -1 as end of array marker to support
// variable amounts of LED pins depending on model
// the power LEDs also serve as back lights
#define POWER_LEDS {2,13,-1}
#if RC_MODEL == NOTANK
#define FRONT_LEDS {-1}
#define EFFECT_LEDS {-1}
#else
#define FRONT_LEDS {11,12,-1}
#define EFFECT_LEDS {4,5,-1}
#endif

/*
This is the default speed curve for dynamic speed adjustment, which allows adjusting from
0% to 100%, on a model which does have reversing. Adjusting it is not tested, though, and
may need somewhat more complicated speed calculations: currently models which support
speed adjustment just start at min-45 and max-45, and add those speed adjustments in, which
expands to the whole range.
*/
#define SPEED_MIN -45
#define SPEED_MAX 45

// The receiver should be connected to this pin via a NPN transistor to enable
// power cycling it when the Arduino restarts. With ELRS receivers and binding
// phrases this allows easy switching of controllers during play without having
// to touch the model
#define POWER_PIN A7

// time LED should remain in a specific status. This is per loop - so
// take LOOP_DELAY into account as well.
#define LED_DELAY 10
// slightly delays the loop to allow time for mechanical elements to
// move, at cost of slight input delay.
#define LOOP_DELAY 20

#define DEBUG_DELAY 10

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

#define TX_IBUS 1
#define TX_CRSF 2

#if TX_TYPE <= FLYSKY
#define SERIAL_PROTOCOL TX_IBUS
#else
#define SERIAL_PROTOCOL TX_CRSF
#endif

#if SERIAL_PROTOCOL == TX_CRSF
// defines how many loop cycles we wait when link has been lost before
// resetting.
#define LINK_DOWN_DELAY 1000
#endif

#if SERIAL_PROTOCOL == TX_IBUS
#include <IBusBM.h>
IBusBM IBus;
#elif SERIAL_PROTOCOL == TX_CRSF
#include <AlfredoCRSF.h>
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
int debug_state=0;
#ifdef LINK_DOWN_DELAY
int linkdown_state=0;
#endif
int throttle_channel;
int steering_channel;

int power_leds[]=POWER_LEDS;
int front_leds[]=FRONT_LEDS;
int effect_leds[]=EFFECT_LEDS;

enum led_effects{
  led_on,
  led_off,
  led_cycle
};

channel_mapping get_tx_mapping(int type){
  switch(type){
    case FLYSKY_ST: return flysky_st_mapping;
    case RADIOMASTER_ST: return radiomaster_st_mapping;
    case RADIOMASTER_PG: return radiomaster_pg_mapping;
    default: return default_mapping;
  }
}

int current_tx = TX_TYPE;
channel_mapping tx_mapping = get_tx_mapping(current_tx);

void setup_controls(){
  int controls;
  if (tx_mapping.control_channel >= 1){
#if SERIAL_PROTOCOL == TX_IBUS
    controls = IBus.readChannel(tx_mapping.control_channel-1);
#elif SERIAL_PROTOCOL == TX_CRSF
    controls = crsf.getChannel(tx_mapping.control_channel);
  } else
#endif
    controls = RX_MIN;

  if (controls <= RX_MIN)
    controls = RX_MIN;

  /*
     For IBUS channel numbers are C indexes, i.e., one less than your transmitter
     shows you. For CRSF the channel numbers match the numbers on the transmitter.

     We adjust the numbers when reading from ibus channels, so we can use the actual
     channel numbers here - and avoid having duplicate defines for IBUS/CRSF
  */
#if RC_MODEL == NOTANK
  // throttle is only supported right due to support for reversing
  if (controls == RX_MID){
    // throttle right, steering left
    throttle_channel=2;
    steering_channel=4;
  } else {
    // throttle and steering right
    steering_channel=1;
    throttle_channel=2;
  }
#else
  if (controls == RX_MID){
    // throttle right, steering left
    throttle_channel=2;
    steering_channel=4;
  } else if (controls == RX_MAX) {
    // throttle left, steering right
    steering_channel=1;
    throttle_thannel=3;
  } else {
    // throttle and steering right
    steering_channel=1;
    throttle_channel=2;
  }
#endif
}

void set_led(int leds[], int cycle, int effect){
  int i=0;
  while (leds[i] != -1){
    i++;
    switch(effect){
      case led_on:
        digitalWrite(leds[i], HIGH);
        break;
      case led_off:
        digitalWrite(leds[i], LOW);
        break;
      case led_cycle:
        if (cycle < 0){
          digitalWrite(leds[i], LOW);
        } else {
          digitalWrite(leds[i], HIGH);
        }
        break;
    }
  }
}

void setup_led(int leds[]){
  int i=0;
  while (leds[i] != -1){
    i++;
    pinMode(leds[i], OUTPUT);
  }
}

void setup() {
  setup_led(power_leds);
  setup_led(effect_leds);
  setup_led(front_leds);

  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  delay(500);
  digitalWrite(POWER_PIN, HIGH);

#ifdef ARDUINO_AVR_NANO_EVERY
#ifdef DEBUG_MESSAGES
#define debug_print if (debug_state == 0) Serial.print
#define debug_println if (debug_state == 0) Serial.println
#else
#define debug_print(...) ((void)0)
#define debug_println(...) ((void)0)
#endif
  Serial.begin(115200);
  // on the Every the internal timer doesn't work - so disable it here,
  // and manually call loop() in the loop
#if SERIAL_PROTOCOL == TX_IBUS
  IBus.begin(Serial1,IBUSBM_NOTIMER);
#elif SERIAL_PROTOCOL == TX_CRSF
  Serial1.begin(CRSF_SPEED);
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

  wdt_enable(WDTO_2S);

#if SERIAL_PROTOCOL == TX_CRSF
  crsf.update();
  int ch4 = crsf.getChannel(4);
  int loop = 0;
  while(ch4 <= RX_MIN - RX_TOLERANCE - 1){
    debug_print(ch4);
    debug_print(" waiting for CRSF transmitter...");
    debug_println(loop);
    loop++;
    delay(500);
    crsf.update();
    ch4 = crsf.getChannel(4);
    if (loop <= 60)
      wdt_reset();
  }

#if TX_TYPE == RADIOMASTER
  debug_println("");
  debug_print("Channel 4: ");
  debug_println(ch4);
  if (ch4 >= RX_MIN - RX_TOLERANCE && ch4 <= RX_MIN + RX_TOLERANCE){
    current_tx = RADIOMASTER_PG;
  } else if (ch4 >= RX_MID - RX_TOLERANCE && ch4 <= RX_MID + RX_TOLERANCE){
    current_tx = RADIOMASTER_ST;
  } else {
    debug_print("Unable to identify remote type");
    delay(10000);
  }

  // this relies on the model restarting to detect remote control changes.
  // if we want to do that during runtime that needs to move to setup_controls
  tx_mapping = get_tx_mapping(current_tx);
#endif
#endif

  setup_controls();
}

#if RC_MODEL == NOTANK
void controls(int pwm_adjusted, int steer_pwm, int throttle_max){
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

  // ability to slow down improves handling quite a bit, but will require additional
  // steering optimisation as that leads one of the tracks more often into low
  // speed cutoff now
  pwm_left = map(pwm_left, 0, 180, 45 - throttle_max, 135 + throttle_max);
  pwm_right = map(pwm_right, 0, 180, 45 - throttle_max, 135 + throttle_max);

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
void controls(int pwm_adjusted, int steer_pwm, int throttle_max){
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
void controls(int pwm_adjusted, int steer_pwm, int throttle_max){
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
  wdt_reset();

  debug_print("tx: ");
  debug_print(current_tx);
  debug_print(" ");

#if SERIAL_PROTOCOL == TX_IBUS
  IBus.loop();
#elif SERIAL_PROTOCOL == TX_CRSF
  crsf.update();
#endif
  if (led_state >= LED_DELAY)
    led_state = LED_DELAY*-1;
  led_state++;

#ifdef LINK_DOWN_DELAY
  if (crsf.isLinkUp() == false){
    ignition = RX_MIN;
    debug_print(" link down: ");
    debug_print(linkdown_state);
    debug_print(" ");
    if (linkdown_state >= LINK_DOWN_DELAY)
      delay(5000);

    linkdown_state++;
  } else
      linkdown_state = 0;
#endif

  setup_controls();

  if (tx_mapping.ignition_channel >= 1){
#if SERIAL_PROTOCOL == TX_IBUS
    ignition = IBus.readChannel(tx_mapping.ignition_channel-1);
#elif SERIAL_PROTOCOL == TX_CRSF
    if (crsf.isLinkUp() == true)
      ignition = crsf.getChannel(tx_mapping.ignition_channel);
#endif
  }

  debug_print(" Ignition(");
  debug_print(tx_mapping.ignition_channel);
  debug_print("): ");
  debug_print(ignition);
  debug_print(" ");

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
    steer = IBus.readChannel(steering_channel-1);
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
    throttle = IBus.readChannel(throttle_channel-1);
#elif SERIAL_PROTOCOL == TX_CRSF
    throttle = crsf.getChannel(throttle_channel);
#endif

    // TODO, for the snow mobile we probably want a different fallback
    throttle = rx_adjust(throttle, RX_MID);

    debug_print(" Throttle: ");
    debug_print(throttle);

    // TODO: max speed is currently ignored, as that'd only apply to maximum
    //       forward speed. Should be applied to both forward/reverse. But
    //       also might be dropped completely - motor seems to be correctly
    //       sized here, so might not be needed at all.
    int throttle_max_raw = 1500;
    if (tx_mapping.speed_channel >= 1){
#if SERIAL_PROTOCOL == TX_IBUS
      throttle_max_raw = IBus.readChannel(tx_mapping.speed_channel-1);
#elif SERIAL_PROTOCOL == TX_CRSF
      throttle_max_raw = crsf.getChannel(tx_mapping.speed_channel);
#endif
    }


    int throttle_max = map(throttle_max_raw, RX_MIN, RX_MAX, SPEED_MIN, SPEED_MAX);
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

    controls(pwm_adjusted, steer_pwm, throttle_max);
  }

#if SERIAL_PROTOCOL == TX_CRSF
  debug_print(" | ");
#ifdef DEBUG_CHANNELS
  print_channels();
#endif
#endif

  debug_println();

  debug_state++;
  if (debug_state >= DEBUG_DELAY)
    debug_state = 0;

  delay(LOOP_DELAY);
}
