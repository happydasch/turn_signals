// turnsignal.ino
#include <FastLED.h>

// count of leds for every side
#define NUM_LEDS_LEFT 9             // number of leds on the left side
#define NUM_LEDS_RIGHT 9            // number of leds on the right side
// total count of leds per part
#define NUM_LEDS NUM_LEDS_LEFT + NUM_LEDS_RIGHT

// colors for turn signal, front and back
#define COLOR_FRONT 0xffffff        // white
#define COLOR_BACK  0x8b0000        // dark red
#define COLOR_TURN  0xff6700        // safety orange
#define COLOR_OFF   0x000000        // black

// pins being used
#define PIN_BTN_LEFT 5              // pin button left
#define PIN_BTN_RIGHT 6             // pin button right
#define PIN_LGHT_FRONT 3            // pin led data light front
#define PIN_LGHT_BACK 4             // pin led data light back
#define PIN_LGHT_CTRL 7             // pin light switch
#define PIN_LGHT_REQ 8              // pin light request from controller
#define PIN_LED 9                   // pin status led (PWM)
#define PIN_BR A0                   // pin brightness sensor (A0/14)

// flag definitions
#define FLAG_NONE             0x00  // 0000 0000 - none
#define FLAG_CLICK            0x01  // 0000 0001 - single click
#define FLAG_DOUBLE_CLICK     0x02  // 0000 0010 - double click
#define FLAG_TRIBBLE_CLICK    0x04  // 0000 0100 - tripple click
#define FLAG_HOLD             0x08  // 0000 1000 - single click hold
#define FLAG_INVALID          0xFF  // 1111 1111 - invalid

// function definitions
#define FUNC_NONE             0x00  // 0000 0000 - none
#define FUNC_MAIN             0x01  // 0000 0001 - main light
#define FUNC_ADDITIONAL       0x02  // 0000 0010 - additional light (led)
#define FUNC_SOS              0x03  // 0000 0011 - sos
#define FUNC_TURN_LEFT        0x11  // 0001 0001 - turn left
#define FUNC_TURN_RIGHT       0x12  // 0001 0010 - turn right
#define FUNC_TURN_LEFT_FORCE  0x13  // 0001 0011 - turn left force
#define FUNC_TURN_RIGHT_FORCE 0x14  // 0001 0100 - turn right force
#define FUNC_LIGHTER          0x21  // 0010 0001 - lighter
#define FUNC_DARKER           0x22  // 0010 0010 - darker
#define FUNC_INVALID          0xFF  // 1111 1111 - invalid

// draw definitions
#define DRAW_DEFAULT          0x00  // 0000 0000 - no special draw
#define DRAW_LEFT             0x01  // 0000 0001 - left animation
#define DRAW_RIGHT            0x02  // 0000 0010 - right animation
#define DRAW_SOS              0x03  // 0000 0011 - sos animation

// number of brightness readings
#define NUM_READ              50
// threshold for automatic light switch on
#define THRESHOLD_AUTO_ON     250
// debug messages
#define DEBUG                 false
// frames / drawing
#define FRAMES_PER_SECOND     60
#define LED_TYPE              WS2812B
#define COLOR_ORDER           GRB

// led values
CRGBArray<NUM_LEDS> g_leds_front;     // front leds (left and right)
CRGBArray<NUM_LEDS> g_leds_back;      // back leds (left and right)

// common values
uint64_t now = 0;
uint64_t g_blink_timer = 0;           // ms for blink timer
uint64_t g_blink_timeout = 20000;     // ms for blink timeout
uint64_t g_click_gap = 500;           // ms click period: between clicks for multiple clicks
uint64_t g_hold_time = 800;           // ms hold period: how long to wait for press+hold event
uint64_t g_blink_interval = 400;      // ms between blink
uint64_t g_blink_duration = 600;      // ms for blink light on
int g_led_brightness = 0;             // brightness for status led
int g_brightness = 100;               // brightness for lights
int g_brightness_change = 30;         // brightness change rate
int g_blink_brightness = 240;         // blink brightness for turn lights
bool g_blink_active = false;          // will be true while blink duration is active (turn lights are on)
float g_blink_grow_factor = 0.4;      // blink grow factor (part of time to use for grow animation)
float g_blink_darken_factor = 0.96;   // blink darken factor (how much to dim the light)
float g_front_brightness_factor = 0.9;  // front leds brightness factor
float g_back_brightness_factor = 1.1;   // back leds brightness factor

// button values
uint64_t g_time_left = 0;             // ms for left button timer
uint64_t g_time_right = 0;            // ms for right button timer
int g_state_left = HIGH;              // last state of left button
int g_state_right = HIGH;             // last state of right button
int g_flags_left = 0x00;              // flags for left button
int g_flags_right = 0x00;             // flags for right button

// brightness values
uint64_t g_brightness_timer = 0;      // ms for brightness timer
int g_brightness_interval = 200;      // ms for brightness level update
int g_sensor_brightness = 0;          // current brightness from sensor
int g_brightness_readings[NUM_READ];  // brightness readings
int g_brightness_avg;                 // brightness readings average
bool g_brightness_autolight = true;   // should light be switched on automatically when brightness is low
bool g_brightness_on = false;         // is light on by brightness sensor

// draw values
uint64_t g_draw_mode_timer = 0;       // time a draw mode was set last
uint64_t g_draw_timer = 0;            // time a scene was drawn last
bool g_draw_lights = false;           // are lights drawn (will be set when switching on)
int g_draw_mode = DRAW_DEFAULT;       // draw mode
int g_draw_step = 0;                  // draw step (used for default scene)
int g_draw_duration = 800;            // draw duration for animations
int g_redraw_interval = 1000 / FRAMES_PER_SECOND; // ms for redraw interval

// light values
bool g_lights_on = false;             // is lights active
bool g_lights_controller = false;     // is lights activated in controller
bool g_lights_additional = false;     // is additional lights active
bool g_left_active = false;           // is left turn light active
bool g_right_active = false;          // is right turn light active
bool g_sos_active = false;            // is sos mode active

/**
 * @brief Updates the average of brightness readings.
 */
void update_sensor_brightness() {
  // check if a update is needed
  if (now < g_brightness_timer + g_brightness_interval) {
    return;
  }
  // shift readings
  g_sensor_brightness = analogRead(PIN_BR);
  for (int i = NUM_READ - 1; i > 0; i--) {
    g_brightness_readings[i] = g_brightness_readings[i - 1];
  }
  // add new reading
  g_brightness_readings[0] = g_sensor_brightness;

  // calculate average
  int sum = 0;
  int total = 0;
  for (int i = 0; i < NUM_READ; i++) {
    if (g_brightness_readings[i] > 0) {
      sum += g_brightness_readings[i];
      total++;
    }
  }
  if (total > 0) {
    g_brightness_avg = sum / total;
  } else {
    g_brightness_avg = 999;
  }
  g_brightness_timer = now;
  if (g_brightness_autolight) {
    if (!g_brightness_on && g_brightness_avg <= THRESHOLD_AUTO_ON) {
      // only switch lights on one time when reading is below threshold
      g_lights_on = true;
      g_brightness_on = true;
    } else if (g_brightness_avg > THRESHOLD_AUTO_ON) {
      g_brightness_on = false;
    }
  }
}

/**
 * @brief Updates the light request from the controller.
 */
void update_light_request_from_controller() {
  bool current = (bool)digitalRead(PIN_LGHT_REQ);
  if (current != g_lights_controller) {
    // update light state only if the state has changed
    g_lights_controller = current;
    g_lights_on = current;
  }
}

/**
 * @brief Checks the button for a complete sequence and processes
 *
 * button flags within sequence.
 *
 * @param pin The pin to check.
 * @param time The time of the last button state change.
 * @param state The previous button state.
 * @param flags The button flags.
 * @return True if a complete sequence was detected.
 */
bool check_button_state(int pin, uint64_t *time, int *state, int *flags) {
  bool result = false;
  int stateNew = digitalRead(pin);

  // check for valid time
  if (*time == 0) {
    *time = now;
  }

  // check for state change
  if (stateNew != *state) {
    // state changed
    *time = now;
    *state = stateNew;
    if (*state == LOW) {
      if (*flags & FLAG_DOUBLE_CLICK) {
        *flags = FLAG_TRIBBLE_CLICK;
      } else if (*flags & FLAG_CLICK) {
        *flags = FLAG_DOUBLE_CLICK;
      } else if (*flags == FLAG_NONE) {
        *flags = FLAG_CLICK;
      } else {
        *flags = FLAG_INVALID;
      }
    }
  } else {
    // state did not change
    if (*state == LOW) {
      if (now - *time > g_hold_time) {
        // button hold
        *flags |= FLAG_HOLD;
      }
    }
  }

  if (*flags != FLAG_NONE) {
    // if any flags are set, a sequence is active so check for end of sequence
    if (*state == HIGH && now - *time > g_click_gap) {
      // sequence finished if button not pressed again within click gap
      result = true;
    }
    if (*flags & FLAG_HOLD || *flags == FLAG_INVALID) {
      // sequence is finished if long hold or invalid flag is set
      result = true;
    }
  }

  return result;
}

/**
 * @brief Reads all sensors and updates the global variables.
 */
void read_sensors() {
  update_sensor_brightness();
  update_light_request_from_controller();
}

/**
 * @brief Checks the button flags and returns the light function.
 *
 * @return The light function.
 */
int process_light_flags() {
  if (g_flags_left == FLAG_INVALID || g_flags_right == FLAG_INVALID) {
    return FUNC_INVALID;
  }
  // single click on both buttons - main on/off
  if (g_flags_left & FLAG_CLICK && g_flags_right & FLAG_CLICK) {
    return FUNC_MAIN;
  }
  // double click on both buttons - additional on/off
  if (g_flags_left & FLAG_DOUBLE_CLICK && g_flags_right & FLAG_DOUBLE_CLICK) {
    return FUNC_ADDITIONAL;
  }
  // tribble click on both buttons - sos
  if (g_flags_left & FLAG_TRIBBLE_CLICK && g_flags_right & FLAG_TRIBBLE_CLICK) {
    return FUNC_SOS;
  }
  // double click on right button - turn right forced
  if (g_flags_right & FLAG_DOUBLE_CLICK && g_flags_left == FLAG_NONE) {
    return FUNC_TURN_RIGHT_FORCE;
  }
  // tribble click or hold on right button - lighter
  if (g_flags_right & FLAG_HOLD || g_flags_right & FLAG_TRIBBLE_CLICK) {
    return FUNC_LIGHTER;
  }
  // double click on left button - turn left forced
  if (g_flags_left & FLAG_DOUBLE_CLICK && g_flags_right == FLAG_NONE) {
    return FUNC_TURN_LEFT_FORCE;
  }
  // tribble click hold on left button - darker
  if (g_flags_left & FLAG_HOLD || g_flags_left & FLAG_TRIBBLE_CLICK) {
    return FUNC_DARKER;
  }
  // single click on left button - turn left
  if (g_flags_left & FLAG_CLICK && g_flags_right == FLAG_NONE) {
    return FUNC_TURN_LEFT;
  }
  // single click on right button - turn right
  if (g_flags_right & FLAG_CLICK && g_flags_left == FLAG_NONE) {
    return FUNC_TURN_RIGHT;
  }
  return FUNC_NONE;
}


/**
 * @brief Returns the front brightness.
 *
 * @return int
 */
int get_front_brightness() {
  return constrain(g_brightness * g_front_brightness_factor, 0, 255);
}

/**
 * @brief Returns the back brightness.
 *
 * @return int
 */
int get_back_brightness() {
  return constrain(g_brightness * g_back_brightness_factor, 0, 255);
}

/**
 * @brief Sets the light switch to the given state.
 */
void set_light_switch(bool state) {
  int current = digitalRead(PIN_LGHT_CTRL);
  if (current != (int)state) {
    digitalWrite(PIN_LGHT_CTRL, state ? HIGH : LOW);
  }
}

/**
 * @brief Returns if the lights are blinking.
 *
 * @return true
 * @return false
 */
bool is_light_blinking() {
  return g_sos_active || g_left_active || g_right_active;
}

/**
 * @brief Set the current draw mode.
 *
 * @param mode the draw mode to set
 */
void set_draw_mode(int mode) {
  g_draw_mode = mode;
  g_draw_mode_timer = now;
  g_draw_step = 0;
  g_draw_timer = 0;
  if (!is_light_blinking()) {
    g_blink_timer = 0;
  }
}

/**
 * @brief Sets the light function.
 *
 * @param func The light function.
 */
void process_light_function(int func) {
  if (func == FUNC_INVALID) return;

  bool isBlinking = is_light_blinking();

  switch (func) {
    case FUNC_MAIN:
      if (g_sos_active) {
        g_sos_active = false;
      } else {
        if (g_lights_on) {
          g_lights_on = false;
        } else {
          g_lights_on = true;
        }
      }
      set_draw_mode(DRAW_DEFAULT);
      break;
    case FUNC_ADDITIONAL:
      if (g_sos_active) {
        g_sos_active = false;
      } else {
        if (g_lights_additional) {
          g_lights_additional = false;
        } else {
          g_lights_additional = true;
        }
      }
      set_draw_mode(DRAW_DEFAULT);
      break;
    case FUNC_SOS:
      g_sos_active = bool(!g_sos_active);
      g_left_active = false;
      g_right_active = false;
      if (g_sos_active) {
        set_draw_mode(DRAW_SOS);
      } else {
        set_draw_mode(DRAW_DEFAULT);
      }
      break;
    case FUNC_TURN_LEFT:
      g_left_active = bool(!isBlinking);
      g_right_active = false;
      if (g_left_active) {
        set_draw_mode(DRAW_LEFT);
      } else {
        set_draw_mode(DRAW_DEFAULT);
      }
      break;
    case FUNC_TURN_RIGHT:
      g_right_active = int(!isBlinking);
      g_left_active = false;
      if (g_right_active) {
        set_draw_mode(DRAW_RIGHT);
      } else {
        set_draw_mode(DRAW_DEFAULT);
      }
      break;
    case FUNC_TURN_LEFT_FORCE:
      g_left_active = true;
      g_right_active = false;
      if (!g_sos_active) {
        set_draw_mode(DRAW_LEFT);
      }
      break;
    case FUNC_TURN_RIGHT_FORCE:
      g_right_active = true;
      g_left_active = false;
      if (!g_sos_active) {
        set_draw_mode(DRAW_RIGHT);
      }
      break;
    case FUNC_LIGHTER:
      g_brightness += g_brightness_change;
      g_brightness = min(g_brightness, 255);
      if (!g_lights_additional) {
        g_lights_additional = true;
      }
      break;
    case FUNC_DARKER:
      g_brightness -= g_brightness_change;
      g_brightness = max(g_brightness, 0);
      if (g_brightness == 0) {
        g_lights_additional = false;
      } else if (!g_lights_additional) {
        g_lights_additional = true;
      }
      break;
  }
}

/**
 * @brief Updates the light switch that controls the lights.
 */
void update_light_switch() {
  if (g_lights_on) {
    if (g_lights_additional && g_draw_lights) {
      set_light_switch(true);
    } else if (!g_lights_additional) {
      set_light_switch(true);
    }
  } else {
    if (g_lights_additional) {
      set_light_switch(false);
    } else if (!g_lights_additional && !g_draw_lights) {
      set_light_switch(false);
    }
  }
}

/**
 * @brief Updates the blink timer.
 *
 * The globally available state of the blink light is updated here.
 */
void update_blink_timer() {
  if (g_blink_timer + g_blink_duration + g_blink_interval < now) {
    g_blink_timer = now;
  }
  uint64_t until = g_blink_timer + g_blink_duration;

  if (is_light_blinking() && until > now) {
    g_blink_active = true;
  } else {
    g_blink_active = false;
  }
}

/**
 * @brief Updates the status led.
 */
void update_status_led() {
  int brightness = g_lights_additional ? 50 : 100;

  if (g_blink_active) {
    brightness = 150;
  }
  if (g_led_brightness != brightness) {
    analogWrite(PIN_LED, brightness);
    g_led_brightness = brightness;
  }
}

/**
 * @brief Notifies the user using the status led.
 *
 * @param count The number of flashes.
 * @param duration The duration of each flash.
 * @param pause The pause between each flash.
 */
void notify_status_led(int count, int duration=250, int pause=-1) {
  if (pause == -1) {
    pause = duration;
  }
  for (int i = 0; i < count; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(duration);
    digitalWrite(PIN_LED, LOW);
    if (pause > 0) {
      delay(pause);
    }
  }
  g_led_brightness = 0;
}

/**
 * @brief Notifies the user about the selected light function.
 *
 * Only some functions are being used for notifications.
 *
 * @param func The light function.
 */
void notify_light_function(int func) {
  switch (func) {
    case FUNC_MAIN:
      // 1 x led default
      notify_status_led(1);
      break;
    case FUNC_ADDITIONAL:
      // 2 x led default
      notify_status_led(2);
      break;
    case FUNC_SOS:
      // 3 x led default
      notify_status_led(3);
      break;
    case FUNC_LIGHTER:
    case FUNC_DARKER:
      // 1 x led short
      notify_status_led(1, 50);
      break;
    case FUNC_INVALID:
      // 1 led - long with long delay
      notify_status_led(1, 100, 200);
      break;
  }
}

/**
 * @brief Draws the left side of the blinking light.
 */
void draw_left_side() {
  uint64_t timeDiff = now - g_blink_timer;
  int durationStep = g_blink_duration / NUM_LEDS_LEFT;
  int ledsOn = min(NUM_LEDS_LEFT, (timeDiff / (float)(durationStep * g_blink_grow_factor)));

  for (int i = 0; i < ledsOn; i++) {
    g_leds_front[i] = COLOR_TURN;
    g_leds_back[i] = COLOR_TURN;
    g_leds_front[i].maximizeBrightness(g_blink_brightness);
    g_leds_back[i].maximizeBrightness(g_blink_brightness);
  }
}

/**
 * @brief Draws the right side of the blinking light.
 */
void draw_right_side() {
  uint64_t timeDiff = now - g_blink_timer;
  int durationStep = g_blink_duration / NUM_LEDS_RIGHT;
  int ledsOn = min(NUM_LEDS_RIGHT, (timeDiff / (float)(durationStep * g_blink_grow_factor)));

  for (int i = 0; i < ledsOn; i++) {
    g_leds_front[i + NUM_LEDS_LEFT] = COLOR_TURN;
    g_leds_back[i + NUM_LEDS_LEFT] = COLOR_TURN;
    g_leds_front[i + NUM_LEDS_LEFT].maximizeBrightness(g_blink_brightness);
    g_leds_back[i + NUM_LEDS_LEFT].maximizeBrightness(g_blink_brightness);
  }
}

/**
 * @brief Draws the scene: On Animation.
 *
 * The scene will fade from black to color from top to bottom.
 *
 * @param step current step of animation
 * @return true if the scene animation is complete
 * @return false while the scene animation is not complete
 */
bool scene_anim_on(int step) {
  int steps = g_draw_duration / g_redraw_interval;
  if (step > steps) {
    return true;
  }
  int frontBrightness = get_front_brightness();
  int backBrightness = get_back_brightness();
  float curFrontBrightness = round((float)frontBrightness / steps) * step;
  float curBackBrightness = round((float)backBrightness / steps) * step;

  for (int i = 0; i < NUM_LEDS; i++) {
    g_leds_front[i] = COLOR_FRONT;
    g_leds_back[i] = COLOR_BACK;
    g_leds_front[i].maximizeBrightness(constrain(curFrontBrightness, 0, frontBrightness));
    g_leds_back[i].maximizeBrightness(constrain(curBackBrightness, 0, backBrightness));
  }
  return false;
}

/**
 * @brief Draws the scene: Off Animation.
 *
 * The scene will fade from black to color from top to bottom.
 *
 * @param step current step of animation
 * @return true if the scene animation is complete
 * @return false while the scene animation is not complete
 */
bool scene_anim_off(int step) {
  int steps = g_draw_duration / g_redraw_interval;
  if (step > steps) {
    return true;
  }
  int frontBrightness = get_front_brightness();
  int backBrightness = get_back_brightness();
  float curFrontBrightness = frontBrightness - (round((float)frontBrightness / steps) * step);
  float curBackBrightness = backBrightness - (round((float)backBrightness / steps) * step);

  for (int i = 0; i < NUM_LEDS; i++) {
    g_leds_front[i] = COLOR_FRONT;
    g_leds_back[i] = COLOR_BACK;
    g_leds_front[i].maximizeBrightness(constrain(curFrontBrightness, 0, frontBrightness));
    g_leds_back[i].maximizeBrightness(constrain(curBackBrightness, 0, backBrightness));
  }
  return false;
}

/**
 * @brief Draws the scene: On.
 */
void scene_on() {
  int frontBrightness = get_front_brightness();
  int backBrightness = get_back_brightness();

  for (int i = 0; i < NUM_LEDS; i++) {
    g_leds_front[i] = COLOR_FRONT;
    g_leds_back[i] = COLOR_BACK;
    g_leds_front[i].maximizeBrightness(frontBrightness);
    g_leds_back[i].maximizeBrightness(backBrightness);
  }
  if (!g_draw_lights) {
    if (scene_anim_on(g_draw_step)) {
      g_draw_lights = true;
    } else {
      g_draw_step++;
    }
  }
}

/**
 * @brief Draws the scene: Off.
 */
void scene_off() {
  g_leds_front.fill_solid(COLOR_OFF);
  g_leds_back.fill_solid(COLOR_OFF);
  for (int i = 0; i < NUM_LEDS; i++) {
    g_leds_front[i] = COLOR_OFF;
    g_leds_back[i] = COLOR_OFF;
  }
  if (g_draw_lights) {
    if (scene_anim_off(g_draw_step)) {
      g_draw_lights = false;
    } else {
      g_draw_step++;
    }
  }
}

/**
 * @brief Draws the scene: SOS Animation.
 *
 * @param step current step of animation
 * @return true if the scene animation is complete
 * @return false while the scene animation is not complete
 */
void scene_anim_sos() {
  if (g_lights_additional) {
    scene_on();
  } else {
    scene_off();
  }
  // darken light
  for (int i = 0; i < NUM_LEDS; i++) {
    g_leds_front[i].fadeLightBy(255 * g_blink_darken_factor);
    g_leds_back[i].fadeLightBy(255 * g_blink_darken_factor);
  }
  if (g_blink_active) {
    draw_left_side();
    draw_right_side();
  }
}

/**
 * @brief Draws the scene: Left Animation.
 *
 * @param step current step of animation
 * @return true if the scene animation is complete
 * @return false while the scene animation is not complete
 */
void scene_anim_left() {
  if (g_lights_additional) {
    scene_on();
  } else {
    scene_off();
  }
  // darken the active side
  for (int i = 0; i < NUM_LEDS_LEFT; i++) {
    int idx = i;
    g_leds_front[idx].fadeLightBy(255 * g_blink_darken_factor);
    g_leds_back[idx].fadeLightBy(255 * g_blink_darken_factor);
  }
  if (g_blink_active) {
    draw_left_side();
  }
}

/**
 * @brief Draws the scene: Right Animation.
 *
 * @param step current step of animation
 * @return true if the scene animation is complete
 * @return false while the scene animation is not complete
 */
void scene_anim_right() {
  if (g_lights_additional) {
    scene_on();
  } else {
    scene_off();
  }
  // darken the active side
  for (int i = 0; i < NUM_LEDS_RIGHT; i++) {
    int idx = i + NUM_LEDS_LEFT;
    g_leds_front[idx].fadeLightBy(255 * g_blink_darken_factor);
    g_leds_back[idx].fadeLightBy(255 * g_blink_darken_factor);
  }
  if (g_blink_active) {
    draw_right_side();
  }
}

/**
 * Draws the light scene.
 */
void draw_scene() {
  uint64_t nextRedraw = g_draw_timer + g_redraw_interval;  // next redraw, in ms
  uint64_t nextForceRedraw = g_draw_timer + (10 * g_redraw_interval);  // next forced redraw, in ms
  int drawMode = g_draw_mode;

  // check if a scene update is needed
  if (nextRedraw > now) {
    return;
  }

  if (drawMode != DRAW_DEFAULT) {
    if (g_draw_mode_timer + g_blink_timeout < now) {
      // blink timeout reached, switch back to default
      if (!g_sos_active) {
        // only switch back to default if not in sos mode
        g_left_active = false;
        g_right_active = false;
        set_draw_mode(DRAW_DEFAULT);
      }
    }
  }

  // ensure correct draw mode is set, prioritze sos over turn signals
  if (g_sos_active) {
    drawMode = DRAW_SOS;
  } else if (g_left_active) {
    drawMode = DRAW_LEFT;
  } else if (g_right_active) {
    drawMode = DRAW_RIGHT;
  }

  // draw the scene
  switch (drawMode) {
    case DRAW_DEFAULT:
      // default scene on
      if (g_lights_additional) {
        // lights are on and shown, just refresh
        if (g_draw_lights && now < nextForceRedraw) {
          return;
        }
        scene_on();
      }
      // default scene off
      if (!g_lights_additional) {
        // lights are off and not shown, just refresh
        if (!g_draw_lights && now < nextForceRedraw) {
          return;
        }
        scene_off();
      }
      break;
    case DRAW_SOS:
      scene_anim_sos();
      break;
    case DRAW_LEFT:
      scene_anim_left();
      break;
    case DRAW_RIGHT:
      scene_anim_right();
      break;
  }

  // finish current scene
  FastLED.show();
  g_draw_timer = now;
}

/**
 * @brief Button left press interrupt handler.
 */
void _btn_left_press() {
  process_light_function(FUNC_TURN_LEFT);
}

/**
 * @brief Button right release interrupt handler.
 */
void _btn_right_press() {
  process_light_function(FUNC_TURN_RIGHT);
}

/*
 * Setup
 */
void setup() {
  // serial setup
  #if DEBUG == true
  Serial.begin(9600);
  #endif
  // pin setup
  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);    // right button
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);   // left button
  pinMode(PIN_LGHT_REQ, INPUT_PULLUP);    // light request from controller
  pinMode(PIN_BR, INPUT);                 // brightness sensor
  pinMode(PIN_LGHT_CTRL, OUTPUT);         // light switch
  pinMode(PIN_LGHT_FRONT, OUTPUT);        // front led data
  pinMode(PIN_LGHT_BACK, OUTPUT);         // back led data
  pinMode(PIN_LED, OUTPUT);               // status led
  // prepare leds
  FastLED.addLeds<LED_TYPE, PIN_LGHT_FRONT, COLOR_ORDER>(g_leds_front, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, PIN_LGHT_BACK, COLOR_ORDER>(g_leds_back, NUM_LEDS);
  FastLED.setCorrection(TypicalLEDStrip);
  FastLED.setTemperature(UncorrectedTemperature);
  FastLED.setBrightness(255); // set default brightness to max
  FastLED.setDither(0);  // stops flikering in animations.
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 1100);
  // attach interupts for buttons
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_LEFT), _btn_left_press, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_RIGHT), _btn_right_press, RISING);
  // set light output to inital values
  digitalWrite(PIN_LGHT_CTRL, g_lights_on);
  digitalWrite(PIN_LGHT_REQ, g_lights_controller);
}

/*
 * Main loop
 */
void loop() {
  now = millis();

  // check sensor states (light, brightness)
  read_sensors();

  // check button states
  if (check_button_state(PIN_BTN_LEFT, &g_time_left, &g_state_left, &g_flags_left)
      || check_button_state(PIN_BTN_RIGHT, &g_time_right, &g_state_right, &g_flags_right)) {
    // button sequence finished
    int function = process_light_flags();
    // process the light function
    process_light_function(function);
    // reset flags after processing
    g_flags_left = FLAG_NONE;
    g_flags_right = FLAG_NONE;
    g_time_left = 0;
    g_time_right = 0;
    // notify about selected function
    notify_light_function(function);
  } else {
    // nothing changed, check status led
    update_status_led();
  }

  // update blink timer
  update_blink_timer();

  // draw light scene
  draw_scene();

  // update light switch
  update_light_switch();

  #if DEBUG == true
  // debug output
  Serial.print("Lights: ");
  Serial.print(g_lights_on);
  Serial.print(", Controller: ");
  Serial.print(g_lights_controller);
  Serial.print(", Additional: ");
  Serial.print(g_lights_additional);
  Serial.print(", Left active: ");
  Serial.print(g_left_active);
  Serial.print(", Right active: ");
  Serial.print(g_right_active);
  Serial.print(", Sos active: ");
  Serial.print(g_sos_active);
  Serial.print(", Draw lights: ");
  Serial.print(g_draw_lights);
  Serial.print(", Draw mode: ");
  Serial.print(g_draw_mode);
  Serial.println();
  #endif
}
