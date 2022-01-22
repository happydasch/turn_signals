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
#define PIN_LGHT_CTRL 7             // pin light controller
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
#define DRAW_LEFT             0x03  // 0000 0011 - left animation
#define DRAW_RIGHT            0x04  // 0000 0100 - right animation
#define DRAW_SOS              0x05  // 0000 0101 - sos animation

// number of brightness readings
#define NUM_READ 50
// debug messages
#define DEBUG 0
// frames / drawing
#define FRAMES_PER_SECOND   60
#define LED_TYPE            WS2812B
#define COLOR_ORDER         GRB

// led values
CRGBArray<NUM_LEDS> gLedsFront;     // front leds (left and right)
CRGBArray<NUM_LEDS> gLedsBack;      // back leds (left and right)

// common values
uint64_t now = 0;
uint64_t gBlinkTimer = 0;           // ms for blink timer
uint64_t gBlinkTimeout = 20000;     // ms for blink timeout
int gLedBrightness = 0;             // brightness for status led
int gClickGap = 500;                // max ms between clicks for multiple clicks
int gHoldTime = 750;                // ms hold period: how long to wait for press+hold event
int gBrightness = 100;              // brightness for lights
int gBrightnessChange = 30;         // brightness change rate
int gBlinkBrightness = 240;         // blink brightness for turn lights
int gBlinkInterval = 900;           // ms between blink
int gBlinkDuration = 750;           // ms for blink light on
bool gBlinkActive = false;          // will be true while blink duration is active (turn lights are on)
float gFrontBrightnessFactor = 0.9; // front leds brightness factor
float gBackBrightnessFactor = 1.1;  // back leds brightness factor
float gBlinkGrowFactor = 0.4;       // blink grow factor (part of time to use for grow animation)
float gBlinkDarkenFactor = 0.96;    // blink darken factor (how much to dim the light)

// button values
uint64_t gTimeLeft = 0;             // ms for left button timer
uint64_t gTimeRight = 0;            // ms for right button timer
int gStateLeft = HIGH;              // last state of left button
int gStateRight = HIGH;             // last state of right button
int gFlagsLeft = 0x00;              // flags for left button
int gFlagsRight = 0x00;             // flags for right button

// brightness values
uint64_t gBrightnessTimer = 0;      // ms for brightness timer
int gBrightnessInterval = 200;      // ms for brightness level update
int gSensorBrightness = 0;          // current brightness from sensor
int gBrightnessReadings[NUM_READ];  // brightness readings
int gBrightnessAvg;                 // brightness readings average

// draw values
uint64_t gDrawModeTimer = 0;        // time a draw mode was set last
uint64_t gDrawTimer = 0;            // time a scene was drawn last
bool gDrawLights = false;           // are lights drawn (will be set when switching on)
int gDrawMode = DRAW_DEFAULT;       // draw mode
int gDrawStep = 0;                  // draw step (used for default scene)
int gDrawDuration = 800;            // draw duration for animations
int gRedrawInterval = 1000 / FRAMES_PER_SECOND; // ms for redraw interval

// light values
bool gLightsOn = false;             // is lights active
bool gLightsAdditional = false;     // is additional lights (led) active
bool gLeftActive = false;           // is left turn light active
bool gRightActive = false;          // is right turn light active
bool gSosActive = false;            // is sos mode active

/*
 * Setup
 */
void setup() {
  // delay for recovery
  delay(2000);
  // serial setup
  Serial.begin(9600);

  // pin setup
  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);    // right button
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);   // left button
  pinMode(PIN_BR, INPUT);                 // brightness sensor
  pinMode(PIN_LGHT_CTRL, OUTPUT);         // light switch
  pinMode(PIN_LGHT_FRONT, OUTPUT);        // front led data
  pinMode(PIN_LGHT_BACK, OUTPUT);         // back led data
  pinMode(PIN_LED, OUTPUT);               // status led

  // prepare leds
  FastLED.addLeds<LED_TYPE, PIN_LGHT_FRONT, COLOR_ORDER>(gLedsFront, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, PIN_LGHT_BACK, COLOR_ORDER>(gLedsBack, NUM_LEDS);
  FastLED.setCorrection(TypicalLEDStrip);
  FastLED.setTemperature(UncorrectedTemperature);
  FastLED.setBrightness(255); // set default brightness to max
  FastLED.setDither(0);  // stops flikering in animations.
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 1100);

  // attach interupts for buttons
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_LEFT), _btnLeftPress, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_RIGHT), _btnRightPress, RISING);
}

/*
 * Main loop
 */
void loop() {
  now = millis();

  // check sensor states (light, brightness)
  readSensors();

  // check button states
  if (checkButtonState(PIN_BTN_LEFT, &gTimeLeft, &gStateLeft, &gFlagsLeft)
      || checkButtonState(PIN_BTN_RIGHT, &gTimeRight, &gStateRight, &gFlagsRight)) {
    // button sequence finished
    int function = processLightFlags();
    // process the light function
    processLightFunction(function);
    // reset flags after processing
    gFlagsLeft = FLAG_NONE;
    gFlagsRight = FLAG_NONE;
    gTimeLeft = 0;
    gTimeRight = 0;
    // notify about selected function
    notifyLightFunction(function);
  } else {
    // nothing changed, check status led
    updateStatusLed();
  }

  // update blink timer
  updateBlinkTimer();

  // draw light scene
  drawScene();

  // update light switch
  updateLightSwitch();
}

/**
 * Draws the light scene.
 */
void drawScene() {
  uint64_t nextRedraw = gDrawTimer + gRedrawInterval;  // next redraw, in ms
  uint64_t nextForceRedraw = gDrawTimer + (10 * gRedrawInterval);  // next forced redraw, in ms
  int drawMode = gDrawMode;

  // check if a scene update is needed
  if (nextRedraw > now) {
    return;
  }

  if (drawMode != DRAW_DEFAULT) {
    if (gDrawModeTimer + gBlinkTimeout < now) {
      // blink timeout reached, switch back to default
      if (!gSosActive) {
        // only switch back to default if not in sos mode
        gLeftActive = false;
        gRightActive = false;
        setDrawMode(DRAW_DEFAULT);
      }
    }
  }

  // ensure correct draw mode is set, prioritze sos over turn signals
  if (gSosActive) {
    drawMode = DRAW_SOS;
  } else if (gLeftActive) {
    drawMode = DRAW_LEFT;
  } else if (gRightActive) {
    drawMode = DRAW_RIGHT;
  }

  // draw the scene
  switch (drawMode) {
    case DRAW_DEFAULT:
      // default scene on
      if (gLightsAdditional) {
        // lights are on and shown, just refresh
        if (gDrawLights && now < nextForceRedraw) {
          return;
        }
        sceneOn();
      }
      // default scene off
      if (!gLightsAdditional) {
        // lights are off and not shown, just refresh
        if (!gDrawLights && now < nextForceRedraw) {
          return;
        }
        sceneOff();
      }
      break;
    case DRAW_SOS:
      sceneAnimSos();
      break;
    case DRAW_LEFT:
      sceneAnimLeft();
      break;
    case DRAW_RIGHT:
      sceneAnimRight();
      break;
  }

  // finish current scene
  FastLED.show();
  gDrawTimer = now;
}

/**
 * @brief Set the current draw mode.
 *
 * @param mode the draw mode to set
 */
void setDrawMode(int mode) {
  gDrawMode = mode;
  gDrawModeTimer = now;
  gDrawStep = 0;
  gDrawTimer = 0;
  if (!isLightBlinking()) {
    gBlinkTimer = 0;
  }
}

/**
 * @brief Draws the scene: On.
 */
void sceneOn() {
  int frontBrightness = getFrontBrightness();
  int backBrightness = getBackBrightness();

  for (int i = 0; i < NUM_LEDS; i++) {
    gLedsFront[i] = COLOR_FRONT;
    gLedsBack[i] = COLOR_BACK;
    gLedsFront[i].maximizeBrightness(frontBrightness);
    gLedsBack[i].maximizeBrightness(backBrightness);
  }
  if (!gDrawLights) {
    if (sceneAnimOn(gDrawStep)) {
      gDrawLights = true;
    } else {
      gDrawStep++;
    }
  }
}

/**
 * @brief Draws the scene: Off.
 */
void sceneOff() {
  gLedsFront.fill_solid(COLOR_OFF);
  gLedsBack.fill_solid(COLOR_OFF);
  for (int i = 0; i < NUM_LEDS; i++) {
    gLedsFront[i] = COLOR_OFF;
    gLedsBack[i] = COLOR_OFF;
  }
  if (gDrawLights) {
    if (sceneAnimOff(gDrawStep)) {
      gDrawLights = false;
    } else {
      gDrawStep++;
    }
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
bool sceneAnimOn(int step) {
  int steps = gDrawDuration / gRedrawInterval;
  if (step > steps) {
    return true;
  }
  int frontBrightness = getFrontBrightness();
  int backBrightness = getBackBrightness();
  float curFrontBrightness = round((float)frontBrightness / steps) * step;
  float curBackBrightness = round((float)backBrightness / steps) * step;

  for (int i = 0; i < NUM_LEDS; i++) {
    gLedsFront[i] = COLOR_FRONT;
    gLedsBack[i] = COLOR_BACK;
    gLedsFront[i].maximizeBrightness(constrain(curFrontBrightness, 0, frontBrightness));
    gLedsBack[i].maximizeBrightness(constrain(curBackBrightness, 0, backBrightness));
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
bool sceneAnimOff(int step) {
  int steps = gDrawDuration / gRedrawInterval;
  if (step > steps) {
    return true;
  }
  int frontBrightness = getFrontBrightness();
  int backBrightness = getBackBrightness();
  float curFrontBrightness = frontBrightness - (round((float)frontBrightness / steps) * step);
  float curBackBrightness = backBrightness - (round((float)backBrightness / steps) * step);

  for (int i = 0; i < NUM_LEDS; i++) {
    gLedsFront[i] = COLOR_FRONT;
    gLedsBack[i] = COLOR_BACK;
    gLedsFront[i].maximizeBrightness(constrain(curFrontBrightness, 0, frontBrightness));
    gLedsBack[i].maximizeBrightness(constrain(curBackBrightness, 0, backBrightness));
  }
  return false;
}

/**
 * @brief Draws the scene: SOS Animation.
 *
 * @param step current step of animation
 * @return true if the scene animation is complete
 * @return false while the scene animation is not complete
 */
void sceneAnimSos() {
  if (gLightsAdditional) {
    sceneOn();
  } else {
    sceneOff();
  }
  // darken light
  for (int i = 0; i < NUM_LEDS; i++) {
    gLedsFront[i].fadeLightBy(255 * gBlinkDarkenFactor);
    gLedsBack[i].fadeLightBy(255 * gBlinkDarkenFactor);
  }
  if (gBlinkActive) {
    drawLeftSide();
    drawRightSide();
  }
}

/**
 * @brief Draws the scene: Left Animation.
 *
 * @param step current step of animation
 * @return true if the scene animation is complete
 * @return false while the scene animation is not complete
 */
void sceneAnimLeft() {
  if (gLightsAdditional) {
    sceneOn();
  } else {
    sceneOff();
  }
  // darken the active side
  for (int i = 0; i < NUM_LEDS_LEFT; i++) {
    int idx = i;
    gLedsFront[idx].fadeLightBy(255 * gBlinkDarkenFactor);
    gLedsBack[idx].fadeLightBy(255 * gBlinkDarkenFactor);
  }
  if (gBlinkActive) {
    drawLeftSide();
  }
}

/**
 * @brief Draws the scene: Right Animation.
 *
 * @param step current step of animation
 * @return true if the scene animation is complete
 * @return false while the scene animation is not complete
 */
void sceneAnimRight() {
  if (gLightsAdditional) {
    sceneOn();
  } else {
    sceneOff();
  }
  // darken the active side
  for (int i = 0; i < NUM_LEDS_RIGHT; i++) {
    int idx = i + NUM_LEDS_LEFT;
    gLedsFront[idx].fadeLightBy(255 * gBlinkDarkenFactor);
    gLedsBack[idx].fadeLightBy(255 * gBlinkDarkenFactor);
  }
  if (gBlinkActive) {
    drawRightSide();
  }
}

/**
 * @brief Draws the left side of the blinking light.
 */
void drawLeftSide() {
  uint64_t timeDiff = now - gBlinkTimer;
  int durationStep = gBlinkDuration / NUM_LEDS_LEFT;
  int ledsOn = min(NUM_LEDS_LEFT, (timeDiff / (float)(durationStep * gBlinkGrowFactor)));

  for (int i = 0; i < ledsOn; i++) {
    gLedsFront[i] = COLOR_TURN;
    gLedsBack[i] = COLOR_TURN;
    gLedsFront[i].maximizeBrightness(gBlinkBrightness);
    gLedsBack[i].maximizeBrightness(gBlinkBrightness);
  }
}

/**
 * @brief Draws the right side of the blinking light.
 */
void drawRightSide() {
  uint64_t timeDiff = now - gBlinkTimer;
  int durationStep = gBlinkDuration / NUM_LEDS_RIGHT;
  int ledsOn = min(NUM_LEDS_RIGHT, (timeDiff / (float)(durationStep * gBlinkGrowFactor)));

  for (int i = 0; i < ledsOn; i++) {
    gLedsFront[i + NUM_LEDS_LEFT] = COLOR_TURN;
    gLedsBack[i + NUM_LEDS_LEFT] = COLOR_TURN;
    gLedsFront[i + NUM_LEDS_LEFT].maximizeBrightness(gBlinkBrightness);
    gLedsBack[i + NUM_LEDS_LEFT].maximizeBrightness(gBlinkBrightness);
  }
}

/**
 * @brief Sets the light switch to the given state.
 */
void setLightSwitch(bool state) {
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
bool isLightBlinking() {
  return gSosActive || gLeftActive || gRightActive;
}

/**
 * @brief Reads all sensors and updates the global variables.
 */
void readSensors() {
  updateSensorBrightness();
}

/**
 * @brief Updates the average of brightness readings.
 */
void updateSensorBrightness() {
  // check if a update is needed
  if (now < gBrightnessTimer + gBrightnessInterval) {
    return;
  }
  // shift readings
  gSensorBrightness = analogRead(PIN_BR);
  for (int i = NUM_READ - 1; i > 0; i--) {
    gBrightnessReadings[i] = gBrightnessReadings[i - 1];
  }
  // add new reading
  gBrightnessReadings[0] = gSensorBrightness;

  // calculate average
  int sum = 0;
  int total = 0;
  for (int i = 0; i < NUM_READ; i++) {
    if (gBrightnessReadings[i] > 0) {
      sum += gBrightnessReadings[i];
      total++;
    }
  }
  if (total > 0) {
    gBrightnessAvg = sum / total;
  } else {
    gBrightnessAvg = 0;
  }
  gBrightnessTimer = now;
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
bool checkButtonState(int pin, uint64_t *time, int *state, int *flags) {
  bool result = false;
  int stateNew = digitalRead(pin);

  Serial.print("pin: ");
  Serial.print(pin);
  Serial.print(" ");
  Serial.print("state: ");
  Serial.print(*state);
  Serial.print(" ");
  Serial.print("stateNew: ");
  Serial.println(stateNew);

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
      if (now - *time > gHoldTime) {
        // button hold
        *flags |= FLAG_HOLD;
      }
    }
  }

  if (*flags != FLAG_NONE) {
    // if any flags are set, a sequence is active so check for end of sequence
    if (*state == HIGH && now - *time > gClickGap) {
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
 * @brief Checks the button flags and returns the light function.
 *
 * @return The light function.
 */
int processLightFlags() {
  if (gFlagsLeft == FLAG_INVALID || gFlagsRight == FLAG_INVALID) {
    return FUNC_INVALID;
  }
  // single click on both buttons - main on/off
  if (gFlagsLeft & FLAG_CLICK && gFlagsRight & FLAG_CLICK) {
    return FUNC_MAIN;
  }
  // double click on both buttons - additional on/off
  if (gFlagsLeft & FLAG_DOUBLE_CLICK && gFlagsRight & FLAG_DOUBLE_CLICK) {
    return FUNC_ADDITIONAL;
  }
  // tribble click on both buttons - sos
  if (gFlagsLeft & FLAG_TRIBBLE_CLICK && gFlagsRight & FLAG_TRIBBLE_CLICK) {
    return FUNC_SOS;
  }
  // double click on right button - turn right forced
  if (gFlagsRight & FLAG_DOUBLE_CLICK && gFlagsLeft == FLAG_NONE) {
    return FUNC_TURN_RIGHT_FORCE;
  }
  // tribble click or hold on right button - lighter
  if (gFlagsRight & FLAG_HOLD || gFlagsRight & FLAG_TRIBBLE_CLICK) {
    if (gLightsOn) {
      return FUNC_LIGHTER;
    }
    return FUNC_NONE;
  }
  // double click on left button - turn left forced
  if (gFlagsLeft & FLAG_DOUBLE_CLICK && gFlagsRight == FLAG_NONE) {
    return FUNC_TURN_LEFT_FORCE;
  }
  // tribble click hold on left button - darker
  if (gFlagsLeft & FLAG_HOLD || gFlagsLeft & FLAG_TRIBBLE_CLICK) {
    if (gLightsOn) {
      return FUNC_DARKER;
    }
    return FUNC_NONE;
  }
  // single click on left button - turn left
  if (gFlagsLeft & FLAG_CLICK && gFlagsRight == FLAG_NONE) {
    return FUNC_TURN_LEFT;
  }
  // single click on right button - turn right
  if (gFlagsRight & FLAG_CLICK && gFlagsLeft == FLAG_NONE) {
    return FUNC_TURN_RIGHT;
  }
  return FUNC_NONE;
}

/**
 * @brief Updates the light switch that controls the lights.
 */
void updateLightSwitch() {
  if (gLightsOn) {
    if (gLightsAdditional && gDrawLights) {
      setLightSwitch(true);
    } else if (!gLightsAdditional) {
      setLightSwitch(true);
    }
  } else {
    if (gLightsAdditional) {
      setLightSwitch(false);
    } else if (!gLightsAdditional && !gDrawLights) {
      setLightSwitch(false);
    }
  }
}

/**
 * @brief Updates the blink timer.
 *
 * The globally available state of the blink light is updated here.
 */
void updateBlinkTimer() {
  if (gBlinkTimer + gBlinkDuration + gBlinkInterval < now) {
    gBlinkTimer = now;
  }
  uint64_t until = gBlinkTimer + gBlinkDuration;

  if (isLightBlinking() && until > now) {
    gBlinkActive = true;
  } else {
    gBlinkActive = false;
  }
}

/**
 * @brief Updates the status led.
 */
void updateStatusLed() {
  int brightness = gLightsAdditional ? 50 : 100;

  if (gBlinkActive) {
    brightness = 150;
  }
  if (brightness != gLedBrightness) {
    analogWrite(PIN_LED, brightness);
    gLedBrightness = brightness;
  }
}

/**
 * @brief Notifies the user using the status led.
 *
 * @param count The number of flashes.
 * @param duration The duration of each flash.
 * @param pause The pause between each flash.
 */
void notifyStatusLed(int count, int duration=250, int pause=-1) {
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
  gLedBrightness = 0;
}

/**
 * @brief Notifies the user about the selected light function.
 *
 * Only some functions are being used for notifications.
 *
 * @param func The light function.
 */
void notifyLightFunction(int func) {
  switch (func) {
    case FUNC_MAIN:
      // 1 x led default
      notifyStatusLed(1);
      break;
    case FUNC_ADDITIONAL:
      // 2 x led default
      notifyStatusLed(2);
      break;
    case FUNC_SOS:
      // 3 x led default
      notifyStatusLed(3);
      break;
    case FUNC_LIGHTER:
    case FUNC_DARKER:
      // 1 x led short
      notifyStatusLed(1, 50);
      break;
    case FUNC_INVALID:
      // 1 led - long with long delay
      notifyStatusLed(1, 100, 200);
      break;
  }
}

/**
 * @brief Sets the light function.
 *
 * @param func The light function.
 */
void processLightFunction(int func) {
  if (func == FUNC_INVALID) return;

  bool isBlinking = isLightBlinking();

  switch (func) {
    case FUNC_MAIN:
      if (gSosActive) {
        gSosActive = false;
      } else {
        if (gLightsOn) {
          gLightsOn = false;
        } else {
          gLightsOn = true;
        }
      }
      setDrawMode(DRAW_DEFAULT);
      break;
    case FUNC_ADDITIONAL:
      if (gSosActive) {
        gSosActive = false;
      } else {
        if (gLightsAdditional) {
          gLightsAdditional = false;
        } else {
          gLightsAdditional = true;
        }
      }
      setDrawMode(DRAW_DEFAULT);
      break;
    case FUNC_SOS:
      gSosActive = bool(!gSosActive);
      gLeftActive = false;
      gRightActive = false;
      if (gSosActive) {
        setDrawMode(DRAW_SOS);
      } else {
        setDrawMode(DRAW_DEFAULT);
      }
      break;
    case FUNC_TURN_LEFT:
      gLeftActive = bool(!isBlinking);
      gRightActive = false;
      if (gLeftActive) {
        setDrawMode(DRAW_LEFT);
      } else {
        setDrawMode(DRAW_DEFAULT);
      }
      break;
    case FUNC_TURN_RIGHT:
      gRightActive = int(!isBlinking);
      gLeftActive = false;
      if (gRightActive) {
        setDrawMode(DRAW_RIGHT);
      } else {
        setDrawMode(DRAW_DEFAULT);
      }
      break;
    case FUNC_TURN_LEFT_FORCE:
      gLeftActive = true;
      gRightActive = false;
      if (!gSosActive) {
        setDrawMode(DRAW_LEFT);
      }
      break;
    case FUNC_TURN_RIGHT_FORCE:
      gRightActive = true;
      gLeftActive = false;
      if (!gSosActive) {
        setDrawMode(DRAW_RIGHT);
      }
      break;
    case FUNC_LIGHTER:
      gBrightness += gBrightnessChange;
      gBrightness = min(gBrightness, 255);
      break;
    case FUNC_DARKER:
      gBrightness -= gBrightnessChange;
      gBrightness = max(gBrightness, 0);
      break;
  }
}

/**
 * @brief Returns the front brightness.
 *
 * @return int
 */
int getFrontBrightness() {
  return constrain(gBrightness * gFrontBrightnessFactor, 0, 255);
}

/**
 * @brief Returns the back brightness.
 *
 * @return int
 */
int getBackBrightness() {
  return constrain(gBrightness * gBackBrightnessFactor, 0, 255);
}

/**
 * @brief Button left press interrupt handler.
 */
void _btnLeftPress() {
  processLightFunction(FUNC_TURN_LEFT);
}

/**
 * @brief Button right release interrupt handler.
 */
void _btnRightPress() {
  processLightFunction(FUNC_TURN_RIGHT);
}
