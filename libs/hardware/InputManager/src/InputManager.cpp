#include "InputManager.h"

// Recorded ADC values from real devices
// BACK CONF LEFT RGHT   UP DOWN
// 3597 2760 1530    6 2300    6
// 3470 2666 1480    6 2222    5
// 3470 2655 1470    3 2205    3

// Averages
// BACK CONF LEFT RGHT   UP DOWN
// 3512 2694 1493    5 2242    5

// Setup ranges, if ADC value is between value `i` and `i + 1`, button `i` is being pressed
// These ranges are based on real world values above, and are much more tolerant of different
// devices than a fixed threshold check
// These values are calculated by taking the midpoint of the pairs of averaged values above
const int InputManager::ADC_RANGES_1[] = {ADC_NO_BUTTON, 3100, 2090, 750, INT32_MIN};
const int InputManager::ADC_RANGES_2[] = {ADC_NO_BUTTON, 1120, INT32_MIN};
const char* InputManager::BUTTON_NAMES[] = {"Back", "Confirm", "Left", "Right", "Up", "Down", "Power"};

InputManager::InputManager()
    : currentState(0),
      lastState(0),
      pressedEvents(0),
      releasedEvents(0),
      lastDebounceTime(0),
      buttonPressStart(0),
      buttonPressFinish(0) {}

void InputManager::begin() {
  pinMode(BUTTON_ADC_PIN_1, INPUT);
  pinMode(BUTTON_ADC_PIN_2, INPUT);
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);
}

int InputManager::getButtonFromADC(const int adcValue, const int ranges[], const int numButtons) {
  for (int i = 0; i < numButtons; i++) {
    if (ranges[i + 1] < adcValue && adcValue <= ranges[i]) {
      return i;
    }
  }

  return -1;
}

uint8_t InputManager::getState() {
  uint8_t state = 0;

  // Read GPIO1 buttons
  const int adcValue1 = analogRead(BUTTON_ADC_PIN_1);
  const int button1 = getButtonFromADC(adcValue1, ADC_RANGES_1, NUM_BUTTONS_1);
  if (button1 >= 0) {
    state |= (1 << button1);
  }

  // Read GPIO2 buttons
  const int adcValue2 = analogRead(BUTTON_ADC_PIN_2);
  const int button2 = getButtonFromADC(adcValue2, ADC_RANGES_2, NUM_BUTTONS_2);
  if (button2 >= 0) {
    state |= (1 << (button2 + 4));
  }

  // Read power button (digital, active LOW)
  const int gpio3Value = digitalRead(POWER_BUTTON_PIN);
  if (gpio3Value == LOW) {
    state |= (1 << BTN_POWER);
  }

#if BUTTON_DEBUG_TRACE
  static unsigned long lastTrace = 0;
  const unsigned long traceNow = millis();
  if (traceNow - lastTrace >= 20) {  // 50 Hz max
    Serial.printf("[BTN] adc1=%4d adc2=%4d gpio3=%d state=0x%02X commit=0x%02X dt=%lums\n",
                  adcValue1, adcValue2, gpio3Value, state, currentState, traceNow - lastTrace);
    lastTrace = traceNow;
  }
#endif

  return state;
}

void InputManager::update() {
  const unsigned long currentTime = millis();

#if BUTTON_DEBUG_CADENCE
  static uint16_t buckets[8] = {0};  // <10ms, 10-20, 20-50, 50-100, 100-200, 200-500, 500-1000, >1000
  static unsigned long lastUpdate = 0;
  static unsigned long lastReport = 0;
  const unsigned long cadenceDt = currentTime - lastUpdate;
  lastUpdate = currentTime;
  int b = (cadenceDt < 10)   ? 0
          : (cadenceDt < 20)  ? 1
          : (cadenceDt < 50)  ? 2
          : (cadenceDt < 100) ? 3
          : (cadenceDt < 200) ? 4
          : (cadenceDt < 500) ? 5
          : (cadenceDt < 1000) ? 6
                               : 7;
  buckets[b]++;
  if (currentTime - lastReport > 5000) {
    Serial.printf("[BTN-CADENCE] <10:%u 10-20:%u 20-50:%u 50-100:%u 100-200:%u 200-500:%u 500-1k:%u >1k:%u\n",
                  buckets[0], buckets[1], buckets[2], buckets[3],
                  buckets[4], buckets[5], buckets[6], buckets[7]);
    lastReport = currentTime;
  }
#endif

  const uint8_t state = getState();

  // Always clear events first
  pressedEvents = 0;
  releasedEvents = 0;

  // Debounce
  if (state != lastState) {
    lastDebounceTime = currentTime;
    lastState = state;
  }

  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (state != currentState) {
      // Calculate pressed and released events
      pressedEvents = state & ~currentState;
      releasedEvents = currentState & ~state;

      // If pressing buttons and wasn't before, start recording time
      if (pressedEvents > 0 && currentState == 0) {
        buttonPressStart = currentTime;
      }

      // If releasing a button and no other buttons being pressed, record finish time
      if (releasedEvents > 0 && state == 0) {
        buttonPressFinish = currentTime;
      }

      currentState = state;
    }
  }

  // Suppress events until all buttons are released (used during activity transitions)
  if (suppressUntilRelease) {
    pressedEvents = 0;
    releasedEvents = 0;
    if (currentState == 0) {
      suppressUntilRelease = false;
    }
  }
}

void InputManager::suppressUntilAllReleased() {
  suppressUntilRelease = true;
  pressedEvents = 0;
  releasedEvents = 0;
  buttonPressStart = millis();  // Reset held time so getHeldTime() starts fresh
}

bool InputManager::isPressed(const uint8_t buttonIndex) const {
  if (suppressUntilRelease) return false;
  return currentState & (1 << buttonIndex);
}

bool InputManager::wasPressed(const uint8_t buttonIndex) const {
  return pressedEvents & (1 << buttonIndex);
}

bool InputManager::wasAnyPressed() const {
  return pressedEvents > 0;
}

bool InputManager::wasReleased(const uint8_t buttonIndex) const {
  return releasedEvents & (1 << buttonIndex);
}

bool InputManager::wasAnyReleased() const {
  return releasedEvents > 0;
}

unsigned long InputManager::getHeldTime() const {
  // Still hold a button
  if (currentState > 0) {
    return millis() - buttonPressStart;
  }

  return buttonPressFinish - buttonPressStart;
}

const char* InputManager::getButtonName(const uint8_t buttonIndex) {
  if (buttonIndex <= BTN_POWER) {
    return BUTTON_NAMES[buttonIndex];
  }
  return "Unknown";
}

bool InputManager::isPowerButtonPressed() const {
  return isPressed(BTN_POWER);
}
