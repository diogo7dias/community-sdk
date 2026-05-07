#pragma once

#include <Arduino.h>

class InputManager {
 public:
  InputManager();
  void begin();
  uint8_t getState();

  /**
   * Updates the button states. Should be called regularly in the main loop.
   */
  void update();

  /**
   * Returns true if the button was being held at the time of the last #update() call.
   *
   * @param buttonIndex the button indexes
   * @return the button current press state
   */
  bool isPressed(uint8_t buttonIndex) const;

 /**
   * Returns true if the button went from unpressed to pressed between the last two #update() calls.
   *
   * This differs from #isPressed() in that pressing and holding a button will cause this function
   * to return true after the first #update() call, but false on subsequent calls, whereas #isPressed()
   * will continue to return true.
   *
   * @param buttonIndex
   * @return the button pressed state
   */
  bool wasPressed(uint8_t buttonIndex) const;

  /**
   * Returns true if any button started being pressed between the last two #update() calls
   *
   * @return true if any button started being pressed between the last two #update() calls
   */
  bool wasAnyPressed() const;

  /**
   * Returns true if the button went from pressed to unpressed between the last two #update() calls
   *
   * @param buttonIndex the button indexes
   * @return the button release state
   */
  bool wasReleased(uint8_t buttonIndex) const;

  /**
   * Returns true if any button was released between the last two #update() calls
   *
   * @return  true if any button was released between the last two #update() calls
   */
  bool wasAnyReleased() const;

  /**
   * Returns the time between any button starting to be depressed and all buttons between released
   *
   * @return duration in milliseconds
   */
  unsigned long getHeldTime() const;

  /**
   * Suppresses all press/release events until every button is released.
   * Also resets the held-time origin so getHeldTime() starts fresh.
   * Use this during activity transitions to prevent stale input from
   * leaking into the next screen.
   */
  void suppressUntilAllReleased();

  // Button indices
  static constexpr uint8_t BTN_BACK = 0;
  static constexpr uint8_t BTN_CONFIRM = 1;
  static constexpr uint8_t BTN_LEFT = 2;
  static constexpr uint8_t BTN_RIGHT = 3;
  static constexpr uint8_t BTN_UP = 4;
  static constexpr uint8_t BTN_DOWN = 5;
  static constexpr uint8_t BTN_POWER = 6;

  // Pins
  static constexpr int BUTTON_ADC_PIN_1 = 1;
  static constexpr int BUTTON_ADC_PIN_2 = 2;
  static constexpr int POWER_BUTTON_PIN = 3;

  // Power button methods
  bool isPowerButtonPressed() const;

  // Button names
  static const char* getButtonName(uint8_t buttonIndex);

 private:
  int getButtonFromADC(int adcValue, const int ranges[], int numButtons);
  static int readAdcAveraged(uint8_t pin);

  uint8_t currentState;
  uint8_t lastState;
  uint8_t pressedEvents;
  uint8_t releasedEvents;
  unsigned long lastDebounceTime;
  unsigned long buttonPressStart;
  unsigned long buttonPressFinish;
  bool suppressUntilRelease = false;

  static constexpr int NUM_BUTTONS_1 = 4;
  static const int ADC_RANGES_1[];

  static constexpr int NUM_BUTTONS_2 = 2;
  static const int ADC_RANGES_2[];

  static constexpr int ADC_NO_BUTTON = 3800;
  // Restored to upstream 5 ms after issue #138 (sluggish navigation).
  // The 15 ms bump from #136 was redundant once 8-sample ADC oversampling
  // (below) eliminated the resistor-ladder bucket flicker at press edges,
  // and the extra latency was perceptible during menu/page navigation.
  // Mechanical bounce on these tactile buttons is well under 5 ms.
  static constexpr unsigned long DEBOUNCE_DELAY = 5;
  // Dropped 8 → 1 after issue #138: averaging was eating legitimate press
  // edges in the reader (page-turn taps needed extra force to register).
  // Match upstream / inx single-sample behaviour. If resistor-ladder bucket
  // flicker (#136) returns on real hardware, raise back to 4 first, not 8.
  static constexpr int ADC_OVERSAMPLE = 1;

  static const char* BUTTON_NAMES[];
};
