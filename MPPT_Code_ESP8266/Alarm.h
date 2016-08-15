
#define CHIRP_VOLTAGE 12.25
#define LOW_VOLTAGE 12.1

#define CHIRP_EVERY_SECONDS 20

// This class handles the beeper on the specified pin
// If the reported voltage is less than 12.25, we chirp
// about every 10-15 seconds. If it goes less than 12.1,
// we leave the beeper on. Hopefully someone notices.

class Alarm {
  public:
  /**
   * Constructor
   *
   * @param pin       pin number for this alarm
   */
  Alarm(const uint8_t pin) {
    this->pin = pin;
    pinMode(pin, OUTPUT);
    outputting = false;
    chirping = false;
  }

  void SendVoltage(const uint32_t& now, const float& v) {
    if (v < LOW_VOLTAGE) {
      if (!outputting) {
        newState(true);
        last_chirp = now;
      }
      chirping = false;
      return;
    }
    if (v <= CHIRP_VOLTAGE) {
      if (now - last_chirp > CHIRP_EVERY_SECONDS) {
        last_chirp = now;
        newState(true);
        chirping = true;
        return;
      }
    }
    // voltage above CHIRP_VOLTAGE; no noise
    newState(false);
    chirping = false;
  }

  bool IsChirping() {
    return chirping;
  }

  private:
  void newState(bool state) {
    if (state != outputting) {
      outputting = state;
      digitalWrite(pin, state ? 1 : 0);
    }
  }
  
  uint8_t pin;
  bool outputting;
  bool chirping;
  uint32_t last_chirp;

      
};

