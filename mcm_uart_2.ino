// === Mock ME: single-thruster STC sender ============================
// Protocol (nibble):
//   stc[0]   = direction (0/1)    <-- LSB
//   stc[1:3] = magnitude (0..7)   (use 0..5; receiver can do speed = 2*mag)
//
// Quad thruster (not used here, helper provided):
//   uint16_t word = [stc:TL][stc:TR][stc:BL][stc:BR]
//   bit ranges:     [0:3]   [4:7]   [8:11]  [12:15]
//
// UART: 115200 8E1

// ---------------- Pins ----------------
const uint8_t PIN_POT      = A0;   // throttle
const uint8_t PIN_DIR_SW   = 8;   // direction switch (to GND, INPUT_PULLUP)
const uint8_t PIN_GO_BTN   = 7;    // Go/NoGo button (to GND, INPUT_PULLUP)

// ---------------- UART ----------------
const unsigned long BAUD   = 115200;

// ---------------- Rate / debounce -----
const uint16_t SEND_HZ     = 20;           // send up to 20 msgs/sec
const uint32_t SEND_PERIOD = 1000UL / SEND_HZ;
uint32_t lastSendMs = 0;

// Track last sent nibble to avoid spam (send on change or at low rate)
uint8_t last_stc = 0xFF;

// ---------------- Helpers -------------
static inline uint8_t build_stc(uint8_t dir, uint8_t mag)
{
  // dir in {0,1}; mag in {0..7}; we enforce 0..5 below
  mag = (mag > 5) ? 5 : mag;        // use only 0..5 as per spec
  dir = dir ? 1 : 0;
  // Bits: [1:3]=mag, [0]=dir
  return (uint8_t)((mag << 1) | dir) & 0x0F;
}

static inline uint16_t pack_quad(uint8_t stc_TL, uint8_t stc_TR,
                                 uint8_t stc_BL, uint8_t stc_BR)
{
  // Each stc is 4 bits; place in [0:3][4:7][8:11][12:15]
  uint16_t w = 0;
  w |= ((uint16_t)(stc_TL & 0x0F)) << 0;
  w |= ((uint16_t)(stc_TR & 0x0F)) << 4;
  w |= ((uint16_t)(stc_BL & 0x0F)) << 8;
  w |= ((uint16_t)(stc_BR & 0x0F)) << 12;
  return w;
}

void setup()
{
  Serial.begin(BAUD, SERIAL_8E1);

  pinMode(PIN_DIR_SW, INPUT_PULLUP);  // switch to GND
  pinMode(PIN_GO_BTN, INPUT_PULLUP);  // button to GND
  pinMode(PIN_POT,    INPUT);
}

void loop()
{
  const uint32_t now = millis();

  // ---- Read inputs ----
  // Button: pressed = LOW (to GND with pull-up)
  const bool goPressed = (digitalRead(PIN_GO_BTN) == LOW);

  // Direction: choose your convention. Here: switch CLOSED (=LOW) => dir=1,
  // OPEN (=HIGH) => dir=0.
  const uint8_t dir = (digitalRead(PIN_DIR_SW) == LOW) ? 1 : 0;

  // Throttle → magnitude 0..5 (only used if Go is pressed)
  // 6 buckets (0..5) using integer math
  uint16_t raw = analogRead(PIN_POT);      // 0..1023
  uint8_t  mag = (uint8_t)((raw * 6UL) / 1024);  // 0..5
  if (mag > 5) mag = 5;
  if (!goPressed) mag = 0;               // NoGo forces magnitude 0

  // ---- Build stc nibble ----
  const uint8_t stc = build_stc(dir, mag);

  // ---- Send when changed or at SEND_HZ ----
  if (stc != last_stc || (now - lastSendMs) >= SEND_PERIOD) {
    lastSendMs = now;
    last_stc   = stc;

    // Single-thruster mode: send 1 byte with STC in low nibble (upper nibble zeroed)
    const uint8_t byteToSend = stc & 0x0F;
    // Serial.write(byteToSend);
    // Serial.write((uint8_t)0x00);
    

    // (Optional) If you later want quad, e.g. mirroring same STC on all:
    uint16_t quad = pack_quad(stc, stc, stc, stc);
    // Send little-endian: low byte, then high byte
    Serial.write((uint8_t)(quad & 0xFF));
    Serial.write((uint8_t)(quad >> 8));
  }

  // Small idle to reduce CPU churn
  delay(1);
}
