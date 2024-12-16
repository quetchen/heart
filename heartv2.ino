// This code was developed for the ATtiny85 using ATTinyCore. It uses some
// tiny85-specific registers, so will need some tweaks to get it to run
// on anything else.

#include <FastLED.h>
#include <avr/sleep.h>
#include <EEPROM.h>

#define ELEPHANT


#define LED_PIN PIN_PB3
#define TOUCH_PIN1 PIN_PB4
#define TOUCH_PIN2 PIN_PB1

#define LED_POWER_ENABLE PIN_PB0

#ifdef ELEPHANT
#define BUTTON_PRESSED LOW
#define LED_COUNT 11
#define LEDNUM(x) leds[(x)]
#define BRIGHT_INIT 16
#else
#define LED_COUNT 10
// Because of the physical location of the LED data output, LED 0
// is not at the bottom or top of the heart, so need to rotate the
// LEDs clockwise by 8
#define LEDNUM(x) leds[(x+8) % 10]
#define BUTTON_PRESSED HIGH
#define BRIGHT_INIT 64
#endif

#define FLAG_EEPROM_CORRUPTION 0  // flash red lights if pattern is invalid
#define ENABLE_CALIBRATION 0      // enable supply voltage calibration
#define BUTTON_HOLD_TIME 3000

// SCALE_STEP - increment to the color scale argument
// power of 2 for best results
#define SCALE_STEP 4
#define LOOP_DELAY 16 // milliseconds
#define VOLT_CHECK_INTERVAL 1024 // milliseconds
#define TWINKLE_DELAY 64 // milliseconds
#define TWINKLE_PROB 128
#define TWINKLE_N_SCALE_STEPS 7

CRGB leds[LED_COUNT];

// supposedly, page 0 is most likely to be corrupted in case
// of weird behavior with low supply voltage (although BOD
// should prevent this). Pages for ATTiny85 are 4 bytes so
// start at address 4.
#define EEPROM_COLOR_MODE 4
#define EEPROM_PATTERN    5
#define EEPROM_LOCKED     6
#define EEPROM_UVLO       7

#define ADC_DELAY 1 // delay in ms between writing ADMUX and starting conversion
                    // 1 ms according to the datasheet
#define UVLO_ENGAGE   93
#define UVLO_RELEASE 87

uint8_t color_mode = 0;
uint8_t pattern = 0;
bool lock_buttons = false;

#define BUTTON_DEBOUNCE_MS 100
const PROGMEM uint8_t button_pin[] = {TOUCH_PIN1, TOUCH_PIN2};
volatile uint8_t button_count[] = {0, 0};
volatile bool button_pressed[] = {false, false};
volatile bool button_read[] = {false, false};
volatile uint16_t button_held[] = {0, 0};

bool uvlo = false;

int8_t mode = 0;

ISR(TIMER0_COMPA_vect){
  // note - this function gets called every 2.048 ms (the details of which are
  // buried in wiring.c within ATTinyCore, but the timer 0 prescaler gets
  // set to 64 there).

  // ignore touch within 500 ms of power-on. This is because the power switch
  // is very close to the touch sensors and it's easy to bump them while
  // hitting the switch.
  uint8_t buttons_on = 0;

  if (millis() < 500) return;
  
  for (uint8_t i = 0; i < sizeof(button_count); i++){
    if (digitalRead(pgm_read_byte(&button_pin[i])) == BUTTON_PRESSED){
      if (!button_read[i]) button_count[i]++;
      button_held[i] += 2;
      buttons_on++;
    } else {
      button_count[i] = 0;
      button_read[i] = false;
      button_held[i] = 0;
    }
    if (button_count[i] > BUTTON_DEBOUNCE_MS/2) button_pressed[i] = true;
  }
  if (buttons_on > 1){
    for (uint8_t i = 0; i < sizeof(button_count); i++){
      button_count[i] = 0;
      button_read[i] = false;
      button_pressed[i] = false;
    }
  }
}

bool get_button(uint8_t button){
  // return true if the selected button passed as argument has been pressed
  // since last called
  bool r;
  noInterrupts();
  r = button_pressed[button];
  if (r){
    button_pressed[button] = false;
    button_read[button] = true;
    button_count[button] = 0;
  }
  interrupts();
  return r;
}

void setup() {

  // turn off ADC until we need it
  ADCSRA &= ~(1 << ADEN);

  // turn off analog comparator; we don't need it
  ACSR   |= (1 << ACD);
#ifdef ELEPHANT
  pinMode(TOUCH_PIN1, INPUT_PULLUP);
  pinMode(TOUCH_PIN2, INPUT_PULLUP);
  pinMode(SCK, INPUT);
  // for the elephant, PB0 is an input that gets grounded
  // when the charger is connected
  pinMode(PIN_PB0, INPUT_PULLUP);
#else
  pinMode(TOUCH_PIN1, INPUT);
  pinMode(TOUCH_PIN2, INPUT);
  pinMode(LED_POWER_ENABLE, OUTPUT);
  digitalWrite(LED_POWER_ENABLE, HIGH);
#endif

  color_mode =   EEPROM.read(EEPROM_COLOR_MODE);
  pattern =      EEPROM.read(EEPROM_PATTERN);
  lock_buttons = EEPROM.read(EEPROM_LOCKED);

  FastLED.setBrightness(BRIGHT_INIT);
  FastLED.addLeds<WS2812B, LED_PIN>(leds, LED_COUNT);
  FastLED.setCorrection(Typical8mmPixel);

  // turn on the timer 0 output compare A interrupt
  // for button debouncing. This avoids using timer 1,
  // but with the disadvantage that the period is
  // determined by obscure and arcane magic deep within
  // ATTinyCore
  OCR0A = 0x80;
  TIMSK |= (1 << OCIE0A);

  // enable WDT interrupt, prescaler = 1001 (8 sec)
  WDTCR = 0b11111001;

}

#define CMODE_RAINBOW   0
#define CMODE_WHITE     1
#define CMODE_REDGREEN  2
#define CMODE_CANDYCANE 3
#define CMODE_SNOWFLAKE 4

#define NUM_CMODES 5;

#define PATTERN_ROTATE   0
#define PATTERN_VERTICAL 1
#define PATTERN_TWINKLE  2
#define PATTERN_SOLID    3

#define NUM_PATTERNS 4

CHSV scheme_color(uint8_t step, uint8_t n, uint16_t seq){
  // return the value of the current color scheme at step step
  // out of n total steps, with seq incremented each cycle
  // each color scheme is periodic w.r.t seq, but the period
  // depends on the scheme.
  uint16_t seq_step = seq / SCALE_STEP / 16;
  uint16_t theta = (256/n) * step + seq;
  uint16_t br = 0;

  switch(color_mode){
    case CMODE_RAINBOW:
      return CHSV(theta, 255, 255);
    case CMODE_WHITE:
      return CHSV(0, 0, sin8(theta));
    case CMODE_REDGREEN:

      /* classic 1990s chasing effect:
       
      (seq_step % 10) 0 1 2 3 4 5 6 7 8 9
                    0 R - - - - G - - - - -
                    1 R G - - - G R - - -
                    2 - G - - - - R - - - 
                    3 - G R - - - R G - -
                    4 - - R - - - - G - -
                    5 - - R G - - - G R -
                    6 - - - G - - - - R -
                    7 - - - G R - - - R G
                    8 - - - - R - - - - G
                    9 R - - - R G - - - G
                    0 R - - - - G - - - -
      */  
      if (n==1) return CHSV((seq_step & 1) * 96, 255, 255);
      switch(seq_step % 2){
        case 1:
          return CHSV(((step % 2) ? 0 : 96), 255, 255*((step % 5) == ((seq_step/2) % 5)));
        case 0:
          return CHSV(((step % 2) ? 0 : 96), 255, 255*(((step % 5) == ((seq_step/2) % 5))||(((step + 1)%5) == ((seq_step/2) % 5))));
      }

    case CMODE_CANDYCANE:
      return CHSV(0, sin8(theta), 255);
    case CMODE_SNOWFLAKE:
      return(CHSV(145, sin8(theta), 255));
    default:
      return CHSV(0, 0, 0);
  }
}

void flash(CRGB color){
  FastLED.clear(true);
  delay(200);
  fill_solid(leds, LED_COUNT, color);
  FastLED.show();
  delay(200);
  FastLED.clear(true);
  delay(200);
}

ISR(WDT_vect){
  // have to reenable the WDT interrupt every time; otherwise
  // it will reset next time instead
  WDTCR |= (1 << WDIE);

  // don't have to do anything else in the ISR; the point of the
  // interrupt is just to wake up the CPU
}

void loop() {
  static uint16_t cycle = 0;
  static unsigned long button_t = 0;
#if ENABLE_CALIBRATION
  static bool cal_mode = false;
#endif


  if (cycle % (VOLT_CHECK_INTERVAL / LOOP_DELAY) == 0){
    // every 1024 ms, check supply voltage by reading the output
    // of the onboard 1.1V bandgap reference
    // since analogRead doesn't include the required settling time
    // we have to roll our own (although maybe this doesn't matter
    // except for the first conversion)

    // note: if in UVLO, cycle doesn't get incremented
    // so this will run every time until out of UVLO
    // turn on ADC
    //         1------- enable ADC
    //         -0------ don't start conversion
    //         --0----- auto trigger disable
    //         ---0---- clear interrupt flag
    //         ----0--- interrupt disable
    //         -----110 prescaler = 64 (8 MHz / 64 = 125 kHz; datasheet recommends 50-200 kHz)
    ADCSRA = 0b10000110;
  
    // set ADC mux

    //        00-x---- ADC voltage reference to Vcc
    //        --1----- left adjust result (don't need 2 lowest bits)
    //        ----1100 read 1.1V reference
    ADMUX = 0b00101100;

    delay(ADC_DELAY);                     // settling time 1ms per datasheet

    ADCSRA |= (1 << ADSC);        // start conversion
    while (ADCSRA & (1 << ADSC)); // wait for conversion to complete
    uint8_t volt_rdg = ADCH;

#if ENABLE_CALIBRATION
    if (button_held[0] > BUTTON_HOLD_TIME || button_held[1] > BUTTON_HOLD_TIME){ // 6 seconds
      cal_mode = true;
    }

    if (cal_mode){
      for (uint8_t i = 0; i < LED_COUNT; i++){
        if (volt_rdg & (1 << i))
          LEDNUM(i) = CRGB(255, 255, 255);
        else
          LEDNUM(i) = CRGB(0, 0, 0); 
      }
      FastLED.show();
      delay(10);
      return;
    }
#endif

    if (!uvlo && (volt_rdg > UVLO_ENGAGE || EEPROM.read(EEPROM_UVLO)
#ifdef ELEPHANT
       || digitalRead(PIN_PB0)==LOW

#endif     
      )){
      // voltage reference is 1.1 V +/- 0.1 V
      // so supply voltage is inversely proportional
      // to the voltage reference reading
      // hence volt_rdg > UVLO_ENGAGE means voltage is low
      // note formula is approximately Vbat = 255 * 1.1 / rdg

      uvlo = true;
      FastLED.clear(true);

      // if we don't do this the LEDs will still be able to sink current through the data input
      pinMode(LED_PIN, INPUT);

      // now disable LED power
#ifndef ELEPHANT
      digitalWrite(LED_POWER_ENABLE, LOW);
#endif

    } else if (uvlo && volt_rdg < UVLO_RELEASE
#ifdef ELEPHANT
      && digitalRead(PIN_PB0) == HIGH
#endif
    ){
      uvlo = false;
      pinMode(LED_PIN, OUTPUT);
#ifndef ELEPHANT
      digitalWrite(LED_POWER_ENABLE, HIGH);
#endif
    }

    EEPROM.update(EEPROM_UVLO, uvlo);

    if (uvlo){

      // SLEEP_MODE_PWR_DOWN is the lowest sleep mode (all clocks off except WDT)
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);

      // disable the ADC
      ADCSRA &= ~(1 << ADEN);

      sleep_enable();
      // disable BOD to save power
      // this only works on ATTiny85 revision C and newer
      // BOD is needed to prevent EEPROM corruption
      // this doesn't seem to work; I'm not sure why. But it doesn't make
      // a huge difference anyway.
      cli(); // disable interrupts during timed sequence
      sleep_bod_disable();
      sei();
      sleep_cpu();

      // the following is recommended by the datasheet
      // not super necessary, just makes it harder to accidentally sleep
      sleep_disable();
      return;
    }
  }

  switch(pattern % NUM_PATTERNS){
  case PATTERN_ROTATE:
    for(uint8_t i = 0; i < LED_COUNT; i++){
      LEDNUM(i) = scheme_color(i, LED_COUNT, cycle * SCALE_STEP); 
    }
    break;
  case PATTERN_VERTICAL:
    for(uint8_t i = 0; i < LED_COUNT/2; i++){
      LEDNUM(i) = scheme_color(i, LED_COUNT/2, cycle * SCALE_STEP);
      LEDNUM(LED_COUNT-i-1) = LEDNUM(i);
    }
    if (LED_COUNT & 1) LEDNUM(LED_COUNT/2) = LEDNUM(LED_COUNT/2-1);
    break;
  case PATTERN_TWINKLE:
    if ((cycle % (TWINKLE_DELAY / LOOP_DELAY)) == 0 && random8() < TWINKLE_PROB){
      LEDNUM(random8(LED_COUNT)) = scheme_color(random8(0,2), 1, random8(0,TWINKLE_N_SCALE_STEPS)*(256 / TWINKLE_N_SCALE_STEPS));
    }
    break;
  case PATTERN_SOLID:
    fill_solid(leds, LED_COUNT, scheme_color(0, 1, cycle * SCALE_STEP));
    break;
  default:
    //invalid pattern
#if FLAG_EEPROM_CORRUPTION
    fill_solid(leds, LED_COUNT, CRGB(255, 0, 0));
    FastLED.show();
    delay(200);
    fill_solid(leds, LED_COUNT, CRGB(0, 0, 0));
    FastLED.show();
    delay(200);
#else
    pattern = PATTERN_VERTICAL;
    return;
#endif
  }

  if (!lock_buttons){
      if (get_button(0)) {
        color_mode = (color_mode + 1) % NUM_CMODES;
        FastLED.clear(true);
        button_t = millis();
    }
    if (get_button(1)){
      pattern = (pattern + 1) % (NUM_PATTERNS * 2);
      FastLED.clear(true);
      button_t = millis();
    }
  } else {
    if (get_button(0) || get_button(1)) flash(CRGB(255,0,0));
  }

  if (button_held[0] > BUTTON_HOLD_TIME && button_held[1] > BUTTON_HOLD_TIME){
    lock_buttons = !lock_buttons;
    EEPROM.write(EEPROM_LOCKED, lock_buttons);
    button_held[0] = 0;
    button_held[1] = 0;
    if (lock_buttons)
      flash(CRGB(255,0,0));
    else
      flash(CRGB(0, 255, 0));
  }

  // wait 1 second after the last button press before saving the settings to EEPROM.
  // this is so any accidental button presses while turning the power off will be
  // ignored.
  if (button_t != 0 && millis() >= button_t + 1000){
    EEPROM.write(EEPROM_COLOR_MODE, color_mode);
    EEPROM.write(EEPROM_PATTERN, pattern);
    button_t = 0;
  }

  FastLED.show();
  delay((pattern < NUM_PATTERNS ? 1 : 3) * LOOP_DELAY - ((cycle % (VOLT_CHECK_INTERVAL / LOOP_DELAY) == 0) ? ADC_DELAY : 0));
  if (color_mode == CMODE_REDGREEN){
    // cycle should roll over when cycle / 16 is a multiple of 10 and cycle * SCALE_STEP is less than UINT16_MAX
    // I don't know if this actually works, because I'm not going to stare at it for 261120 ms to see if it glitches
    cycle = (cycle + 1)% ((LED_COUNT * 16) * (UINT16_MAX / SCALE_STEP / LED_COUNT / 16));
  } else {
    // cycle should roll over at a multiple of 256*SCALE_STEP
    cycle++;
  }
}
