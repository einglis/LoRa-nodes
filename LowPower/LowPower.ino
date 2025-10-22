
#include <STM32LoRaWAN.h>
#include <STM32CubeWL/LoRaWAN/Mac/Region/RegionEU868.h>
#include <STM32CubeWL/SubGHz_Phy/radio.h>
#include <STM32LowPower.h>

// Download STM32CubeProgrammer

// In Arduino Settings, add extra board URL:
// https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
// Install "STM32 MCU based boards"
// Select "LoRa Boards",  part number "LoRa-E5 mini"

// Install libraries (to Arduino/libraries/):
// https://github.com/stm32duino/STM32LoRaWAN
// https://github.com/stm32duino/STM32LowPower
// https://github.com/stm32duino/STM32RTC

#define DEVBOARD
#ifdef DEVBOARD
  #define INPUT_PIN         D9
  #define LED_PIN           19
#else
  #define INPUT_PIN         A4
  #define LED_PIN           D9
#endif

volatile int events_count = 0;
volatile int sleeps_count = 0;
volatile int alarms_count = 0;

void event_fn(void) { events_count++; }
void alarm_fn(void *) { alarms_count++; }

uint8_t my_id[2] = { 0 };
uint8_t seq = 0;

void send(uint8_t value)
{
  Serial.print("send ");
  Serial.println(value);

  digitalWrite(LED_PIN, LOW); // active low
  delay(20);
  digitalWrite(LED_PIN, HIGH);

  const int vref = analogRead(AVREF);
  const int millivolts = 1212 * 1023 / vref; // 1.212 is nominal reference voltage
  const int charge = max(0, min(255, (millivolts - 1800) * 255 / (3600 - 1800)));

  uint8_t buf[] = { 0x38, 0x05, my_id[0], my_id[1], seq, value, (uint8_t)charge /*battery*/ };
  Radio.Send( &buf[0], sizeof(buf) );
  seq++;
}

void OnRadioTxDone( void )
{
  Serial.println("TxDone");
  sleeps_count++;
}
void OnRadioTxTimeout( void )
{
  Serial.println("TxTimeout");
  sleeps_count++; // oh well!
}

RadioEvents_t RadioEvents = { 0 };
STM32RTC& rtc = STM32RTC::getInstance();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);

  Serial.println("");
  Serial.println("");
  Serial.println("I am booting");

  const uint32_t unique_id = LL_FLASH_GetUDN();
  my_id[0] =  unique_id       & 0xff;
  my_id[1] = (unique_id >> 8) & 0xff;

  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.setBinaryMode(STM32RTC::MODE_MIX);
  rtc.begin(true, STM32RTC::HOUR_24);
  UTIL_TIMER_Init(rtc.getHandle());

  RadioEvents.TxDone = OnRadioTxDone;
  RadioEvents.TxTimeout = OnRadioTxTimeout;
  Radio.Init( &RadioEvents );

  Radio.SetPublicNetwork( false /*private network*/ );
  Radio.SetChannel( 868000000 );

  Radio.SetTxConfig(
    MODEM_LORA,
    EU868_DEFAULT_TX_POWER,
    0, // freq dev
    0, // bandwidth = 125kHz
    7, // spreading factor
    RADIO_LORA_CR_4_5,
    8, // preamble
    false, // == variable length packets
    true, // == crc on
    false, // == no frequency hopping
    0, // n/a (hop period)
    false, // IQ not inverted
    1000 // tx timeout (ms)
  );

  Radio.Sleep();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // active low

  pinMode(INPUT_PIN, INPUT_PULLUP);
  LowPower.begin();
  LowPower.attachInterruptWakeup(INPUT_PIN, event_fn, CHANGE, DEEP_SLEEP_MODE);
  rtc.attachInterrupt(alarm_fn, STM32RTC::ALARM_A);
}


int alarms_seen = 0;
int events_seen = -1; // force an event on startup
int sleeps_seen = 0;

static const int intervals_sec[] = { 3, 10, 60, 15*60 };
static const int num_intervals = sizeof(intervals_sec)/sizeof(intervals_sec[0]);

static bool debounce = false;
static int last_sent = -1;

void send_and_reschedule(bool event)
{
  int input = digitalRead(INPUT_PIN);

  int nextInterval = intervals_sec[0];

  if (event) {
    if (debounce) {
        Serial.println("currently in debounce; ignore");
        return;
    } else {
        Serial.println("send now, restart sequence");
        nextInterval = intervals_sec[0];
        debounce = true;
    }
  } else { // alarm
    if (debounce && (input != last_sent)) {
        Serial.println("completed debounce but change of state; restart sequence");
        nextInterval == intervals_sec[0];
        debounce = true;
    } else {
        Serial.println("regular alarm; cancel debounce; advance sequence");
        debounce = false;

        for (int i = 0; i < num_intervals - 1; i++) {
          if (nextInterval <= intervals_sec[i]) {
            nextInterval = intervals_sec[i+1];
            break;
          }
        }
    }
  }

  send(input);
  last_sent = input;

  Serial.print("Resend in ");
  Serial.println(nextInterval);
  Serial.print(" Now ");
  Serial.println(rtc.getEpoch());
  Serial.print("Wake ");
  Serial.println(rtc.getEpoch() + nextInterval);

  rtc.disableAlarm(STM32RTC::ALARM_A);
  rtc.setAlarmEpoch(rtc.getEpoch() + nextInterval, STM32RTC::MATCH_DHHMMSS, 0, STM32RTC::ALARM_A);
}

void loop()
{
  LoRaMacProcess();

  if (events_count != events_seen) {
    events_seen = events_count;

    send_and_reschedule(true);
  }

  if (alarms_count != alarms_seen) {
    alarms_seen = alarms_count;

    send_and_reschedule(false);
  }

  if (sleeps_count != sleeps_seen) {
    sleeps_seen = sleeps_count;

    Serial.println("zzzzz");
    delay(100);

    Radio.Sleep();
    LowPower.deepSleep();

    Serial.begin(115200);
    while (!Serial);
    Serial.println("awake");
  }
}
