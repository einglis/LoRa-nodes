//
#include <STM32LoRaWAN.h>
#include <STM32CubeWL/LoRaWAN/Mac/Region/RegionEU868.h>
#include <STM32CubeWL/SubGHz_Phy/radio.h>

// Download STM32CubeProgrammer

// In Arduino Settings, add extra board URL:
// https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
// Install "STM32 MCU based boards"
// Select "LoRa Boards",  part number "LoRa-E5 mini"

// Install libraries (to Arduino/libraries/):
// https://github.com/stm32duino/STM32LoRaWAN
// https://github.com/stm32duino/STM32RTC

#define LED_PIN        19
#define INPUT_PIN      D9
#define OPEN_LED_PIN   A3
#define LOCKED_LED_PIN A4

long lastSendTime = millis();
uint8_t my_id[2] = { 0 };
uint8_t seq = 0;

void send(bool locked)
{
  lastSendTime = millis();

  digitalWrite(LED_PIN, LOW); // active low
  delay(20);
  digitalWrite(LED_PIN, HIGH);

  uint8_t buf[] = { 0x38, 0x05, my_id[0], my_id[1], seq, locked, 100 /*battery*/ };
  radio_status_t send_rc = Radio.Send( &buf[0], sizeof(buf) );
  Serial.println("Sent!");
  seq++;
}


void OnRadioTxDone( void )
{
  Serial.println("TxDone");
  Radio.Rx(0); // continuous reception
}
void OnRadioTxTimeout( void )
{
  Serial.println("TxTimeout");
  Radio.Rx(0); // continuous reception
}

const int max_nodes = 8;
uint16_t unlocked_nodes[max_nodes] = { 0 };
int num_unlocked = 0;

void OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  Serial.println("RxDone");

  if (size != 7) {
    Serial.println("Unxepected length");
    Serial.println(size);
    return;
  }

  const uint16_t check = (payload[0] << 8) | (payload[1]);
  const uint16_t node  = (payload[2] << 8) | (payload[3]);
  const bool locked = payload[5];

  Serial.println(payload[5]);
  if (check != 0x3805) {
    Serial.println("Unxepected check");
    Serial.println(check);
    return;
  }

  Serial.print("Node ");
  Serial.print(node, HEX);
  if (locked) {
    Serial.println(" is locked");

    for (int i = 0; i < max_nodes; i++) {
      if (unlocked_nodes[i] == node) {
        unlocked_nodes[i] = 0;
      }
    }
  } else {
    Serial.println(" is unlocked");

    bool found = false;
    for (int i = 0; i < max_nodes; i++) {
      if (unlocked_nodes[i] == node) {
        found = true;
      }
    }
    if (!found) {
      for (int i = 0; i < max_nodes; i++) {
        if (unlocked_nodes[i] == 0) {
          unlocked_nodes[i] = node;
          break;
        }
      }
    }

  }

  int count = 0;
  for (int i = 0; i < max_nodes; i++) {
    if (unlocked_nodes[i] != 0) {
      count++;
    }
  }
  num_unlocked = count;
}

void OnRadioRxTimeout()
{
  Serial.println("RxTimeout");
}
void OnRadioRxError()
{
  Serial.println("RxError");
}


RadioEvents_t RadioEvents = { 0 };

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);

  Serial.println("--------------");

  pinMode(INPUT_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // active low

  pinMode(OPEN_LED_PIN, OUTPUT);
  digitalWrite(OPEN_LED_PIN, LOW);
  pinMode(LOCKED_LED_PIN, OUTPUT);
  digitalWrite(LOCKED_LED_PIN, LOW);


  const uint32_t unique_id = LL_FLASH_GetUDN();
  my_id[0] =  unique_id       & 0xff;
  my_id[1] = (unique_id >> 8) & 0xff;


  STM32RTC& rtc = STM32RTC::getInstance();
  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.setBinaryMode(STM32RTC::MODE_MIX);
  rtc.begin(true, STM32RTC::HOUR_24);
  UTIL_TIMER_Init(rtc.getHandle());


  RadioEvents.TxDone = OnRadioTxDone;
  RadioEvents.TxTimeout = OnRadioTxTimeout;
  RadioEvents.RxDone = OnRadioRxDone;
  RadioEvents.RxTimeout = OnRadioRxTimeout;
  RadioEvents.RxError = OnRadioRxError;
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

  Radio.SetRxConfig(
    MODEM_LORA,
    0, // bandwidth = 125kHz
    7, // spreading factor
    RADIO_LORA_CR_4_5,
    0, // n/a
    8, // preamble
    8, // ?? - symbol timeout
    false, // == variable length packets
    0, // n/a (fixed payload length)
    true, // == crc on
    false, // == no frequency hopping
    0, // n/a (hop period)
    false, // IQ not inverted
    true // continuous receive
  );

  Radio.Rx(0); // continuous reception
}


bool update_blink(long now) {
  static int count = 0;
  static bool tick_tock = true;

  static long last_tick = now;
  if (now - last_tick >= 500) {
    last_tick = now;

    if (tick_tock) {
      count ++;
      if (count > num_unlocked + 4)
        count = 0;
    }

    tick_tock = !tick_tock;
  }

  return tick_tock && (num_unlocked > count);
}

void loop()
{
  const long now = millis();

  LoRaMacProcess();

  // Update the state immediately to make the LEDs
  // resonsive, but debounce heavily at send-time.
  const bool locked = digitalRead(INPUT_PIN);

  const bool blink = update_blink(now);
  digitalWrite(  OPEN_LED_PIN,  locked && !blink);
  digitalWrite(LOCKED_LED_PIN, !locked && !blink);


  static const long intervals[] = { 3*1000, 10*1000, 60*1000, 15*60*1000 };
  static long nextInterval = intervals[0];

  static bool last_sent = !locked;
  if (last_sent ^ locked)
    nextInterval = intervals[0]; // rate limit

  if (now - lastSendTime > nextInterval) {
    send(locked);
    last_sent = locked;

    for (int i = 0; i < sizeof(intervals)/sizeof(intervals[0]) - 1; i++) {
      if (nextInterval <= intervals[i]) {
        nextInterval = intervals[i+1];
        break;
      }
    }

    Serial.print("Resend in ");
    Serial.print(nextInterval / 1000);
    Serial.println(" seconds");
  }
}
