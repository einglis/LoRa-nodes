
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

bool sleep_now = false;
bool send_now = true;

#define LED_PIN  D9
#define INPUT_PIN  A4

void event_fn( void )
{
  send_now = true;
}

uint8_t my_id[2] = { 0 };
uint8_t seq = 0;

void send()
{
  const int vref = analogRead(AVREF);
  const int millivolts = 1212 * 1023 / vref; // 1.212 is nominal reference voltage
  const int charge = max(0, min(255, (millivolts - 1800) * 255 / (3600 - 1800)));

  uint8_t buf[] = { 0x38, 0x05, my_id[0], my_id[1], seq, digitalRead(INPUT_PIN) /*input*/, (uint8_t)charge /*battery*/ };
  //radio_status_t send_rc = 
    Radio.Send( &buf[0], sizeof(buf) );
  //Serial.println("Sent!");
  //Serial.println(send_rc);
  seq++;

}


void OnRadioTxDone( void )
{
  //Serial.println("TxDone");
  sleep_now = true;
}
void OnRadioTxTimeout( void )
{
  //Serial.println("TxTimeout");
  sleep_now = true; // oh well!
}

RadioEvents_t RadioEvents = { 0 };


void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);

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

  // Radio.SetRxConfig(
  //   MODEM_LORA,
  //   0, // bandwidth = 125kHz
  //   7, // spreading factor
  //   RADIO_LORA_CR_4_5,
  //   0, // n/a
  //   8, // preamble
  //   8, // ?? - symbol timeout
  //   false, // == variable length packets
  //   0, // n/a (fixed payload length)
  //   true, // == crc on
  //   false, // == no frequency hopping
  //   0, // n/a (hop period)
  //   false, // IQ not inverted
  //   true // continuous receive
  // );

  Radio.Sleep();

  LowPower.begin();
  pinMode(INPUT_PIN, INPUT);
  LowPower.attachInterruptWakeup(INPUT_PIN, event_fn, CHANGE, DEEP_SLEEP_MODE);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  if (0)
  {
    Serial.println("Wakeup delay");
    delay(10*1000);
  }
}

void loop()
{
  LoRaMacProcess();

  if (send_now)
  {
    send_now = false;

    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);


    send();
  }

  if (sleep_now)
  {
    sleep_now = false;
    LowPower.deepSleep(60*1000 /*ms*/);
    send_now = true;
  }
}
