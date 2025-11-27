/**
   @file Environment_Monitoring.ino
   @author rakwireless.com
   @brief This sketch demonstrate how to get environment data from BME680
      and send the data to lora gateway.
   @version 0.1
   @date 2020-07-28
   @copyright Copyright (c) 2020
**/

#include <Arduino.h>
#include <LoRaWan-RAK4630.h>  // Click to install library: http://librarymanager/ALL#SX126x-Arduino
#include "bsec.h"             // Click to download library: https://github.com/BoschSensortec/BSEC-Arduino-library then install it as .ZIP
#include <Servo.h>

// Comment the next line if you want DEBUG output. But the power savings are not as good then!!!!!!!
//#define MAX_SAVE

String output;

bool doOTAA = true;
// OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                                       /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_3                                     /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_0                               /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 5                                        /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;                           /* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_AU915;           /* Region:AU915*/
lmh_confirm g_CurrentConfirm = LMH_CONFIRMED_MSG;                 /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;                              /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = { LORAWAN_ADR_OFF, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF };

// Foward declaration
static void lorawan_has_joined_handler(void);
void lorawan_join_fail(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);
void Sensor_parse_data(void);
/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = { BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                           lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_fail };

//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Device EUI
uint8_t nodeAppEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };    // Application EUI};
uint8_t nodeAppKey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Application Key

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 128                                           /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 20000                                               /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];               //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = { m_lora_app_data_buffer, 0, 0, 0, 0 };  //< Lora user application data structure.

TimerEvent_t appTimer;
static uint32_t timers_init(void);
static uint32_t count = 0;
static uint32_t count_fail = 0;

float co2;  // The magic variable that will store the CO2 level.

unsigned long previousMillis = 0;  // Will store last time the Servo was turned.

#define BAD_CO2 1000   //Constant of CO2 ppm that will considered to trigger the Servo.
#define GOOD_CO2 1000  //Constant of CO2 ppm that will considered to trigger the Servo.

#define INTERVAL_GOOD 10000  //Sampling interval when the air is clean in (ms)
#define INTERVAL_BAD 5000    //Sampling interval when the air is polluted in (ms)

long INTERVAL = INTERVAL_GOOD;

#define ALIVE_DG 0   // Servo position when the air is clean (Deg)
#define DIED_DG 180  // Servo position when the air is polluted (Deg)


int pos = 0;  // Variable to store the servo position

// Create an object of the class Bsec
Bsec iaqSensor;  // create Bsec object to read the sensor data
Servo myservo;   // create servo object to control a servo


void setup() {
  // Initialize the built in LED
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);

  // Initialize Serial for debug output
  Serial.begin(115200);

  time_t serial_timeout = millis();
  // On nRF52840 the USB serial is not available immediately
  while (!Serial) {
    if ((millis() - serial_timeout) < 5000) {
      delay(100);
      digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    } else {
      break;
    }
    digitalWrite(LED_GREEN, LOW);
  }

  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWan!!!");
  if (doOTAA) {
    Serial.println("Type: OTAA");
  } else {
    Serial.println("Type: ABP");
  }

  Serial.println("Region: AU915");

  Serial.println("=====================================");

  // Initialize LoRa chip.
  lora_rak4630_init();

  //creat a user timer to send data to server period
  uint32_t err_code;

  err_code = timers_init();

  if (err_code != 0) {
    Serial.printf("timers_init failed - %d\n", err_code);
    return;
  }

  // Setup the EUIs and Keys
  if (doOTAA) {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }

  // Initialize LoRaWan
  err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0) {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  Serial.println("Node Initiated Correctly");

  Wire.begin();

  myservo.attach(PIN_SERIAL1_RX);  // Attaches the servo on pin RX to the servo object

  //Initialize Sensor Communication
  iaqSensor.begin(BME68X_I2C_ADDR_LOW, Wire);

  //Check sensor status
  checkIaqSensorStatus();

  //Define the sensor variables you need from the BME680
  bsec_virtual_sensor_t sensorList[1] = {
    BSEC_OUTPUT_CO2_EQUIVALENT,
  };

  //Subscribe to the selected variable
  iaqSensor.updateSubscription(sensorList, 1, BSEC_SAMPLE_RATE_LP);

  //Check sensor status
  checkIaqSensorStatus();

  // Tell servo to go to initial position
  myservo.write(pos);

  Sensor_parse_data();

  lmh_join();
}

void loop() {

  if (iaqSensor.run()) {  // If new data is available

    co2 = iaqSensor.co2Equivalent;  //Store the CO2 level on variable "co2"

  } else {
    checkIaqSensorStatus();
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= INTERVAL) {
    // Save the last time the servo was running
    previousMillis = currentMillis;
#ifndef MAX_SAVE
    Serial.print("CO2: ");
    Serial.println(co2);

#endif

    if (co2 > BAD_CO2) {  // If the CO2 level exceeded the BAD_CO2 param, turn down the servo to DIED_DG
      INTERVAL = INTERVAL_BAD;
      //digitalWrite(LED_GREEN, LOW);
      for (pos; pos <= DIED_DG; pos += 1) {  // goes to DIED position
        // in steps of 1 degree
        myservo.write(pos);  // tell servo to go to position in variable 'pos'
        delay(15);           // waits 15ms for the servo to reach the position
      }
    } else if (co2 <= GOOD_CO2) {  // If the CO2 level returns to normal, turn up the servo to ALIVE_DG
      //digitalWrite(LED_GREEN, HIGH);
      INTERVAL = INTERVAL_GOOD;
      for (pos; pos >= ALIVE_DG; pos -= 1) {  // goes to ALIVE position
        // in steps of 1 degree
        myservo.write(pos);  // tell servo to go to position in variable 'pos'
        delay(15);           // waits 15ms for the servo to reach the position
      }
    }
  }
}

// Helper function definitions
void checkIaqSensorStatus(void) {
  if (iaqSensor.bsecStatus != BSEC_OK) {
    if (iaqSensor.bsecStatus < BSEC_OK) {

      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    if (iaqSensor.bme68xStatus < BME68X_OK) {

      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
    }
  }
}

void errLeds(void) {
  delay(100);
  digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
}

/**@brief LoRa function for failed Join event
*/
void lorawan_join_fail(void) {
  Serial.println("OTAA join failed!");
  //digitalWrite(LED_BLUE, LOW);
}

/**@brief LoRa function for handling HasJoined event.
*/
void lorawan_has_joined_handler(void) {
  Serial.println("OTAA Mode, Network Joined!");
  //digitalWrite(LED_BLUE, HIGH);
  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS) {
    delay(1000);
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);
  }
}

/**@brief Function for handling LoRaWan received data from Gateway
   @param[in] app_data  Pointer to rx data
*/
void lorawan_rx_handler(lmh_app_data_t *app_data) {
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
                app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class) {
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void send_lora_frame(void) {
  if (lmh_join_status_get() != LMH_SET) {
    //Not joined, try again later
    Serial.println("Not joined, try again later");
    return;
  }

  Sensor_parse_data();

  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);

  if (error == LMH_SUCCESS) {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  } else {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }

  if (count_fail >= 3) {
    Serial.println("Reseting Board!");
    NVIC_SystemReset();
  }
}

/**@brief Function for handling user timerout event.
*/
void tx_lora_periodic_handler(void) {
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);
  Serial.println("Sending frame now...");
  send_lora_frame();
}

/**@brief Function for the Timer initialization.
   @details Initializes the timer module. This creates and starts application timers.
*/
uint32_t timers_init(void) {
  TimerInit(&appTimer, tx_lora_periodic_handler);
  return 0;
}


void Sensor_parse_data(void) {
  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;

  // Convertimos el float de CO2 a entero sin signo (uint16)
  // El CO2 en ppm es un número entero (ej. 450, 1200, etc.)
  uint16_t co2_ppm = (uint16_t)co2;

  // --- Empaquetado Cayenne LPP ---

  // Canal 1
  m_lora_app_data.buffer[i++] = 0x01;

  // Tipo: 0x65 (Illuminance / Lux Sensor)
  // Usamos este porque permite valores hasta 65,535.
  // "Analog Input" (0x02) solo llega a 327.67, lo cual rompería tus datos de CO2.
  m_lora_app_data.buffer[i++] = 0x65;

  // Valor (High Byte y Low Byte)
  m_lora_app_data.buffer[i++] = (uint8_t)(co2_ppm >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)co2_ppm;

  m_lora_app_data.buffsize = i;
}
