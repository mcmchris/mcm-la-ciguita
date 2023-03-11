/*
   La Cigüita firmware

   @Author: Christopher Mendez | MCMCHRIS
   @Date: 03/10/2023 (mm/dd/yy)
   @Brief:
   This firmware runs on a nRF52840 (RAK4631) and using a BME680 sensor, analyze Gas resistance,
   if the air quality is bad, controls a servo motor to let you know, dropping down La Cigüita

   Average power consumption: 6.12ma 
   With a 3000mah battery should last for 21 days.
*/

#include "main.h"
#include "bsec.h"  // Click to download library: https://github.com/BoschSensortec/BSEC-Arduino-library then install it as .ZIP
#include <Servo.h>

#define PIN_VBAT WB_A0  // Pin where the battery voltage is measured

// Battery level measurement variables
uint32_t vbat_pin = PIN_VBAT;
float vbat_mv;
uint8_t vbat_per;

float gas;  // The magic variable that will store the Gas resistance

#define VBAT_MV_PER_LSB (0.73242188F)  // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.73)       // Compensation factor for the VBAT divider, depend on the board

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

#define BAD_GAS 9000  //Constant of Gas resistance that will considered to trigger the Servo.

#define INTERVAL_GOOD 10000  //10 * 60 * 1000 Sampling interval when the air is clean in (ms)
#define INTERVAL_BAD 5000    //5 * 60 * 1000 Sampling interval when the air is polluted in (ms)

long INTERVAL = INTERVAL_GOOD;

#define ALIVE_DG 0   // Servo position when the air is clean (Deg)
#define DIED_DG 165  // Servo position when the air is polluted (Deg)
#define BATT_DG 90   // Servo position when the battery is low (Deg)

int pos = 0;  // Variable to store the servo position

// Create an object of the class Bsec
Bsec iaqSensor;  // create Bsec object to read the sensor data
Servo myservo;   // create servo object to control a servo

/** Semaphore used by events to wake up loop task */
SemaphoreHandle_t taskEvent = NULL;

/** Timer to wakeup task frequently and send message */
SoftwareTimer taskWakeupTimer;

/**
   @brief Get RAW Battery Voltage
*/
float readVBAT(void) {
  float raw;

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(vbat_pin);

  return raw * REAL_VBAT_MV_PER_LSB;
}

/**
   @brief Convert from raw mv to percentage
   @param mvolts
      RAW Battery Voltage
*/
uint8_t mvToPercent(float mvolts) {
  if (mvolts < 3300)
    return 0;

  if (mvolts < 3600) {
    mvolts -= 3300;
    return mvolts / 30;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F);  // thats mvolts /6.66666666
}

/**
 * @brief Flag for the event type
 * -1 => no event
 * 0 => LoRaWan data received
 * 1 => Timer wakeup
 * 2 => tbd
 * ...
 */
uint8_t eventType = -1;

/**
 * @brief Timer event that wakes up the loop task frequently
 * 
 * @param unused 
 */
void periodicWakeup(TimerHandle_t unused) {

  eventType = 1;
  // Give the semaphore, so the loop task will wake up
  xSemaphoreGiveFromISR(taskEvent, pdFALSE);
}

/**
 * @brief Arduino setup function. Called once after power-up or reset
 * 
 */
void setup(void) {

  Wire.begin();  // Initialize I2C communication.

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12);  // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get a single ADC sample and throw it away
  readVBAT();

  //Initialize Sensor Communication
  iaqSensor.begin(BME68X_I2C_ADDR_LOW, Wire);
  //Check sensor status
  checkIaqSensorStatus();

  //Define the sensor variables you need from the BME680
  bsec_virtual_sensor_t sensorList[1] = {
    BSEC_OUTPUT_RAW_GAS,
  };

  //Subscribe to the selected variable
  iaqSensor.updateSubscription(sensorList, 1, BSEC_SAMPLE_RATE_LP);

  //Check sensor status
  checkIaqSensorStatus();

  // Create the LoRaWan event semaphore
  taskEvent = xSemaphoreCreateBinary();
  // Initialize semaphore
  xSemaphoreGive(taskEvent);

#ifndef MAX_SAVE
  // Initialize Serial for debug output
  Serial.begin(115200);
#endif

#ifndef MAX_SAVE
  Serial.println("=====================================");
  Serial.println("RAK4631 Deep Sleep Test");
  Serial.println("=====================================");
#endif

  // Take the semaphore so the loop will go to sleep until an event happens
  xSemaphoreTake(taskEvent, 10);
  taskWakeupTimer.begin(SLEEP_TIME, periodicWakeup);
  taskWakeupTimer.start();
}

/**
 * @brief Arduino loop task. Called in a loop from the FreeRTOS task handler
 * 
 */
void loop(void) {

  // Sleep until we are woken up by an event
  if (xSemaphoreTake(taskEvent, portMAX_DELAY) == pdTRUE) {

    // Check the wake up reason
    switch (eventType) {

      case 1:  // Wakeup reason is timer

        if (iaqSensor.run()) {  // If new data is available

          gas = iaqSensor.gasResistance;

          //sensordata = String(temp);
          // Get a raw ADC reading
          vbat_mv = readVBAT();

          // Convert from raw mv to percentage (based on LIPO chemistry)
          vbat_per = mvToPercent(vbat_mv);

          // If the battery is low, show it setting the servo to BATT_DG degrees.
          if (vbat_per <= 25) {
            if (pos > BATT_DG) {
              for (pos; pos >= BATT_DG; pos -= 1) {  // goes from 0 degrees to BATT_DG degrees
                // in steps of 1 degree
                myservo.write(pos);  // tell servo to go to position in variable 'pos'
                delay(15);           // waits 15ms for the servo to reach the position
              }
            } else {
              for (pos; pos <= BATT_DG; pos += 1) {  // goes from 0 degrees to BATT_DG degrees
                // in steps of 1 degree
                myservo.write(pos);  // tell servo to go to position in variable 'pos'
                delay(15);           // waits 15ms for the servo to reach the position
              }
            }
          }

          if (gas < BAD_GAS && vbat_per > 25) {  // If the Gas resistance is lower than the BAD_Gas param, turn down the servo to DIED_DG
            INTERVAL = INTERVAL_BAD;
            for (pos; pos <= DIED_DG; pos += 1) {  // goes to DIED position
              // in steps of 1 degree
              myservo.write(pos);  // tell servo to go to position in variable 'pos'
              delay(15);           // waits 15ms for the servo to reach the position
            }
          } else if (gas >= BAD_GAS && vbat_per > 25) {  // If the Gas resistance is higher than the BAD_Gas param, turn up the servo to ALIVE_DG
            INTERVAL = INTERVAL_GOOD;
            for (pos; pos >= ALIVE_DG; pos -= 1) {  // goes to ALIVE position
              // in steps of 1 degree
              myservo.write(pos);  // tell servo to go to position in variable 'pos'
              delay(15);           // waits 15ms for the servo to reach the position
            }
          }
#ifndef MAX_SAVE
          Serial.print("Gas: ");
          Serial.println(gas);
          Serial.print("Battery %: ");
          Serial.println(vbat_per);
#endif
        } else {
          checkIaqSensorStatus();
        }
        break;
      default:
        break;
    }

    // Go back to sleep
    xSemaphoreTake(taskEvent, 10);
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
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
