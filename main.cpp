/************************ STM32NUCLEO IOT Contest ******************************
 *
 *                   Green Building IoT Solution for
 *                Plant Life Monitoring And Maintenance
 *
 *                           Authored by
 *                        Dien Hoa Truong
 *                 Muhammad Haziq Bin Kamarul Azman
 *                        
 *                            for the
 *            eSAME 2016 STM32NUCLEO IoT Contest in Sophia-Antipolis
 *
 *  main.cpp | Program main
 *
 *  See LICENCE.txt for information on copyrights
 *
 ******************************************************************************/

/** Includes **/
#include "mbed.h"                                                               // ARM mbed               library
#include "x_nucleo_iks01a1.h"                                                   // STM32NUCLEO board      library
#include "ble/BLE.h"                                                            // Bluetooth LE           library
#include "GreenBuildingService.h"                                               // Green Building service library



/** Defines **/
#define GB_SOIL_MOISTURE_MAX 70                                                 // Soil moisture threshold value



/** Device declarations **/

// Board-specific
PwmOut pumpPWM(PC_8);                                                           // PWM motor control out pin
DigitalOut led1(LED1, 1);                                                       // Debug pin instance
AnalogIn moisture_sensor(PB_1);                                                 // Moisture sensor
static X_NUCLEO_IKS01A1 *mems_expansion_board = X_NUCLEO_IKS01A1::Instance(D14, D15); // Expansion board instance
static HumiditySensor   *humidity_sensor = mems_expansion_board->ht_sensor;     // Expansion board humidity sensor instance
static TempSensor       *temp_sensor     = mems_expansion_board->ht_sensor;     // Expansion board temperature sensor instance

// BLE-specific
BLE&                  ble = BLE::Instance(BLE::DEFAULT_INSTANCE);               // BLE device instance
const static char     DEVICE_NAME[] = "GB-Sensor";                              // Device name
static const uint16_t uuid16_list[] = {GreenBuildingService::UUID_GREEN_BUILDING_SERVICE};
GreenBuildingService *gbServicePtr;                                             // Service pointer

// Program-specific
float getMoistureValue();
float getHumidityValue();
float getTemperatureValue();
void errorLoop(void);
void activateFastSensorPoll();
void deactivateFastSensorPoll();
void pumpActivateCallback(void);
void pumpDeactivateCallback(void);

Ticker sanityTicker;
Ticker sensorPollTicker;
Ticker fastSensorPollTicker;
Timeout pumpWaitTimeout;
uint8_t usersConnected;
bool sensorPolling;
bool fastSensorPolling;
bool pumpActivate;
bool waitOnce;
bool bleActive;
bool pumpActive;


/** Callbacks **/

// BLE-specific callback
void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)    // Callback for everytime the connection gets disconnected
{
    ble.gap().startAdvertising();                                               // Restart advertising
    if((!pumpActive)||(!usersConnected))
        deactivateFastSensorPoll();
    bleActive = false;
    --usersConnected;
//    printf("\r\n> BLE  : Disconnected. Advertising restarted.");
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params)          // Callback for everytime the connection is established
{
    ble.gap().stopAdvertising();                                                // Stop advertising
    activateFastSensorPoll();
    bleActive = true;
    ++usersConnected;
//    printf("\r\n> BLE  : Connected to %x. Accept no subsequent connections.", params->peerAddr);
}

void onBleInitError(BLE &ble, ble_error_t error)
{
//    printf("\r\n> BLE  : Init error encountered. Error returned: %d", error);
    errorLoop();
}

void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {                                              // Check to see init errors
        onBleInitError(ble, error);
        errorLoop();
    }

    if (ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {                         // If this is not default instance (double instanciation?)
//        printf("\r\n> BLE  : BLE controller instance is invalid.");
        errorLoop();
    }
    
    ble.gap().onDisconnection(disconnectionCallback);                           // Register disconnection callback
    ble.gap().onConnection(connectionCallback);                                 // Register connection callback

    gbServicePtr = new GreenBuildingService(ble);                               // Init service with initial value
    
    /* Setup advertising. */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS,(uint8_t *)uuid16_list, sizeof(uuid16_list));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms */
    ble.gap().startAdvertising();
    
//    printf("\r\n> BLE  : BLE Init done.");
}

// Helper functions for retrieving data from sensors
float getMoistureValue()
{
    float moisture = 0;
    for (int i = 1;i<=10;i++) {
        moisture += moisture_sensor.read();                                     // Get ten samples
    }
    moisture = moisture / 10;
    moisture = moisture * 3300;                                                 // Change the value to be in the 0 to 3300 range
    moisture = moisture / 33;                                                   // Convert to percentage
    return moisture;
}

float getHumidityValue()
{
    float humidity = 0;
    humidity_sensor->GetHumidity(&humidity);
    return humidity;
}

float getTemperatureValue()
{
    float temperature = 0;
    temp_sensor->GetTemperature(&temperature);
    return temperature;
}


// Miscellaneous callbacks & functions
void sanityCallback(void)
{
    led1 = !led1;                                                               // Blink LED1 to indicate system sanity
}

void sensorPollCallback(void)
{
    sensorPolling = true;
}

void fastSensorPollCallback(void)
{
    fastSensorPolling = true;
}

void pumpActivateCallback(void)
{
    pumpActivate = true;
}

void pumpDeactivateCallback(void)
{
    pumpActivate = false;
}

void activateFastSensorPoll(void)
{
    fastSensorPolling = true;
    fastSensorPollTicker.attach(&fastSensorPollCallback, 0.9);
}

void deactivateFastSensorPoll(void)
{
    fastSensorPolling = false;
    fastSensorPollTicker.detach();
}


void errorLoop(void)
{
    sanityTicker.detach();
    sensorPollTicker.detach();
    ble.shutdown();
//    printf("\r\n> ERROR : Error encountered. Infinite looping.");
    while(true)
    {
        led1 != led1;        
    }
}



/** Pre-main inits **/



/** Main loop **/
int main(void)
{
    pumpPWM.write(1);
    pumpPWM.period(1.0f);
    
    printf("\r\n/**\r\n * Green Building Sensor Device: Debug Info\r\n */");
    
    sensorPolling = false;
    fastSensorPolling = false;
    pumpActivate = false;
    waitOnce = true;
    bleActive = false;
    pumpActive = false;
    
    sanityTicker.attach(sanityCallback, 1.1);                                   // LED sanity checker
    sensorPollTicker.attach(sensorPollCallback, 4.9);                           // Sensor poll ticker
    
    printf("\r\n> MAIN : Tickers initialized.");
 
    volatile GreenBuildingService::PlantEnvironmentType_t peVal;                // Plant environment var
    uint8_t pumpWaitTime = 3;                                                   // Pump waiting time
 
    ble.init(bleInitComplete);                                                  // Pass BLE init complete function upon init
    
//    while(ble.hasInitialized() == false);
    
    printf("\r\n> MAIN : BLE Init procedure done.");
    
    // Infinite loop
    while (true) {
        
        if(sensorPolling || fastSensorPolling)
        {
            sensorPolling = false;                                              // Deassert polling bit
            fastSensorPolling = false;
            
            peVal.soilMoisture   = (uint8_t) getMoistureValue();                // Update all measurements
            peVal.airHumidity    = (uint8_t) getHumidityValue();
            peVal.airTemperature = (int8_t)  getTemperatureValue();
            
            if(ble.getGapState().connected)                                     // Update characteristic if connected
                gbServicePtr->updatePlantEnvironment(peVal);
            
//            printf("\r\n> MAIN : Current soil moisture    = %d", peVal.soilMoisture);
//            printf("\r\n> MAIN : Current air humidity     = %d", peVal.airHumidity);
//            printf("\r\n> MAIN : Current air temperature  = %d", peVal.airTemperature);
            printf("%d\t%d\t%d\r\n", peVal.airTemperature, peVal.airHumidity, peVal.soilMoisture);
            
            // If moisture is below 50% of max when user is present
            //    or if less than 30% of max
            if( ( ((peVal.soilMoisture < 0.5*GB_SOIL_MOISTURE_MAX) &&  ble.getGapState().connected)   ||
                  ((peVal.soilMoisture < 0.3*GB_SOIL_MOISTURE_MAX) && !ble.getGapState().connected) ) &&
                waitOnce
            )
            {
                pumpWaitTimeout.attach(&pumpActivateCallback, pumpWaitTime);    // Waiting time is hard coded but may be calculated, I think
                activateFastSensorPoll();
                waitOnce = false;
                pumpActive = true;
            }
            else if((peVal.soilMoisture >= 0.6*GB_SOIL_MOISTURE_MAX) && pumpActivate) // Stop condition: when soil moisture is at 60% of max
            {
                pumpPWM.write(1);
                pumpWaitTimeout.detach();
                pumpDeactivateCallback();
                if(!bleActive)
                    deactivateFastSensorPoll();
                waitOnce = true;
                pumpActive = false;
            }
            
            if(pumpActivate)
            {
//                printf("\r\n> MAIN : Activating water pump.");
                pumpPWM.write(0.7);                
                pumpActivate = false;
                pumpWaitTimeout.attach(&pumpActivateCallback, 1);
            }
            
        }
        else
            ble.waitForEvent();                                                 //Low power wait for event   
        
    }
}
