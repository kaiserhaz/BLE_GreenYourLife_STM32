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
 *
 * GreenBuildingService.h | Green Building Service header
 *
 * See LICENCE.txt for information on copyrights
 *
 ******************************************************************************/

#ifndef __BLE_GREEN_BUILDING_SERVICE_H__
#define __BLE_GREEN_BUILDING_SERVICE_H__

#include "ble/BLE.h"

/**
 * @class GreenBuildingService
 * @brief Custom service derived from mbed's BLE Environmental Service. This service provides air temperature, humidity and soil moisture measurement.
 */
class GreenBuildingService {
public:
    
    static const uint16_t UUID_GREEN_BUILDING_SERVICE  = 0xEB00;                /**< Green Building service UUID */
    static const uint16_t UUID_PLANT_ENVIRONMENT_CHAR  = 0xEB01;                /**< Plant environment characteristic UUID */
    
    /** Plant environment data type
     *
     */
    typedef struct
    {
        uint8_t airTemperature;                                                 /**< Air temperature value */
        uint8_t airHumidity   ;                                                 /**< Air humidity value */
        uint8_t soilMoisture  ;                                                 /**< Soil moisture value */
    } PlantEnvironmentType_t;

    /**
     * @brief   GreenBuildingService constructor.
     * @param   ble Reference to BLE device.
     */
    GreenBuildingService(BLE& _ble) :
        ble(_ble),
        plantEnvironmentCharacteristic(GreenBuildingService::UUID_PLANT_ENVIRONMENT_CHAR,
                                       &plantEnvironment                              ,
                                       GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY)
    {        
        static bool serviceAdded = false;                                       // Ensure only single service is added
        if (serviceAdded) {
            return;
        }
        
        GattCharacteristic *charTable[] = { &plantEnvironmentCharacteristic };  // Compile characteristics

        GattService greenBuildingService(GreenBuildingService::UUID_GREEN_BUILDING_SERVICE,
                                         charTable                               ,
                                         sizeof(charTable) / sizeof(GattCharacteristic *)); // Create GATT service

        ble.gattServer().addService(greenBuildingService);                      // Register service with GATT server

        serviceAdded = true;
    }

    /**
     * @brief   Update plant environment characteristic.
     * @param   newPlantEnvironmentVal New plant environment measurement.
     */
    void updatePlantEnvironment(PlantEnvironmentType_t newPlantEnvironmentVal)
    {
        plantEnvironment = (PlantEnvironmentType_t) (newPlantEnvironmentVal);
        ble.gattServer().write(plantEnvironmentCharacteristic.getValueHandle(), (uint8_t *) &plantEnvironment, sizeof(PlantEnvironmentType_t));
    }

private:
    BLE& ble;                                                                   /*< Local BLE controller reference */

    ReadOnlyGattCharacteristic<PlantEnvironmentType_t> plantEnvironmentCharacteristic; /*< Plant environment read-only characteristic */

    PlantEnvironmentType_t plantEnvironment;                                    /*< Local var for plant environment */
};

#endif /* #ifndef __BLE_GREEN_BUILDING_SERVICE_H__*/