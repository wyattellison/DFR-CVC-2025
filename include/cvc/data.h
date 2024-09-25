/*
* data.h
*
* Created on September 19, 2024
* Andrei Gerashchenko
*/
#ifndef CVC_DATA_H
#define CVC_DATA_H

#include <stdint.h>

typedef enum {
    EMUS_OverallParameters,
    EMUS_DiagnosticCodes,
    EMUS_BatteryVoltageOverallParameters,
    EMUS_CellModuleTemperatureOverallParameters,
    EMUS_CellTemperatureOverallParameters,
    EMUS_CellBalancingRateOverallParameters,
    EMUS_StateOfChargeParameters,
    EMUS_ConfigurationParameters,
    EMUS_ContactorControl,
    EMUS_EnergyParameters,
    EMUS_Events,
    VDM_GPSLatitudeLongitude,
    VDM_GPSData,
    VDM_GPSDateTime,
    VDM_AccelerationData,
    VDM_YawRateData,
    DASHBOARD_Selector,
    SENSORBOARD_Data,
    INVERTER1_Temp1,
    INVERTER1_Temp2,
    INVERTER1_Temp3TorqueShudder,
    INVERTER1_AnalogInputStatus,
    INVERTER1_DigitalInputStatus,
    INVERTER1_MotorPositionParameters,
    INVERTER1_CurrentParameters,
    INVERTER1_VoltageParameters,
    INVERTER1_FluxParameters,
    INVERTER1_InternalVoltageParameters,
    INVERTER1_InternalStateParameters,
    INVERTER1_FaultCodes,
    INVERTER1_TorqueTimerParameters,
    INVERTER1_ModulationIndexFluxWeakening,
    INVERTER1_FirmwareInformation,
    INVERTER1_DiagnosticData,
    INVERTER1_HighSpeedParameters,
    INVERTER1_TorqueCapability,
    INVERTER2_Temp1,
    INVERTER2_Temp2,
    INVERTER2_Temp3TorqueShudder,
    INVERTER2_AnalogInputStatus,
    INVERTER2_DigitalInputStatus,
    INVERTER2_MotorPositionParameters,
    INVERTER2_CurrentParameters,
    INVERTER2_VoltageParameters,
    INVERTER2_FluxParameters,
    INVERTER2_InternalVoltageParameters,
    INVERTER2_InternalStateParameters,
    INVERTER2_FaultCodes,
    INVERTER2_TorqueTimerParameters,
    INVERTER2_ModulationIndexFluxWeakening,
    INVERTER2_FirmwareInformation,
    INVERTER2_DiagnosticData,
    INVERTER2_HighSpeedParameters,
    INVERTER2_TorqueCapability,
    // === Length of CVC data array ===
    // This must be the last value in the enum
    NUM_MESSAGES,
} CAN_Message_Index;

typedef enum {
    // === CVC Data ===
    CVC_THROTTLE_ADC,
    CVC_THROTTLE,
    CVC_THROTTLE_VALID,
    CVC_DRIVE_MODE, // 0 = neutral, 1 = drive, 2 = reverse
    CVC_STATE,
    CVC_LV_VOLTAGE,
    CVC_LEFT_TORQUE, // Left motor torque
    CVC_RIGHT_TORQUE, // Right motor torque
    CVC_LEFT_DIRECTION, // Left motor direction
    CVC_RIGHT_DIRECTION, // Right motor direction
    CVC_PRECHARGE_BTN, // Precharge button state
    CVC_LV_CHARGE_STATE, // LV charge state
    CVC_AIR_1_STATE, // Air 1 state
    CVC_AIR_2_STATE, // Air 2 state
    CVC_DCDC_STATE, // DCDC state
    CVC_COCKPIT_BRB_STATE, // Cockpit BRB state
    CVC_BOT_STATE, // BOT state
    CVC_IMD_STATE, // IMD state
    CVC_BMS_STATE, // BMS state
    // === Front Sensor Board ==
    SENSOR_THROTTLE_ADC,
    SENSOR_STEERING_ANGLE,
    SENSOR_LEFT_WHEELSPEED,
    SENSOR_RIGHT_WHEELSPEED,
    SENSOR_BRAKESWITCH,
    // === Dashboard ===
    DASH_REQUESTED_STATE, // neutral, drive, reverse, none
    DASH_SELECTOR_VALUE,
    // === EMUS BMS ===
    // Overall Parameters
    BMS_IGNITION,
    BMS_CHARGER_MAINS,
    BMS_FAST_CHARGE,
    BMS_LEAKAGE_DETECTED,
    BMS_CHARGER_ENABLED,
    BMS_HEATER_ENABLED,
    BMS_BATTERY_CONTACTOR,
    BMS_BATTERY_FAN,
    BMS_POWER_REDUCTION,
    BMS_CHARGING_INTERLOCK,
    BMS_DCDC_ENABLED,
    BMS_CONTACTOR_PRECHARGE,
    BMS_CHARGING_STATE,
    BMS_CHARGING_DURATION,
    BMS_LAST_CHARGING_ERROR,
    BMS_LIVE_CELL_COUNT,
    // Diagnostic Codes
    BMS_UNDERVOLTAGE,
    BMS_OVERVOLTAGE,
    BMS_DISCHARGE_OVERCURRENT,
    BMS_CHARGE_OVERCURRENT,
    BMS_CELL_MODULE_OVERHEAT,
    BMS_LEAKAGE,
    BMS_NO_CELL_COMMUNICATION,
    BMS_WARN_LOW_CELL_VOLTAGE,
    BMS_WARN_HIGH_DISCHARGE_CURRENT,
    BMS_WARN_HIGH_CELL_MODULE_TEMP,
    BMS_CELL_OVERHEAT,
    BMS_NO_CURRENT_SENSOR,
    BMS_PACK_UNDERVOLTAGE,
    BMS_CELL_VOLTAGE_VALID,
    BMS_CELL_MODULE_TEMP_VALID,
    BMS_BALANCE_RATE_VALID,
    BMS_LIVE_CELL_COUNT_VALID,
    BMS_CHARGING_FINISHED,
    BMS_CELL_TEMP_VALID,
    // Battery Voltage Overall Parameters
    BMS_MIN_CELL_VOLTAGE,
    BMS_MAX_CELL_VOLTAGE,
    BMS_AVG_CELL_VOLTAGE,
    BMS_TOTAL_VOLTAGE,
    // Cell Module Temperature Overall Parameters
    BMS_MIN_CELL_MODULE_TEMP,
    BMS_MAX_CELL_MODULE_TEMP,
    BMS_AVG_CELL_MODULE_TEMP,
    // Cell Module Temperature Overall Parameters
    BMS_MIN_CELL_TEMP,
    BMS_MAX_CELL_TEMP,
    BMS_AVG_CELL_TEMP,
    // Cell Balancing Rate Overall Parameters
    BMS_MIN_BALANCE_RATE,
    BMS_MAX_BALANCE_RATE,
    BMS_AVG_BALANCE_RATE,
    // State of Charge Parameters
    BMS_CURRENT,
    BMS_ESTIMATED_CHARGE,
    BMS_ESTIMATED_SOC,
    // Configuration Parameters - TODO: Figure out how to handle this
    // Contactor Control
    BMS_CONTACTOR_STATE,
    // Energy Parameters
    BMS_ESTIMATED_CONSUMPTION,
    BMS_ESTIMATED_ENERGY,
    BMS_ESTIMATED_DISTANCE_REMAINING,
    BMS_DISTANCE_TRAVELED,
    // === AEM 30-22O6 VDM ===
    // GPS Data
    VDM_GPS_LATITUDE, 
    VDM_GPS_LONGITUDE, 
    VDM_GPS_SPEED,
    VDM_GPS_ALTITUDE, 
    VDM_GPS_TRUE_COURSE, 
    VDM_GPS_SATELLITES_IN_USE, 
    VDM_GPS_DATA_VALID,
    // GPS Date Time Parameters
    VDM_GPS_DATE_TIME_VALID, 
    VDM_UTC_YEAR, 
    VDM_UTC_MONTH,
    VDM_UTC_DAY,
    VDM_UTC_HOUR,
    VDM_UTC_MINUTE,
    VDM_UTC_SECOND, 
    // Acceleration data
    VDM_ACCELERATION_X,
    VDM_ACCELERATION_Y,
    VDM_ACCELERATION_Z,
    // Yaw rate Data
    VDM_YAW_RATE_X,
    VDM_YAW_RATE_Y,
    VDM_YAW_RATE_Z,
    // Events - TODO: Figure out how to handle this


    // === Inverter ===
    //  Temperature Parameters
    INVERTER1_POWER_MODULE_A_TEMP,
    INVERTER1_POWER_MODULE_B_TEMP,
    INVERTER1_POWER_MODULE_C_TEMP,
    INVERTER1_GATE_DRIVER_BOARD_TEMP, 
    INVERTER1_CONTROL_BOARD_TEMP,
    INVERTER1_RTD_INPUT_1_TEMP,
    INVERTER1_RTD_INPUT_2_TEMP,
    INVERTER1_STALL_BURST_MODEL_TEMP,
    INVERTER1_COOLANT_TEMP,
    INVERTER1_HOT_SPOT_TEMP,
    INVERTER1_MOTOR_TEMP,
    INVERTER2_POWER_MODULE_A_TEMP,
    INVERTER2_POWER_MODULE_B_TEMP,
    INVERTER2_POWER_MODULE_C_TEMP,
    INVERTER2_GATE_DRIVER_BOARD_TEMP, 
    INVERTER2_CONTROL_BOARD_TEMP,
    INVERTER2_RTD_INPUT_1_TEMP,
    INVERTER2_RTD_INPUT_2_TEMP,
    INVERTER2_STALL_BURST_MODEL_TEMP,
    INVERTER2_COOLANT_TEMP,
    INVERTER2_HOT_SPOT_TEMP,
    INVERTER2_MOTOR_TEMP,
    // Torque Parameters
    INVERTER1_TORQUE_SHUDDER,
    INVERTER2_TORQUE_SHUDDER,
    // Analog Input Status Parameters
    INVERTER1_ANALOG_INPUT_1,
    INVERTER1_ANALOG_INPUT_2,
    INVERTER1_ANALOG_INPUT_3,
    INVERTER1_ANALOG_INPUT_4,
    INVERTER1_ANALOG_INPUT_5,
    INVERTER1_ANALOG_INPUT_6,
    INVERTER2_ANALOG_INPUT_1,
    INVERTER2_ANALOG_INPUT_2,
    INVERTER2_ANALOG_INPUT_3,
    INVERTER2_ANALOG_INPUT_4,
    INVERTER2_ANALOG_INPUT_5,
    INVERTER2_ANALOG_INPUT_6,
    // Digital Input Status Parameters
    INVERTER1_FORWARD_SWITCH,
    INVERTER1_REVERSE_SWITCH,
    INVERTER1_BRAKE_SWITCH,
    INVERTER1_REGEN_DISABLE_SWITCH,
    INVERTER1_IGNITION_SWITCH,
    INVERTER1_START_SWITCH,
    INVERTER1_VALET_MODE,
    INVERTER2_FORWARD_SWITCH,
    INVERTER2_REVERSE_SWITCH,
    INVERTER2_BRAKE_SWITCH,
    INVERTER2_REGEN_DISABLE_SWITCH,
    INVERTER2_IGNITION_SWITCH,
    INVERTER2_START_SWITCH,
    INVERTER2_VALET_MODE,
    // Motor Position Parameters
    INVERTER1_MOTOR_ANGLE, 
    INVERTER1_MOTOR_SPEED,
    INVERTER1_MOTOR_ACCELERATION,
    INVERTER1_ELECTRICAL_OUTPUT_FREQUENCY,
    INVERTER1_DELTA_RESOLVER_FILTERED,
    INVERTER2_MOTOR_ANGLE, 
    INVERTER2_MOTOR_SPEED,
    INVERTER2_MOTOR_ACCELERATION,
    INVERTER2_ELECTRICAL_OUTPUT_FREQUENCY,
    INVERTER2_DELTA_RESOLVER_FILTERED,
    // Current Partameters
    INVERTER1_PHASE_A_CURRENT,
    INVERTER1_PHASE_B_CURRENT,
    INVERTER1_PHASE_C_CURRENT,
    INVERTER1_DC_BUS_CURRENT,
    INVERTER2_PHASE_A_CURRENT,
    INVERTER2_PHASE_B_CURRENT,
    INVERTER2_PHASE_C_CURRENT,
    INVERTER2_DC_BUS_CURRENT,
    // Voltage Parameters
    INVERTER1_DC_BUS_VOLTAGE,
    INVERTER1_OUTPUT_VOLTAGE, 
    INVERTER1_VAB_VD_VOLTAGE,
    INVERTER1_VAB_VQ_VOLTAGE,
    INVERTER2_DC_BUS_VOLTAGE,
    INVERTER2_OUTPUT_VOLTAGE, 
    INVERTER2_VAB_VD_VOLTAGE,
    INVERTER2_VAB_VQ_VOLTAGE,
    // Flux Parameters
    INVERTER1_FLUX_COMMAND,
    INVERTER1_FLUX_FEEDBACK, 
    INVERTER1_ID_CURRENT,
    INVERTER1_IQ_CURRENT,
    INVERTER2_FLUX_COMMAND,
    INVERTER2_FLUX_FEEDBACK, 
    INVERTER2_ID_CURRENT,
    INVERTER2_IQ_CURRENT,
    // Internal Voltage Parameters
    INVERTER1_1_5_REFERENCE_VOLTAGE,
    INVERTER1_2_5_REFERENCE_VOLTAGE,
    INVERTER1_5_0_REFERENCE_VOLTAGE,
    INVERTER1_12_0_REFERENCE_VOLTAGE,
    INVERTER2_1_5_REFERENCE_VOLTAGE,
    INVERTER2_2_5_REFERENCE_VOLTAGE,
    INVERTER2_5_0_REFERENCE_VOLTAGE,
    INVERTER2_12_0_REFERENCE_VOLTAGE,
    // Internal State Parameters
    INVERTER1_VSM_STATE,
    INVERTER1_PWM_FREQUENCY,
    INVERTER1_STATE,
    INVERTER1_RELAY_1_STATUS,
    INVERTER1_RELAY_2_STATUS,
    INVERTER1_RELAY_3_STATUS,
    INVERTER1_RELAY_4_STATUS,
    INVERTER1_RELAY_5_STATUS,
    INVERTER1_RELAY_6_STATUS,
    INVERTER1_RUN_MODE,
    INVERTER1_SELF_SENSING_ASSIST,
    INVERTER1_ACTIVE_DISCHARGE_STATE,
    INVERTER1_COMMAND_MODE,
    INVERTER1_ROLLING_COUNTER_VALUE,
    INVERTER1_ENABLE_STATE,
    INVERTER1_BURST_MODEL_MODE,
    INVERTER1_START_MODE_ACTIVE,
    INVERTER1_ENABLE_LOCKOUT,
    INVERTER1_DIRECTION_COMMAND,
    INVERTER1_BMS_ACTIVE,
    INVERTER1_BMS_LIMITING_TORQUE,
    INVERTER1_LIMIT_MAX_SPEED,
    INVERTER1_LIMIT_HOT_SPOT,
    INVERTER1_LOW_SPEED_LIMITING,
    INVERTER1_COOLANT_TEMP_LIMITING,
    INVERTER1_LIMIT_STALL_BURST_MODEL,
    INVERTER2_VSM_STATE,
    INVERTER2_PWM_FREQUENCY,
    INVERTER2_STATE,
    INVERTER2_RELAY_1_STATUS,
    INVERTER2_RELAY_2_STATUS,
    INVERTER2_RELAY_3_STATUS,
    INVERTER2_RELAY_4_STATUS,
    INVERTER2_RELAY_5_STATUS,
    INVERTER2_RELAY_6_STATUS,
    INVERTER2_RUN_MODE,
    INVERTER2_SELF_SENSING_ASSIST,
    INVERTER2_ACTIVE_DISCHARGE_STATE,
    INVERTER2_COMMAND_MODE,
    INVERTER2_ROLLING_COUNTER_VALUE,
    INVERTER2_ENABLE_STATE,
    INVERTER2_BURST_MODEL_MODE,
    INVERTER2_START_MODE_ACTIVE,
    INVERTER2_ENABLE_LOCKOUT,
    INVERTER2_DIRECTION_COMMAND,
    INVERTER2_BMS_ACTIVE,
    INVERTER2_BMS_LIMITING_TORQUE,
    INVERTER2_LIMIT_MAX_SPEED,
    INVERTER2_LIMIT_HOT_SPOT,
    INVERTER2_LOW_SPEED_LIMITING,
    INVERTER2_COOLANT_TEMP_LIMITING,
    INVERTER2_LIMIT_STALL_BURST_MODEL,
    // Fault codes
    INVERTER1_POST_FAULT_LO,
    INVERTER1_POST_FAULT_HI, 
    INVERTER1_RUN_FAULT_LO, 
    INVERTER1_RUN_FAULT_HI,
    INVERTER2_POST_FAULT_LO,
    INVERTER2_POST_FAULT_HI, 
    INVERTER2_RUN_FAULT_LO, 
    INVERTER2_RUN_FAULT_HI,
    // High Speed Parameters
    INVERTER1_TORQUE_COMMAND_HS,
    INVERTER1_TORQUE_FEEDBACK_HS,
    INVERTER1_MOTOR_SPEED_HS,
    INVERTER1_DC_BUS_VOLTAGE_HS,
    INVERTER2_TORQUE_COMMAND_HS,
    INVERTER2_TORQUE_FEEDBACK_HS,
    INVERTER2_MOTOR_SPEED_HS,
    INVERTER2_DC_BUS_VOLTAGE_HS,

    // === PLC ===
    PLC_OUTPUTS,
    PLC_INPUTS,
    LED_STATE,
    // === Length of CVC data array ===
    // This must be the last value in the enum
    NUM_DATA_VALUES,
} CVC_Data_Index;

// Main CAN data vector
// Data array stored as single 64-bit integer
extern volatile uint64_t CAN_data[NUM_MESSAGES];
// CVC data vector
// Data stored as single 64-bit integer
extern volatile uint64_t CVC_data[NUM_DATA_VALUES];

#endif // CVC_DATA_H