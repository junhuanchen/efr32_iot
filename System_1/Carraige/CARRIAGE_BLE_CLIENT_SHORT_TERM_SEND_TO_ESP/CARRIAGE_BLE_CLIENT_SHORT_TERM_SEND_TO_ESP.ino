/*
 * BLE Custom Data Client (GPS NMEA/Timepulse Integration - Periodic Sync)
 *
 * - Combines Silabs BLE Client logic with NMEA/PPS parsing.
 * - Scans for "CustomDataSender".
 * - Connects and discovers characteristics (Indication, Time Sync, Offset).
 * - Uses NMEA/PPS derived time via getPreciseTimestamp().
 * - Sends Unix timestamp (4 bytes) + Precise Milliseconds (4 bytes) together
 * to server's Time Sync characteristic every 10 seconds using Write Without Response.
 * - Sends Offset (1 byte) to server's Offset characteristic every 10 seconds
 * using Write Without Response.
 * - Enables indications on 0x2A1C.
 * - Receives 10-byte data (Est. Unix Time + Server MS + ADC) from server indications.
 * - **Collects 10 LoadCell ADC values per second based on ServerSensorTime.**
 * - **Collects 10 Analog Pin 6 values per second based on Client GPS Time.**
 * - **Collects 10 Encoder Velocity (mm/s) per second based on Client GPS Time.**
 * - **Prints a consolidated data packet per Client GPS Unix Second.**
 * - Periodically prints detailed GPS status to Serial Monitor.
 */

#include <Arduino.h>
#include <sl_bt_api.h> // Silabs BLE API
#include <stdio.h>      // For sprintf/printf
#include <string.h>     // For strncmp, strlen, strchr, memset
#include <stdlib.h>     // For atoi, atof
#include <time.h>       // For time_t, tm struct, mktime()
#include <math.h>       // For round(), isnan(), isinf() (for Encoder)
#include <stdint.h>     // For int16_t (for Encoder)

// --- Hardware Pins & Settings ---
#define GPS_SERIAL Serial1
#define GPS_BAUD_RATE 9600
#define TIME_PULSE_PIN 3
#define ANALOG_PIN_NUMBER 6 // Use Digital Pin 6 as Analog Input (ADC6 on many boards)
HardwareSerial &gpsSerial = GPS_SERIAL;

// --- NMEA Buffer ---
const int MAX_NMEA_BUFFER = 100; char nmeaBuffer[MAX_NMEA_BUFFER]; int bufferIndex = 0;

// --- NMEA/PPS Time Storage ---
volatile int nmeUtcYear=0; volatile int nmeUtcMonth=0; volatile int nmeUtcDay=0;
volatile int nmeUtcHour=0; volatile int nmeUtcMinute=0; volatile int nmeUtcSecond=0;
volatile bool nmeTimeValid=false; volatile unsigned long ppsTimestampMicros=0;
volatile int ppsUtcYear=0; volatile int ppsUtcMonth=0; volatile int ppsUtcDay=0;
volatile int ppsUtcHour=0; volatile int ppsUtcMinute=0; volatile int ppsUtcSecond=0;
volatile bool ppsTimeValid=false;

// --- Location Storage ---
volatile float latitude=0.0; volatile float longitude=0.0; volatile bool gpsFix=false;

// --- BLE Global Variables & Constants ---
enum conn_state_t {
    ST_BOOT, ST_SCAN, ST_CONNECT, ST_SERVICE_DISCOVER, ST_TEMP_CHAR_DISCOVER,
    ST_TIME_SYNC_CHAR_DISCOVER, ST_TIME_OFFSET_CHAR_DISCOVER,
    ST_ENABLE_INDICATIONS, ST_RECEIVE_DATA
};
const uint8_t thermometer_service_uuid[] = {0x09,0x18};
const sl_bt_uuid_16_t temp_measurement_characteristic_uuid = {.data={0x1C,0x2A}};
const uint8_t advertised_name[] = "CustomDataSender";
const uint8_t UUID_TIME_SYNC_CHAR[16] = {
    0x01, 0xf5, 0xd9, 0x46, 0xc3, 0x45, 0x59, 0x82, 0x71, 0x48, 0x02, 0x83, 0x02, 0x0d, 0xa8, 0xf0 };
const uint8_t UUID_TIME_OFFSET_CHAR[16] = {
    0x02, 0xf5, 0xd9, 0x46, 0xc3, 0x45, 0x59, 0x82, 0x71, 0x48, 0x02, 0x83, 0x02, 0x0d, 0xa8, 0xf0 };
uint8_t current_connection_handle = 0xFF;
uint32_t thermometer_service_handle = __UINT32_MAX__;
uint16_t temp_measurement_char_handle = 0xFFFF;
uint16_t time_sync_char_handle = 0xFFFF;
uint16_t time_offset_char_handle = 0xFFFF;
conn_state_t connection_state = ST_BOOT;


// *** Global Variables for Load Cell Data Collection (Server Time) ***
const int NUM_SAMPLES_PER_SECOND = 10;
uint16_t loadCellValues[NUM_SAMPLES_PER_SECOND];
bool valueReceived[NUM_SAMPLES_PER_SECOND]; // Tracks if server sent data for this slot
int valuesCollectedCount = 0; // Counts collected load cell values for its server second
uint32_t currentCollectionSecond = 0; // Server-timed second for load cell data

// ***** Global Variables for Analog Pin 6 Data Collection (Client GPS Time) *****
uint16_t analogPin6Values[NUM_SAMPLES_PER_SECOND];
bool pin6ValueReceivedThisSlot[NUM_SAMPLES_PER_SECOND]; // Tracks if client collected data for this slot
int pin6ValuesCollectedCount = 0; // Counts collected Pin6 values for the active client second

// --- Encoder Pins & Settings ---
#define ENCODER_1_PIN 14
#define ENCODER_2_PIN 13
#define ENCODER_SELECT_PIN 4

// --- Encoder and Wheel Properties ---
const float WHEEL_CIRCUMFERENCE_METERS = 0.6f;
const int TICKS_PER_REVOLUTION = 1;

// --- Global variables for Encoder ---
volatile int distance_ticks = 0;
volatile bool moving_forward = false;
volatile unsigned long last_tick_time_us = 0;
float distance_per_tick = 0.0f;

volatile bool newDataAvailable = false;
volatile unsigned long captured_tick_time_us = 0;
volatile bool captured_direction_forward = false;
volatile bool captured_direction_backward = false;
volatile int latest_captured_distance_ticks = 0;
volatile int16_t g_currentVelocityMmPs = 0;

// ***** Global Variables for Encoder Velocity Data Collection (Client GPS Time) *****
int16_t encoderVelocityValues[NUM_SAMPLES_PER_SECOND];
bool encoderValueReceivedThisSlot[NUM_SAMPLES_PER_SECOND];
int encoderValuesCollectedCount = 0; // Counts collected encoder values for the active client second
int startSecondEncoderTickCount = 0; // Stores ticks at the start of the client collection second

// Tracks the client second for which data is actively being collected
uint32_t activeClientCollectionUnixSecond = 0;
uint32_t lastLocationPrintSecond = 0;


// --- Forward Declarations ---
static bool find_complete_local_name_in_advertisement(sl_bt_evt_scanner_legacy_advertisement_report_t *response);
void sl_bt_on_event(sl_bt_msg_t *evt);
static void discover_time_sync_characteristic();
static void discover_time_offset_characteristic();
static void send_periodic_sync_data();
void readGpsSerial(); void parseNmeaSentence(char *sentence); float nmeaToDecimalDegrees(char *nmeaCoord, char hemisphere);
bool validateChecksum(char *sentence); byte hexCharToByte(char c); void ppsISR();
bool getPreciseTimestamp(unsigned long*, unsigned int*, unsigned int*, unsigned int*, unsigned int*, unsigned int*, unsigned long*);
bool getCurrentLocation(float *pLat, float *pLon);
int8_t get_offset();

static void resetPin6CollectionForNewSecond();
static void resetEncoderCollectionForNewSecond();
// static void resetLoadCellCollectionInternal(); // <<< REMOVED FORWARD DECLARATION
static void printConsolidatedDataPacket(uint32_t clientUnixSec);

// Encoder Function declarations
bool processEncoderUpdate(int16_t* pVelocity, int* pTicks, unsigned long* pIntervalUs, bool* pDirForward, bool* pDirBackward);
int getCurrentGlobalTickCount();
int16_t getCurrentVelocityMmPs();
void encoder1_ISR();
void encoder2_ISR();


#ifndef LED_BUILTIN_ACTIVE
#define LED_BUILTIN_ACTIVE HIGH
#endif
#ifndef LED_BUILTIN_INACTIVE
#define LED_BUILTIN_INACTIVE LOW
#endif
#ifndef app_assert_status
#define app_assert_status(sc) \
    do{if((sc)!=SL_STATUS_OK){Serial.print("BLE Assert Error: 0x");Serial.println(sc,HEX);while(1);}}while(0)
#endif

// ========================================================================
// SETUP
// ========================================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
    Serial.begin(115200); while (!Serial);
    Serial.println("BLE Client + NMEA/PPS + Sync + Consolidated Data Packet per Client Second");
    gpsSerial.begin(GPS_BAUD_RATE); memset(nmeaBuffer, 0, MAX_NMEA_BUFFER); bufferIndex = 0;
    Serial.print("GPS Listening on Serial1, Baud: "); Serial.println(GPS_BAUD_RATE);
    pinMode(TIME_PULSE_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(TIME_PULSE_PIN), ppsISR, RISING);
    Serial.print("PPS Listening on Pin: "); Serial.println(TIME_PULSE_PIN);

    pinMode(ANALOG_PIN_NUMBER, INPUT);

    currentCollectionSecond = 0;
    valuesCollectedCount = 0;
    memset(valueReceived, false, sizeof(valueReceived));
    memset(loadCellValues, 0, sizeof(loadCellValues));

    pin6ValuesCollectedCount = 0;
    memset(pin6ValueReceivedThisSlot, false, sizeof(pin6ValueReceivedThisSlot));
    memset(analogPin6Values, 0, sizeof(analogPin6Values));

    encoderValuesCollectedCount = 0;
    memset(encoderValueReceivedThisSlot, false, sizeof(encoderValueReceivedThisSlot));
    memset(encoderVelocityValues, 0, sizeof(encoderVelocityValues));
    startSecondEncoderTickCount = 0;

    activeClientCollectionUnixSecond = 0;
    lastLocationPrintSecond = 0;

    Serial.println("Initializing Encoder...");
    if (TICKS_PER_REVOLUTION > 0) {
        distance_per_tick = WHEEL_CIRCUMFERENCE_METERS / (float)TICKS_PER_REVOLUTION;
    } else {
        Serial.println("ERROR: TICKS_PER_REVOLUTION is zero or negative. Halting.");
        while(1);
    }
    Serial.print("Encoder: Distance per tick: "); Serial.print(distance_per_tick, 6); Serial.println(" m");
    pinMode(ENCODER_SELECT_PIN, OUTPUT);digitalWrite(ENCODER_SELECT_PIN, HIGH);
    pinMode(ENCODER_1_PIN, INPUT_PULLUP);
    pinMode(ENCODER_2_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), encoder1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), encoder2_ISR, FALLING);
    last_tick_time_us = micros();

    Serial.println("Initializing BLE Stack...");
}

// ========================================================================
// LOOP
// ========================================================================
void loop() {
    readGpsSerial();

    int16_t event_velocity_val; int event_ticks_val; unsigned long event_interval_val_us;
    bool event_dir_fwd_val, event_dir_bwd_val;
    processEncoderUpdate(&event_velocity_val, &event_ticks_val, &event_interval_val_us, &event_dir_fwd_val, &event_dir_bwd_val);

    unsigned long now_millis = millis();

    unsigned long gps_derived_year, gps_derived_microsecond;
    unsigned int gps_derived_month, gps_derived_day, gps_derived_hour, gps_derived_minute, gps_derived_second_part;
    uint32_t current_client_unix_seconds = 0;
    uint32_t current_client_milliseconds = 0;

    bool client_time_available = getPreciseTimestamp(&gps_derived_year, &gps_derived_month, &gps_derived_day,
                                                 &gps_derived_hour, &gps_derived_minute, &gps_derived_second_part,
                                                 &gps_derived_microsecond);

    if (client_time_available) {
        struct tm t_client_curr;
        t_client_curr.tm_year = gps_derived_year - 1900; t_client_curr.tm_mon = gps_derived_month - 1; t_client_curr.tm_mday = gps_derived_day;
        t_client_curr.tm_hour = gps_derived_hour; t_client_curr.tm_min = gps_derived_minute; t_client_curr.tm_sec = gps_derived_second_part;
        t_client_curr.tm_isdst = 0;
        time_t temp_unix_time = mktime(&t_client_curr);

        if (temp_unix_time != (time_t)-1) {
            current_client_unix_seconds = (uint32_t)temp_unix_time;
            current_client_milliseconds = gps_derived_microsecond / 1000;

            if (activeClientCollectionUnixSecond == 0 && current_client_unix_seconds != 0) {
                // This is the very first valid client second we are starting to collect for.
                activeClientCollectionUnixSecond = current_client_unix_seconds;
                resetPin6CollectionForNewSecond();
                resetEncoderCollectionForNewSecond();
                startSecondEncoderTickCount = getCurrentGlobalTickCount();
            }
            
            if (current_client_unix_seconds > activeClientCollectionUnixSecond) {
                // The previous second (activeClientCollectionUnixSecond) has just finished.
                // Print the consolidated data for the second that just completed.
                if (activeClientCollectionUnixSecond != 0) { // Ensure it was a valid second
                     printConsolidatedDataPacket(activeClientCollectionUnixSecond);
                }

                // Start collecting for the new current_client_unix_seconds
                activeClientCollectionUnixSecond = current_client_unix_seconds;

                resetPin6CollectionForNewSecond();
                resetEncoderCollectionForNewSecond();
                startSecondEncoderTickCount = getCurrentGlobalTickCount();
            }


            // If we have an active collection second, store data into bins
            if (activeClientCollectionUnixSecond != 0 && current_client_unix_seconds == activeClientCollectionUnixSecond) {
                int index = current_client_milliseconds / 100;

                if (index >= 0 && index < NUM_SAMPLES_PER_SECOND) {
                    // Store Analog Pin 6 Data
                    if (!pin6ValueReceivedThisSlot[index]) {
                        uint16_t pin6Value = analogRead(ANALOG_PIN_NUMBER);
                        analogPin6Values[index] = pin6Value;
                        pin6ValueReceivedThisSlot[index] = true;
                        pin6ValuesCollectedCount++;
                    }

                    // Store Encoder Velocity Data
                    if (!encoderValueReceivedThisSlot[index]) {
                        int16_t velocity = getCurrentVelocityMmPs();
                        encoderVelocityValues[index] = velocity;
                        encoderValueReceivedThisSlot[index] = true;
                        encoderValuesCollectedCount++;
                    }
                }
            }
        }
    }

    delay(1);

    static unsigned long lastGpsCheck = 0;
    if (now_millis - lastGpsCheck >= 5000) {
        lastGpsCheck = now_millis;
        unsigned long year_chk, us_chk; unsigned int mo_chk, d_chk, h_chk, mi_chk, s_chk;
        bool timeAvailable_chk = getPreciseTimestamp(&year_chk, &mo_chk, &d_chk, &h_chk, &mi_chk, &s_chk, &us_chk);
        if(!timeAvailable_chk){
            Serial.println("Precise UTC Time (5s Check): Not available");
        }
    }

    static unsigned long lastSyncSend = 0;
    if (connection_state == ST_RECEIVE_DATA && current_connection_handle != 0xFF &&
        time_sync_char_handle != 0xFFFF && time_offset_char_handle != 0xFFFF &&
        (now_millis - lastSyncSend >= 10000)) {
        lastSyncSend = now_millis;
        send_periodic_sync_data();
    }
}

// ========================================================================
// ENCODER SPECIFIC FUNCTIONS (unchanged)
// ========================================================================
void encoder1_ISR() { int encoder_1_state = digitalRead(ENCODER_1_PIN); if (encoder_1_state) { moving_forward = false; } else { moving_forward = true; } }
void encoder2_ISR() { unsigned long current_event_capture_time = micros(); int encoder_1_state_on_falling_edge = digitalRead(ENCODER_1_PIN); bool direction_confirmed_this_tick_forward = false; bool direction_confirmed_this_tick_backward = false; if (moving_forward) { if (encoder_1_state_on_falling_edge) { distance_ticks++; direction_confirmed_this_tick_forward = true; } } else { if (!encoder_1_state_on_falling_edge) { distance_ticks--; direction_confirmed_this_tick_backward = true; } } if (direction_confirmed_this_tick_forward || direction_confirmed_this_tick_backward) { if (!newDataAvailable) { captured_tick_time_us = current_event_capture_time; captured_direction_forward = direction_confirmed_this_tick_forward; captured_direction_backward = direction_confirmed_this_tick_backward; latest_captured_distance_ticks = distance_ticks; newDataAvailable = true; } } }
int getCurrentGlobalTickCount() { int ticks_snapshot; noInterrupts(); ticks_snapshot = distance_ticks; interrupts(); return ticks_snapshot; }
int16_t getCurrentVelocityMmPs() { return g_currentVelocityMmPs; }
bool processEncoderUpdate(int16_t* pVelocity, int* pTicks, unsigned long* pIntervalUs, bool* pDirForward, bool* pDirBackward) { if (newDataAvailable) { unsigned long current_event_time_us_local; unsigned long previous_event_time_us_local; bool dir_fwd_local, dir_bwd_local; int ticks_local; noInterrupts(); current_event_time_us_local = captured_tick_time_us; previous_event_time_us_local = last_tick_time_us; dir_fwd_local = captured_direction_forward; dir_bwd_local = captured_direction_backward; ticks_local = latest_captured_distance_ticks; newDataAvailable = false; interrupts(); last_tick_time_us = current_event_time_us_local; *pDirForward = dir_fwd_local; *pDirBackward = dir_bwd_local; *pTicks = ticks_local; *pIntervalUs = current_event_time_us_local - previous_event_time_us_local; int16_t calculated_vel = 0; if (*pIntervalUs > 0 && (dir_fwd_local || dir_bwd_local)) { float speed_mps = (distance_per_tick * 1000000.0f) / (float)(*pIntervalUs); float signed_velocity_mmps_float = (dir_fwd_local ? 1.0f : -1.0f) * speed_mps * 1000.0f; if (isnan(signed_velocity_mmps_float) || isinf(signed_velocity_mmps_float)) { calculated_vel = 0; } else { float rounded_velocity = round(signed_velocity_mmps_float); if (rounded_velocity > 32767.0f) calculated_vel = 32767; else if (rounded_velocity < -32768.0f) calculated_vel = -32768; else calculated_vel = (int16_t)rounded_velocity; } } else { calculated_vel = 0; } *pVelocity = calculated_vel; noInterrupts(); g_currentVelocityMmPs = calculated_vel; interrupts(); return true; } return false; }

// ========================================================================
// get_offset & send_periodic_sync_data (unchanged)
// ========================================================================
int8_t get_offset() { return 25;}
static void send_periodic_sync_data() { unsigned long year, microsecond; unsigned int month, day, hour, minute, second_part; bool timeAvailable = getPreciseTimestamp(&year, &month, &day, &hour, &minute, &second_part, &microsecond); if (timeAvailable) { struct tm t; t.tm_year = year - 1900; t.tm_mon = month - 1; t.tm_mday = day; t.tm_hour = hour; t.tm_min = minute; t.tm_sec = second_part; t.tm_isdst = 0; time_t unix_time = mktime(&t); uint32_t ms_part = microsecond / 1000; if (unix_time != (time_t)-1) { uint8_t time_buffer[8]; time_buffer[0] = (uint8_t)(unix_time >> 0); time_buffer[1] = (uint8_t)(unix_time >> 8); time_buffer[2] = (uint8_t)(unix_time >> 16); time_buffer[3] = (uint8_t)(unix_time >> 24); time_buffer[4] = (uint8_t)(ms_part >> 0); time_buffer[5] = (uint8_t)(ms_part >> 8); time_buffer[6] = (uint8_t)(ms_part >> 16); time_buffer[7] = (uint8_t)(ms_part >> 24); sl_status_t sc_time = sl_bt_gatt_write_characteristic_value_without_response( current_connection_handle, time_sync_char_handle, sizeof(time_buffer), time_buffer, NULL); if (sc_time != SL_STATUS_OK) { Serial.print("BLE: Failed Time+MS Write Wo Rsp: 0x"); Serial.println(sc_time, HEX); } } else { Serial.println("BLE: Error converting GPS time to Unix timestamp."); } int8_t offset_val = get_offset(); uint8_t offset_buffer[1] = {(uint8_t)offset_val}; sl_status_t sc_offset = sl_bt_gatt_write_characteristic_value_without_response( current_connection_handle, time_offset_char_handle, sizeof(offset_buffer), offset_buffer, NULL); if (sc_offset != SL_STATUS_OK) { Serial.print("BLE: Failed Offset Write Wo Rsp: 0x"); Serial.println(sc_offset, HEX); } } else { Serial.println("BLE: Cannot send sync data, GPS time not valid."); } }


// ***** Function to Reset Analog Pin 6 Data Collection Variables for a New Second *****
static void resetPin6CollectionForNewSecond() {
    pin6ValuesCollectedCount = 0;
    memset(analogPin6Values, 0, sizeof(analogPin6Values)); 
    memset(pin6ValueReceivedThisSlot, false, sizeof(pin6ValueReceivedThisSlot));
}

// ***** Function to Reset Encoder Data Collection Variables for a New Second *****
static void resetEncoderCollectionForNewSecond() {
    encoderValuesCollectedCount = 0;
    memset(encoderVelocityValues, 0, sizeof(encoderVelocityValues)); 
    memset(encoderValueReceivedThisSlot, false, sizeof(encoderValueReceivedThisSlot));
}

// ***** Function to Print Consolidated Data Packet for a Specific Client Unix Second *****
static void printConsolidatedDataPacket(uint32_t clientUnixSec) {
    float currentLat, currentLon;
    bool locationAvailable = getCurrentLocation(&currentLat, &currentLon);

    Serial.print("UnixS:");
    Serial.print(clientUnixSec);

    Serial.print(",Lat:");
    if (locationAvailable) {
        Serial.print(currentLat, 6);
    } else {
        Serial.print("N/A");
    }
    Serial.print(",Lon:");
    if (locationAvailable) {
        Serial.print(currentLon, 6);
    } else {
        Serial.print("N/A");
    }

    Serial.print(",LC_ServS:");
    Serial.print(currentCollectionSecond); 
    Serial.print(",LC_Vals:[");
  
    for (int i = 0; i < NUM_SAMPLES_PER_SECOND; i++) {
        if (valueReceived[i]) {
            Serial.print(loadCellValues[i]);
        } else {
            Serial.print("---");
        }
        if (i < NUM_SAMPLES_PER_SECOND - 1) {
            Serial.print(";");
        }
    }
    Serial.print("]");

    Serial.print(",A6_Vals:[");
    for (int i = 0; i < NUM_SAMPLES_PER_SECOND; i++) {
        if (pin6ValueReceivedThisSlot[i]) { 
            Serial.print(analogPin6Values[i]);
        } else {
            Serial.print("---");
        }
        if (i < NUM_SAMPLES_PER_SECOND - 1) {
            Serial.print(";");
        }
    }
    Serial.print("]");

    Serial.print(",Enc_StartT:");
    Serial.print(startSecondEncoderTickCount);

    Serial.print(",Enc_Vels:[");
    for (int i = 0; i < NUM_SAMPLES_PER_SECOND; i++) {
        if (encoderValueReceivedThisSlot[i]) { 
            Serial.print(encoderVelocityValues[i]);
        } else {
            Serial.print("---");
        }
        if (i < NUM_SAMPLES_PER_SECOND - 1) {
            Serial.print(";");
        }
    }
    Serial.print(" mm/s]");

    Serial.println(); 
}


// ========================================================================
// BLE Functions
// ========================================================================
static void discover_time_sync_characteristic() { Serial.println("BLE: Discovering Time Sync characteristic..."); sl_status_t sc = sl_bt_gatt_discover_characteristics_by_uuid(current_connection_handle, thermometer_service_handle, sizeof(UUID_TIME_SYNC_CHAR), UUID_TIME_SYNC_CHAR); app_assert_status(sc); connection_state = ST_TIME_SYNC_CHAR_DISCOVER; }
static void discover_time_offset_characteristic() { Serial.println("BLE: Discovering Time Offset characteristic..."); sl_status_t sc = sl_bt_gatt_discover_characteristics_by_uuid(current_connection_handle, thermometer_service_handle, sizeof(UUID_TIME_OFFSET_CHAR), UUID_TIME_OFFSET_CHAR); app_assert_status(sc); connection_state = ST_TIME_OFFSET_CHAR_DISCOVER; }

void sl_bt_on_event(sl_bt_msg_t *evt) {
    sl_status_t sc;
    switch (SL_BT_MSG_ID(evt->header)) {
        case sl_bt_evt_system_boot_id:
            Serial.println("BLE: System Booted."); sc = sl_bt_scanner_set_parameters(sl_bt_scanner_scan_mode_active, 16, 16); app_assert_status(sc); sc = sl_bt_scanner_start(sl_bt_scanner_scan_phy_1m, sl_bt_scanner_discover_generic); app_assert_status(sc); Serial.println("BLE: Started scanning..."); connection_state = ST_SCAN; break;

        case sl_bt_evt_scanner_legacy_advertisement_report_id:
            if (connection_state == ST_SCAN) { if (find_complete_local_name_in_advertisement(&(evt->data.evt_scanner_legacy_advertisement_report))) { Serial.println("BLE: Target device found"); sc = sl_bt_scanner_stop(); app_assert_status(sc); sc = sl_bt_connection_open(evt->data.evt_scanner_legacy_advertisement_report.address, evt->data.evt_scanner_legacy_advertisement_report.address_type, sl_bt_gap_phy_1m, &current_connection_handle); if (sc == SL_STATUS_OK) { connection_state = ST_CONNECT; Serial.print("BLE: Connection pending, Handle: "); Serial.println(current_connection_handle); } else { connection_state = ST_SCAN; sc = sl_bt_scanner_start(sl_bt_scanner_scan_phy_1m, sl_bt_scanner_discover_generic); if(sc != SL_STATUS_OK && sc != SL_STATUS_INVALID_STATE) app_assert_status(sc); } } } break;

        case sl_bt_evt_connection_opened_id:
            if (current_connection_handle == evt->data.evt_connection_opened.connection){ Serial.print("BLE: Connection opened successfully, Handle: "); Serial.println(current_connection_handle); digitalWrite(LED_BUILTIN, LED_BUILTIN_ACTIVE); sc = sl_bt_gatt_discover_primary_services_by_uuid(current_connection_handle, sizeof(thermometer_service_uuid), thermometer_service_uuid); app_assert_status(sc); connection_state = ST_SERVICE_DISCOVER; } break;

        case sl_bt_evt_connection_closed_id:
            Serial.print("BLE: Connection closed, handle: "); Serial.print(evt->data.evt_connection_closed.connection); Serial.print(", reason: 0x"); Serial.println(evt->data.evt_connection_closed.reason, HEX);
            digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
            if (current_connection_handle == evt->data.evt_connection_closed.connection) {
                current_connection_handle = 0xFF;
                thermometer_service_handle = __UINT32_MAX__;
                temp_measurement_char_handle = 0xFFFF;
                time_sync_char_handle = 0xFFFF;
                time_offset_char_handle = 0xFFFF;
                connection_state = ST_SCAN;

                currentCollectionSecond = 0; valuesCollectedCount = 0; memset(valueReceived, false, sizeof(valueReceived)); memset(loadCellValues, 0, sizeof(loadCellValues)); // Also clear load cell values on disconnect
                
                activeClientCollectionUnixSecond = 0; 
                resetPin6CollectionForNewSecond();    
                resetEncoderCollectionForNewSecond(); 
                startSecondEncoderTickCount = 0;

                lastLocationPrintSecond = 0;

                sc = sl_bt_scanner_start(sl_bt_scanner_scan_phy_1m, sl_bt_scanner_discover_generic);
                if (sc != SL_STATUS_OK && sc != SL_STATUS_INVALID_STATE) app_assert_status(sc);
                Serial.println("BLE: Restarted scanning...");
            }
            break;

        case sl_bt_evt_gatt_service_id:
             if (connection_state == ST_SERVICE_DISCOVER && evt->data.evt_gatt_service.connection == current_connection_handle && memcmp(evt->data.evt_gatt_service.uuid.data, thermometer_service_uuid, sizeof(thermometer_service_uuid)) == 0) { Serial.println("BLE: GATT service 0x1809 discovered"); thermometer_service_handle = evt->data.evt_gatt_service.service; } break;

        case sl_bt_evt_gatt_characteristic_id:
            if(evt->data.evt_gatt_characteristic.connection == current_connection_handle) { if (connection_state == ST_TEMP_CHAR_DISCOVER) { if (evt->data.evt_gatt_characteristic.uuid.len == 2 && memcmp(evt->data.evt_gatt_characteristic.uuid.data, temp_measurement_characteristic_uuid.data, 2) == 0) { temp_measurement_char_handle = evt->data.evt_gatt_characteristic.characteristic; Serial.println(" -> Matched Characteristic 0x2A1C (Indications)"); } } else if (connection_state == ST_TIME_SYNC_CHAR_DISCOVER) { if (evt->data.evt_gatt_characteristic.uuid.len == 16 && memcmp(evt->data.evt_gatt_characteristic.uuid.data, UUID_TIME_SYNC_CHAR, 16) == 0) { time_sync_char_handle = evt->data.evt_gatt_characteristic.characteristic; Serial.println(" -> Matched Time Sync Characteristic (Write)"); } } else if (connection_state == ST_TIME_OFFSET_CHAR_DISCOVER) { if (evt->data.evt_gatt_characteristic.uuid.len == 16 && memcmp(evt->data.evt_gatt_characteristic.uuid.data, UUID_TIME_OFFSET_CHAR, 16) == 0) { time_offset_char_handle = evt->data.evt_gatt_characteristic.characteristic; Serial.println(" -> Matched Time Offset Characteristic (Write)"); } } } break;

        case sl_bt_evt_gatt_procedure_completed_id:
            if(evt->data.evt_gatt_procedure_completed.connection == current_connection_handle) { if (evt->data.evt_gatt_procedure_completed.result != SL_STATUS_OK) { Serial.print("BLE: GATT Procedure failed! State: "); Serial.print(connection_state); sl_bt_connection_close(current_connection_handle); break; } switch(connection_state) { case ST_SERVICE_DISCOVER: if (thermometer_service_handle != __UINT32_MAX__) { Serial.println("BLE: Discovering char 0x2A1C..."); sc = sl_bt_gatt_discover_characteristics_by_uuid(current_connection_handle, thermometer_service_handle, sizeof(temp_measurement_characteristic_uuid.data), temp_measurement_characteristic_uuid.data); app_assert_status(sc); connection_state = ST_TEMP_CHAR_DISCOVER; } break; case ST_TEMP_CHAR_DISCOVER: if (temp_measurement_char_handle != 0xFFFF) { discover_time_sync_characteristic(); } break; case ST_TIME_SYNC_CHAR_DISCOVER: if (time_sync_char_handle != 0xFFFF) { discover_time_offset_characteristic(); } break; case ST_TIME_OFFSET_CHAR_DISCOVER: if (time_offset_char_handle != 0xFFFF) { Serial.println("BLE: Discovery complete. Enabling indications..."); sc = sl_bt_gatt_set_characteristic_notification(current_connection_handle, temp_measurement_char_handle, sl_bt_gatt_indication); app_assert_status(sc); connection_state = ST_ENABLE_INDICATIONS; } break; case ST_ENABLE_INDICATIONS: Serial.println("BLE: Indications enabled. Ready for data & periodic sync."); connection_state = ST_RECEIVE_DATA; break; default: break; } } break;

        case sl_bt_evt_gatt_characteristic_value_id: { // Handle incoming indications (LoadCell data)
            sl_bt_evt_gatt_characteristic_value_t *read_evt = &evt->data.evt_gatt_characteristic_value;
            if(read_evt->connection == current_connection_handle &&
               read_evt->characteristic == temp_measurement_char_handle &&
               read_evt->att_opcode == sl_bt_gatt_handle_value_indication) {

                uint8_t* data = read_evt->value.data; uint8_t length = read_evt->value.len;
                if (length == 10) { // Expected data format: 4b UnixS, 4b ServerMS, 2b ADC
                    uint32_t server_unix_seconds_from_packet = (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
                    uint32_t server_milliseconds = (uint32_t)data[4] | (uint32_t)data[5] << 8 | (uint32_t)data[6] << 16 | (uint32_t)data[7] << 24;
                    uint16_t adcValue = (uint16_t)data[8] | (uint16_t)data[9] << 8;

                    // <<< REVISED Load Cell Data Handling Logic >>>
                    if (currentCollectionSecond == 0 && server_unix_seconds_from_packet != 0) {
                        // First packet ever for load cell, or after a full system/BLE reset.
                        currentCollectionSecond = server_unix_seconds_from_packet;
                        valuesCollectedCount = 0;
                        memset(valueReceived, false, sizeof(valueReceived));
                        // Optional: Clear loadCellValues if you want to be certain no old data lingers visually
                        // memset(loadCellValues, 0, sizeof(loadCellValues));
                    }

                    if (server_unix_seconds_from_packet > currentCollectionSecond) {
                        // Server second has advanced. Data for the 'old' currentCollectionSecond is now finalized.
                        // Update to the new server second and reset collection variables FOR THE NEW SECOND.
                        currentCollectionSecond = server_unix_seconds_from_packet;
                        valuesCollectedCount = 0; 
                        memset(valueReceived, false, sizeof(valueReceived)); 
                        // Optional: memset(loadCellValues, 0, sizeof(loadCellValues)); 
                    } else if (server_unix_seconds_from_packet < currentCollectionSecond) {
                        // Packet is for a previous server second (already processed or too late).
                        // Serial.print("WARN LoadCell: Ignoring old packet for server second: "); Serial.println(server_unix_seconds_from_packet);
                        // Send confirmation for the indication and stop processing this packet for data.
                        sc = sl_bt_gatt_send_characteristic_confirmation(current_connection_handle);
                        if (sc != SL_STATUS_OK) { Serial.print("BLE: Failed indication confirm (old LC): 0x"); Serial.println(sc, HEX); }
                        break; 
                    }

                    // If the packet's server second matches the one we are currently collecting for:
                    if (server_unix_seconds_from_packet == currentCollectionSecond) {
                        int index = server_milliseconds / 100;
                        if (index >= 0 && index < NUM_SAMPLES_PER_SECOND) {
                            if (!valueReceived[index]) { 
                                loadCellValues[index] = adcValue;
                                valueReceived[index] = true;
                                valuesCollectedCount++;
                                // No reset of valueReceived or valuesCollectedCount here even if count == 10.
                                // The data for currentCollectionSecond remains until currentCollectionSecond itself advances.
                            }
                        }
                    }
                    // <<< END REVISED Load Cell Data Handling Logic >>>
                } else { 
                    Serial.print("BLE Indication Recv: Unexpected length: "); Serial.println(length); 
                }
                // Send confirmation for the indication
                sc = sl_bt_gatt_send_characteristic_confirmation(current_connection_handle);
                if (sc != SL_STATUS_OK) {
                    Serial.print("BLE: Failed indication confirm: 0x"); Serial.println(sc, HEX);
                }
            }
        } break; 

        default: break;
    }
}


// ========================================================================
// NMEA/PPS Functions (Corrected ParseNmeaSentence)
// ========================================================================
void ppsISR() { unsigned long cM = micros(); if (nmeTimeValid) { ppsTimestampMicros = cM; noInterrupts(); ppsUtcYear=nmeUtcYear; ppsUtcMonth=nmeUtcMonth; ppsUtcDay=nmeUtcDay; ppsUtcHour=nmeUtcHour; ppsUtcMinute=nmeUtcMinute; ppsUtcSecond=nmeUtcSecond; interrupts(); ppsTimeValid = true; } else { ppsTimeValid = false; } }
bool getPreciseTimestamp(unsigned long *pY,unsigned int *pMo,unsigned int *pD,unsigned int *pH,unsigned int *pMi,unsigned int *pS,unsigned long *pUs) { unsigned long fCM=micros(); unsigned long lPM; int y,mo,d,h,mi,s; bool v; noInterrupts(); lPM=ppsTimestampMicros; y=ppsUtcYear; mo=ppsUtcMonth; d=ppsUtcDay; h=ppsUtcHour; mi=ppsUtcMinute; s=ppsUtcSecond; v=ppsTimeValid; interrupts(); if (!v) return false; unsigned long uSP; if (fCM >= lPM) { uSP = fCM - lPM; } else { uSP = (0xFFFFFFFF - lPM) + fCM + 1; } if (uSP >= 1000000) { ppsTimeValid = false; return false; } *pY=y; *pMo=mo; *pD=d; *pH=h; *pMi=mi; *pS=s; *pUs=uSP; return true; }
bool getCurrentLocation(float *pLat, float *pLon) { bool vF; float cLat, cLon; noInterrupts(); vF=gpsFix; cLat=latitude; cLon=longitude; interrupts(); if(vF){*pLat=cLat; *pLon=cLon; return true;}else{return false;} }
void readGpsSerial() { while(gpsSerial.available()>0){ char c=gpsSerial.read(); if(c=='$'||bufferIndex>=MAX_NMEA_BUFFER-1){bufferIndex=0; memset(nmeaBuffer,0,MAX_NMEA_BUFFER);} if(c!='\n'&&c!='\r'&&bufferIndex<MAX_NMEA_BUFFER-1){nmeaBuffer[bufferIndex++]=c;} if(c=='\n'){nmeaBuffer[bufferIndex]=0; if(nmeaBuffer[0]=='$'){parseNmeaSentence(nmeaBuffer);} bufferIndex=0; memset(nmeaBuffer,0,MAX_NMEA_BUFFER);}}}
void parseNmeaSentence(char *sentence) {
    if(strncmp(sentence+1,"GNRMC",5)==0||strncmp(sentence+1,"GPRMC",5)==0){
        if(!validateChecksum(sentence)) return;
        char *f[20]; int fI=0; f[fI++]=sentence; char* p=sentence;
        while((p=strchr(p,','))!=NULL&&fI<20){*p=0; f[fI++]=++p;}
        p=strchr(f[fI-1],'*'); if(p!=NULL) *p=0;

        if(fI>=10){
            int d_val=0,mo_val=0,y_val=0,h_val=0,mi_val=0,s_val=0;
            bool fix_status_val=false;
            float lat_val_nmea = 0.0, lon_val_nmea = 0.0; // <<< Corrected from previous error
            bool pOk=true;

            if(strlen(f[2])>0){fix_status_val=(f[2][0]=='A');}else pOk=false;
            if(fix_status_val){
                if(strlen(f[1])>=6){char hh[3]={f[1][0],f[1][1],0},mm[3]={f[1][2],f[1][3],0},ss[3]={f[1][4],f[1][5],0}; h_val=atoi(hh);mi_val=atoi(mm);s_val=atoi(ss);}else pOk=false;
                if(pOk&&strlen(f[9])==6){char d_s[3]={f[9][0],f[9][1],0},m_s[3]={f[9][2],f[9][3],0},y_s[3]={f[9][4],f[9][5],0}; d_val=atoi(d_s);mo_val=atoi(m_s);y_val=atoi(y_s)+2000;}else pOk=false;
                if(pOk&&strlen(f[3])>0&&strlen(f[4])>0){lat_val_nmea=nmeaToDecimalDegrees(f[3],f[4][0]);}else pOk=false;
                if(pOk&&strlen(f[5])>0&&strlen(f[6])>0){lon_val_nmea=nmeaToDecimalDegrees(f[5],f[6][0]);}else pOk=false;
                if(pOk){
                    noInterrupts();
                    nmeUtcYear=y_val;nmeUtcMonth=mo_val;nmeUtcDay=d_val;nmeUtcHour=h_val;nmeUtcMinute=mi_val;nmeUtcSecond=s_val;nmeTimeValid=true;
                    latitude=lat_val_nmea;longitude=lon_val_nmea;gpsFix=true;
                    interrupts();
                }else{noInterrupts();nmeTimeValid=false;gpsFix=false;interrupts();}
            }else{noInterrupts();nmeTimeValid=false;gpsFix=false;interrupts();}
        }else{noInterrupts();nmeTimeValid=false;gpsFix=false;interrupts();}
    }
}
float nmeaToDecimalDegrees(char *nC, char hem) { if(strlen(nC)==0)return 0.0; float rC=atof(nC); int deg=(int)(rC/100.0); float min_val=rC-(deg*100.0); float dD=deg+(min_val/60.0); if(hem=='S'||hem=='W')dD=-dD; else if(hem!='N'&&hem!='E')return 0.0; return dD;}
byte hexCharToByte(char c) { if(c>='0'&&c<='9')return c-'0'; if(c>='A'&&c<='F')return c-'A'+10; if(c>='a'&&c<='f')return c-'a'+10; return 255; }
bool validateChecksum(char *sentence) { if(sentence[0]!='$')return false; char *cs_ptr=strchr(sentence,'*'); if(cs_ptr==NULL||strlen(cs_ptr)<3)return false; byte calc_cs=0; for(int i=1;sentence[i]!='*'&&sentence[i]!=0;i++)calc_cs^=sentence[i]; byte hi=hexCharToByte(*(cs_ptr+1)); byte lo=hexCharToByte(*(cs_ptr+2)); if(hi==255||lo==255)return false; byte recv_cs=(hi<<4)+lo; return calc_cs==recv_cs; }
static bool find_complete_local_name_in_advertisement(sl_bt_evt_scanner_legacy_advertisement_report_t *response) { int i=0; bool f=false; uint8_t nL=strlen((const char*)advertised_name); while(i<(response->data.len-1)){ uint8_t adL=response->data.data[i]; if(adL==0||(i+adL)>=response->data.len)break; uint8_t adT=response->data.data[i+1]; if(adT==0x09){if((adL-1)==nL){if(memcmp(response->data.data+i+2, advertised_name, nL)==0){f=true; break;}}} i=i+adL+1; } return f; }