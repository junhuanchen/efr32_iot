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
 * - ***ADDED: Prints raw Analog Pin 6 value + Client Time immediately on read.***
 * - Prints collected arrays when full or on second rollover (with double-print fix).
 * - **Prints Location once per client GPS second.**
 * - Prints client's receive timestamp alongside server's sensor timestamp for context (optional).
 * - Periodically prints detailed GPS status to Serial Monitor.
 */

#include <Arduino.h>
#include <sl_bt_api.h> // Silabs BLE API
#include <stdio.h>      // For sprintf/printf
#include <string.h>     // For strncmp, strlen, strchr, memset
#include <stdlib.h>     // For atoi, atof
#include <time.h>       // For time_t, tm struct, mktime()

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
enum conn_state_t { // State machine simplified for periodic writes
    ST_BOOT, ST_SCAN, ST_CONNECT, ST_SERVICE_DISCOVER, ST_TEMP_CHAR_DISCOVER,
    ST_TIME_SYNC_CHAR_DISCOVER, ST_TIME_OFFSET_CHAR_DISCOVER, // Discover all needed handles
    ST_ENABLE_INDICATIONS,      // Enable indications
    ST_RECEIVE_DATA        // Operational state (receive indications, send periodic writes)
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
uint16_t temp_measurement_char_handle = 0xFFFF; // Indication OUT
uint16_t time_sync_char_handle = 0xFFFF;      // Time+MS IN (8 bytes)
uint16_t time_offset_char_handle = 0xFFFF;    // Offset IN (1 byte)
conn_state_t connection_state = ST_BOOT;


// *** Global Variables for Load Cell Data Collection (Server Time) ***
const int NUM_SAMPLES_PER_SECOND = 10;
uint16_t loadCellValues[NUM_SAMPLES_PER_SECOND];
bool valueReceived[NUM_SAMPLES_PER_SECOND];
int valuesCollectedCount = 0;
uint32_t currentCollectionSecond = 0;
// *** END Load Cell ***

// ***** REVISED: Global Variables for Analog Pin 6 Data Collection (Client GPS Time) *****
uint16_t analogPin6Values[NUM_SAMPLES_PER_SECOND];
bool pin6ValueReceivedThisSlot[NUM_SAMPLES_PER_SECOND];
int pin6ValuesCollectedCount = 0;
uint32_t currentPin6CollectionSecond = 0;
// Flag to prevent double printing Pin 6 data
bool pin6DataPrinted = false;
// Tracker for periodic location print
uint32_t lastLocationPrintSecond = 0;
// ***** END REVISED *****


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
static void printAndResetCollection(); // Existing for LoadCell (Server Time)
static void printAndResetPin6Collection(); // For Pin 6 (Client Time)


// Define LED states & assert macro
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
    Serial.println("BLE Client + NMEA/PPS + Sync + Collections + Location Print + Pin 6 Array + Live Pin 6 Print"); // Title updated
    gpsSerial.begin(GPS_BAUD_RATE); memset(nmeaBuffer, 0, MAX_NMEA_BUFFER); bufferIndex = 0;
    Serial.print("GPS Listening on Serial1, Baud: "); Serial.println(GPS_BAUD_RATE);
    pinMode(TIME_PULSE_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(TIME_PULSE_PIN), ppsISR, RISING);
    Serial.print("PPS Listening on Pin: "); Serial.println(TIME_PULSE_PIN);

    pinMode(ANALOG_PIN_NUMBER, INPUT); // Set Pin 6 as Input

    // Initialize Load Cell Collection Variables (Server Time)
    currentCollectionSecond = 0;
    valuesCollectedCount = 0;
    memset(valueReceived, false, sizeof(valueReceived));
    memset(loadCellValues, 0, sizeof(loadCellValues));

    // Initialize Pin 6 Collection Variables (Client Time)
    currentPin6CollectionSecond = 0;
    pin6ValuesCollectedCount = 0;
    memset(pin6ValueReceivedThisSlot, false, sizeof(pin6ValueReceivedThisSlot));
    memset(analogPin6Values, 0, sizeof(analogPin6Values));
    pin6DataPrinted = false; // Initialize print flag
    lastLocationPrintSecond = 0; // Initialize location print tracker


    Serial.println("Initializing BLE Stack...");
}

// ========================================================================
// LOOP
// ========================================================================
void loop() {
    readGpsSerial(); // Poll GPS

    // Get current time once for this loop iteration
    unsigned long now_millis = millis(); // Use Arduino millis() for general timing checks

    // ***** Analog Pin 6 Reading/Binning and Location Print Section *****
    unsigned long pin6_year, pin6_microsecond;
    unsigned int pin6_month, pin6_day, pin6_hour, pin6_minute, pin6_second_part;
    uint32_t pin6_client_unix_seconds = 0;
    uint32_t pin6_client_milliseconds = 0;

    bool pin6_time_available = getPreciseTimestamp(&pin6_year, &pin6_month, &pin6_day,
                                                  &pin6_hour, &pin6_minute, &pin6_second_part,
                                                  &pin6_microsecond);

    if (pin6_time_available) {
        struct tm t_pin6;
        t_pin6.tm_year = pin6_year - 1900; t_pin6.tm_mon = pin6_month - 1; t_pin6.tm_mday = pin6_day;
        t_pin6.tm_hour = pin6_hour; t_pin6.tm_min = pin6_minute; t_pin6.tm_sec = pin6_second_part;
        t_pin6.tm_isdst = 0;
        time_t temp_unix_time = mktime(&t_pin6);

        if (temp_unix_time != (time_t)-1) {
            pin6_client_unix_seconds = (uint32_t)temp_unix_time;
            pin6_client_milliseconds = pin6_microsecond / 1000;

            // --- Periodic Location Print (Once per Client GPS Second) ---
            //if (pin6_client_unix_seconds != lastLocationPrintSecond) {
            //    float currentLat, currentLon;
            //    bool locationAvailable = getCurrentLocation(&currentLat, &currentLon);
            //    if (locationAvailable) {
            //        char lBuf[64]; char laS[12]; char loS[12];
            //        dtostrf(currentLat, 9, 6, laS); dtostrf(currentLon, 10, 6, loS);
            //        sprintf(lBuf, "Location (GPS): Lat=%s, Lon=%s", laS, loS);
            //        Serial.println(lBuf);
            //    } else {
            //        Serial.println("Location (GPS): No Fix");
            //    }
            //    lastLocationPrintSecond = pin6_client_unix_seconds; // Update tracker
            //}
            // --- End Location Print ---


            // --- Pin 6 Binning Logic (Client Time - Array collection + DP Fix) ---
            // 1. Check for Client Second Rollover for Pin 6
            if (pin6_client_unix_seconds > currentPin6CollectionSecond) {
                // Check if data exists AND if it wasn't already printed by count
                if (pin6ValuesCollectedCount > 0 && !pin6DataPrinted) {
                    printAndResetPin6Collection(); // Print previous partial second
                }
                // Start collecting for the new client second
                currentPin6CollectionSecond = pin6_client_unix_seconds;
                pin6ValuesCollectedCount = 0; // Reset counter
                memset(pin6ValueReceivedThisSlot, false, sizeof(pin6ValueReceivedThisSlot)); // Reset flags
                pin6DataPrinted = false; // Reset print flag for the new second
            } else if (currentPin6CollectionSecond == 0 && pin6_client_unix_seconds != 0) {
                 // First valid time reading
                currentPin6CollectionSecond = pin6_client_unix_seconds;
                pin6ValuesCollectedCount = 0;
                memset(pin6ValueReceivedThisSlot, false, sizeof(pin6ValueReceivedThisSlot));
                pin6DataPrinted = false;
            }

            // 2. Store Pin 6 Data
            if (pin6_client_unix_seconds == currentPin6CollectionSecond) {
                int index = pin6_client_milliseconds / 100; // Determine 100ms slot index
                if (index >= 0 && index < NUM_SAMPLES_PER_SECOND) {
                    // Check if we already have data for this slot
                    if (!pin6ValueReceivedThisSlot[index]) {
                        uint16_t pin6Value = analogRead(ANALOG_PIN_NUMBER); // Read value

                        // ***** START: ADDED PRINT STATEMENT *****
                        // Print the raw analog value and the client timestamp when it was read
                        char printBuf[128]; // Buffer for formatted output
                        //sprintf(printBuf, "Analog Pin 6 Read: ClientTime=[%lus %lums] Pin6_ADC=%u",
                        //        (unsigned long)pin6_client_unix_seconds, // Cast to unsigned long for %lu format specifier
                        //        (unsigned long)pin6_client_milliseconds, // Cast to unsigned long for %lu format specifier
                        //        pin6Value);                            // %u for uint16_t
                        //Serial.println(printBuf);
                        // ***** END: ADDED PRINT STATEMENT *****

                        analogPin6Values[index] = pin6Value; // Store value
                        pin6ValueReceivedThisSlot[index] = true; // Mark slot received
                        pin6ValuesCollectedCount++; // Increment count

                        // 3. Check if collection complete
                        if (pin6ValuesCollectedCount == NUM_SAMPLES_PER_SECOND) {
                            printAndResetPin6Collection(); // Print the full array
                            pin6DataPrinted = true; // Set flag: printed because full
                        }
                    } // else: Already have data for this slot
                } // else: Invalid index
            }
            // --- End Pin 6 Binning Logic ---
        } // End mktime check
    } // End time available check


    delay(1); // Keep delay if needed for stability or yielding


    // --- Periodically Print GPS Status (Original 5-second check, Location part commented out) ---
    static unsigned long lastGpsCheck = 0;
    // unsigned long now_millis = millis(); // Already called above
    if (now_millis - lastGpsCheck >= 5000) {
        lastGpsCheck = now_millis;
        unsigned long year_chk, us_chk; unsigned int mo_chk, d_chk, h_chk, mi_chk, s_chk; // Use different var names
        bool timeAvailable_chk = getPreciseTimestamp(&year_chk, &mo_chk, &d_chk, &h_chk, &mi_chk, &s_chk, &us_chk);
        if(!timeAvailable_chk){
             Serial.println("Precise UTC Time (5s Check): Not available"); // Added label
        }
        // Location print removed here as it's printed once per second above
    }

    // --- Send Sync Data Every 10 Seconds ---
    static unsigned long lastSyncSend = 0;
    if (connection_state == ST_RECEIVE_DATA &&
        current_connection_handle != 0xFF &&
        time_sync_char_handle != 0xFFFF &&
        time_offset_char_handle != 0xFFFF &&
        (now_millis - lastSyncSend >= 10000)) { // Use now_millis
        lastSyncSend = now_millis;
        send_periodic_sync_data();
    }

} // End loop()

// ========================================================================
// get_offset (Placeholder) - UNCHANGED
// ========================================================================
int8_t get_offset() { return 60;}

// ========================================================================
// send_periodic_sync_data() - UNCHANGED
// ========================================================================
static void send_periodic_sync_data() {
    unsigned long year, microsecond;
    unsigned int month, day, hour, minute, second_part;
    bool timeAvailable = getPreciseTimestamp(&year, &month, &day, &hour, &minute, &second_part, &microsecond);
    if (timeAvailable) {
        struct tm t;
        t.tm_year = year - 1900; t.tm_mon = month - 1; t.tm_mday = day;
        t.tm_hour = hour; t.tm_min = minute; t.tm_sec = second_part; t.tm_isdst = 0;
        time_t unix_time = mktime(&t);
        uint32_t ms_part = microsecond / 1000;
        if (unix_time != (time_t)-1) {
            uint8_t time_buffer[8];
            time_buffer[0] = (uint8_t)(unix_time >> 0); time_buffer[1] = (uint8_t)(unix_time >> 8);
            time_buffer[2] = (uint8_t)(unix_time >> 16); time_buffer[3] = (uint8_t)(unix_time >> 24);
            time_buffer[4] = (uint8_t)(ms_part >> 0); time_buffer[5] = (uint8_t)(ms_part >> 8);
            time_buffer[6] = (uint8_t)(ms_part >> 16); time_buffer[7] = (uint8_t)(ms_part >> 24);
            sl_status_t sc_time = sl_bt_gatt_write_characteristic_value_without_response(
                current_connection_handle, time_sync_char_handle, sizeof(time_buffer), time_buffer, NULL);
            if (sc_time != SL_STATUS_OK) { Serial.print("BLE: Failed Time+MS Write Wo Rsp: 0x"); Serial.println(sc_time, HEX); }
        } else { Serial.println("BLE: Error converting GPS time to Unix timestamp."); }
        int8_t offset_val = get_offset();
        uint8_t offset_buffer[1] = {(uint8_t)offset_val};
        sl_status_t sc_offset = sl_bt_gatt_write_characteristic_value_without_response(
            current_connection_handle, time_offset_char_handle, sizeof(offset_buffer), offset_buffer, NULL);
        if (sc_offset != SL_STATUS_OK) { Serial.print("BLE: Failed Offset Write Wo Rsp: 0x"); Serial.println(sc_offset, HEX); }
    } else { Serial.println("BLE: Cannot send sync data, GPS time not valid."); }
}

// *** Function to Print Collected LoadCell Data and Reset (Server Time Based) - UNCHANGED ***
static void printAndResetCollection() {
    if (valuesCollectedCount > 0) {
        Serial.print("Collected LoadCell Data for Server Second: ");
        Serial.print(currentCollectionSecond);
        Serial.print(" [");
        for (int i = 0; i < NUM_SAMPLES_PER_SECOND; i++) {
            if (valueReceived[i]) { Serial.print(loadCellValues[i]); } else { Serial.print("---"); }
            if (i < NUM_SAMPLES_PER_SECOND - 1) { Serial.print(", "); }
        }
        Serial.println("]");
    }
    valuesCollectedCount = 0;
    memset(valueReceived, false, sizeof(valueReceived));
}

// ***** REVISED: Function to Print Collected Pin 6 Data and Reset (Client Time Based) *****
static void printAndResetPin6Collection() {
    if (pin6ValuesCollectedCount > 0) {
        Serial.print("Collected Analog Pin 6 Data for Client Second: ");
        Serial.print(currentPin6CollectionSecond); // Print the client second this data belongs to
        Serial.print(" [");
        for (int i = 0; i < NUM_SAMPLES_PER_SECOND; i++) {
            if (pin6ValueReceivedThisSlot[i]) {
                Serial.print(analogPin6Values[i]);
            } else {
                Serial.print("---"); // Indicate missing data for this slot
            }
            if (i < NUM_SAMPLES_PER_SECOND - 1) {
                Serial.print(", ");
            }
        }
        Serial.println("]");
    }
    // Reset state variables for the next Pin 6 collection cycle
    pin6ValuesCollectedCount = 0;
    memset(pin6ValueReceivedThisSlot, false, sizeof(pin6ValueReceivedThisSlot));
    // Note: We do NOT reset pin6DataPrinted here; it's reset on rollover in loop()
}
// ***** END REVISED *****


// ========================================================================
// BLE Functions - All UNCHANGED except Disconnect Reset
// ========================================================================
static void discover_time_sync_characteristic() { /* ... UNCHANGED ... */ Serial.println("BLE: Discovering Time Sync characteristic..."); sl_status_t sc = sl_bt_gatt_discover_characteristics_by_uuid(current_connection_handle, thermometer_service_handle, sizeof(UUID_TIME_SYNC_CHAR), UUID_TIME_SYNC_CHAR); app_assert_status(sc); connection_state = ST_TIME_SYNC_CHAR_DISCOVER; }
static void discover_time_offset_characteristic() { /* ... UNCHANGED ... */ Serial.println("BLE: Discovering Time Offset characteristic..."); sl_status_t sc = sl_bt_gatt_discover_characteristics_by_uuid(current_connection_handle, thermometer_service_handle, sizeof(UUID_TIME_OFFSET_CHAR), UUID_TIME_OFFSET_CHAR); app_assert_status(sc); connection_state = ST_TIME_OFFSET_CHAR_DISCOVER; }

void sl_bt_on_event(sl_bt_msg_t *evt) {
    sl_status_t sc;
    switch (SL_BT_MSG_ID(evt->header)) {
        case sl_bt_evt_system_boot_id:
            /* ... UNCHANGED ... */
              Serial.println("BLE: System Booted."); sc = sl_bt_scanner_set_parameters(sl_bt_scanner_scan_mode_active, 16, 16); app_assert_status(sc); sc = sl_bt_scanner_start(sl_bt_scanner_scan_phy_1m, sl_bt_scanner_discover_generic); app_assert_status(sc); Serial.println("BLE: Started scanning..."); connection_state = ST_SCAN; break;

        case sl_bt_evt_scanner_legacy_advertisement_report_id:
            /* ... UNCHANGED ... */
             if (connection_state == ST_SCAN) { if (find_complete_local_name_in_advertisement(&(evt->data.evt_scanner_legacy_advertisement_report))) { Serial.println("BLE: Target device found"); sc = sl_bt_scanner_stop(); app_assert_status(sc); sc = sl_bt_connection_open(evt->data.evt_scanner_legacy_advertisement_report.address, evt->data.evt_scanner_legacy_advertisement_report.address_type, sl_bt_gap_phy_1m, &current_connection_handle); if (sc == SL_STATUS_OK) { connection_state = ST_CONNECT; Serial.print("BLE: Connection pending, Handle: "); Serial.println(current_connection_handle); } else { /* Handle immediate connect fail */ connection_state = ST_SCAN; sc = sl_bt_scanner_start(sl_bt_scanner_scan_phy_1m, sl_bt_scanner_discover_generic); if(sc != SL_STATUS_OK && sc != SL_STATUS_INVALID_STATE) app_assert_status(sc); } } } break;

        case sl_bt_evt_connection_opened_id:
            /* ... UNCHANGED ... */
             if (current_connection_handle == evt->data.evt_connection_opened.connection){ Serial.print("BLE: Connection opened successfully, Handle: "); Serial.println(current_connection_handle); digitalWrite(LED_BUILTIN, LED_BUILTIN_ACTIVE); sc = sl_bt_gatt_discover_primary_services_by_uuid(current_connection_handle, sizeof(thermometer_service_uuid), thermometer_service_uuid); app_assert_status(sc); connection_state = ST_SERVICE_DISCOVER; } else { /* Handle unexpected connection */ } break;

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

                // Reset LoadCell collection state (Server Time)
                currentCollectionSecond = 0; valuesCollectedCount = 0; memset(valueReceived, false, sizeof(valueReceived));

                // Reset Pin 6 collection state (Client Time)
                currentPin6CollectionSecond = 0;
                pin6ValuesCollectedCount = 0; // Reset counter
                memset(pin6ValueReceivedThisSlot, false, sizeof(pin6ValueReceivedThisSlot)); // Reset flags
                pin6DataPrinted = false; // Reset print flag

                // Reset location print tracker
                lastLocationPrintSecond = 0;

                // Restart scanning
                sc = sl_bt_scanner_start(sl_bt_scanner_scan_phy_1m, sl_bt_scanner_discover_generic);
                if (sc != SL_STATUS_OK && sc != SL_STATUS_INVALID_STATE) app_assert_status(sc);
                Serial.println("BLE: Restarted scanning...");
            }
            break;

        case sl_bt_evt_gatt_service_id:
            /* ... UNCHANGED ... */
             if (connection_state == ST_SERVICE_DISCOVER && evt->data.evt_gatt_service.connection == current_connection_handle && memcmp(evt->data.evt_gatt_service.uuid.data, thermometer_service_uuid, sizeof(thermometer_service_uuid)) == 0) { Serial.println("BLE: GATT service 0x1809 discovered"); thermometer_service_handle = evt->data.evt_gatt_service.service; } break;

        case sl_bt_evt_gatt_characteristic_id:
            /* ... UNCHANGED ... */
             if(evt->data.evt_gatt_characteristic.connection == current_connection_handle) {
                 if (connection_state == ST_TEMP_CHAR_DISCOVER) { if (evt->data.evt_gatt_characteristic.uuid.len == 2 && memcmp(evt->data.evt_gatt_characteristic.uuid.data, temp_measurement_characteristic_uuid.data, 2) == 0) { temp_measurement_char_handle = evt->data.evt_gatt_characteristic.characteristic; Serial.println(" -> Matched Characteristic 0x2A1C (Indications)"); } }
                 else if (connection_state == ST_TIME_SYNC_CHAR_DISCOVER) { if (evt->data.evt_gatt_characteristic.uuid.len == 16 && memcmp(evt->data.evt_gatt_characteristic.uuid.data, UUID_TIME_SYNC_CHAR, 16) == 0) { time_sync_char_handle = evt->data.evt_gatt_characteristic.characteristic; Serial.println(" -> Matched Time Sync Characteristic (Write)"); } }
                 else if (connection_state == ST_TIME_OFFSET_CHAR_DISCOVER) { if (evt->data.evt_gatt_characteristic.uuid.len == 16 && memcmp(evt->data.evt_gatt_characteristic.uuid.data, UUID_TIME_OFFSET_CHAR, 16) == 0) { time_offset_char_handle = evt->data.evt_gatt_characteristic.characteristic; Serial.println(" -> Matched Time Offset Characteristic (Write)"); } }
             } break;

        case sl_bt_evt_gatt_procedure_completed_id: // Handles discovery and indication enable confirmations
            /* ... UNCHANGED ... */
              if(evt->data.evt_gatt_procedure_completed.connection == current_connection_handle) { if (evt->data.evt_gatt_procedure_completed.result != SL_STATUS_OK) { Serial.print("BLE: GATT Procedure failed! State: "); Serial.print(connection_state); /*...*/ sl_bt_connection_close(current_connection_handle); break; }
                switch(connection_state) {
                    case ST_SERVICE_DISCOVER: if (thermometer_service_handle != __UINT32_MAX__) { Serial.println("BLE: Discovering char 0x2A1C..."); sc = sl_bt_gatt_discover_characteristics_by_uuid(current_connection_handle, thermometer_service_handle, sizeof(temp_measurement_characteristic_uuid.data), temp_measurement_characteristic_uuid.data); app_assert_status(sc); connection_state = ST_TEMP_CHAR_DISCOVER; } else { /* Handle service not found */ } break;
                    case ST_TEMP_CHAR_DISCOVER: if (temp_measurement_char_handle != 0xFFFF) { discover_time_sync_characteristic(); } else { /* Handle char not found */ } break;
                    case ST_TIME_SYNC_CHAR_DISCOVER: if (time_sync_char_handle != 0xFFFF) { discover_time_offset_characteristic(); } else { /* Handle char not found */ } break;
                    case ST_TIME_OFFSET_CHAR_DISCOVER: if (time_offset_char_handle != 0xFFFF) { Serial.println("BLE: Discovery complete. Enabling indications..."); sc = sl_bt_gatt_set_characteristic_notification(current_connection_handle, temp_measurement_char_handle, sl_bt_gatt_indication); app_assert_status(sc); connection_state = ST_ENABLE_INDICATIONS; } else { /* Handle char not found */ } break;
                    case ST_ENABLE_INDICATIONS: Serial.println("BLE: Indications enabled. Ready for data & periodic sync."); connection_state = ST_RECEIVE_DATA; break;
                    default: break;
                }
              } break;

        case sl_bt_evt_gatt_characteristic_value_id: { // Handle incoming indications (LoadCell data)
            // ***** THIS ENTIRE CASE REMAINS UNCHANGED *****
             /* ... UNCHANGED ... */
             sl_bt_evt_gatt_characteristic_value_t *read_evt = &evt->data.evt_gatt_characteristic_value;

             if(read_evt->connection == current_connection_handle &&
                read_evt->characteristic == temp_measurement_char_handle &&
                read_evt->att_opcode == sl_bt_gatt_handle_value_indication) {

                 unsigned long client_year, client_microsecond; // Vars for debug print only
                 unsigned int client_month, client_day, client_hour, client_minute, client_second;
                 bool client_time_available = getPreciseTimestamp(&client_year, &client_month, &client_day, &client_hour, &client_minute, &client_second, &client_microsecond);
                 time_t client_unix_time = (time_t)-1; uint32_t client_ms = 0;
                 if (client_time_available) { /* Convert for debug print */ struct tm t_client; t_client.tm_year = client_year - 1900; t_client.tm_mon = client_month - 1; t_client.tm_mday = client_day; t_client.tm_hour = client_hour; t_client.tm_min = client_minute; t_client.tm_sec = client_second; t_client.tm_isdst = 0; client_unix_time = mktime(&t_client); client_ms = client_microsecond / 1000; if (client_unix_time == (time_t)-1) { client_time_available = false; } }

                 uint8_t* data = read_evt->value.data; uint8_t length = read_evt->value.len;

                 if (length == 10) {
                     uint32_t server_unix_seconds = (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
                     uint32_t server_milliseconds = (uint32_t)data[4] | (uint32_t)data[5] << 8 | (uint32_t)data[6] << 16 | (uint32_t)data[7] << 24;
                     uint16_t adcValue = (uint16_t)data[8] | (uint16_t)data[9] << 8;

                     // DEBUG PRINT BLOCK
                     //Serial.print("BLE Indication Recv: "); Serial.print("ClientRxTime=["); if (client_time_available) { Serial.print((uint32_t)client_unix_time); Serial.print("s "); Serial.print(client_ms); Serial.print("ms] "); } else { Serial.print("Unavailable] "); } Serial.print("ServerSensorTime=["); Serial.print(server_unix_seconds); Serial.print("s "); Serial.print(server_milliseconds); Serial.print("ms] "); Serial.print("LoadCell="); Serial.println(adcValue);
                     // END DEBUG PRINT BLOCK

                     // LoadCell Data Collection Logic (Using SERVER Time)
                     if (server_unix_seconds > currentCollectionSecond) { if (valuesCollectedCount > 0) { Serial.print("INFO: LoadCell Server Time Rollover. "); printAndResetCollection(); } currentCollectionSecond = server_unix_seconds; valuesCollectedCount = 0; memset(valueReceived, false, sizeof(valueReceived)); } else if (server_unix_seconds < currentCollectionSecond && currentCollectionSecond != 0) { Serial.print("WARN LoadCell: Ignoring old data for server second: "); Serial.println(server_unix_seconds); sc = sl_bt_gatt_send_characteristic_confirmation(current_connection_handle); if (sc != SL_STATUS_OK) { Serial.print("BLE: Failed indication confirm (old data): 0x"); Serial.println(sc, HEX); } break; } else if (currentCollectionSecond == 0 && server_unix_seconds != 0) { currentCollectionSecond = server_unix_seconds; valuesCollectedCount = 0; memset(valueReceived, false, sizeof(valueReceived)); }
                     if (server_unix_seconds == currentCollectionSecond) { int index = server_milliseconds / 100; if (index >= 0 && index < NUM_SAMPLES_PER_SECOND) { if (!valueReceived[index]) { loadCellValues[index] = adcValue; valueReceived[index] = true; valuesCollectedCount++; if (valuesCollectedCount == NUM_SAMPLES_PER_SECOND) { printAndResetCollection(); } } } else { Serial.print("WARN LoadCell: Invalid server milliseconds value "); Serial.print(server_milliseconds); Serial.print(" -> index "); Serial.print(index); Serial.println(". Data ignored."); } }
                     // End LoadCell Logic
                 } else { Serial.print("BLE Indication Recv: Unexpected length: "); Serial.println(length); }
                 sc = sl_bt_gatt_send_characteristic_confirmation(current_connection_handle); if (sc != SL_STATUS_OK) { Serial.print("BLE: Failed indication confirm: 0x"); Serial.println(sc, HEX); }
             }
        } break; // End case sl_bt_evt_gatt_characteristic_value_id

        default: break;
    }
}


// ========================================================================
// NMEA/PPS Functions (Unchanged from original)
// ========================================================================
void ppsISR() { /* ... UNCHANGED ... */ unsigned long cM = micros(); if (nmeTimeValid) { ppsTimestampMicros = cM; noInterrupts(); ppsUtcYear=nmeUtcYear; ppsUtcMonth=nmeUtcMonth; ppsUtcDay=nmeUtcDay; ppsUtcHour=nmeUtcHour; ppsUtcMinute=nmeUtcMinute; ppsUtcSecond=nmeUtcSecond; interrupts(); ppsTimeValid = true; } else { ppsTimeValid = false; } }
bool getPreciseTimestamp(unsigned long *pY,unsigned int *pMo,unsigned int *pD,unsigned int *pH,unsigned int *pMi,unsigned int *pS,unsigned long *pUs) { /* ... UNCHANGED ... */ unsigned long fCM=micros(); unsigned long lPM; int y,mo,d,h,mi,s; bool v; noInterrupts(); lPM=ppsTimestampMicros; y=ppsUtcYear; mo=ppsUtcMonth; d=ppsUtcDay; h=ppsUtcHour; mi=ppsUtcMinute; s=ppsUtcSecond; v=ppsTimeValid; interrupts(); if (!v) return false; unsigned long uSP; if (fCM >= lPM) { uSP = fCM - lPM; } else { uSP = (0xFFFFFFFF - lPM) + fCM + 1; } if (uSP >= 1000000) { ppsTimeValid = false; return false; } *pY=y; *pMo=mo; *pD=d; *pH=h; *pMi=mi; *pS=s; *pUs=uSP; return true; }
bool getCurrentLocation(float *pLat, float *pLon) { /* ... UNCHANGED ... */ bool vF; float cLat, cLon; noInterrupts(); vF=gpsFix; cLat=latitude; cLon=longitude; interrupts(); if(vF){*pLat=cLat; *pLon=cLon; return true;}else{return false;} }
void readGpsSerial() { /* ... UNCHANGED ... */ while(gpsSerial.available()>0){ char c=gpsSerial.read(); if(c=='$'||bufferIndex>=MAX_NMEA_BUFFER-1){bufferIndex=0; memset(nmeaBuffer,0,MAX_NMEA_BUFFER);} if(c!='\n'&&c!='\r'&&bufferIndex<MAX_NMEA_BUFFER-1){nmeaBuffer[bufferIndex++]=c;} if(c=='\n'){nmeaBuffer[bufferIndex]=0; if(nmeaBuffer[0]=='$'){parseNmeaSentence(nmeaBuffer);} bufferIndex=0; memset(nmeaBuffer,0,MAX_NMEA_BUFFER);}}}
void parseNmeaSentence(char *sentence) { /* ... UNCHANGED ... */ if(strncmp(sentence+1,"GNRMC",5)==0||strncmp(sentence+1,"GPRMC",5)==0){ if(!validateChecksum(sentence)) return; char *f[20]; int fI=0; f[fI++]=sentence; char* p=sentence; while((p=strchr(p,','))!=NULL&&fI<20){*p=0; f[fI++]=++p;} p=strchr(f[fI-1],'*'); if(p!=NULL) *p=0; if(fI>=10){int d=0,mo=0,y=0,h=0,mi=0,s=0; bool fix=false; float lat=0.0,lon=0.0; bool pOk=true; if(strlen(f[2])>0){fix=(f[2][0]=='A');}else pOk=false; if(fix){if(strlen(f[1])>=6){char hh[3]={f[1][0],f[1][1],0},mm[3]={f[1][2],f[1][3],0},ss[3]={f[1][4],f[1][5],0}; h=atoi(hh);mi=atoi(mm);s=atoi(ss);}else pOk=false; if(pOk&&strlen(f[9])==6){char d_s[3]={f[9][0],f[9][1],0},m_s[3]={f[9][2],f[9][3],0},y_s[3]={f[9][4],f[9][5],0}; d=atoi(d_s);mo=atoi(m_s);y=atoi(y_s)+2000;}else pOk=false; if(pOk&&strlen(f[3])>0&&strlen(f[4])>0){lat=nmeaToDecimalDegrees(f[3],f[4][0]);}else pOk=false; if(pOk&&strlen(f[5])>0&&strlen(f[6])>0){lon=nmeaToDecimalDegrees(f[5],f[6][0]);}else pOk=false; if(pOk){noInterrupts();nmeUtcYear=y;nmeUtcMonth=mo;nmeUtcDay=d;nmeUtcHour=h;nmeUtcMinute=mi;nmeUtcSecond=s;nmeTimeValid=true;latitude=lat;longitude=lon;gpsFix=true;interrupts();}else{noInterrupts();nmeTimeValid=false;gpsFix=false;interrupts();}}else{noInterrupts();nmeTimeValid=false;gpsFix=false;interrupts();}}else{noInterrupts();nmeTimeValid=false;gpsFix=false;interrupts();}}}
float nmeaToDecimalDegrees(char *nC, char hem) { /* ... UNCHANGED ... */ if(strlen(nC)==0)return 0.0; float rC=atof(nC); int deg=(int)(rC/100.0); float min=rC-(deg*100.0); float dD=deg+(min/60.0); if(hem=='S'||hem=='W')dD=-dD; else if(hem!='N'&&hem!='E')return 0.0; return dD;}
byte hexCharToByte(char c) { /* ... UNCHANGED ... */ if(c>='0'&&c<='9')return c-'0'; if(c>='A'&&c<='F')return c-'A'+10; if(c>='a'&&c<='f')return c-'a'+10; return 255; }
bool validateChecksum(char *sentence) { /* ... UNCHANGED ... */ if(sentence[0]!='$')return false; char *cs_ptr=strchr(sentence,'*'); if(cs_ptr==NULL||strlen(cs_ptr)<3)return false; byte calc_cs=0; for(int i=1;sentence[i]!='*'&&sentence[i]!=0;i++)calc_cs^=sentence[i]; byte hi=hexCharToByte(*(cs_ptr+1)); byte lo=hexCharToByte(*(cs_ptr+2)); if(hi==255||lo==255)return false; byte recv_cs=(hi<<4)+lo; return calc_cs==recv_cs; }
static bool find_complete_local_name_in_advertisement(sl_bt_evt_scanner_legacy_advertisement_report_t *response) { /* ... UNCHANGED ... */ int i=0; bool f=false; uint8_t nL=strlen((const char*)advertised_name); while(i<(response->data.len-1)){ uint8_t adL=response->data.data[i]; if(adL==0||(i+adL)>=response->data.len)break; uint8_t adT=response->data.data[i+1]; if(adT==0x09){if((adL-1)==nL){if(memcmp(response->data.data+i+2, advertised_name, nL)==0){f=true; break;}}} i=i+adL+1; } return f; }