/*
 * BLE Custom Data Server (Time Sync Receive & ADC Send) - CORRECTED OFFSET LOGIC
 * OPTIMIZED WITH CONDITIONAL SERIAL DEBUG
 *
 * - Receives Time+MS (8 bytes) on Time Sync characteristic.
 * - Receives Offset (1 byte) on Offset characteristic and stores it persistently.
 * - Applies the stored offset IMMEDIATELY to the received Time+MS.
 * - Uses this adjusted base time for calculating timestamps in indications.
 * - Sends 10-byte indication (Adjusted Unix Time + Server MS + ADC).
 * - Includes CONDITIONAL Serial debugging output via ENABLE_SERIAL_DEBUG flag.
 */

#include <Arduino.h>
// Ensure necessary Silabs headers are included
#include <sl_bt_api.h>
// #include "sl_bluetooth.h" // May also be needed
// #include "app_assert.h" // For app_assert_status definition (using local placeholder)

// --- Conditional Serial Debugging ---
// Uncomment the line below to enable all Serial output (begin, print, println)
//#define ENABLE_SERIAL_DEBUG

#ifdef ENABLE_SERIAL_DEBUG
  #define DEBUG_BEGIN(...) Serial.begin(__VA_ARGS__)
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__) // If you use printf
  #define DEBUG_WAIT_FOR_SERIAL() while(!Serial) // Optional wait
#else
  #define DEBUG_BEGIN(...) // Does nothing
  #define DEBUG_PRINT(...) // Does nothing
  #define DEBUG_PRINTLN(...) // Does nothing
  #define DEBUG_PRINTF(...) // Does nothing
  #define DEBUG_WAIT_FOR_SERIAL() // Does nothing
#endif

// --- Forward Declarations ---
static void handle_temperature_indication();
static void ble_initialize_gatt_db();
static void ble_start_advertising();
void sl_bt_on_event(sl_bt_msg_t *evt);

// --- Global Variables ---
const uint8_t advertised_name[] = "CustomDataSender";
uint8_t connection_handle = 0u;
uint16_t temp_measurement_characteristic_handle = 0xFFFF; // For indications OUT
bool indication_enabled = false;
uint16_t time_sync_char_handle = 0xFFFF;     // For Time+MS IN
uint16_t time_offset_char_handle = 0xFFFF;   // For Offset IN
volatile uint32_t g_received_utc_sec = 0;     // Stores ADJUSTED base seconds
volatile uint32_t g_received_utc_ms = 0;      // Stores ADJUSTED base milliseconds
volatile int8_t g_received_offset_ms = 0;     // Stores the LATEST received offset value
volatile bool g_use_received_time = false;    // Flag to indicate if synced time is available
volatile uint32_t g_last_sync_server_millis = 0; // Stores server's millis() at last client time sync

// --- Define Custom UUIDs using the uuid_128 struct type ---
// UUID for Time+MS Characteristic (8 bytes, Writeable)
const uuid_128 UUID_TIME_SYNC_CHAR = {{
    0x01, 0xf5, 0xd9, 0x46, 0xc3, 0x45, 0x59, 0x82,
    0x71, 0x48, 0x02, 0x83, 0x02, 0x0d, 0xa8, 0xf0
}};
// UUID for Offset Characteristic (1 byte, Writeable)
const uuid_128 UUID_TIME_OFFSET_CHAR = {{
    0x02, 0xf5, 0xd9, 0x46, 0xc3, 0x45, 0x59, 0x82,
    0x71, 0x48, 0x02, 0x83, 0x02, 0x0d, 0xa8, 0xf0
}};


// Define LED states
#ifndef LED_BUILTIN_ACTIVE
#define LED_BUILTIN_ACTIVE HIGH
#endif
#ifndef LED_BUILTIN_INACTIVE
#define LED_BUILTIN_INACTIVE LOW
#endif

// Placeholder for app_assert_status (Includes LED blink - Does not use Serial)
#ifndef app_assert_status
#define app_assert_status(sc) \
  do {                       \
    if ((sc) != SL_STATUS_OK) { \
      pinMode(LED_BUILTIN, OUTPUT); \
      while (1){ digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); \
      delay(100); } \
    } \
  } while (0)
#endif

// --- Arduino Setup ---
void setup()
{
  // --- User Hardware Setup ---
  analogReference(AR_EXTERNAL_1V25);
  pinMode(4, OUTPUT); digitalWrite(4, HIGH); pinMode(3, OUTPUT); digitalWrite(3, LOW);
  pinMode(2, OUTPUT); digitalWrite(2, HIGH); pinMode(1, OUTPUT); digitalWrite(1, LOW);
  pinMode(14, OUTPUT); digitalWrite(14, LOW); pinMode(13, OUTPUT); digitalWrite(13, HIGH);
  pinMode(12, OUTPUT); pinMode(11, OUTPUT); pinMode(10, OUTPUT); digitalWrite(10, HIGH);
  pinMode(9, OUTPUT); digitalWrite(9, LOW);
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
  pinMode(PA7, INPUT); // ADC Input Pin
  // --- End User Hardware Setup ---

  // --- Conditional Serial Init ---
  DEBUG_BEGIN(115200);
  // DEBUG_WAIT_FOR_SERIAL(); // Uncomment if you need to wait for Serial Monitor connection
  DEBUG_PRINTLN("\n---------------------------------------------");
  DEBUG_PRINTLN("Custom BLE Server - CORRECTED OFFSET LOGIC");
  DEBUG_PRINTLN("---------------------------------------------");
  // Initializing message moved to boot event
}

// --- Arduino Loop ---
void loop()
{
  // External Tick for Bluetooth stack is handled elsewhere usually (e.g., platform specific)
  // We just handle indications when enabled
  if (indication_enabled) {
    handle_temperature_indication(); // Contains its own delay logic now
  } else {
    // Minimal delay when not indicating to allow BLE stack processing if needed
    delay(10);
  }
}

// --- Constants for ADC Reading ---
const int adc_samples = 16;
const int inamp_bridge_drive_delay = 50; // us
const int settle_time = 350; // us

// --- Custom Data Sending Logic ---
static void handle_temperature_indication()
{
  if (!indication_enabled) { return; }

  uint32_t current_timestamp_s = 0;
  uint32_t current_timestamp_ms = 0;

  // Calculate current time based on last sync (which already includes offset)
  if (g_use_received_time) {
    uint32_t current_server_millis = millis();
    uint32_t elapsed_ms_since_sync;

    // Calculate elapsed millis correctly handling rollover
    if (current_server_millis >= g_last_sync_server_millis) {
        elapsed_ms_since_sync = current_server_millis - g_last_sync_server_millis;
    } else { // Rollover occurred
        elapsed_ms_since_sync = (0xFFFFFFFF - g_last_sync_server_millis) + current_server_millis + 1;
    }

    uint32_t elapsed_sec = elapsed_ms_since_sync / 1000;
    uint32_t elapsed_ms_part = elapsed_ms_since_sync % 1000;

    uint32_t base_s = g_received_utc_sec;
    uint32_t base_ms = g_received_utc_ms;

    uint32_t temp_ms = base_ms + elapsed_ms_part;
    current_timestamp_s = base_s + elapsed_sec + (temp_ms / 1000);
    current_timestamp_ms = temp_ms % 1000;

  } else { // Fallback if no time sync received yet
    uint32_t current_millis = millis();
    current_timestamp_s = current_millis / 1000;
    current_timestamp_ms = current_millis % 1000;
  }

  // --- ADC Reading ---
  digitalWrite(4, LOW); digitalWrite(14, HIGH); digitalWrite(9, HIGH); delayMicroseconds(inamp_bridge_drive_delay);
  digitalWrite(13, LOW); delayMicroseconds(settle_time); uint32_t adc_sum = 0;
  for(int i=0; i<adc_samples; i++){ adc_sum += analogRead(PA7); } uint16_t rawAnalogValue = adc_sum / adc_samples;
  digitalWrite(4, HIGH); digitalWrite(14, LOW); digitalWrite(13, HIGH); digitalWrite(9, LOW);
  // --- End ADC Reading ---

  // Pack data for indication
  uint8_t buffer[10];
  buffer[0]=(uint8_t)(current_timestamp_s>>0); buffer[1]=(uint8_t)(current_timestamp_s>>8);
  buffer[2]=(uint8_t)(current_timestamp_s>>16); buffer[3]=(uint8_t)(current_timestamp_s>>24);
  buffer[4]=(uint8_t)(current_timestamp_ms>>0); buffer[5]=(uint8_t)(current_timestamp_ms>>8);
  buffer[6]=(uint8_t)(current_timestamp_ms>>16); buffer[7]=(uint8_t)(current_timestamp_ms>>24);
  buffer[8]=(uint8_t)(rawAnalogValue>>0); buffer[9]=(uint8_t)(rawAnalogValue>>8);

  // Send indication
  sl_status_t sc1 = sl_bt_gatt_server_send_indication(connection_handle, temp_measurement_characteristic_handle, sizeof(buffer), buffer);
  
  
  sl_status_t sc2 = 0;
  if(sc1>0){
  DEBUG_PRINTLN("Indication Result:");
  DEBUG_PRINTLN(sc1);
  delay(20);
  sc2 = sl_bt_gatt_server_send_indication(connection_handle, temp_measurement_characteristic_handle, sizeof(buffer), buffer);
  DEBUG_PRINTLN("Indication Result 2 :");
  DEBUG_PRINTLN(sc2);
  }
  if(sc2>0){
  delay(20);
  sl_status_t sc3 = sl_bt_gatt_server_send_indication(connection_handle, temp_measurement_characteristic_handle, sizeof(buffer), buffer);
  DEBUG_PRINTLN("Indication Result 3 :");
  DEBUG_PRINTLN(sc3);
  }
  // Delay logic
  //DEBUG_PRINTLN(current_timestamp_ms); // Your debug print, now conditional
  //DEBUG_PRINTLN(millis()%100);       // Your debug print, now conditional
  uint32_t calculated_delay = 100;
  uint32_t ms_remainder = current_timestamp_ms % 100;
  if (calculated_delay > ms_remainder) {
    calculated_delay = calculated_delay - ms_remainder;
  } else {
    calculated_delay = 0; // Avoid negative delay
  }
  delay(calculated_delay); // Adjust delay
}

// --- BLE GATT Database Initialization ---
static void ble_initialize_gatt_db()
{
  sl_status_t sc;
  uint16_t gattdb_session_id;
  uint16_t generic_access_service_handle;
  uint16_t thermometer_service_handle;
  uint16_t device_name_characteristic_handle;
  uint16_t temp_type_characteristic_handle;

  sc = sl_bt_gattdb_new_session(&gattdb_session_id);
  app_assert_status(sc);

  // Generic Access Service
  const uint8_t generic_access_service_uuid[] = { 0x00, 0x18 };
  sc = sl_bt_gattdb_add_service(gattdb_session_id, sl_bt_gattdb_primary_service, 0, sizeof(generic_access_service_uuid), generic_access_service_uuid, &generic_access_service_handle); app_assert_status(sc);
  const sl_bt_uuid_16_t device_name_characteristic_uuid = { .data = { 0x00, 0x2A } };
  sc = sl_bt_gattdb_add_uuid16_characteristic(gattdb_session_id, generic_access_service_handle, SL_BT_GATTDB_CHARACTERISTIC_READ, 0x01, 0x02, device_name_characteristic_uuid, sl_bt_gattdb_variable_length_value, sizeof(advertised_name) - 1, sizeof(advertised_name) - 1, advertised_name, &device_name_characteristic_handle); app_assert_status(sc);
  sc = sl_bt_gattdb_start_service(gattdb_session_id, generic_access_service_handle); app_assert_status(sc);

  // Health Thermometer Service (Container)
  const uint8_t thermometer_service_uuid[] = { 0x09, 0x18 };
  sc = sl_bt_gattdb_add_service(gattdb_session_id, sl_bt_gattdb_primary_service, SL_BT_GATTDB_ADVERTISED_SERVICE, sizeof(thermometer_service_uuid), thermometer_service_uuid, &thermometer_service_handle); app_assert_status(sc);

  // Characteristic for Data OUT (0x2A1C - Temp Measurement)
  const sl_bt_uuid_16_t temp_measurement_characteristic_uuid = { .data = { 0x1C, 0x2A } };
  uint8_t custom_initial_value[10] = { 0 };
  sc = sl_bt_gattdb_add_uuid16_characteristic(gattdb_session_id, thermometer_service_handle, SL_BT_GATTDB_CHARACTERISTIC_INDICATE, 0x00, 0x10, temp_measurement_characteristic_uuid, sl_bt_gattdb_fixed_length_value, 10, 10, custom_initial_value, &temp_measurement_characteristic_handle); app_assert_status(sc);

  // Temperature Type characteristic (0x2A1D - Unused but present)
  const sl_bt_uuid_16_t temp_type_characteristic_uuid = { .data = { 0x1D, 0x2A } };
  uint8_t temp_type_initial_value = 0;
  sc = sl_bt_gattdb_add_uuid16_characteristic(gattdb_session_id, thermometer_service_handle, SL_BT_GATTDB_CHARACTERISTIC_READ, 0x01, 0x02, temp_type_characteristic_uuid, sl_bt_gattdb_fixed_length_value, 1, 1, &temp_type_initial_value, &temp_type_characteristic_handle); app_assert_status(sc);

  // --- Add Custom Writable Characteristics ---
  // 1. Time Sync Characteristic (UUID_TIME_SYNC_CHAR)
  uint8_t time_sync_initial_value[8] = { 0 };
  sc = sl_bt_gattdb_add_uuid128_characteristic(
      gattdb_session_id, thermometer_service_handle,
      (SL_BT_GATTDB_CHARACTERISTIC_WRITE | SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE),
      0x02, (0x08 | 0x04), UUID_TIME_SYNC_CHAR, sl_bt_gattdb_fixed_length_value,
      8, 8, time_sync_initial_value, &time_sync_char_handle);
  app_assert_status(sc);
  DEBUG_PRINT("Added Time Sync Char (8 byte, W+W/O Rsp), Handle: 0x"); DEBUG_PRINTLN(time_sync_char_handle, HEX);

  // 2. Time Offset Characteristic (UUID_TIME_OFFSET_CHAR)
  uint8_t time_offset_initial_value[1] = { 0 };
  sc = sl_bt_gattdb_add_uuid128_characteristic(
      gattdb_session_id, thermometer_service_handle,
      (SL_BT_GATTDB_CHARACTERISTIC_WRITE | SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE),
      0x02, (0x08 | 0x04), UUID_TIME_OFFSET_CHAR, sl_bt_gattdb_fixed_length_value,
      1, 1, time_offset_initial_value, &time_offset_char_handle);
  app_assert_status(sc);
  DEBUG_PRINT("Added Time Offset Char (1 byte, W+W/O Rsp), Handle: 0x"); DEBUG_PRINTLN(time_offset_char_handle, HEX);


  // --- Start and Commit ---
  sc = sl_bt_gattdb_start_service(gattdb_session_id, thermometer_service_handle); app_assert_status(sc);
  sc = sl_bt_gattdb_commit(gattdb_session_id); app_assert_status(sc);
  DEBUG_PRINTLN("GATT Database Initialized.");
}


// --- BLE Advertising Logic ---
static void ble_start_advertising()
{
  static uint8_t advertising_set_handle = 0xff; static bool init = true; sl_status_t sc;
  if (init) { sc = sl_bt_advertiser_create_set(&advertising_set_handle); app_assert_status(sc); sc = sl_bt_advertiser_set_timing(advertising_set_handle, 160, 160, 0, 0); app_assert_status(sc); init = false; }
  sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle, sl_bt_advertiser_general_discoverable); app_assert_status(sc);
  sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
  if (sc == SL_STATUS_OK) { DEBUG_PRINTLN("Advertising started..."); }
  else if (sc == SL_STATUS_ALREADY_EXISTS) { DEBUG_PRINTLN("Already advertising."); }
  else { app_assert_status(sc); }
}


// --- BLE Event Handler ---
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      DEBUG_PRINTLN("BLE Event: System Booted.");
      ble_initialize_gatt_db();
      ble_start_advertising();
      break;

    case sl_bt_evt_connection_opened_id:
      connection_handle = evt->data.evt_connection_opened.connection;
      digitalWrite(LED_BUILTIN, LED_BUILTIN_ACTIVE);
      indication_enabled = false; g_use_received_time = false;
      g_received_offset_ms = 0; g_last_sync_server_millis = 0;
      DEBUG_PRINT("BLE Event: Connection opened, handle: "); DEBUG_PRINTLN(connection_handle);
      break;

    case sl_bt_evt_connection_closed_id:
      DEBUG_PRINT("BLE Event: Connection closed, handle: "); DEBUG_PRINT(evt->data.evt_connection_closed.connection);
      DEBUG_PRINT(", reason: 0x"); DEBUG_PRINTLN(evt->data.evt_connection_closed.reason, HEX);
      connection_handle = 0u; indication_enabled = false; g_use_received_time = false;
      g_received_offset_ms = 0; g_last_sync_server_millis = 0;
      digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
      ble_start_advertising(); // Restart advertising
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id: {
      sl_bt_evt_gatt_server_characteristic_status_t *status_evt = &evt->data.evt_gatt_server_characteristic_status;
      if (status_evt->characteristic == temp_measurement_characteristic_handle) {
        if (status_evt->status_flags == sl_bt_gatt_server_client_config) {
          if (status_evt->client_config_flags == sl_bt_gatt_indication) { indication_enabled = true; DEBUG_PRINTLN("BLE Event: Indications Enabled by Client."); }
          else if (status_evt->client_config_flags == sl_bt_gatt_disable) { indication_enabled = false; DEBUG_PRINTLN("BLE Event: Indications Disabled by Client."); }
        }
      }
    } break; // End characteristic_status case

    // Handle Client Writes
    case sl_bt_evt_gatt_server_attribute_value_id: {
      sl_bt_evt_gatt_server_attribute_value_t *write_evt = &evt->data.evt_gatt_server_attribute_value;

      DEBUG_PRINTLN("---------------------------------");
      DEBUG_PRINT("BLE Event: GATT Attribute Write");
      DEBUG_PRINT(" | Handle: 0x"); DEBUG_PRINT(write_evt->attribute, HEX);
      DEBUG_PRINT(" | Conn: "); DEBUG_PRINT(write_evt->connection);
      DEBUG_PRINT(" | Len: "); DEBUG_PRINT(write_evt->value.len);
      DEBUG_PRINT(" | Offset: "); DEBUG_PRINTLN(write_evt->offset);
      DEBUG_PRINT("  Raw Data: ");
      #ifdef ENABLE_SERIAL_DEBUG // Only loop/print if debug enabled
        for(int i=0; i < write_evt->value.len; i++) {
          if(write_evt->value.data[i] < 0x10) DEBUG_PRINT("0"); // Pad
          DEBUG_PRINT(write_evt->value.data[i], HEX); DEBUG_PRINT(" ");
        }
      #endif
      DEBUG_PRINTLN();

      // Toggle LED briefly
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(30);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

      // Check if it's the Time Offset characteristic FIRST
      if (write_evt->attribute == time_offset_char_handle) {
           DEBUG_PRINTLN(" -> Write matched Offset Handle.");
           if (write_evt->value.len == 1) {
               DEBUG_PRINTLN(" -> Length is 1 byte (Correct for Offset). Parsing...");
               g_received_offset_ms = (int8_t)write_evt->value.data[0];
               DEBUG_PRINT(" -> Parsed Offset OK: Offset="); DEBUG_PRINT(g_received_offset_ms); DEBUG_PRINTLN(" ms stored.");
           } else {
               DEBUG_PRINT(" -> ERROR: Incorrect length for Time Offset write! Expected 1, Got ");
               DEBUG_PRINTLN(write_evt->value.len);
           }
      }
      // Check if it's the Time Sync characteristic
      else if (write_evt->attribute == time_sync_char_handle) {
           DEBUG_PRINTLN(" -> Write matched Time Sync Handle.");
           if (write_evt->value.len == 8) {
               DEBUG_PRINTLN(" -> Length is 8 bytes (Correct for Time+MS). Parsing...");

               uint32_t base_sec = (uint32_t)write_evt->value.data[0] | (uint32_t)write_evt->value.data[1] << 8 | (uint32_t)write_evt->value.data[2] << 16 | (uint32_t)write_evt->value.data[3] << 24;
               uint32_t base_ms  = (uint32_t)write_evt->value.data[4] | (uint32_t)write_evt->value.data[5] << 8 | (uint32_t)write_evt->value.data[6] << 16 | (uint32_t)write_evt->value.data[7] << 24;

               DEBUG_PRINT(" -> Received Base Time: Sec="); DEBUG_PRINT(base_sec);
               DEBUG_PRINT(", Ms="); DEBUG_PRINTLN(base_ms);
               DEBUG_PRINT(" -> Applying Stored Offset: "); DEBUG_PRINTLN(g_received_offset_ms);

               // Apply the currently stored offset immediately
               int64_t total_ms_raw = (int64_t)base_ms + g_received_offset_ms;
               int32_t sec_adjust = 0;
               uint32_t adjusted_ms_part = 0;

               if (total_ms_raw >= 1000) {
                   sec_adjust = total_ms_raw / 1000;
                   adjusted_ms_part = total_ms_raw % 1000;
               } else if (total_ms_raw < 0) {
                   sec_adjust = (total_ms_raw / 1000) - 1;
                   adjusted_ms_part = 1000 + (total_ms_raw % 1000);
                   if (adjusted_ms_part == 1000) adjusted_ms_part = 0;
               } else {
                   adjusted_ms_part = (uint32_t)total_ms_raw;
               }

               // Store the *adjusted* time globally
               g_received_utc_sec = base_sec + sec_adjust;
               g_received_utc_ms = adjusted_ms_part;

               g_use_received_time = true;
               g_last_sync_server_millis = millis();
               if (g_last_sync_server_millis == 0) g_last_sync_server_millis = 1; // Avoid edge case

               DEBUG_PRINT(" -> Stored Adjusted Time: Sec="); DEBUG_PRINT(g_received_utc_sec);
               DEBUG_PRINT(", Ms="); DEBUG_PRINTLN(g_received_utc_ms);

           } else {
               DEBUG_PRINT(" -> ERROR: Incorrect length for Time Sync write! Expected 8, Got ");
               DEBUG_PRINTLN(write_evt->value.len);
           }
      } else {
           DEBUG_PRINTLN(" -> Write to different/unknown attribute handle.");
      }
      DEBUG_PRINTLN("---------------------------------"); // Separator
    } break; // End attribute_value case

    case sl_bt_evt_gatt_server_indication_timeout_id:
      DEBUG_PRINTLN("BLE Event: Indication timeout.");
      break;

    default: break; // Ignore other events silently
  }
}

// --- NMEA/PPS Functions from Client (If needed on server? Assume not needed based on description) ---
// Remove these functions if they are not used on the server side.

// --- Helper function from Client (If needed on server?) ---
// Remove this function if not used on server side.
// static bool find_complete_local_name_in_advertisement(...) { ... }