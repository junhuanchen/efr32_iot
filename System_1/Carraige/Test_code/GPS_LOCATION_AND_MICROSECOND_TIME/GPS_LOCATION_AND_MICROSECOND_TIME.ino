#include <Arduino.h> // Standard Arduino header

// Define the serial port for GPS communication (Hardware Serial 1)
HardwareSerial &gpsSerial = Serial1;

// --- PPS Pin Definition ---
const int PPS_PIN = 3; // GPS PPS signal connected here

// Buffer for NMEA sentences
const int MAX_NMEA_BUFFER = 100;
char nmeaBuffer[MAX_NMEA_BUFFER];
int bufferIndex = 0;

// --- NMEA Time Storage (Updated by NMEA parser, read by ISR) ---
volatile int nmeUtcYear = 0;
volatile int nmeUtcMonth = 0;
volatile int nmeUtcDay = 0;
volatile int nmeUtcHour = 0;
volatile int nmeUtcMinute = 0;
volatile int nmeUtcSecond = 0;
volatile bool nmeTimeValid = false;

// --- PPS Time Storage (Updated by ISR, read by getPreciseTimestamp) ---
volatile unsigned long ppsTimestampMicros = 0;
volatile int ppsUtcYear = 0;
volatile int ppsUtcMonth = 0;
volatile int ppsUtcDay = 0;
volatile int ppsUtcHour = 0;
volatile int ppsUtcMinute = 0;
volatile int ppsUtcSecond = 0;
volatile bool ppsTimeValid = false;
volatile bool newPpsFlag = false;

// --- Location Storage (Updated by NMEA parser, read by getCurrentLocation) ---
volatile float latitude = 0.0;
volatile float longitude = 0.0;
volatile bool gpsFix = false; // Overall fix status based on last GNRMC

// --- Function Prototypes ---
void readGpsSerial();
void parseNmeaSentence(char *sentence);
float nmeaToDecimalDegrees(char *nmeaCoord, char hemisphere);
bool validateChecksum(char *sentence);
void printGpsData(); // Kept for optional basic NMEA printing
byte hexCharToByte(char c);
void ppsISR();
bool getPreciseTimestamp(unsigned long *pYear, unsigned int *pMonth, unsigned int *pDay,
                         unsigned int *pHour, unsigned int *pMinute, unsigned int *pSecond,
                         unsigned long *pMicrosecond);
bool getCurrentLocation(float *pLat, float *pLon); // Added prototype


// ========================================================================
// SETUP
// ========================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Manual NMEA + PPS Parser Started");
  Serial.println("--------------------------------");
  Serial.println("Location: Newcastle, NSW, Australia"); // Added context
  Serial.print("Current System Time: "); // Placeholder for system time if needed
  // Note: Arduino doesn't automatically know local time unless set externally (e.g., via NTP or RTC)
  // GPS provides UTC time. Current time (May 4 2025, ~8:02 PM AEST) corresponds to May 4 2025, ~10:02 AM UTC.
  Serial.println("(System time is approximate, GPS provides UTC)");
  Serial.println("Listening on Serial1 (9600 baud) for $GNRMC sentences...");
  Serial.println("Listening on Pin 3 for PPS signal...");


  gpsSerial.begin(9600);
  memset(nmeaBuffer, 0, MAX_NMEA_BUFFER);
  bufferIndex = 0;

  // --- PPS Pin Setup ---
  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsISR, RISING);
}

// ========================================================================
// LOOP
// ========================================================================
void loop() {
  readGpsSerial(); // Process incoming GPS data

  // Example: Get and print precise timestamp AND location periodically
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 2000) { // Every 2 seconds
    lastCheck = millis();

    Serial.println("\n--- Reading Timestamp & Location ---");

    // --- Get Precise Timestamp ---
    unsigned long year, microsecond;
    unsigned int month, day, hour, minute, second;
    bool timeAvailable = getPreciseTimestamp(&year, &month, &day, &hour, &minute, &second, &microsecond);

    // --- Get Current Location ---
    float currentLat, currentLon;
    bool locationAvailable = getCurrentLocation(&currentLat, &currentLon);

    // --- Print Combined Results ---
    if (timeAvailable) {
       Serial.print("Precise Timestamp: ");
       Serial.print(year); Serial.print("/");
       if (month < 10) Serial.print('0'); Serial.print(month); Serial.print("/");
       if (day < 10) Serial.print('0'); Serial.print(day); Serial.print(" ");
       if (hour < 10) Serial.print('0'); Serial.print(hour); Serial.print(":");
       if (minute < 10) Serial.print('0'); Serial.print(minute); Serial.print(":");
       if (second < 10) Serial.print('0'); Serial.print(second); Serial.print(".");
       // Print microseconds, padding with leading zeros to 6 digits
       if(microsecond < 100000) Serial.print('0'); if(microsecond < 10000) Serial.print('0');
       if(microsecond < 1000) Serial.print('0'); if(microsecond < 100) Serial.print('0');
       if(microsecond < 10) Serial.print('0'); Serial.println(microsecond);
    } else {
       Serial.println("Precise Timestamp: Not available (waiting for NMEA/PPS sync)");
    }

    if (locationAvailable) {
      Serial.print("Current Location:  Lat=");
      Serial.print(currentLat, 6); // Print with 6 decimal places
      Serial.print(", Lon=");
      Serial.println(currentLon, 6);
    } else {
      Serial.println("Current Location:  No valid fix");
    }
     Serial.println("------------------------------------"); // Separator
  }
}

// ========================================================================
// PPS Interrupt Service Routine (ISR)
// ========================================================================
void ppsISR() {
  unsigned long currentMicros = micros();
  if (nmeTimeValid) { // Only update if NMEA time is considered valid
    ppsTimestampMicros = currentMicros;
    noInterrupts(); // --- Start Atomic Block ---
    ppsUtcYear = nmeUtcYear; ppsUtcMonth = nmeUtcMonth; ppsUtcDay = nmeUtcDay;
    ppsUtcHour = nmeUtcHour; ppsUtcMinute = nmeUtcMinute; ppsUtcSecond = nmeUtcSecond;
    interrupts(); // --- End Atomic Block ---
    ppsTimeValid = true;
  }
  newPpsFlag = true;
}


// ========================================================================
// getPreciseTimestamp: Calculates the precise time of its call
// ========================================================================
bool getPreciseTimestamp(unsigned long *pYear, unsigned int *pMonth, unsigned int *pDay,
                         unsigned int *pHour, unsigned int *pMinute, unsigned int *pSecond,
                         unsigned long *pMicrosecond)
{
  unsigned long functionCallMicros = micros();
  unsigned long lastPpsMicros;
  int year, month, day, hour, minute, second;
  bool valid;

  noInterrupts(); // --- Start Atomic Block ---
  lastPpsMicros = ppsTimestampMicros; year = ppsUtcYear; month = ppsUtcMonth; day = ppsUtcDay;
  hour = ppsUtcHour; minute = ppsUtcMinute; second = ppsUtcSecond; valid = ppsTimeValid;
  interrupts(); // --- End Atomic Block ---

  if (!valid) { return false; }

  unsigned long microsSincePps = functionCallMicros - lastPpsMicros;

  *pYear = year; *pMonth = month; *pDay = day; *pHour = hour; *pMinute = minute; *pSecond = second;
  *pMicrosecond = microsSincePps;

  // Optional check for unusually large microsecond value (> 1 sec)
  // if (microsSincePps >= 1000000) { Serial.println("Warning: Micros > 1M"); }

  return true;
}

// ========================================================================
// getCurrentLocation: Gets the most recent valid GPS coordinates.
// ========================================================================
bool getCurrentLocation(float *pLat, float *pLon) {
  bool validFix;
  float currentLat, currentLon;

  // Atomically read the fix status and coordinates
  noInterrupts(); // --- Start Atomic Block ---
  validFix = gpsFix;
  currentLat = latitude;
  currentLon = longitude;
  interrupts(); // --- End Atomic Block ---

  if (validFix) {
    *pLat = currentLat;
    *pLon = currentLon;
    return true; // Location data is valid
  } else {
    return false; // No valid fix currently available
  }
}


// ========================================================================
// readGpsSerial (Unchanged)
// ========================================================================
void readGpsSerial() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    if (c == '$' || bufferIndex >= MAX_NMEA_BUFFER - 1) { bufferIndex = 0; memset(nmeaBuffer, 0, MAX_NMEA_BUFFER); nmeaBuffer[bufferIndex++] = c; continue; }
    if (c != '\n' && c != '\r' && bufferIndex < MAX_NMEA_BUFFER - 1) { nmeaBuffer[bufferIndex++] = c; }
    if (c == '\n') { nmeaBuffer[bufferIndex] = '\0'; if (nmeaBuffer[0] == '$') { parseNmeaSentence(nmeaBuffer); } bufferIndex = 0; memset(nmeaBuffer, 0, MAX_NMEA_BUFFER); }
  }
}

// ========================================================================
// parseNmeaSentence (Updates volatile vars atomically)
// ========================================================================
void parseNmeaSentence(char *sentence) {
  if (strncmp(sentence + 1, "GNRMC", 5) == 0) {
    if (!validateChecksum(sentence)) { return; }

    char *field[20]; int fieldIndex = 0; field[fieldIndex++] = sentence;
    for (int i = 0; sentence[i] != '\0' && sentence[i] != '*' && fieldIndex < 20; i++) { if (sentence[i] == ',') { sentence[i] = '\0'; field[fieldIndex++] = &sentence[i + 1]; } }
    for (int i = 0; sentence[i] != '\0'; i++) { if (sentence[i] == '*') { sentence[i] = '\0'; break; } }

    if (fieldIndex >= 10) {
      int tempDay = 0, tempMonth = 0, tempYear = 0; int tempHour = 0, tempMinute = 0, tempSecond = 0;
      bool currentFix = false; float tempLat = 0.0, tempLon = 0.0; bool parseOk = true;

      // Field 2: Status
      if (strlen(field[2]) > 0) { currentFix = (field[2][0] == 'A'); }

      if (currentFix) {
        // Parse Time (Field 1)
        if (strlen(field[1]) >= 6) {
          char hh[3] = {field[1][0], field[1][1], '\0'}; char mm[3] = {field[1][2], field[1][3], '\0'}; char ss[3] = {field[1][4], field[1][5], '\0'};
          tempHour = atoi(hh); tempMinute = atoi(mm); tempSecond = atoi(ss);
        } else { parseOk = false; }
        // Parse Date (Field 9 - DDMMYY)
        if (parseOk && strlen(field[9]) == 6) {
          char dd[3] = {field[9][0], field[9][1], '\0'}; char mm[3] = {field[9][2], field[9][3], '\0'}; char yy[3] = {field[9][4], field[9][5], '\0'};
          tempDay = atoi(dd); tempMonth = atoi(mm); tempYear = atoi(yy) + 2000;
        } else { parseOk = false; }
        // Parse Lat (Field 3, 4)
        if (parseOk && strlen(field[3]) > 0 && strlen(field[4]) > 0) { tempLat = nmeaToDecimalDegrees(field[3], field[4][0]); } else { parseOk = false; }
        // Parse Lon (Field 5, 6)
        if (parseOk && strlen(field[5]) > 0 && strlen(field[6]) > 0) { tempLon = nmeaToDecimalDegrees(field[5], field[6][0]); } else { parseOk = false; }

        // --- Update Volatile Vars if everything parsed OK ---
        if (parseOk) {
          noInterrupts(); // --- Start Atomic Block ---
          // Update time
          nmeUtcYear = tempYear; nmeUtcMonth = tempMonth; nmeUtcDay = tempDay;
          nmeUtcHour = tempHour; nmeUtcMinute = tempMinute; nmeUtcSecond = tempSecond;
          nmeTimeValid = true;
          // Update location
          latitude = tempLat; longitude = tempLon; gpsFix = true;
          interrupts(); // --- End Atomic Block ---
        } else {
          // Parsing failed mid-sentence
          noInterrupts(); nmeTimeValid = false; gpsFix = false; interrupts();
        }
      } else { // No fix ('V' status)
        noInterrupts(); nmeTimeValid = false; gpsFix = false; interrupts();
      }
    } else { // Not enough fields
       noInterrupts(); nmeTimeValid = false; gpsFix = false; interrupts();
    }
  } // end GNRMC check
} // end parseNmeaSentence


// ========================================================================
// nmeaToDecimalDegrees (Unchanged)
// ========================================================================
float nmeaToDecimalDegrees(char *nmeaCoord, char hemisphere) {
  if (strlen(nmeaCoord) == 0) { return 0.0; } float rawCoord = atof(nmeaCoord);
  int degrees = (int)(rawCoord / 100.0); float minutes = rawCoord - (degrees * 100.0);
  float decimalDegrees = degrees + (minutes / 60.0);
  if (hemisphere == 'S' || hemisphere == 'W') { decimalDegrees = -decimalDegrees; }
  else if (hemisphere != 'N' && hemisphere != 'E') { return 0.0; } return decimalDegrees;
}


// ========================================================================
// hexCharToByte (Unchanged)
// ========================================================================
byte hexCharToByte(char c) {
  if (c >= '0' && c <= '9') { return c - '0'; } if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; } return 255;
}

// ========================================================================
// validateChecksum (Unchanged - using manual hex version)
// ========================================================================
bool validateChecksum(char *sentence) {
  if (sentence[0] != '$') { return false; } char *checksum_str_ptr = strchr(sentence, '*');
  if (checksum_str_ptr == NULL || strlen(checksum_str_ptr) < 3) { return false; } byte calculated_checksum = 0;
  for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) { calculated_checksum ^= sentence[i]; }
  byte high_nibble = hexCharToByte(*(checksum_str_ptr + 1)); byte low_nibble = hexCharToByte(*(checksum_str_ptr + 2));
  if (high_nibble == 255 || low_nibble == 255) { return false; } byte received_checksum = (high_nibble << 4) + low_nibble;
  return calculated_checksum == received_checksum;
}

// ========================================================================
// printGpsData (Prints NMEA derived data, can be removed if not needed)
// ========================================================================
void printGpsData() {
  bool validFix; float currentLat, currentLon;
  int year, month, day, hour, minute, second; bool validTime;

  noInterrupts(); // Read all volatile vars atomically
  validFix = gpsFix; currentLat = latitude; currentLon = longitude;
  year = nmeUtcYear; month = nmeUtcMonth; day = nmeUtcDay;
  hour = nmeUtcHour; minute = nmeUtcMinute; second = nmeUtcSecond;
  validTime = nmeTimeValid;
  interrupts();

  Serial.print("NMEA Data -> Location: ");
  if (validFix) { Serial.print(currentLat, 6); Serial.print(", "); Serial.print(currentLon, 6); }
  else { Serial.print("No Fix"); }
  Serial.print("  Time (UTC): ");
  if (validTime) { if (hour < 10) Serial.print('0'); Serial.print(hour); Serial.print(":"); if (minute < 10) Serial.print('0'); Serial.print(minute); Serial.print(":"); if (second < 10) Serial.print('0'); Serial.print(second); }
  else { Serial.print("--:--:--"); }
  Serial.print("  Date (UTC): ");
  if (validTime && year != 0) { if (day < 10) Serial.print('0'); Serial.print(day); Serial.print("/"); if (month < 10) Serial.print('0'); Serial.print(month); Serial.print("/"); Serial.print(year); }
  else { Serial.print("----/--/--"); }
  Serial.println();
}