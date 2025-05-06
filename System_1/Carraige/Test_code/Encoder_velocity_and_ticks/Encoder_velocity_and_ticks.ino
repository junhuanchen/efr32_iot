#include <math.h>   // For round(), isnan(), isinf()
#include <stdint.h> // For int16_t

// Pins:
#define ENCODER_1_PIN 14
#define ENCODER_2_PIN 13
#define ENCODER_SELECT_PIN 4

// Encoder and Wheel Properties
const float WHEEL_CIRCUMFERENCE_METERS = 0.6f;
const int TICKS_PER_REVOLUTION = 1;

// Global variables
volatile int distance_ticks = 0;
volatile bool moving_forward = false;
volatile unsigned long last_tick_time_us = 0;
float distance_per_tick = 0.0f;

volatile bool newDataAvailable = false;
volatile unsigned long captured_tick_time_us = 0;
volatile bool captured_direction_forward = false;
volatile bool captured_direction_backward = false;
volatile int latest_captured_distance_ticks = 0;

volatile int16_t g_currentVelocityMmPs = 0; // Holds the latest calculated velocity

// Function declarations
bool processEncoderUpdate(int16_t* pVelocity, int* pTicks, unsigned long* pIntervalUs, bool* pDirForward, bool* pDirBackward);
int getCurrentGlobalTickCount();
int16_t getCurrentVelocityMmPs(); // New accessor for velocity

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Encoder Test - On-Demand Velocity & Ticks Access");

  if (TICKS_PER_REVOLUTION > 0) {
    distance_per_tick = WHEEL_CIRCUMFERENCE_METERS / (float)TICKS_PER_REVOLUTION;
  } else {
    Serial.println("ERROR: TICKS_PER_REVOLUTION is zero or negative.");
    while(1);
  }
  Serial.print("Distance per tick: ");
  Serial.print(distance_per_tick, 6);
  Serial.println(" m");

  pinMode(ENCODER_SELECT_PIN, OUTPUT);
  pinMode(ENCODER_1_PIN, INPUT);
  pinMode(ENCODER_2_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), encoder1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), encoder2_ISR, FALLING);

  last_tick_time_us = micros();
}

void encoder1_ISR() {
  int encoder_1_state = digitalRead(ENCODER_1_PIN);
  if (encoder_1_state) { // If ENCODER_1_PIN is HIGH
    moving_forward = false;
  } else {                // If ENCODER_1_PIN is LOW
    moving_forward = true;
  }
}

void encoder2_ISR() {
  unsigned long current_event_capture_time = micros();
  int encoder_1_state_on_falling_edge = digitalRead(ENCODER_1_PIN);
  bool direction_confirmed_this_tick_forward = false;
  bool direction_confirmed_this_tick_backward = false;

  if (moving_forward) {
    if (encoder_1_state_on_falling_edge) {
        distance_ticks++;
        direction_confirmed_this_tick_forward = true;
    }
  } else {
    if (!encoder_1_state_on_falling_edge) {
        distance_ticks--;
        direction_confirmed_this_tick_backward = true;
    }
  }

  if (direction_confirmed_this_tick_forward || direction_confirmed_this_tick_backward) {
    if (!newDataAvailable) {
        captured_tick_time_us = current_event_capture_time;
        captured_direction_forward = direction_confirmed_this_tick_forward;
        captured_direction_backward = direction_confirmed_this_tick_backward;
        latest_captured_distance_ticks = distance_ticks;
        newDataAvailable = true;
    }
  }
}

// Safely gets the current global tick count
int getCurrentGlobalTickCount() {
  int ticks_snapshot;
  noInterrupts();
  ticks_snapshot = distance_ticks;
  interrupts();
  return ticks_snapshot;
}

// Gets the last calculated velocity
int16_t getCurrentVelocityMmPs() {
  // g_currentVelocityMmPs is volatile and updated by processEncoderUpdate(),
  // which is called from the main loop context. A direct read is safe here.
  return g_currentVelocityMmPs;
}

bool processEncoderUpdate(int16_t* pVelocity, int* pTicks, unsigned long* pIntervalUs, bool* pDirForward, bool* pDirBackward) {
  if (newDataAvailable) {
    unsigned long current_event_time_us_local;
    unsigned long previous_event_time_us_local;

    noInterrupts();
    current_event_time_us_local = captured_tick_time_us;
    previous_event_time_us_local = last_tick_time_us;
    *pDirForward = captured_direction_forward;
    *pDirBackward = captured_direction_backward;
    *pTicks = latest_captured_distance_ticks;
    newDataAvailable = false;
    interrupts();

    last_tick_time_us = current_event_time_us_local;

    *pIntervalUs = current_event_time_us_local - previous_event_time_us_local;
    int16_t calculated_vel = 0;

    if (*pIntervalUs > 0 && (*pDirForward || *pDirBackward)) {
      float speed_mps = (distance_per_tick * 1000000.0f) / (float)(*pIntervalUs);
      float signed_velocity_mmps_float = (*pDirForward ? 1.0f : -1.0f) * speed_mps * 1000.0f;

      if (isnan(signed_velocity_mmps_float) || isinf(signed_velocity_mmps_float)) {
        calculated_vel = 0;
      } else {
        float rounded_velocity = round(signed_velocity_mmps_float);
        if (rounded_velocity > 32767.0f) calculated_vel = 32767;
        else if (rounded_velocity < -32768.0f) calculated_vel = -32768;
        else calculated_vel = (int16_t)rounded_velocity;
      }
    }
    *pVelocity = calculated_vel;
    g_currentVelocityMmPs = calculated_vel; // Update global velocity
    return true;
  }
  return false;
}

// Timer for demonstrating on-demand access in loop()
unsigned long lastDemandReadTime = 0;

void loop() {
  digitalWrite(ENCODER_SELECT_PIN, HIGH); // Manage this pin as per your hardware needs

  // --- Step 1: Always process encoder updates ---
  // This ensures g_currentVelocityMmPs and distance_ticks are kept up-to-date
  // by the ISRs and this processing function.
  int16_t event_velocity_val;
  int event_ticks_val;
  unsigned long event_interval_val_us;
  bool event_dir_fwd_val, event_dir_bwd_val;

  processEncoderUpdate(&event_velocity_val, &event_ticks_val, &event_interval_val_us, &event_dir_fwd_val, &event_dir_bwd_val);


  // --- Step 2: Access velocity and ticks on demand ---
  // You can call getCurrentVelocityMmPs() and getCurrentGlobalTickCount()
  // whenever you need them. For example, based on a timer, a command, a button press etc.

  // Example: Get and print values "on demand" roughly every 1 second
  // Replace this with your actual condition for when you want the data.
  if (millis() - lastDemandReadTime >= 1000) { // Check every 1000 ms
    lastDemandReadTime = millis();

    int16_t current_velocity = getCurrentVelocityMmPs(); // Get latest velocity
    int current_total_ticks = getCurrentGlobalTickCount();   // Get latest total ticks

    Serial.print("ON_DEMAND_DATA (");
    Serial.print(millis()); // Timestamp the on-demand read
    Serial.print("ms) -> Velocity: ");
    Serial.print(current_velocity);
    Serial.print(" mm/s, Total Ticks: ");
    Serial.println(current_total_ticks);

    // Here you would use current_velocity and current_total_ticks
    // to send to your server or for other logic.
    // Example: sendDataToServer(current_velocity, current_total_ticks);
  }

  // Your loop should be free of long delays to ensure responsive
  // processing of encoder ticks and timely on-demand access.
  // Other non-blocking code can go here.
}