#define ENCODER_PIN_A 4  // GPIO pin for encoder A signal
#define ENCODER_PIN_B 5  // GPIO pin for encoder B signal
#define PULSES_PER_REV 2100// Number of pulses per revolution

volatile int pulseCount = 0;     // Pulse counter
volatile bool direction = true; // true: forward, false: backward

unsigned long prevTime = 0;     // Time for RPM calculation

void IRAM_ATTR readEncoder() {
  // Read the state of the B channel
  bool stateB = digitalRead(ENCODER_PIN_B);

  // Update direction based on A and B channels
  direction = stateB != digitalRead(ENCODER_PIN_A); 

  // Increment or decrement pulse count based on direction
  pulseCount += direction ? 1 : -1;
}

void setup() {
  Serial.begin(115200);

  // Configure encoder pins as input with internal pull-ups
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // Attach an interrupt to the A channel
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), readEncoder, RISING); // Trigger on rising edge
}

void loop() {
  // Calculate RPM
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - prevTime;

  if (elapsedTime >= 1000) { // Update every 1 second
    int pulses = pulseCount; // Get the current pulse count
    pulseCount = 0;          // Reset the pulse count for the next calculation

    // Calculate RPM
    float rpm = ((pulses / (float)PULSES_PER_REV) * (60000.0 / elapsedTime))/350;

    // Print RPM and direction
    Serial.print("RPM: ");
    Serial.print(rpm);
    Serial.print(" | Direction: ");
    Serial.println(direction ? "Forward" : "Backward");

    prevTime = currentTime; // Update the previous time
  }
}
