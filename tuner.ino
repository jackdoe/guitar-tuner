#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <math.h>

// ----------------------------------------------------------
// I2S & Audio Settings
// ----------------------------------------------------------
#define I2S_SD_PIN  4  // data pin
#define I2S_WS_PIN  2  // word select pin
#define I2S_SCK_PIN 3  // bit clock pin

// We'll run at 8 kHz for guitar fundamental detection
#define SAMPLE_RATE 8000
#define BUFFER_SIZE 512

// ----------------------------------------------------------
// Guitar Notes for Reference
// ----------------------------------------------------------
static const float NOTES[] = {82.41f, 110.00f, 146.83f, 196.00f, 246.94f, 329.63f};
static const char* NOTE_NAMES[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
static const int NUM_NOTES = 6;

#define MIN_FREQUENCY 70.0f
#define MAX_FREQUENCY 350.0f
#define TUNE_TOLERANCE 1.0f  // +/- 1 Hz

// ----------------------------------------------------------
// OLED Display
// ----------------------------------------------------------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, /*SCL*/ 6, /*SDA*/ 5);

// ----------------------------------------------------------
// Audio Buffer
// ----------------------------------------------------------
int32_t audioBuffer[BUFFER_SIZE];

// ----------------------------------------------------------
// YIN Parameters
// ----------------------------------------------------------
#define YIN_THRESHOLD      0.15f // Lower = more sensitive; higher = more conservative
#define YIN_HALF_BUFFER    (BUFFER_SIZE/2)

// ----------------------------------------------------------
// Ring Buffer for smoothing
// ----------------------------------------------------------
static const int SMOOTH_SIZE = 4; 
float freqHistory[SMOOTH_SIZE];
int freqIndex = 0; // current position in ring

// ----------------------------------------------------------
// Forward Declarations
// ----------------------------------------------------------
void setupI2S();
float getFrequency();
float computeYINFrequency(const float* buffer, int length, float sampleRate);
float computeMedian(float *arr, int size);
float getSmoothedFrequency(float newFreq);
void displayTuning(int noteIndex, float frequency, float difference);

void setup() {
  Serial.begin(115200);

  // Initialize I2S
  setupI2S();

  // Initialize OLED
  u8g2.begin();
  u8g2.setContrast(255);
  u8g2.setBusClock(400000); // 400kHz I2C
  u8g2.setFont(u8g2_font_9x18_tf);

  // Initialize freqHistory with zeros
  for (int i = 0; i < SMOOTH_SIZE; i++) {
    freqHistory[i] = 0.0f;
  }
}

void loop() {
  // 1) Get a new frequency reading from YIN
  float rawFreq = getFrequency();

  // 2) Smooth it (median filter over last SMOOTH_SIZE frames)
  float stableFreq = getSmoothedFrequency(rawFreq);

  // 3) Find the closest note
  int closestNoteIndex = -1;
  float difference = 0;
  float minDifference = 99999.0f;

  if (stableFreq >= MIN_FREQUENCY && stableFreq <= MAX_FREQUENCY) {
    for (int i = 0; i < NUM_NOTES; i++) {
      float currentDiff = fabsf(stableFreq - NOTES[i]);
      if (currentDiff < minDifference) {
        minDifference = currentDiff;
        closestNoteIndex = i;
        difference = (stableFreq - NOTES[i]);
      }
    }
  }

  // 4) Display
  displayTuning(closestNoteIndex, stableFreq, difference);

  // Slow down updates a bit if you prefer
  // delay(40);
}

// ----------------------------------------------------------
// I2S Setup
// ----------------------------------------------------------
void setupI2S() {
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0
  };

  // Initialize freqHistory with zeros
  i2s_pin_config_t pin_config = {
      .mck_io_num = I2S_PIN_NO_CHANGE,
      .bck_io_num = I2S_SCK_PIN,
      .ws_io_num = I2S_WS_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_SD_PIN
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// ----------------------------------------------------------
// Insert frequency into ring buffer, compute median to smooth
// ----------------------------------------------------------
float getSmoothedFrequency(float newFreq) {
  // Insert new frequency into history
  freqHistory[freqIndex] = newFreq;
  freqIndex = (freqIndex + 1) % SMOOTH_SIZE;

  // Copy valid (non-zero) entries to a temp array
  static float temp[SMOOTH_SIZE];
  int count = 0;
  for (int i = 0; i < SMOOTH_SIZE; i++) {
    if (freqHistory[i] > 0.0f) { // ignoring zeros
      temp[count++] = freqHistory[i];
    }
  }
  if (count == 0) {
    return 0.0f; // no valid entries
  }

  // Compute median
  // Sort the valid portion
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (temp[j] < temp[i]) {
        float tmp = temp[i];
        temp[i] = temp[j];
        temp[j] = tmp;
      }
    }
  }

  // If count is odd, pick middle; if even, average the two middle
  if (count % 2 == 1) {
    return temp[count / 2];
  } else {
    float a = temp[(count / 2) - 1];
    float b = temp[count / 2];
    return 0.5f * (a + b);
  }
}

// ----------------------------------------------------------
// Read from I2S and compute pitch via YIN
// ----------------------------------------------------------
float getFrequency() {
  size_t bytesRead = 0;
  i2s_read(I2S_NUM_0, audioBuffer, BUFFER_SIZE * sizeof(int32_t), &bytesRead, portMAX_DELAY);

  // Convert to floats, remove DC offset, also measure RMS amplitude for gating
  double sum = 0.0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += audioBuffer[i];
  }
  float mean = (float)(sum / BUFFER_SIZE);

  // Populate a float buffer suitable for YIN
  static float floatBuf[BUFFER_SIZE];
  double sqSum = 0.0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    float centered = (float)audioBuffer[i] - mean;
    floatBuf[i] = centered;
    sqSum += (double)centered * (double)centered;
  }

  // Compute RMS amplitude to ignore extremely quiet signals
  float rms = sqrtf((float)(sqSum / BUFFER_SIZE));
  // Adjust the amplitude gate to taste (depends on microphone, preamp, etc.)
  if (rms < 0.005f) {
    // Too quiet, likely no note
    return 0.0f;
  }

  // Compute pitch using the YIN method
  float yinFreq = computeYINFrequency(floatBuf, BUFFER_SIZE, (float)SAMPLE_RATE);

  return yinFreq;
}
float computeYINFrequency(const float* buffer, int length, float sampleRate) {
  static float yin[BUFFER_SIZE/2];
  int halfBuffer = length / 2; // e.g. 512 if length=1024

  // 1) Difference function d[tau]
  for (int tau = 0; tau < halfBuffer; tau++) {
    double sum = 0.0;
    for (int i = 0; i < halfBuffer; i++) {
      float diff = buffer[i] - buffer[i + tau];
      sum += diff * diff;
    }
    yin[tau] = (float)sum;
  }

  // 2) Cumulative mean normalized difference
  yin[0] = 1.0f; // avoid divide-by-zero
  double runningSum = 0.0;
  for (int tau = 1; tau < halfBuffer; tau++) {
    runningSum += yin[tau];
    yin[tau] = (float)(yin[tau] * tau / runningSum);
  }

  // 3) Find the first tau that goes below YIN_THRESHOLD,
  //    then apply parabolic interpolation around that tau.
  int tauEstimate = -1;
  for (int tau = 1; tau < halfBuffer; tau++) {
    if (yin[tau] < YIN_THRESHOLD) {
      tauEstimate = tau;
      break;
    }
  }

  // If we didn't find a crossing, no valid pitch
  if (tauEstimate == -1) {
    return 0.0f;
  }

  // ---- Parabolic interpolation ----
  // We'll refine tauEstimate to a floating-point value by looking
  // at yin[tauEstimate-1], yin[tauEstimate], yin[tauEstimate+1].
  float finalTau = (float)tauEstimate;
  if (tauEstimate > 1 && tauEstimate < (halfBuffer - 1)) {
    float y1 = yin[tauEstimate - 1];
    float y2 = yin[tauEstimate];
    float y3 = yin[tauEstimate + 1];
    float denom = (y1 - 2.0f * y2 + y3);

    // Only interpolate if the denominator isn't too small
    if (fabsf(denom) > 1e-7) {
      float shift = 0.5f * (y1 - y3) / denom;
      finalTau = (float)tauEstimate + shift;
    }
  }

  // 4) Convert tau to frequency
  float frequency = sampleRate / finalTau;
  return frequency;
}

// ----------------------------------------------------------
// Display Tuning
// ----------------------------------------------------------
void displayTuning(int noteIndex, float frequency, float difference) {
  u8g2.clearBuffer();
  
  // Just for layout
  int xOffset = 10; 
  int yOffset = 10; 

  if (noteIndex >= 0 && frequency >= MIN_FREQUENCY && frequency <= MAX_FREQUENCY) {
    // Note name
    u8g2.setCursor(xOffset, yOffset);
    u8g2.print(NOTE_NAMES[noteIndex]);

    // Frequency
    u8g2.setCursor(xOffset + 40, yOffset);
    u8g2.print(frequency, 1);
    u8g2.print(" Hz");

    // Tuning indicator
    u8g2.setCursor(xOffset, yOffset + 25);
    float absDiff = fabsf(difference);
    if (absDiff < TUNE_TOLERANCE) {
      u8g2.print("<3 IN TUNE");
    } else {
      if (difference < 0) {
        u8g2.print("T ");
        u8g2.print(difference, 1);
        u8g2.print(" Hz");
      } else {
        u8g2.print("L +");
        u8g2.print(difference, 1);
        u8g2.print(" Hz");
      }
    }
  } else {
    // No valid note
    u8g2.setCursor(xOffset, yOffset);
    u8g2.print("??");
    u8g2.setCursor(xOffset + 30, yOffset);
    u8g2.print(frequency, 1);
    u8g2.print(" Hz");
  }

  // Just a fun message
  u8g2.setCursor(xOffset, 60);
  u8g2.print("YOU ROCK!");
  u8g2.sendBuffer();
}
