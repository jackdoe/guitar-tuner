#include <U8g2lib.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <math.h>

// Adjust these pins for your microphone breakout:
#define I2S_SD_PIN  4   // SD (data) pin
#define I2S_WS_PIN  2   // WS (word select) pin
#define I2S_SCK_PIN 3   // SCK (bit clock) pin

// Define standard guitar string frequencies (E2, A2, D3, G3, B3, E4)
const float NOTES[] = {82.41, 110.00, 146.83, 196.00, 246.94, 329.63};
const char* NOTE_NAMES[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
const int NUM_NOTES = 6;

// Tolerance in Hz for considering a note "in tune"
const float TUNE_TOLERANCE = 1.0;

// Try a higher sample rate for better frequency resolution
#define SAMPLE_RATE 8000

// Increase or decrease as desired (power of 2). Larger → finer resolution, but slower updates.
#define BUFFER_SIZE 4096

// Lower and upper frequency bounds you expect for a guitar note
#define MIN_FREQUENCY 70.0
#define MAX_FREQUENCY 350.0

// Arrays for FFT
float vReal[BUFFER_SIZE];
float vImag[BUFFER_SIZE];

// We create the FFT object
ArduinoFFT<float> FFT(vReal, vImag, BUFFER_SIZE, SAMPLE_RATE);

// OLED (make sure pins match your setup)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 6, 5);
int xOffset = 10; 
int yOffset = 10; 

// Buffer for I2S audio samples
int32_t audioBuffer[BUFFER_SIZE];

// Forward declarations
float processAudioBuffer();
float findPeakFrequency();
float checkHarmonics(float rawFreq);
void displayTuning(int noteIndex, float frequency, float difference);

void setup(void) {
    Serial.begin(115200);

    // I2S config
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, 
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = true,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_SCK_PIN,    // BCK
        .ws_io_num = I2S_WS_PIN,      // WS
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN     // SD
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);

    // OLED init
    u8g2.begin();
    u8g2.setContrast(255);
    u8g2.setBusClock(400000); //400kHz I2C 
    u8g2.setFont(u8g2_font_9x18_tf);
}

void loop(void) {
    size_t bytesRead = 0;

    // Read BUFFER_SIZE * 4 bytes (32-bit samples) from I2S
    i2s_read(I2S_NUM_0, audioBuffer, BUFFER_SIZE * sizeof(int32_t), &bytesRead, portMAX_DELAY);

    // Process the audio to get a raw frequency from the FFT
    float rawFreq = processAudioBuffer();

    // Attempt to correct for harmonics, e.g., if it's picking up the 2nd harmonic
    float dominantFreq = checkHarmonics(rawFreq);

    // Find the closest note
    int closestNoteIndex = -1;
    float difference = 0;
    float minDifference = 9999;
    if (dominantFreq > MIN_FREQUENCY && dominantFreq < MAX_FREQUENCY) {
      for (int i = 0; i < NUM_NOTES; i++) {
          float currentDiff = fabsf(dominantFreq - NOTES[i]);
          if (currentDiff < minDifference) {
              minDifference = currentDiff;
              closestNoteIndex = i;
              difference = (dominantFreq - NOTES[i]);
          }
      }
    }

    // Display the results
    displayTuning(closestNoteIndex, dominantFreq, difference);
}

float processAudioBuffer() {
    // 1) Compute average (DC offset)
    long long sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        int32_t sample = audioBuffer[i];
        sum += sample;
    }
    float avg = (float)sum / BUFFER_SIZE;

    // 2) Subtract average and populate vReal
    for (int i = 0; i < BUFFER_SIZE; i++) {
        float centeredSample = ((float) audioBuffer[i]) - avg;
        vReal[i] = centeredSample;
        vImag[i] = 0.0; 
    }

    // 3) Windowing + FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    // 4) Find peak frequency with sub-bin interpolation
    float freq = findPeakFrequency();
    return freq;
}

/**
 * Manually find the bin of max magnitude in vReal[0..(BUFFER_SIZE/2 - 1)]
 * then use a simple parabolic interpolation around that bin to get sub-bin accuracy.
 */
float findPeakFrequency() {
    // Ignore DC (bin 0) to avoid picking up offset noise
    int startBin = 1;
    int endBin = BUFFER_SIZE / 2; // Nyquist limit

    float maxMagnitude = 0.0;
    int maxIndex = -1;

    // Find the index of the largest bin
    for(int i = startBin; i < endBin; i++) {
        if(vReal[i] > maxMagnitude) {
            maxMagnitude = vReal[i];
            maxIndex = i;
        }
    }

    if (maxIndex <= 0) {
        return 0.0; // No valid peak found
    }

    // Parabolic interpolation:
    // Suppose the largest bin is i. We look at bins i-1 and i+1
    // freqBin = i + 0.5*(A - C) / (A - 2B + C)
    // Where B is the amplitude at i, A at i-1, C at i+1
    float A = vReal[maxIndex - 1];
    float B = vReal[maxIndex];
    float C = vReal[maxIndex + 1];

    float denom = (A - 2 * B + C);
    float interpolation = 0.0;
    if (fabsf(denom) > 1e-6) {
        interpolation = 0.5f * (A - C) / denom;
    }

    float interpolatedIndex = (float)maxIndex + interpolation;

    // Convert bin index to frequency
    // Frequency resolution = sampleRate / FFT_size
    float peakFreq = interpolatedIndex * ((float)SAMPLE_RATE / (float)BUFFER_SIZE);
    return peakFreq;
}

/**
 * Attempt to detect if the rawFreq is actually a strong harmonic:
 * If rawFreq is above ~120 Hz, we check if half that frequency is still
 * within range and might be the "true" fundamental. 
 * This is a simplistic approach—feel free to improve or extend.
 */
float checkHarmonics(float rawFreq) {
    if (rawFreq <= 0) return 0.0f;

    // If the raw freq is suspiciously high (say above G3/B3 range),
    // check if half or a third is also in the valid range.
    // For guitar E2=82.4 up to E4=329.6, so let's do a couple checks:
    if (rawFreq > 140.0f) { 
        float halfF = rawFreq * 0.5f;
        if (halfF >= MIN_FREQUENCY && halfF <= MAX_FREQUENCY) {
            // Optionally, we can see if the magnitude at halfF’s bin is also large
            // but here we just “prefer” half if it’s in range:
            return halfF;
        }
    }

    // You could also check rawFreq * 0.333..., etc., for triple harmonics.
    // We'll keep it simple here.
    return rawFreq;
}

void displayTuning(int noteIndex, float frequency, float difference) {
    u8g2.clearBuffer();
    
    // Only display tuning information if we have a valid frequency
    if (noteIndex >= 0 && frequency > MIN_FREQUENCY && frequency < MAX_FREQUENCY) {
        // Display the note name
        u8g2.setCursor(xOffset, yOffset);
        u8g2.print(NOTE_NAMES[noteIndex]);
        
        // Display the detected frequency
        u8g2.setCursor(xOffset + 30, yOffset);
        u8g2.print(frequency, 1);
        u8g2.print("Hz");
        
        // Display tuning indicator
        u8g2.setCursor(xOffset, yOffset + 25);
        if (fabsf(difference) < TUNE_TOLERANCE) {
            u8g2.print("<3 IN TUNE");
        } else {
            if (difference < 0) {
                // Flat (need to tighten)
                u8g2.print("T ");
                u8g2.print(difference, 1);
                u8g2.print("Hz");
            } else {
                // Sharp (need to loosen)
                u8g2.print("L +");
                u8g2.print(difference, 1);
                u8g2.print("Hz");
            }
        }
    } else {
        u8g2.setCursor(xOffset, yOffset);
        u8g2.print("?");

        u8g2.setCursor(xOffset + 30, yOffset);
        u8g2.print(frequency, 1);
        u8g2.print("Hz");
    }
    
    u8g2.setCursor(xOffset, 60);
    u8g2.print("YOU ROCK!");
    u8g2.sendBuffer();
}
