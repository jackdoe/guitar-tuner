#include <U8g2lib.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <math.h>

#define I2S_SD_PIN  4  // SD (data) pin
#define I2S_WS_PIN  2  // WS (word select) pin
#define I2S_SCK_PIN 3  // SCK (bit clock) pin

#define log2(x) log(x)/log(2)

// Define standard guitar string frequencies (E2, A2, D3, G3, B3, E4)
const float NOTES[] = {82.41, 110.00, 146.83, 196.00, 246.94, 329.63};
const char* NOTE_NAMES[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
const int NUM_NOTES = 6;

// Tolerance in Hz for considering a note "in tune"
const float TUNE_TOLERANCE = 1.0;

#define SAMPLE_RATE 44100
#define BUFFER_SIZE 512
const uint16_t samples = BUFFER_SIZE; // This value MUST ALWAYS be a power of 2
unsigned int samplingPeriod;

float vReal[samples];
float vImag[samples];

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, SAMPLE_RATE); /* Create FFT object */

// there is no 72x40 constructor in u8g2 hence the 72x40 screen is
// mapped in the middle of the 132x64 pixel buffer of the SSD1306 controller
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 6, 5);
int xOffset = 10; 
int yOffset = 10; 

// Add timing and averaging variables
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 100; // 100ms between updates
float frequencyAccumulator = 0.0;
int sampleCount = 0;

// Define buffer size for audio samples
int32_t audioBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// Improve frequency detection with better thresholds and filtering
#define MIN_AMPLITUDE_THRESHOLD 0.01  // Minimum amplitude to consider a frequency valid
#define MIN_FREQUENCY 70.0            // Lowest frequency to detect (below E2)
#define MAX_FREQUENCY 350.0           // Highest frequency to detect (above E4)
#define NOISE_FLOOR 0.005             // Noise floor for signal detection

// Add a confidence measure for detected notes
float lastValidFrequency = 0.0;
int consecutiveReadings = 0;
const int REQUIRED_CONSECUTIVE = 3;   // Number of consecutive readings needed to confirm a note

// Add a buffer to store recent frequency readings
#define FREQ_HISTORY_SIZE 5
float recentFrequencies[FREQ_HISTORY_SIZE] = {0};
int freqHistoryIndex = 0;

void setup(void) {
    Serial.begin(115200);
    
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

    u8g2.begin();
    u8g2.setContrast(255); // set contrast to maximum 
    u8g2.setBusClock(400000); //400kHz I2C 
    u8g2.setFont(u8g2_font_9x18_tf);

    samplingPeriod = round(1000000 * (1.0/SAMPLE_RATE));
}

void loop(void) {
    // Fill the audio buffer
    if (!bufferFull) {
        size_t bytesRead = 0;
        int32_t tempBuffer[samples];
        
        // Read audio samples from I2S
        i2s_read(I2S_NUM_0, tempBuffer, samples * 4, &bytesRead, portMAX_DELAY);
        
        // Add samples to our buffer
        for (int i = 0; i < samples && bufferIndex < BUFFER_SIZE; i++) {
            audioBuffer[bufferIndex++] = tempBuffer[i];
        }
        
        // Check if buffer is full
        if (bufferIndex >= BUFFER_SIZE) {
            bufferFull = true;
        }
    }
    
    // Process data when buffer is full
    if (bufferFull) {
        float dominantFreq = processAudioBuffer();
        
        // Add to frequency history
        recentFrequencies[freqHistoryIndex] = dominantFreq;
        freqHistoryIndex = (freqHistoryIndex + 1) % FREQ_HISTORY_SIZE;
        
        // Get most common frequency from history
        float mostCommonFreq = getMostCommonFrequency();
        
        // Find closest note and calculate tuning
        int closestNoteIndex = -1;
        float minDifference = 1000.0;
        float difference = 0.0;
        
        for (int i = 0; i < NUM_NOTES; i++) {
            float currentDiff = abs(mostCommonFreq - NOTES[i]);
            if (currentDiff < minDifference) {
                minDifference = currentDiff;
                closestNoteIndex = i;
                difference = mostCommonFreq - NOTES[i];
            }
        }
        
        // Display results on OLED
        displayTuning(closestNoteIndex, mostCommonFreq, difference);
        
        // Reset buffer for next cycle
        bufferIndex = 0;
        bufferFull = false;
    }
}

float processAudioBuffer() {
    // Copy chunk to FFT input arrays
    for (int i = 0; i < samples; i++) {
        vReal[i] = (float)audioBuffer[i];
        vImag[i] = 0.0;
    }

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	/* Weigh data */
    FFT.compute(FFTDirection::Forward); /* Compute FFT */
    FFT.complexToMagnitude(); /* Compute magnitudes */
    float x = FFT.majorPeak();
    return x;
}

void displayTuning(int noteIndex, float frequency, float difference) {
    u8g2.clearBuffer();
    
    // Only display tuning information if we have a valid frequency
    if (noteIndex >= 0 && frequency > MIN_FREQUENCY) {
        // Display the note name
        u8g2.setCursor(xOffset, yOffset);
        u8g2.print(NOTE_NAMES[noteIndex]);
        
        // Display the detected frequency
        u8g2.setCursor(xOffset + 30, yOffset);
        u8g2.print(frequency, 1);
        u8g2.print("Hz");
        
        // Display tuning indicator with symbols
        u8g2.setCursor(xOffset, yOffset + 25);
        if (abs(difference) < TUNE_TOLERANCE) {
            // In tune - display heart
            u8g2.print("<3 IN TUNE");
        } else {
            // Calculate position for tuning indicator
            if (difference < 0) {
                // Flat - need to tighten
                u8g2.print("T (");
                u8g2.print(abs(difference), 1);
                u8g2.print("Hz)");
            } else {
                // Sharp - need to loosen
                u8g2.print("L (");
                u8g2.print(abs(difference), 1);
                u8g2.print("Hz)");
            }
        }
    } else {
        // No note detected - simple message
        u8g2.setCursor(xOffset, yOffset);
        u8g2.print("?");
    }
    
    u8g2.setCursor(xOffset, 60);
    u8g2.print("YOU ROCK!");
    u8g2.sendBuffer();
}

// New function to get the most common frequency from recent readings
float getMostCommonFrequency() {
    // Simple approach: group similar frequencies and find the most common group
    
    // First check if we have enough readings
    bool hasValidReadings = false;
    for (int i = 0; i < FREQ_HISTORY_SIZE; i++) {
        if (recentFrequencies[i] > MIN_FREQUENCY) {
            hasValidReadings = true;
            break;
        }
    }
    
    if (!hasValidReadings) {
        return 0; // No valid readings yet
    }
    
    // Group similar frequencies (within 2Hz of each other)
    const float SIMILARITY_THRESHOLD = 2.0;
    int bestGroupSize = 0;
    float bestGroupValue = 0;
    
    for (int i = 0; i < FREQ_HISTORY_SIZE; i++) {
        if (recentFrequencies[i] < MIN_FREQUENCY) continue;
        
        // Count frequencies similar to this one
        int groupSize = 0;
        float sum = 0;
        
        for (int j = 0; j < FREQ_HISTORY_SIZE; j++) {
            if (recentFrequencies[j] < MIN_FREQUENCY) continue;
            
            if (abs(recentFrequencies[i] - recentFrequencies[j]) < SIMILARITY_THRESHOLD) {
                groupSize++;
                sum += recentFrequencies[j];
            }
        }
        
        // If this group is bigger than our best so far, update
        if (groupSize > bestGroupSize) {
            bestGroupSize = groupSize;
            bestGroupValue = sum / groupSize; // Average of the group
        }
    }
    
    return bestGroupValue;
} 
