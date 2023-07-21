#include <limits.h>
#include <Arduino.h>

// FHT, see http://wiki.openmusiclabs.com/wiki/ArduinoFHT
#define LOG_OUT 1 // use the log output function
#define FHT_N 128 // amount of bins to use
#include <FHT.h> // include the library

#define FreqLog // use log scale for FHT frequencies
#define FreqGainFactorBits 0 // 6  // use 6 for justification
//#define FreqSerialBinary

// Macros for faster sampling, see
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Set to true if you want to use the FHT 128 channel analyser to visualize
// the detected frequencies. Will disable beat detection.
const bool LOG_FREQUENCY_DATA = false;

// Set to true if the light should be based on detected beats instead
// of detected amplitudes.
const bool PERFORM_BEAT_DETECTION = false;

const int SOUND_REFERENCE_PIN = 8; // D8
const int HAT_LIGHTS_PIN = 9; // D9
const int HAT_LIGHTS_LOW_PIN = 11; // D11
const int HAT_LIGHTS_HIGH_PIN = 12; // D12
const int HAT_LIGHTS_PULSE_PIN = 13; // D13

const int LIGHT_PULSE_DELAY = 2000;
const int LIGHT_PULSE_DURATION = 500;

const int LIGHT_FADE_OUT_DURATION = 200; // good value range is [100:1000]
const float MINIMUM_LIGHT_INTENSITY = 0.01; // in range [0:1]
const float MAXIMUM_LIGHT_INTENSITY = 1.0; // in range [0:1]

const int MAXIMUM_SIGNAL_VALUE = 1024;
const int INTENSITY_FACTOR = MAXIMUM_SIGNAL_VALUE/3;

const int OVERALL_FREQUENCY_RANGE_START = 2; // should be 0, but first 2 bands produce too much noise
const int OVERALL_FREQUENCY_RANGE_END = FHT_N / 2;
const int OVERALL_FREQUENCY_RANGE = OVERALL_FREQUENCY_RANGE_END - OVERALL_FREQUENCY_RANGE_START;

const int FIRST_FREQUENCY_RANGE_START = 2;
const int FIRST_FREQUENCY_RANGE_END = 4;
const int FIRST_FREQUENCY_RANGE = FIRST_FREQUENCY_RANGE_END - FIRST_FREQUENCY_RANGE_START;

const int SECOND_FREQUENCY_RANGE_START = 2;
const int SECOND_FREQUENCY_RANGE_END = 6;
const int SECOND_FREQUENCY_RANGE = SECOND_FREQUENCY_RANGE_END - SECOND_FREQUENCY_RANGE_START;

const int MAXIMUM_BEATS_PER_MINUTE = 300;
const int MINIMUM_DELAY_BETWEEN_BEATS = 60000L / MAXIMUM_BEATS_PER_MINUTE;
const int SINGLE_BEAT_DURATION = 50; // good value range is [50:150]

const int FREQUENCY_MAGNITUDE_SAMPLES = 7; // good value range is [5:15]

struct runningStats {
  int current;
  long total;
  int average;
  int variance;
  int history[FREQUENCY_MAGNITUDE_SAMPLES];
  size_t historyIndex;

  runningStats() : current(0), total(0), average(0), variance(0), historyIndex(0) { memset(&history, 0 , sizeof(history)); };
  void update(int value);
};

runningStats overallFrequency, firstFrequency, secondFrequency, signal;

long lastBeatTimestamp = 0;
long durationSinceLastBeat = 0;
float beatProbability = 0;
float beatProbabilityThreshold = 0.5;

long lightIntensityBumpTimestamp = 0;
float lightIntensityBumpValue = 0;
float lightIntensityValue = 0;

/**
 * Converts the specified value into an ASCII-art progressbar
 * with the specified length.
 */
String toProgressBar(float value, const int length) {
  int amount = max(0, min(length, value * length));
  String progressBar = "[";
  for (int i = 0; i < amount; i++) {
    progressBar += "=";
  }
  for (int i = 0; i < length - amount; i++) {
    progressBar += " ";
  }
  progressBar += "]";
  return progressBar;
}

void logValue(String name, float value, int length) {
  Serial.print(" | " + name + ": " + value + " " + toProgressBar(value, length));
}

void logValue(String name, float value) {
  logValue(name, value, 10);
}

void logValue(String name, boolean value) {
  logValue(name, value ? 1.0 : 0.0, 1);
}


/**
 * Analog to Digital Conversion needs to be configured to free running mode
 * in order to read the sound sensor values at a high frequency.
 *
 * See: http://maxembedded.com/2011/06/the-adc-of-the-avr/
 */
void setupADC() {
  ADCSRA = 0xe0+7; // "ADC Enable", "ADC Start Conversion", "ADC Auto Trigger Enable" and divider.
  ADMUX = 0x0; // use adc0. Use ARef pin for analog reference (same as analogReference(EXTERNAL)).
  //ADMUX |= 0x40; // Use Vcc for analog reference.
  DIDR0 = 0x01; // turn off the digital input for adc0
}

void runningStats::update(int value) {  
  total -= history[historyIndex]; // subtract the oldest history value from the total
  total += current; // add the current value to the total
  history[historyIndex] = current; // add the current value to the history
  if (++historyIndex >= FREQUENCY_MAGNITUDE_SAMPLES) historyIndex = 0;
  
  average = total / FREQUENCY_MAGNITUDE_SAMPLES;
  
  // update the variance of frequency magnitudes
  long squaredDifferenceSum = 0;
  for (int i = 0; i < FREQUENCY_MAGNITUDE_SAMPLES; i++) {
    long diff = history[i] - average;
    squaredDifferenceSum += (diff * diff);
  }
  variance = squaredDifferenceSum / FREQUENCY_MAGNITUDE_SAMPLES;

  // Finally, save the new value
  // Statistics do not include it, so variance analysis is based on prior samples
  current = value;
}


uint32_t sqrt32 (uint32_t n)
{
    // http://www.codecodex.com/wiki/Calculate_an_integer_square_root#C
    // Another very fast algorithm donated by Tristan.Muntsinger@gmail.com
    // (note: tested across the full 32 bits range, see comment below)

    // 15 iterations (c=1<<15)

    uint32_t c = 0x8000;
    uint32_t g = 0x8000;

    for(;;)
    {
        if (g*g > n)
            g ^= c;
        c >>= 1;
        if (!c)
            return g;
        g |= c;
    }
}


/**
 * Will read the sound sensor values from pin A0.
 */
void readAudioSamples() {
  long currentAverage = 0;
  int currentMaximum = INT_MIN;
  int currentMinimum = INT_MAX;
  
  for (int i = 0; i < FHT_N; i++) { // save 256 samples
    while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
    sbi(ADCSRA, ADIF); // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = ((int) j << 8) | m; // form into an int
    
    k -= 0x0200; // form into a signed int
    currentMinimum = min(currentMinimum, k);
    currentMaximum = max(currentMaximum, k);    
    currentAverage += k;    

    k <<= FreqGainFactorBits;    
    fht_input[i] = k; // put real data into bins
  }
  
  currentAverage /= FHT_N;

  // WM: Original logic - not really sure why this makes any sense??
  //int signalDelta = currentMaximum - currentAverage;
  //signal.current = currentAverage + (2 * signalDelta);  
  //constrain(signal.current, 0, currentMaximum);

  // Signal power  
  // roughly equivalent to the full FHT output ..
/*
  long variance = 0;
  for (int i = 0; i < FHT_N; ++i) {
    long v_offset = (fht_input[i] - currentAverage);
    variance += (v_offset * v_offset);
  }
  variance /= FHT_N;
  signal.update(sqrt32(variance));
*/  

  // vPP
  signal.update(currentMaximum - currentMinimum);
  Serial.print(signal.current);

   
  logValue("S", (float) signal.current / MAXIMUM_SIGNAL_VALUE, 10);
  //logValue("A", (float) signal.average / MAXIMUM_SIGNAL_VALUE, 10);
  //logValue("M", (float) currentMaximum / MAXIMUM_SIGNAL_VALUE, 10);  
}

/**
 * Will run the Fast Hartley Transform to convert the time domain signals
 * to the frequency domain.
 *
 * See: http://wiki.openmusiclabs.com/wiki/ArduinoFHT
 */
void getFrequencyData() {
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the FHT
  fht_run(); // process the data in the FHT
  fht_mag_log(); // get the magnitude of each bin in the FHT
}

void logFrequencyData() {
#ifdef FreqSerialBinary
  // print as binary
  Serial.write(255); // send a start byte
  Serial.write(fht_log_out, FHT_N / 2); // send out the data
#else
  // print as text
  for (int i = 0; i < FHT_N / 2; i++) {
      Serial.print(fht_log_out[i]);
      Serial.print(',');
  }
  Serial.print("\n");
#endif
}


byte getFrequencyMagnitude(byte frequencies[], const int startIndex, const int endIndex) {
  int total = 0;
  int average = 0;
  
  for (int i = startIndex; i < endIndex; i++) {
    int current = frequencies[i];
    total += current;
  }
  
  average = total / (endIndex - startIndex);
  
  logValue("F", (float) average / 128, 10);
  
  return average;
}


/**
 * Will calculate a value in range [0:2] based on the magnitude changes of
 * different frequency bands.
 * Low values are indicating a low beat probability.
 */
float calculateSignalChangeFactor() {
  float aboveAverageSignalFactor;
/*
  auto sq_diff = signal.current - signal.average;
  sq_diff *= sq_diff;
  if (sq_diff < (2*signal.variance)) {
    aboveAverageSignalFactor = 0;
  } else */{
    aboveAverageSignalFactor = ((float) (signal.current - signal.average) / (2*sqrt32(signal.variance)));
    aboveAverageSignalFactor = constrain(aboveAverageSignalFactor, 0, 2);
  }
  
  //logValue("SC", (float) signal.current / 512, 10);
  //logValue("SA", (float) signal.average / 512, 10);  
  logValue("SF", aboveAverageSignalFactor / 2, 2);
  return aboveAverageSignalFactor;
}

/**
 * Will calculate a value in range [0:1] based on the magnitude changes of
 * different frequency bands.
 * Low values are indicating a low beat probability.
 */
float calculateMagnitudeChangeFactor() {   
  // current overall magnitude is higher than the average, probably 
  // because the signal is mainly noise
  float aboveAverageOverallMagnitudeFactor = ((float) overallFrequency.current / overallFrequency.average);
  aboveAverageOverallMagnitudeFactor -= 1.05;
  aboveAverageOverallMagnitudeFactor *= 10;
  aboveAverageOverallMagnitudeFactor = constrain(aboveAverageOverallMagnitudeFactor, 0, 1);
  
  // current magnitude is higher than the average, probably 
  // because the there's a beat right now
  float aboveAverageFirstMagnitudeFactor = ((float) firstFrequency.current / firstFrequency.average);
  aboveAverageOverallMagnitudeFactor -= 0.1;
  aboveAverageFirstMagnitudeFactor *= 1.5;
  aboveAverageFirstMagnitudeFactor = pow(aboveAverageFirstMagnitudeFactor, 3);
  aboveAverageFirstMagnitudeFactor /= 3;
  aboveAverageFirstMagnitudeFactor -= 1.25;
  
  aboveAverageFirstMagnitudeFactor = constrain(aboveAverageFirstMagnitudeFactor, 0, 1);
  
  float aboveAverageSecondMagnitudeFactor = ((float) secondFrequency.current / secondFrequency.average);
  aboveAverageSecondMagnitudeFactor -= 1.01;
  aboveAverageSecondMagnitudeFactor *= 10;
  aboveAverageSecondMagnitudeFactor = constrain(aboveAverageSecondMagnitudeFactor, 0, 1);
  
  float magnitudeChangeFactor = aboveAverageFirstMagnitudeFactor;
  if (magnitudeChangeFactor > 0.15) {
    magnitudeChangeFactor = max(aboveAverageFirstMagnitudeFactor, aboveAverageSecondMagnitudeFactor);
  }
  
  if (magnitudeChangeFactor < 0.5 && aboveAverageOverallMagnitudeFactor > 0.5) {
    // there's no bass related beat, but the overall magnitude changed significantly
    magnitudeChangeFactor = max(magnitudeChangeFactor, aboveAverageOverallMagnitudeFactor);
  } else {
    // this is here to avoid treating signal noise as beats
    //magnitudeChangeFactor *= 1 - aboveAverageOverallMagnitudeFactor;
  }
  
  //float maximumMagnitude = 128; //128;
  
  //logValue("CO", (overallFrequency.current - overallFrequency.average) / maximumMagnitude, 5);
  //logValue("C1", (firstFrequency.current - firstFrequency.average) / maximumMagnitude, 5);
  //logValue("C2", (secondFrequency.current - secondFrequency.average) / maximumMagnitude, 5);

  //logValue("CO", (overallFrequency.current) / maximumMagnitude, 10);
  //logValue("C1", (firstFrequency.current) / maximumMagnitude, 10);
  //logValue("C2", (secondFrequency.current) / maximumMagnitude, 10);

  logValue("AO", aboveAverageOverallMagnitudeFactor, 2);
  logValue("A1", aboveAverageFirstMagnitudeFactor, 10);
  logValue("A2", aboveAverageSecondMagnitudeFactor, 10);
  //logValue("A1|2", max(aboveAverageFirstMagnitudeFactor, aboveAverageSecondMagnitudeFactor), 1);

  
  logValue("M", magnitudeChangeFactor, 1);
  
  return magnitudeChangeFactor;
}

/**
 * Will calculate a value in range [0:1] based on variance in the first and second
 * frequency band over time. The variance will be high if the magnitude of bass
 * frequencies changed in the last few milliseconds.
 * Low values are indicating a low beat probability.
 */
float calculateVarianceFactor() {
  // a beat also requires a high variance in recent frequency magnitudes
  float firstVarianceFactor = ((float) (firstFrequency.variance - 50) / 20) - 1;
  firstVarianceFactor = constrain(firstVarianceFactor, 0, 1);
  
  float secondVarianceFactor = ((float) (secondFrequency.variance - 50) / 20) - 1;
  secondVarianceFactor = constrain(secondVarianceFactor, 0, 1);
  
  float varianceFactor = max(firstVarianceFactor, secondVarianceFactor);
  
  logValue("V", varianceFactor, 1);
  
  return varianceFactor;
}

/**
 * Will calculate a value in range [0:1] based on the recency of the last detected beat.
 * Low values are indicating a low beat probability.
 */
float calculateRecencyFactor() {
  float recencyFactor = 1;
  durationSinceLastBeat = millis() - lastBeatTimestamp;
  
  int referenceDuration = MINIMUM_DELAY_BETWEEN_BEATS - SINGLE_BEAT_DURATION;
  recencyFactor = 1 - ((float) referenceDuration / durationSinceLastBeat);
  recencyFactor = constrain(recencyFactor, 0, 1);
  
  //logValue("R", recencyFactor, 5);
  
  return recencyFactor;
}


/**
 * Will update the beat probability, a value between 0 and 1
 * indicating how likely it is that there's a beat right now.
 */
void updateBeatProbability() {
  beatProbability = 1;
  beatProbability *= calculateSignalChangeFactor();
  //beatProbability *= calculateMagnitudeChangeFactor();
  beatProbability *= max(calculateMagnitudeChangeFactor(), calculateVarianceFactor());
  beatProbability *= calculateRecencyFactor();
  
  if (beatProbability >= beatProbabilityThreshold) {
    lastBeatTimestamp = millis();
    durationSinceLastBeat = 0;
  }
  
  logValue("B", beatProbability, 5);
}

/**
 * Will extract insightful features from the frequency data in order
 * to perform the beat detection.
 */
void processFrequencyData() {
  // each of the methods below will:
  //  - get the current frequency magnitude
  //  - add the current magnitude to the history
  //  - update relevant features
  overallFrequency.update(getFrequencyMagnitude(fht_log_out, OVERALL_FREQUENCY_RANGE_START, OVERALL_FREQUENCY_RANGE_END));
  firstFrequency.update(getFrequencyMagnitude(fht_log_out, FIRST_FREQUENCY_RANGE_START, FIRST_FREQUENCY_RANGE_END));
  secondFrequency.update(getFrequencyMagnitude(fht_log_out, SECOND_FREQUENCY_RANGE_START, SECOND_FREQUENCY_RANGE_END));
}

/**
 * Will update the light intensity bump based on the recency of detected beats.
 */
void updateLightIntensityBasedOnBeats() {
  float intensity = 1 - ((float) durationSinceLastBeat / LIGHT_FADE_OUT_DURATION);
  intensity = constrain(intensity, 0, 1);
  
  if (intensity > lightIntensityValue) {
    lightIntensityBumpValue = intensity;
    lightIntensityBumpTimestamp = millis();
  }
}

/**
 * Will update the light intensity bump based on measured amplitudes.
 */
void updateLightIntensityBasedOnAmplitudes() {
  float intensity;
  if (signal.average < 1 || signal.current < 1) {
    intensity = 0;
  } else {
    intensity = (float) (signal.current - signal.average) / (10*sqrt32(signal.variance));
    intensity *= pow(intensity, 3);
    
    if (intensity < 0.1) {
      intensity = 0;
    } else {
      intensity -= 0.1;
      intensity = pow(1 + intensity, 3) - 1;
      intensity = constrain(intensity, 0, 1);
    }
  }
  
  logValue("I", intensity, 10);
  
  if (intensity > lightIntensityValue) {
    lightIntensityBumpValue = intensity;
    lightIntensityBumpTimestamp = millis();
  }
}

/**
 * Will update the hat lights based on the last light intensity bumps.
 */
void updateLights() {
  long durationSinceLastBump = millis() - lightIntensityBumpTimestamp;
  float fadeFactor = 1 - ((float) durationSinceLastBump / LIGHT_FADE_OUT_DURATION);
  fadeFactor = constrain(fadeFactor, 0, 1);
  
  lightIntensityValue = lightIntensityBumpValue * fadeFactor;
  lightIntensityValue = constrain(lightIntensityValue, 0, 1);
  
  logValue("L", lightIntensityValue, 20);
  
  // scale the intensity to be in range of maximum and minimum
  float scaledLightIntensity = MINIMUM_LIGHT_INTENSITY + (lightIntensityValue * (MAXIMUM_LIGHT_INTENSITY - MINIMUM_LIGHT_INTENSITY));
  
  int pinValue = 255 * scaledLightIntensity;
  analogWrite(HAT_LIGHTS_PIN, pinValue);
  
  // also use the builtin LED, for debugging when no lights are connected
  if (scaledLightIntensity > MAXIMUM_LIGHT_INTENSITY - ((MAXIMUM_LIGHT_INTENSITY - MINIMUM_LIGHT_INTENSITY) / 4)) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void updatePulse() {  
  // update the pulse signal
  static long lastPulseTimestamp = 0;

  long durationSincePulse = millis() - lastPulseTimestamp;
  float fadeFactor = ((float) durationSincePulse / (LIGHT_PULSE_DURATION * 2));
  if (durationSincePulse >= LIGHT_PULSE_DURATION) {
    fadeFactor = 1 - fadeFactor;
  }
  fadeFactor *= 2;
  fadeFactor = constrain(fadeFactor, 0, 1);
  
  // scale the intensity to be in range of maximum and minimum
  float scaledLightIntensity = MINIMUM_LIGHT_INTENSITY + (fadeFactor * (MAXIMUM_LIGHT_INTENSITY - MINIMUM_LIGHT_INTENSITY));
  
  logValue("P", scaledLightIntensity, 10);
  int pinValue = 255 * scaledLightIntensity;
  analogWrite(HAT_LIGHTS_PULSE_PIN, pinValue);
  
  if (durationSincePulse >= LIGHT_PULSE_DELAY) {
    lastPulseTimestamp = millis();
  }
}


void setup() {
  setupADC();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(HAT_LIGHTS_PIN, OUTPUT);
  pinMode(HAT_LIGHTS_LOW_PIN, OUTPUT);
  pinMode(HAT_LIGHTS_HIGH_PIN, OUTPUT);
  pinMode(HAT_LIGHTS_PULSE_PIN, OUTPUT);
  pinMode(SOUND_REFERENCE_PIN, OUTPUT);
  
  digitalWrite(HAT_LIGHTS_PIN, HIGH);
  digitalWrite(SOUND_REFERENCE_PIN, HIGH);
  
  analogWrite(HAT_LIGHTS_LOW_PIN, 255 * MINIMUM_LIGHT_INTENSITY);
  analogWrite(HAT_LIGHTS_HIGH_PIN, 255 * MAXIMUM_LIGHT_INTENSITY);

  Serial.begin(115200);
  Serial.println("Starting Festival Hat Controller");
}


void loop() {
  if (LOG_FREQUENCY_DATA) {
    readAudioSamples();
    getFrequencyData();
    logFrequencyData();
  } else {
    readAudioSamples();
    if (PERFORM_BEAT_DETECTION) {
      getFrequencyData();
      processFrequencyData();
      updateBeatProbability();
      updateLightIntensityBasedOnBeats();
    } else {
      updateLightIntensityBasedOnAmplitudes();
    }
    updateLights();
    Serial.println("");
  }
}