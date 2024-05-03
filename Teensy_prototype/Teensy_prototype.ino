// Include the ADC library
#include <ADC.h>
// Include necessary libraries for signal processing
#include <arduinoFFT.h>
#include <cmath>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
// #include <Adafruit_DRV2605.h>

// Create an instance for the haptics driver
// Adafruit_DRV2605 drv;

// Create a timer instance. This is used for reading the ADC
IntervalTimer timer;

File myFile;
const int chipSelect = BUILTIN_SDCARD;

// Create an ADC object
ADC* adc = new ADC();


// Define the analog input pin
const int analogInPin = A2;
const int PIN_2 = 11;
const int PIN_3 = 12;

int pulse_number = 8;  // How many bursts are needed

volatile int idx = 0;  // Index for the array

const int array_size = 1500;  // Array for incoming sensor readings.

// Timer variables
volatile int i = 0;
volatile int start = 0;     // Used for delaying code
volatile int duration = 0;  // Used for delaying code
volatile int start_echo = 0;
volatile int duration_echo = 0;
int end_echo = 13000;  // Receive for 10ms.



// SAMPLE PARAMETERS
#define numSamples array_size                              // Must be a power of 2
#define samplingFreq 124800                          // Hz
#define signal_length 63                             // length of input signal for cross correlation (CC)
#define convLength (numSamples + signal_length - 1)  // length of array for convolution in CC

unsigned int sampling_period_us;
double timeArray[numSamples];
double vReal[numSamples];
double vImag[numSamples];
double hanning_window[numSamples / 2];
double input_signal[signal_length];
double convArray[convLength];
ArduinoFFT<double> FFT(vReal, vImag, numSamples, samplingFreq);

// Model Parameters
const double speed_of_sound = 340;                           // in meters per second
const double max_distance = 3;                               // in meters
double cutoff_time = 2*(max_distance / speed_of_sound) * 1e6;  // in microseconds
const double trans_time = 1000;                              // in microseconds (for removing the transmit signal)

// Hanning Window Parameters
const double max_freq = (numSamples / 2.0 - 1) / (numSamples)*samplingFreq;  // max on freq axis after FFT (Hz)
const double tran_freq = 40000;                                              // Hz (needs to be set)
const double bandwidth = 5000;                                               // Hz (needs to be set)

// Sliding Window Parameters
const int N = 40;  // must not exceed the length of time_SW (relevant length of timeArray see below)
const double m = 0.1;
const double tau = N * 0.8;  // second threshold (number of sample points that exceeds the first threshold), this has to be less than N points
const int shift = 1;

// Cross Correlation Paramters
const double conv_threshold = 0.3;

int sample_no = 1;

double distance;
int vibration;
void setup() {

  // Start the serial communication
  Serial.begin(115200);
  Wire1.begin();  // join i2c bus (address optional for master)
  pinMode(PIN_2, OUTPUT);
  pinMode(PIN_3, OUTPUT);

  // Set up the ADC
  adc->adc0->setAveraging(0);                                       // set number of averages
  adc->adc0->setResolution(12);                                     // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);  // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);      // change the sampling speed

  // drv.begin();
  // drv.setMode(DRV2605_MODE_INTTRIG);  // Set mode to triggered


 // Wire2.begin();  // join i2c bus (address optional for master)

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    while (1)
      ;
  }
  Serial.println("initialization done.");
  // // Print a message to the serial monitor
  Serial.println("ADC setup complete.");
}

volatile int row_counter = 0;

void loop() {

  //-------------PULSE-----------//

  generateWave2();  // Sends out 8 pulses.
  //chirp();
  //-----------------------------//
  //-------------RECEIVE---------//
  duration_echo = 0;
  start_echo = micros();

  // THIS UPDTAES V REAL
  timer.begin(sensor_read, 8);
  while (duration_echo < end_echo) {
    duration_echo = micros() - start_echo;
  }
  timer.end();
  //Serial.println(duration_echo);
  //Serial.println(idx);
  //-----------------------------//
  //-------------SD CARD---------//
  bool writeFlag = true;  // Set this flag to true when you want to write to the SD card
  if (writeFlag) {
    // Call the function to write data to the SD card
    writeDataToSDCard();
    // Reset the flag
    writeFlag = false;
  }
  //-----------------------------//
  //-----------Process---------//
  double distance = processSignalCC();
  //Print distance on each iteration
  Serial.print(" Distance:  ");
  Serial.print(distance, 4);
  //-----------------------------//
  //------------Send-------------//
  //send_data(distance);
  //-----------------------------//

  // bool writeFlag = true;  // Set this flag to true when you want to write to the SD card
  // if (writeFlag) {
  //   // Call the function to write data to the SD card
  //   //writeDataToSDCard();
  //   writeDouble(distance);
  //   // Reset the flag
  //   writeFlag = false;
  // }

  idx = 0;
  i = 0;
  row_counter++;
  Serial.print(" Row number:  ");
  Serial.println(row_counter);
  //delay(200);
}

//volatile unsigned long lastReadTime = 0;  // Add this line at the top of your code

void sensor_read() {
  if (idx < array_size) {
    // unsigned long currentReadTime = micros();
    // //Serial.println(currentReadTime - lastReadTime);  // Print the time difference to the serial monitor
    // lastReadTime = currentReadTime;
    int sensorvalue = adc->adc0->analogRead(analogInPin);
    vReal[idx] = sensorvalue * (3.3 / 4096);
    //Serial.println(vReal[idx]);
    idx++;
  }
}

void generateWave() {
  int pulse_number = 8;        // Number of pulses
  int i = 0;                   // Pulse counter
  double duty_duration = 12;   // Duration of each state in microseconds
  unsigned long start = 0;     // Start time of each state
  unsigned long duration = 0;  // Duration of each state



  while (i < pulse_number) {  // Keep the turning on and off in the while loop because it is faster.

    while (duration <= duty_duration) {  // when 12 uS have passed change pin state.
      duration = micros() - start;
    }

    // Turn pin 2 on and pin 3 off simultaneously
    digitalWriteFast(PIN_2, HIGH);  // Turn pin 2 on
    digitalWriteFast(PIN_3, LOW);   // Turn pin 3 off

    start = micros();  // Catches the current time
    duration = 0;      // Resets the timer to zero

    while (duration <= duty_duration) {  // Keeps the code running for 12 uS
      duration = micros() - start;
    }

    // Turn pin 2 off and pin 3 on simultaneously
    digitalWriteFast(PIN_2, LOW);   // Turn pin 2 off
    digitalWriteFast(PIN_3, HIGH);  // Turn pin 3 on

    start = micros();  // Catches current time
    duration = 0;      // Resets the timer
    i++;
  }
}

void generateWave2() {

  const int pulse_number = 20;  // Number of pulses
  volatile int i = 0;          // Pulse counter

  // // Enable the cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;


  while (i < pulse_number) {  // Keep the turning on and off in the while loop because it is faster.

    delayCycles(2300);  // 13 uS delay
    // Turn pin 2 on and pin 3 off simultaneously
    digitalWriteFast(PIN_2, HIGH);  // Turn pin 2 on
    digitalWriteFast(PIN_3, LOW);   // Turn pin 3 off

    delayCycles(2300);

    // Turn pin 2 off and pin 3 on simultaneously
    digitalWriteFast(PIN_2, LOW);   // Turn pin 2 off
    digitalWriteFast(PIN_3, HIGH);  // Turn pin 3 on

    i++;
  }

  // Disable the cycle counter
  ARM_DEMCR &= ~ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL &= ~ARM_DWT_CTRL_CYCCNTENA;
}

void delayCycles(uint32_t cycles) {
  uint32_t start, elapsed;
  uint32_t count = 0;

  // Get the current value of the cycle counter
  start = ARM_DWT_CYCCNT;

  do {
    // Calculate the number of cycles elapsed since we started
    elapsed = ARM_DWT_CYCCNT - start;

    // Increment our loop counter
    count++;
  } while (elapsed < cycles);
}

void chirp() {
  //const int freq[8] = {5e3, 20e3, 30e3, 40e3, 40e3, 40e3, 40e3, 0e3};  // Frequencies to cycle through 1/(2*freq) * 180Mhz
  const int delay_c[8] = { 18000, 4500, 3000, 2300, 2300, 2300, 2300, 0 };
  volatile int z = 0;          // Index for cycling through frequencies
  const int pulse_number = 8;  // Number of pulses
  // // Enable the cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  while (z < pulse_number) {  // Keep the turning on and off in the while loop because it is faster.

    delayCycles(delay_c[z]);  // 13 uS delay
    // Turn pin 2 on and pin 3 off simultaneously
    digitalWriteFast(PIN_2, HIGH);  // Turn pin 2 on
    digitalWriteFast(PIN_3, LOW);   // Turn pin 3 off

    delayCycles(delay_c[z]);

    // Turn pin 2 off and pin 3 on simultaneously
    digitalWriteFast(PIN_2, LOW);   // Turn pin 2 off
    digitalWriteFast(PIN_3, HIGH);  // Turn pin 3 on

    z++;
  }
  // Disable the cycle counter
  ARM_DEMCR &= ~ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL &= ~ARM_DWT_CTRL_CYCCNTENA;
}

void send_data(double number) {
  byte* data = (byte*)&number;  // convert double to byte array

  Wire2.beginTransmission(8);  // transmit to device #8
  for (int i = 0; i < sizeof(double); i++) {
    Wire2.write(data[i]);  // sends one byte at a time
  }
  Wire2.endTransmission();  // stop transmitting
}

// double processSignal() {

//   // Generate time array
//   sampling_period_us = (1.0 / samplingFreq) * 1e6;
//   for (int i = 0; i < numSamples; i++) {
//     timeArray[i] = i * sampling_period_us;
//   }

//   // Perform FFT
//   FFT.compute(FFTDirection::Forward);

//   // Delete the second half of the arrays
//   for (int i = numSamples / 2; i < numSamples; i++) {
//     vReal[i] = 0;
//     vImag[i] = 0;
//   }

//   // Calculate Hanning window
//   for (int i = 0; i < numSamples / 2; i++) {
//     double x = (double)i / (numSamples / 2 - 1);
//     double y = 0.5 * (1 + cos((x - tran_freq / max_freq) / (bandwidth / max_freq) * PI));
//     hanning_window[i] = y * ((x >= (tran_freq / max_freq - bandwidth / max_freq)) && (x <= (tran_freq / max_freq + bandwidth / max_freq)));
//   }

//   // Apply Hanning window to real and imaginary parts separately
//   for (int i = 0; i < numSamples / 2; i++) {
//     vReal[i] *= hanning_window[i];
//     vImag[i] *= hanning_window[i];
//   }

//   // Perform Inverse FFT
//   FFT.compute(FFTDirection::Reverse);

//   // Compute the maximum magnitude of vReal and vImag
//   double max_magnitude = 0;
//   for (int i = 0; i < numSamples; i++) {
//     double magnitude = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
//     if (magnitude > max_magnitude) {
//       max_magnitude = magnitude;
//     }
//   }

//   // Normalize vReal and vImag
//   for (int i = 0; i < numSamples; i++) {
//     vReal[i] /= max_magnitude;
//     vImag[i] /= max_magnitude;
//   }


//   // Find index of cutoff
//   double min_cutoff_time = abs(timeArray[0] - cutoff_time);
//   int index_cutoff = 0;
//   for (int i = 1; i < numSamples; i++) {
//     if (abs(timeArray[i] - cutoff_time) < min_cutoff_time) {
//       min_cutoff_time = abs(timeArray[i] - cutoff_time);
//       index_cutoff = i;
//     }
//   }

//   // Fill elements from (including) index_cutoff to the end with 0
//   for (int i = index_cutoff; i < numSamples; i++) {
//     timeArray[i] = 0;
//     vReal[i] = 0;
//     vImag[i] = 0;
//   }

//   // Find dead zone
//   double trans_time = 1000;  // in microseconds
//   int index_trans = 0;
//   double min_trans_time = abs(timeArray[0] - trans_time);

//   for (int i = 1; i < index_cutoff; i++) {
//     double current_diff = abs(timeArray[i] - trans_time);
//     if (current_diff < min_trans_time) {
//       min_trans_time = current_diff;
//       index_trans = i;
//     }
//   }

//   // Set deadzone for vReal and vImag
//   for (int i = 0; i <= index_trans; i++) {
//     vReal[i] = 0;
//     vImag[i] = 0;
//   }

//   // Sliding Window

//   int start_index = index_trans;    // starting sliding window (left side)
//   int end_index = start_index + N;  // starting sliding window (right side)

//   while (true) {

//     int tau_temp = 0;

//     for (int i = start_index; i < end_index; i++) {
//       if (sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]) >= m) {
//         tau_temp++;
//       }
//     }

//     if (tau_temp > tau) {  // threshold check of completed window

//       // Calculate TOF
//       double TOF = timeArray[start_index];

//       // Calculate distance
//       double distance = speed_of_sound * (TOF / 1e6) / 2.0;  // Assuming one-way travel

//       // Return distance
//       return distance;
//     }

//     // Move the sliding window
//     start_index += shift;
//     end_index += shift;

//     if (end_index >= index_cutoff) {  // check if the next window's end_index will exceed the range
//       // Return 0 if the range is exceeded
//       return 0;
//     }
//   }
// }

double processSignalCC() {

  // Generate time array
  sampling_period_us = (1.0 / samplingFreq) * 1e6;
  for (int i = 0; i < numSamples; i++) {
    timeArray[i] = i * sampling_period_us;
  }

  // Compute the maximum magnitude of vReal
  double max_magnitude = 0;
  for (int i = 0; i < numSamples; i++) {
    double magnitude = vReal[i];
    if (magnitude > max_magnitude) {
      max_magnitude = magnitude;
    }
  }

  // Normalize vReal
  for (int i = 0; i < numSamples; i++) {
    vReal[i] /= max_magnitude;
  }

  // Calculate the sum of all elements
  double sum = 0.0;
  for (int i = 0; i < numSamples; i++) {
    sum += vReal[i];
  }

  double mean = sum / numSamples;

  // Subtract the mean from each element in the array
  for (int i = 0; i < numSamples; i++) {
    vReal[i] -= mean;
  }

  // Generate tone-burst signal
  for (int i = 0; i < signal_length; i++) {
    // Calculate the time for the current index directly within the sine function
    input_signal[i] = sin(2 * PI * tran_freq * i / samplingFreq);
  }

  // Calculate Hanning window
  for (int i = 0; i < signal_length; i++) {
    double x = (double)i / (signal_length - 1);
    double y = 0.5 * (1 + cos((x - 0.5) / (0.5) * PI));
    hanning_window[i] = y * ((x >= (0.0)) && (x <= (1.0)));
  }

  // Apply Hanning window to ideal signal
  for (int i = 0; i < signal_length; i++) {
    input_signal[i] *= hanning_window[i];
  }

  // Remove transmitting signal from vReal
  int index_trans = 0;
  double min_trans_time = abs(timeArray[0] - trans_time);

  for (int i = 1; i < numSamples; i++) {
    double current_diff = abs(timeArray[i] - trans_time);
    if (current_diff < min_trans_time) {
      min_trans_time = current_diff;
      index_trans = i;
    }
  }

  // Set deadzone for vReal
  for (int i = 0; i <= index_trans; i++) {
    vReal[i] = 0;
  }

  // Perform convolution
  for (int k = 0; k < convLength; k++) {

    double total = 0;
    int min_j = max(0, k - (signal_length - 1));
    int max_j = min(k, numSamples - 1);

    for (int j = min_j; j <= max_j; j++) {

      double temp = vReal[j] * input_signal[k - j];
      total += temp;
    }

    convArray[k] = total;
  }


  /* OLD CODE */
  //   for (int i = numSamples; i < convLength; i++) {
  //     convArray[i] = 0;
  //   }

  //   // Find index of TOF (it's at the maximum of abs values in convArray)
  //   double conv_current = 0;
  //   double conv_peak = 0;
  //   int index_tof = 0;
  //   for (int i = 0; i < convLength; i++) {
  //     conv_peak = abs(convArray[i]);
  //     if (conv_peak > conv_current) {
  //       conv_current = conv_peak;  // Correct assignment
  //       index_tof = i;
  //     }
  //   }

  //   // Check if convolution result exceeds set threshold
  //   if (conv_current > conv_threshold) {
  //     double TOF = timeArray[index_tof];
  //     double distance = speed_of_sound * (TOF / 1e6) / 2.0;
  //     return distance;
  //   } else {
  //     return 0;
  //   }
  // }



  // Find index of TOF (it's at the maximum of abs values in convArray)
  double conv_current = 0;
  double conv_peak = 0;
  int index_tof = 0;
  for (int i = 0; i < convLength; i++) {
    conv_peak = abs(convArray[i]);
    if (conv_peak > conv_current) {
      conv_current = conv_peak;  // Correct assignment
      index_tof = i;
    }
  }


  // Check if convolution result exceeds set threshold
  if (conv_current > conv_threshold) {
    int index_tof_int = floor((double)(index_tof + 1) / convLength * numSamples);
    double TOF = timeArray[index_tof_int - 1];
    double distance = speed_of_sound * (TOF / 1e6) / 2.0;
    return distance;
  } else {
    return 0;
  }
}

// For voltage prints
void writeDataToSDCard() {
  myFile = SD.open("Cycle20_1500.csv", FILE_WRITE);

  if (myFile) {
    //Serial.print("Writing to test.csv...");
    for (int zz = 0; zz < array_size; zz++) {
      myFile.printf(" %.4f", vReal[zz]);
      if (zz < array_size) {
        myFile.print(",");
      }
    }
    myFile.println();
    myFile.close();
    Serial.print(" Writing status: done.");
  } else {
    Serial.println("error opening test.csv");
  }
}

// For distance prints
// void writeDouble(double value) {
//   myFile = SD.open("FOV0.csv", FILE_WRITE);

//   if (myFile) {
//     //Serial.print("Writing to test.csv...");
//     myFile.printf(" %.4f", value);
//     myFile.println();
//     myFile.close();
//     Serial.print(" Writing status: done.");
//   } else {
//     Serial.println("error opening test.csv");
//   }
// }

// void haptics() {
//   //set condisitons for vibration
//   if (distance < 40) {
//     vibration = 0;
//     drv.setWaveform(0, vibration);
//     drv.setWaveform(1, 0);
//     drv.go();
//     //delay(500);
//   } else if (distance <= 70) {
//     vibration = 17;
//     drv.setWaveform(0, vibration);
//     drv.setWaveform(1, 0);
//     drv.go();
//     //delay(500);
//   } else if (distance <= 100) {
//     vibration = 18;
//     drv.setWaveform(0, vibration);
//     drv.setWaveform(1, 0);
//     drv.go();
//     //delay(500);
//   } else if (distance <= 130) {
//     vibration = 19;
//     drv.setWaveform(0, vibration);
//     drv.setWaveform(1, 0);
//     drv.go();
//     //delay(500);
//   } else if (distance <= 160) {
//     vibration = 21;
//     drv.setWaveform(0, vibration);
//     drv.setWaveform(1, 0);
//     drv.go();
//     //delay(500);
//   } else if (distance <= 190) {
//     vibration = 22;
//     drv.setWaveform(0, vibration);
//     drv.setWaveform(1, 0);
//     drv.go();
//     //delay(500);
//   } else if (distance <= 220) {
//     vibration = 23;
//     drv.setWaveform(0, vibration);
//     drv.setWaveform(1, 0);
//     drv.go();
//     //delay(500);
//   }
// }

// void send_data(double* array, int size) {
//   int test_start = micros();
//   Wire1.beginTransmission(8);  // transmit to device #8
//   for (int i = 0; i < size; i++) {
//     double number = array[i];
//     byte* data = (byte*)&number;  // convert double to byte array
//     for (int j = 0; j < sizeof(number); j++) {
//       Wire1.write(data[j]);  // sends one byte at a time
//     }
//   }
//   Serial.println(micros() - test_start);
//   Wire1.endTransmission();  // stop transmitting
// }

// void send_data_one(double number) {
//   byte* data = (byte*)&number;  // convert double to byte array

//   Wire1.beginTransmission(8);  // transmit to device #8
//   for (int i = 0; i < sizeof(double); i++) {
//     Wire1.write(data[i]);  // sends one byte at a time
//   }
//   Wire1.endTransmission();  // stop transmitting
// }
