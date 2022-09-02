#include <Arduino.h>
#include "STM32LowPower.h"
#include <STM32RTC.h>
#include "arduinoFFT.h"
#include <string.h>
#include <SoftwareSerial.h>

const int RX = PA3;
const int TX = PA2;
SoftwareSerial gprs(RX, TX);

STM32RTC &rtc = STM32RTC::getInstance();

#if !defined(STM32F1xx)
#error "Not applicable (only stm32F1xx save date in backup memory)"
#endif /* !STM32F1xx */

#define INITIAL_SEC 10
#define INITIAL_MIN 59
#define INITIAL_HOUR 11
#define INITIAL_WDAY 5
#define INITIAL_DAY 2
#define INITIAL_MONTH 9
#define INITIAL_YEAR 22

#define SAMPLES 256             // Must be a power of 2 (for FFT algorithm)
#define SAMPLING_FREQUENCY 7000 // Hz, must be less than 10000 due to ADC and double the frequency you are trying to sample
#define num_therm_samples 5     // number of samples to average reading from thermistor
#define num_vwp_samples 3       // number of samples to average reading from vwp
int vwp_divisor = 3;            // if # of vwp samples changes, change this to match!

arduinoFFT FFT = arduinoFFT();

uint32_t subSec;
byte seconds;
byte minutes;
byte hours;
byte am_pm;

/* these values are read in the backUp register */
byte weekDay;
byte day;
byte month;
byte year;

String device_number = "117";

bool initialstatus = false;

// int numSensors = 1;              // Number of sensors attached to sensor
// char sendto[12] = "18005555555"; // Phone number used for SMS message

unsigned int sampling_period_us;

double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long time1[SAMPLES];
unsigned long time2[SAMPLES];

long sum;
float average;
float exp_sampl_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
float nat_freq = 0.00; // Initialize variable
double Bunits;
int vwp_count = 0;
char buf[50];

double peak;
String Sensor;
int freq[num_vwp_samples];
float freq_avg;
int ThermistorPin;
int temp[num_therm_samples];
float temp_avg;
int Vo;
float logR2, R2, T, Tc;
float R1 = 10000;                                                        // Fixed resitance of the series resistor plus wire resistance (calibrated)
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07; // Based on VWP Manual calibration statistics.
String dataString;

void ShowSerialData()
{
    while (gprs.available() != 0)
        Serial1.write(gprs.read());
    delay(5000);
}

void initializeGsm()
{
    if (gprs.available())
        Serial1.write(gprs.read());

    gprs.println("AT");
    delay(1000);
    gprs.println("AT+CPIN?");
    delay(1000);
    gprs.println("AT+CREG?");
    delay(1000);
    gprs.println("AT+CGATT?");
    delay(1000);
    gprs.println("AT+CIPSHUT");
    delay(1000);
    gprs.println("AT+CIPSTATUS");
    delay(2000);
    gprs.println("AT+CIPMUX=0");
    delay(2000);

    ShowSerialData();
    gprs.println("AT+CSTT=\"airtelgprs.com\""); // start task and setting the APN,
    delay(1000);
    ShowSerialData();
    gprs.println("AT+CIICR"); // bring up wireless connection
    delay(3000);
    ShowSerialData();
    gprs.println("AT+CIFSR"); // get local IP adress
    delay(2000);
    ShowSerialData();
    gprs.println("AT+CIPSPRT=0");
    delay(3000);
    ShowSerialData();
}

void sendData(String level, String temp, String date)
{
    gprs.println("AT+CIPSTART=\"TCP\",\"jvearth.com\",\"80\""); // start up the connection
    delay(6000);
    ShowSerialData();

    gprs.println("AT+CIPSEND="); // begin send data to remote server
    delay(4000);

    String str = "GET http://jvearth.com/ostron_gwm/Api/send?user_id=1&water_level=" + level + "&unit=1&temp=" + temp + "&device_number=" + device_number + "&date_time=" + date;
    Serial1.println(str);
    gprs.println(str);

    // http://swmss.in/ostron_gwm/Api/send?user_id=1&water_level=9887.0&unit=1&temp=34.7&device_number=rtc&date_time=22-8-22_16:54:34

    delay(4000);
    ShowSerialData();

    gprs.println((char)26); // sending
    delay(5000);            // waitting for reply, important! the time is base on the condition of internet
    gprs.println();
    ShowSerialData();

    gprs.println("AT+CIPSHUT"); // close the connection
    delay(1000);
    ShowSerialData();
}

void fft()
{
    for (int i = 0; i < num_vwp_samples; i++)
    {
        while ((nat_freq == 0.00) && (vwp_count < 3))
        {
            for (unsigned int freq = 1400; freq <= 3500; freq = freq + 50)
            {                    // sweeping frequency incrementing every 4ms and ending after 150ms to wait for natural frequency pulse
                tone(PA6, freq); // ADJUST!!!
                delay(4);
                noTone(PA6); // ADJUST!!!
            }
            pinMode(PA6, INPUT_ANALOG); // ADJUST!!!
            delay(20);                  // delay 20ms to wait for frequencies other than natural frequency to die out.

            for (int i = 0; i < SAMPLES; i++)
            {
                time1[i] = micros(); // Overflows after around 70 minutes!
                vReal[i] = analogRead(PA6);
                // Serial1.println(vReal[i]);
                while (micros() < (time1[i] + sampling_period_us))
                {
                }
            }

            for (int i = 0; i < SAMPLES; i++)
            {
                vImag[i] = 0;
            }
            for (int i = 1; i < SAMPLES; i++)
            {
                time2[i] = time1[i] - time1[i - 1];
            }
            sum = 0L;
            for (int i = 0; i < SAMPLES; i++)
            {
                sum += time2[i];
            }
            average = sum / (SAMPLES - 1.0);
            // FFT
            FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
            FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
            FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
            peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

            nat_freq = (exp_sampl_period_us / average) * peak;
            // delay(1000);
            // Serial1.println(nat_freq);

            if ((nat_freq < 1400.00) || (nat_freq > 3500))
            {
                nat_freq = 0.00;
                vwp_count = vwp_count + 1;
                delay(2000); // Wait this long to let wire settle before plucking again
            }
        }

        freq[i] = nat_freq;
        delay(4000);
    }

    freq_avg = 0.0;

    for (int i = 0; i < num_vwp_samples; i++)
    {
        freq_avg += freq[i];
        if (freq[i] == 0.0)
        { // Critical to include this step! It removes any zero values from averaging!
            vwp_divisor = vwp_divisor - 1;
        }
    }

    if (vwp_divisor == 0)
    {                   // If all the readings register zero readings, set outcome to zero!
        freq_avg = 0.0; // This would take 2s x 3 tries x 3 readings = 18s per VWP!!! (6s minimum per VWP)
        Bunits = 0;
    }
    else
    {
        freq_avg /= vwp_divisor;
        Bunits = (freq_avg * freq_avg) * (0.001);
    }

    Sensor = "1";         // ADJUST!!!
    pinMode(PA6, OUTPUT); // ADJUST!!!
    digitalWrite(PA6, LOW);
    delay(200); // ADJUST!!!
    // analogReference(EXTERNAL);
    pinMode(PA1, INPUT_ANALOG);
    delay(200);
    digitalWrite(PA4, HIGH); // ADJUST!!!
    delay(100);             // delay for powering up

    for (int i = 0; i < num_therm_samples; i++)
    {
        Vo = analogRead(PA1);
        Serial1.println(Vo);
        R2 = R1 * (1023.0 / (float)Vo - 1.0);
        logR2 = log(R2);
        T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
        Tc = T - 273.15;
        temp[i] = Tc;
        delay(100);
    }

    temp_avg = 0.0;

    for (int i = 0; i < num_therm_samples; i++)
    {
        temp_avg += temp[i];
    }

    temp_avg /= num_therm_samples;

    pinMode(PA5, OUTPUT); // ADJUST!!!
    digitalWrite(PA5, LOW);
    delay(200);             // ADJUST!!!
    digitalWrite(PA4, LOW); // ADJUST!!!

    delay(1000);
    sprintf(buf, "%02d-%02d-%02d_%02d:%02d:%02d", rtc.getDay(), rtc.getMonth(), rtc.getYear(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

    dataString = "";
    dataString += String('a') + String(',') + String(buf) + String(',') + String(Sensor) + String(',') + String(Bunits) + String(',') + String(temp_avg);
    Serial1.println(dataString);

    nat_freq = 0.00;
    vwp_count = 0;
    vwp_divisor = 3;
}

void setup()
{
    gprs.begin(9600);
    Serial1.begin(9600);
    Serial1.println("Setup start");
    rtc.setClockSource(STM32RTC::LSE_CLOCK);
    bool reset_bkup = false;
    rtc.begin(reset_bkup);
    rtc.getDate(&weekDay, &day, &month, &year);

    if (reset_bkup)
    {
        Serial1.printf("reset the date to %02d/%02d/%02d\n", day, month, year);
    }
    else
    {
        Serial1.printf("date from the BackUp %02d/%02d/%02d\n", day, month, year);
    }

    if ((day == 1) && (month == RTC_MONTH_JANUARY) && (year == 1))
    {
        // Set the time
        rtc.setHours(INITIAL_HOUR);
        rtc.setMinutes(INITIAL_MIN);
        rtc.setSeconds(INITIAL_SEC);
        // Set the date
        rtc.setWeekDay(INITIAL_WDAY);
        rtc.setDay(INITIAL_DAY);
        rtc.setMonth(INITIAL_MONTH);
        rtc.setYear(INITIAL_YEAR);
    }

    pinMode(PB12, INPUT_PULLDOWN);

    pinMode(PB13, OUTPUT);

    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LOW);

    pinMode(PA4, OUTPUT);
    digitalWrite(PA4, LOW);

    pinMode(PA6, OUTPUT);
    digitalWrite(PA6, LOW);

    pinMode(PA1, OUTPUT);
    digitalWrite(PA1, LOW);

    sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

    digitalWrite(PC13, HIGH);

    Serial1.println("Setup Complete");

    LowPower.begin();
}

void loop()
{
    if (digitalRead(PB12) == HIGH)
    {
        if (!initialstatus)
        {
            digitalWrite(PB13, HIGH);
            delay(15000);
            while (!gprs)
            {
                ;
            }
            initializeGsm();
            initialstatus = true;
        }
        Serial1.println("Taking rapid initial readings");
        fft();
        if (gprs)
        {
            sendData(String(Bunits), String(temp_avg), String(buf));
        }
        delay(50000);
    }
    else
    {
        fft();
        if (Bunits != 0)
        {
            if (!initialstatus)
            {
                digitalWrite(PB13, HIGH);
                delay(15000);
                while (!gprs)
                {
                    ;
                }
                initializeGsm();
                initialstatus = true;
            }

            delay(1000);
            if (gprs)
            {
                sendData(String(Bunits), String(temp_avg), String(buf));
                digitalWrite(PB13, LOW);
                initialstatus = false;
                Serial1.println("Going into Sleep for 12 hours");
                delay(1000);
                LowPower.deepSleep(43200000);
            }
        }
        else
        {
            Serial1.println("Taking Another REading");
        }
        delay(50000);
    }
}