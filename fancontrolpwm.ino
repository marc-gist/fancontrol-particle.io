// This #include statement was automatically added by the Particle IDE.
#include <map>
#include "ds18x20.h"
#include "onewire.h"

// This #include statement was automatically added by the Particle IDE.
#include "FreqPeriodCounter.h"

long previousMillis = 0;
long interval = 5000;

// Used to debounce Button
boolean button_was_pressed;
unsigned long pulseDuration;
unsigned long millisNow = 0;
#define rpmCalcDelay            2500

/*#define pwmFan D2
#define pwmFan2 D3
#define pwmFan3 RX
#define pwmFan4 TX

#define rpmPin A0
#define rpmPin2 A1
#define rpmPin3 A2
#define rpmPin4 A3*/

#define tempPin D4 // one wire pin; could use multiple in order to easily determine temp inputs

const int num = 4; // NUMBER OF FANS? CONNECTED
const int pwmFans[] = {D2, D3, RX, TX};
const int rpmPins[] = {A0, A1, A2, A3};

const int rpmLow = 80; // ~30%
const int rpmMed = 128; // ~50%
const int rpmHigh = 192; // ~75%
const int rpmMax = 252; // ~100% (put slightly lower in order to pwm?)

#define PWM_FREQ 25000

//const int counter_interrupt_pin = rpmPin; // aka rpmPin

const int k_seconds_per_minute = 60;
const int k_pulses_per_revolution = 2;
const int k_hertz_to_RPM_conversion_factor = k_seconds_per_minute / k_pulses_per_revolution;

Timer getTempTimer(5000, getTemp);

void setup() {
    for(int i=0; i<num; i++) {
      pinMode(pwmFans[i], OUTPUT);
      pinMode(rpmPins[i], INPUT_PULLUP);
      analogWrite(pwmFans[i], rpmMed, PWM_FREQ);
    }

    ow_setPin(tempPin); //OneWire

    pinMode(A4, INPUT); //testing pin, used for variable resistor

    Serial.begin(9600);
    Particle.publish("fan-mon", "init");
    delay(2000);
    Serial.println("OK");

    //getTempTimer.start();
}

uint8_t sensors[80];
void log(char* msg)
{
    //Particle.publish("log", msg);
    Serial.println(msg);
    delay(500);
}


double tempArry[4];
static constexpr int sensorTemp[] = {62, 97, 191, 194};

void getTemp() {
    double retval = 0;
    uint8_t subzero, cel, cel_frac_bits;
    char msg[100];
    log("Starting measurement");

    DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL ); //Asks all DS18x20 devices to start temperature measurement, takes up to 750ms at max resolution
    delay(1000); //If your code has other tasks, you can store the timestamp instead and return when a second has passed.

    uint8_t numsensors = ow_search_sensors(10, sensors);
    if (numsensors > 4) numsensors = 4; //limit to 4 for this program
    sprintf(msg, "Found %i sensors", numsensors);
    log(msg);

    for (uint8_t i=0; i<numsensors; i++)
    {
        if (sensors[i*OW_ROMCODE_SIZE+0] == 0x10 || sensors[i*OW_ROMCODE_SIZE+0] == 0x28) //0x10=DS18S20, 0x28=DS18B20
        {
            //log("Found a DS18B20");
			if ( DS18X20_read_meas( &sensors[i*OW_ROMCODE_SIZE], &subzero, &cel, &cel_frac_bits) == DS18X20_OK ) {
				char sign = (subzero) ? '-' : '+';
				int frac = cel_frac_bits*DS18X20_FRACCONV;
        int item = -1;
				/*sprintf(msg, "Sensor# %d (%02X%02X%02X%02X%02X%02X%02X%02X) =  : %c%d.%04d\r\n",i+1,
				sensors[(i*OW_ROMCODE_SIZE)+0],
				sensors[(i*OW_ROMCODE_SIZE)+1],
				sensors[(i*OW_ROMCODE_SIZE)+2],
				sensors[(i*OW_ROMCODE_SIZE)+3],
				sensors[(i*OW_ROMCODE_SIZE)+4],
				sensors[(i*OW_ROMCODE_SIZE)+5],
				sensors[(i*OW_ROMCODE_SIZE)+6],
				sensors[(i*OW_ROMCODE_SIZE)+7],
				sign,
				cel,
				frac
				);*/
        sprintf(msg, "Sensor# %d (%02X %d) =  : %c%d.%04d\r\n",i+1,
        sensors[(i*OW_ROMCODE_SIZE)+0],
				sensors[(i*OW_ROMCODE_SIZE)+7],
				sign,
				cel,
				frac
				);
				log(msg);

        // convert to floating point value
        // another option is to round up, as we don't really care?
        retval = ((cel*10000)+frac) / 10000.0;
        if(subzero) retval *= -1;

        switch(sensors[(i*OW_ROMCODE_SIZE)+7]) {
          case sensorTemp[0]:
            item = 0;
            break;
          case sensorTemp[1]:
            item = 1;
            break;
          case sensorTemp[2]:
              item = 2;
              break;
          case sensorTemp[3]:
                item = 3;
                break;
        }
        Serial.printf("float %d: ", item);
        tempArry[item] = retval;
        Serial.println(retval);
			}
			else
			{
			    Spark.publish("log", "CRC Error (lost connection?)");
			}
        }
    }
}

// setup global, pass first rpmPin here just to setup, will override in function
FreqPeriodCounter counter(rpmPins[0], micros, 0);

int counter_read_rpm(int pin, int retry=3) {
    int period;
    long hertz;
    int RPM = -1;
    Serial.print("Reading Pin: ");
    Serial.println(pin);
    // override counter object with new one.
    counter.setPin(pin);

    /*counter = FreqPeriodCounter(pin, micros, 0);*/

    if(!attachInterrupt(pin, counter_interrupt_service_routine, CHANGE)) return -2;
    for(int i=0; i < retry; i++) {
        delay(1000);
        if(counter.ready()) {
            period = counter.period;
            hertz = counter.hertz();
            RPM = hertz * k_hertz_to_RPM_conversion_factor;


            Serial.print("Period: ");
            Serial.print(period);
            Serial.print("\t");

            Serial.print("Hertz: ");
            Serial.print(hertz);
            Serial.print("\t");

            Serial.print("RPM: ");
            Serial.print(RPM);
            Serial.print("\t");

            Serial.println();
            return RPM;
        }
        else {
            Serial.print("...");
        }
    }
    detachInterrupt(pin);

    return RPM;
}

void counter_interrupt_service_routine()
{
    counter.poll();
}

void loop() {
    int i = 0;

    //Serial.print("RPM:");

    getTemp(); // also can enable timers.
        
    // this is working, so start skipping so we can do other work
    /*for(i = 0; i < num; i++) {
      counter_read_rpm(rpmPins[i]);
      // TODO: check return values, and alert if issue?
      delay(250);
    }*/


    Serial.println("DELAY");
    delay(interval);


    int v = analogRead(A4);
    v = v/16;
    if( v < 35 ) v = 35;
    if( v > 250) v = 250;

    Serial.print("A: ");
    Serial.println(v/252.0*100.0);

    for(i=0; i<num; i++) {
      analogWrite(pwmFans[i], v, PWM_FREQ);
    }

}
