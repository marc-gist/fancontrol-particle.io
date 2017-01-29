// This #include statement was automatically added by the Particle IDE.
#include "ds18x20.h"
#include "onewire.h"

// This #include statement was automatically added by the Particle IDE.
#include "FreqPeriodCounter.h"

#define tempPin D2 // one wire pin; could use multiple in order to easily determine temp inputs

long previousMillis = 0;
long interval = 2000;

// Used to debounce Button
boolean button_was_pressed;
unsigned long pulseDuration;
unsigned long millisNow = 0;
#define rpmCalcDelay            2500
int fanRPM = 0;


#define pulsePin D5
#define pulsePin2 D6
const int counter_interrupt_pin = pulsePin; // aka pulsePin

const int k_seconds_per_minute = 60;
const int k_pulses_per_revolution = 2;
const int k_hertz_to_RPM_conversion_factor = k_seconds_per_minute / k_pulses_per_revolution;

void setup() {
    pinMode(D3, OUTPUT);
    ow_setPin(tempPin);
    pinMode(pulsePin, INPUT_PULLUP);
    pinMode(pulsePin2, INPUT_PULLUP);
    pinMode(A3, INPUT);

    Serial.begin(9600);
    delay(2000);
    Serial.println("OK");
}

uint8_t sensors[80];
void log(char* msg)
{
    //Particle.publish("log", msg);
    Serial.println(msg);
    delay(500);
}

double getTemp() {
    uint8_t subzero, cel, cel_frac_bits;
    char msg[100];
    log("Starting measurement");

    DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL ); //Asks all DS18x20 devices to start temperature measurement, takes up to 750ms at max resolution
    delay(1000); //If your code has other tasks, you can store the timestamp instead and return when a second has passed.

    uint8_t numsensors = ow_search_sensors(10, sensors);
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
				sprintf(msg, "Sensor# %d (%02X%02X%02X%02X%02X%02X%02X%02X) =  : %c%d.%04d\r\n",i+1,
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
				);
				log(msg);
			}
			else
			{
			    Spark.publish("log", "CRC Error (lost connection?)");
			}
        }
    }
}

FreqPeriodCounter counter(counter_interrupt_pin, micros, 0);

int counter_read_rpm(int pin, int retry=5) {
    int period;
    long hertz;
    int RPM = -1;

    counter = FreqPeriodCounter(pin, micros, 0);

    if(!attachInterrupt(pin, counter_interrupt_service_routine, CHANGE)) return -2;
    for(int i=0; i < retry; i++) {

        delay(500);
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
    delay(interval);


    int v = analogRead(A3);
    v = v/16;
    if( v < 85 ) v = 85;
    if( v > 250) v = 250;
    analogWrite(D3, v, 25000);

    Serial.print("A: ");
    Serial.println(v/250.0*100.0);

    Serial.print("RPM:");
    counter_read_rpm(pulsePin);
    delay(500);
    //counter_read_rpm(pulsePin2);
    // Serial.println(fanRPM);
    getTemp();
}
