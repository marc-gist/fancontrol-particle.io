// This #include statement was automatically added by the Particle IDE.
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

boolean fanChanged = true;

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
const int pwmFans[] = {RX, TX, D2, D3};
const int rpmPins[] = {A0, A1, A2, A3};

// use to auto change fan?
bool doFanAutoTempChange = true;
double lowTemp[] = {30.0, 30.0, 30.0, 30.0};
double highTemp[] = {48.0, 48.0, 48.0, 48.0};
bool tempChange[] = {true, true, true, true}; // change fan with temp?

const int rpmLow = 80; // ~30%
const int rpmMed = 128; // ~50%
const int rpmHigh = 192; // ~75%
const int rpmMax = 252; // ~100% (put slightly lower in order to pwm?)

#define PWM_FREQ 25000

double tempArry[4];
static constexpr int sensorTemp[] = {62, 97, 191, 194};

//const int counter_interrupt_pin = rpmPin; // aka rpmPin

const int k_seconds_per_minute = 60;
const int k_pulses_per_revolution = 2;
const int k_hertz_to_RPM_conversion_factor = k_seconds_per_minute / k_pulses_per_revolution;
String tempString = String("Call gettemp first");

// Timer getTempTimer(5000, getTemp);

int tempChangeCheck() {
  if( !doFanAutoTempChange ) return 0;
  for(int i = 0; i < num; i++) {
    if(tempArry[i] > 20 ) {
      if(tempArry[i] >= highTemp[i] && tempChange[i]) {
        Serial.printlnf("Fan %d set to High", i);
        analogWrite(pwmFans[i], rpmHigh, PWM_FREQ);
      } else
      if(tempArry[i] > lowTemp[i] && tempChange[i]) {
        Serial.printlnf("Fan %d set to Med", i);
        analogWrite(pwmFans[i], rpmMed, PWM_FREQ);
      } else {
        Serial.printlnf("Fan %d set to Low", i);
        analogWrite(pwmFans[i], rpmLow, PWM_FREQ);
      }
    }
  }

  return 1;
}


void setup() {
    Particle.function("setrpm", setRpm);
    Particle.function("gettemp", getTempApi);
    Particle.function("scantemp", getTemp);
    Particle.variable("temps", tempString);
    for(int i=0; i<num; i++) {
      pinMode(pwmFans[i], OUTPUT);
      pinMode(rpmPins[i], INPUT_PULLUP);
      analogWrite(pwmFans[i], rpmMed, PWM_FREQ);
    }

    ow_setPin(tempPin); //OneWire

    pinMode(A4, INPUT); //testing pin, used for variable resistor

    Serial.begin(9600);
    //Particle.publish("fan-mon", "init");
    delay(2000);
    Serial.println("OK");

    //getTempTimer.start();
}

uint8_t sensors[80];
void log(char* msg)
{
    //Particle.publish("log", msg);
    Serial.println(msg);
    delay(100);
}


//return the integer value of the temp for args=0-num
//will be 1000x to keep decimals, need to devide by 1000 to get float value again
int getTempApi(String args) {
  int i = args.toInt();
  return tempArry[i]*1000;
}

// scan bus for DS18x20's and store in tempArry[]
int getTemp(String args) {
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

        // this gets the entire address of the chip
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

        // store according to maped codes based upon just last byte of address
        // TODO: make this programable from web interface?
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
        //Serial.printf("float %d: ", item);
        tempArry[item] = retval;
        //Serial.println(retval);
			}
			else
			{
			    log("CRC Error (lost connection?)");
			}
        }
    }//end for
    tempString = String::format("[%.3f, %.3f, %.3f, %.3f]",
      tempArry[0], tempArry[1], tempArry[2], tempArry[3]);

    return 1;
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

    getTemp("all"); // also can enable timers.

    // this is working, so start skipping so we can do other work
    /*for(i = 0; i < num; i++) {
      counter_read_rpm(rpmPins[i]);
      // TODO: check return values, and alert if issue?
      delay(250);
    }*/


    Serial.println("DELAY");
    delay(interval);

    // int v = analogRead(A4);
    // v = v/16;
    // if( v < 35 ) v = 35;
    // if( v > 250) v = 250;

    // Serial.print("A: ");
    // Serial.println(v/252.0*100.0);

    // for(i=0; i<num; i++) {
    //   analogWrite(pwmFans[i], v, PWM_FREQ);
    // }



}

/*
* used to set the RPM (PWM 0-250) on a pin
* syntax would be '<FAN>=<PWM>'
* 0=250
* would set pwmFans[0] pin to 250 PWM duty cycle (approx 100%)
*/
int setRpm(String args) {
    int pos = args.indexOf('=');
    if( pos == -1 ) return -2;
    String fan_s = args.substring(0, pos);
    String val_s = args.substring(pos+1);
    Serial.printlnf("fan_s: %s", fan_s.c_str());
    Serial.printlnf("val_s: %s", val_s.c_str());
    int fan = fan_s.toInt();
    int val = val_s.toInt();
    if( fan == num ) {
      for(int i=0; i<num; i++) {
        analogWrite(pwmFans[i], val, PWM_FREQ);
      }
      return num;
    } else if( fan < 0 || fan > num ) {
      return -3;
    } else {
      if( val > 35 && val < 255 )
        analogWrite(pwmFans[fan], val, PWM_FREQ);
      else
        return -1;

      delay(2000);
      return counter_read_rpm(rpmPins[fan]);
    }
    return -5;
}
