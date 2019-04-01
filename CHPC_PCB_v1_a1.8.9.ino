/*
	Cheap Heat Pump Controller (CHPC) firmware.
	Copyright (C) 2018-2019 Gonzho (gonzho@web.de)

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	
	See https://github.com/gonzho000/chpc/ for more details
*/



//-----------------------USER OPTIONS-----------------------
#define BOARD_TYPE_G 		//Type "G"
//#define BOARD_TYPE_F		//Type "F"

//#define DISPLAY_096 		1
#define DISPLAY_1602 		2 		// patch "inline size_t LiquidCrystal_I2C::write(uint8_t value)" if only 1st character appears: "return 1" instead of "return 0"
//#define DISPLAY_NONE		-1

//#define INPUTS_AS_BUTTONS	1  		//pulldown resistors required!
//#define RS485_PYTHON		1  	
#define RS485_HUMAN   		2

//#define WATCHDOG          			//only if u know what to do
//-----------------------TUNING OPTIONS -----------------------
#define MAX_WATTS		1170.0		//user for power protection

#define DEFFERED_STOP_HOTCIRCLE	3000000		//5 mins

#define POWERON_PAUSE     	300000;    	//5 mins
#define MINCYCLE_POWEROFF 	300000;    	//5 mins
#define MINCYCLE_POWERON  	3600000;  	//60 mins

#define MAGIC     		0x39   		//change if u want to reinit T sensors
//-----------------------USER OPTIONS END -----------------------

//#define EEV_SUPPORT
//???#define HUMAN_AUTOINFO	20  		//!!!send periodical info, seconds, default = 20
//#define INPUTS_AS_INPUTS	2  		//!!!
//define RS485_MACHINE 		3 		//?? or part of Python?



//-----------------------changelog-----------------------
/*
v1.0:
- Displays support
- no more softserial
- define TYPE F/G and rearrange ports
- multi-DS18b20 support on lane
- skip non-important DS18B20 during init
- rewrite Main Cycle to unification: some sensors can be absent, ex: T_hot_out can be absent because i'ts used as target
- 2 on-board buttons support: +/- aim
- DISPLAY: indication: real and aim
- RS485_HUMAN: remote commands +,-,G,0x20/?/Enter
- buttons: < > increase_decrease t
- simpliest thermostat scheme: only T target
- rename all procs
- RS485_PYTHON: print to console inspite of mode diring init proc
- faster wattage overload processing
- write aim value to EE if needed, period: 15 mins (eq. 1041 days)
- deferred stop of hot side circle
- 80 microseconds at 9600

//TODO:
- HUMAN_AUTOINFO time
- current sensor optional
- few devices at same lane for RS485_HUMAN
- EEV_Support
- EEV_recalibration_time to stop HP and recalibrate EEV from zero level ex: every 24 hours
- valve_4way
- rewite re-init proc from MAGIC to emergency jumper removal at board start
- emergency jumper support
? periodical start of hot side circle
- Liquid ref. T protection
*/
//-----------------------changelog END-----------------------

// DS18B20 pins: GND DATA VDD

//Connections:
//DS18B20 Pinout (Left to Right, pins down, flat side toward you)
//- Left   = Ground
//- Center = Signal (Pin N of arduino):  (with 3.3K to 4.7K resistor to +5 or 3.3 )
//- Right  = +5 or +3.3 V   
//


//
// high volume scheme:        +---- +5V (12V not tested)
//                            |
//                       +----+
//                    1MOhm   piezo
//                       +----+
//                            |(C)
// pin -> 1.6 kOhms -> (B) 2n2222        < front here
//                            |(E)
//                            +--- GND
//

/*
scheme SCT-013-000:

2 pins used: tip and sleeve, center (ring) not used http://cms.35g.tw/coding/wp-content/uploads/2014/09/SCT-013-000_UNO-1.jpg
pins are interchangeable due to AC

32 Ohms (22+10) between sensor pins  (35 == ideal)

Pin1: 
- via elect. cap. to GND
- via ~10K..470K resistor to GND
- via ~10K..470K resistor to +5 (same as prev.)
if 10K+10K used: current is 25mA
use 100K+100K for 3 phases

Pin2:
- to analog pin
- via 32..35 Ohms resistor to Pin1

+5 -------------------------+
                            |                  
                            |                                            
                            # R1 10K+                        
                            |                                
                            |                                
                            |~2.5 at this point              
            +---------------+--------------------------------------+----+
            |               |                                      |    |
            #_ elect. cap.  # R2 10K+ (same as R1)     SCT-013-000 $    # R3 = 35 Ohms (ideal case), 32 used  
            |               |                                      |    |
GND --------+---------------+                                      +----+--------> to Analog pin


WARNING: calibrate 3 sensors together, from different sellers, due to case of incorrectly worked 1 of 3 sensor

P(watts)=220*220/R(Ohms)
*/

//
//MAX 485 voltage - 5V
//
// use resistor at RS-485 GND
// 1st test: 10k result lot of issues
// 2nd test: 1k, issues
// 3rd test: 100, see discussions


//16-ch Multiplexer EN pin: active LOW, connect to GND

//used pins:
//!!! ACTUALISE
//2: Z
//3: S3
//4: S2
//5: S1
//6: S0
//7: relay 2
//8: relay 3
//9: speaker
//10: relay 4
//11-13: rs485
//A0: relay 1
//A1: power monitor

/*
relay 1: heat pump
relay 2: hot side pump
relay 3: cold side pump
relay 4: (future) heatpump sump heater

t0: room
t1: heatpump sump
t2: cold in
t3: cold out
t4: hot in
t5: hot out
t6: before condenser
t7: condenser-evaporator
t8: after evaporator
t9: outer
tA: warm floor

wattage1

*/

String fw_version = "1.0";

#ifdef DISPLAY_096
	#define DISPLAY DISPLAY_096
	#include <Wire.h>
	#include "SSD1306Ascii.h"
	#include "SSD1306AsciiWire.h"
	#define I2C_ADDRESS 0x3C
	SSD1306AsciiWire oled;
#endif 

#ifdef DISPLAY_1602
	#define DISPLAY DISPLAY_1602
	#include <Wire.h>
	#include "LiquidCrystal_I2C.h"
	LiquidCrystal_I2C lcd(0x3f,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#endif 

#ifdef DISPLAY_NONE
	#define DISPLAY DISPLAY_NONE
#endif 

#ifndef DISPLAY
	#define DISPLAY -1
#endif

//

#ifdef INPUTS_AS_BUTTONS
	#define  INPUTS INPUTS_AS_BUTTONS
#endif 

#ifdef INPUTS_AS_INPUTS
	#define  INPUTS INPUTS_AS_INPUTS
#endif 

//

#ifdef RS485_PYTHON
	#define  RS485 RS485_PYTHON
	char ishuman = 0;
#endif

#ifdef RS485_HUMAN
	#define  RS485 RS485_HUMAN
	char ishuman = 1;
#endif

//hardware resources
#define OW_BUS_ALLTSENSORS    12
#define SerialTxControl       13   //RS485 Direction control DE and RE to this pin
#define speakerOut            6
#define em_pin1               A6
#define EMERGENCY_PIN         A7

#ifdef BOARD_TYPE_G
	String hw_version = "Type G v1.0.x";
	#define RELAY_HEATPUMP        	8
	#define RELAY_HOTSIDE_CIRCLE  	9
	#define RELAY_COLDSIDE_CIRCLE 	7
	#define RELAY_SUMP_HEATER     	10
	#define RELAY_4WAY_VALVE      	11
	#ifdef INPUTS_AS_BUTTONS
		#define BUT_RIGHT 	A0
		#define BUT_LEFT  	A3
	#endif
#elif BOARD_TYPE_F
	//!!!!
#endif
//---------------------------memory debug
#ifdef __arm__
	// should use uinstd.h to define sbrk but Due causes a conflict
	extern "C" char* sbrk(int incr);
#else // __ARM__
	extern char *__brkval;
#endif // __arm__
     
int freeMemory() {
	char top;
	#ifdef __arm__
		return &top - reinterpret_cast<char*>(sbrk(0));
	#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
		return &top - __brkval;
	#else // __arm__
		return __brkval ? &top - __brkval : &top - __malloc_heap_start;
	#endif // __arm__
}
//---------------------------memory debug END

    
#include <avr/wdt.h>
#include <EEPROM.h>
//#include <FastCRC.h>
/*FastCRC16 CRC16;
union _crc {
	unsigned int   integer;
	char           bytes[2];
} crc;
*/

#include <SoftwareSerial.h>

#define SerialRX        0   //RX connected to RO - Receiver Output  
#define SerialTX        1   //TX connected to DI - Driver Output Pin
#define RS485Transmit    HIGH
#define RS485Receive     LOW

const char devID  = 0x44; //0x3B == ;
const char hostID = 0x30;

SoftwareSerial RS485Serial(SerialRX, SerialTX); // RX, TX

#include <OneWire.h>
#include <DallasTemperature.h>
//library's DEVICE_DISCONNECTED_C -127.0

OneWire ow_ALLTSENSORS(OW_BUS_ALLTSENSORS);
DallasTemperature s_allTsensors(&ow_ALLTSENSORS);

typedef struct {
	DeviceAddress addr;
	bool e;	//enabled
	double        T;
} st_tsens;

DeviceAddress dev_addr;		//temp

st_tsens Tae	;
st_tsens Tbe	;
st_tsens Ttarget;
st_tsens Tsump;
st_tsens Tci	;
st_tsens Tco	;
st_tsens Thi	;
st_tsens Tho	;
st_tsens Tbc	;
st_tsens Tac	;
st_tsens Touter;
st_tsens Ts1	;
st_tsens Ts2	;

#define BIT_Tae   	0
#define BIT_Tbe     	1
#define BIT_Ttarget 	2
#define BIT_Tsump   	3
#define BIT_Tci     	4
#define BIT_Tco     	5
#define BIT_Thi     	6
#define BIT_Tho     	7
#define BIT_Tbc     	8
#define BIT_Tac     	9
#define BIT_Touter  	10
#define BIT_Ts1     	11
#define BIT_Ts2     	12

unsigned int used_sensors = 0	;    			//bit array

double T_setpoint 			= 26.5;  
double T_setpoint_lastsaved		= T_setpoint;
const double cT_setpoint_max 		= 45.0;  
const double cT_heat_delta_min 		= 2.0;
const double cT_sump_min 		= 12.0;
const double cT_sump_max 		= 101.0;
const double cT_sump_heat_threshold 	= 16.0;
//const double cT_sump_outerT_threshold	= 18.0;    	//?? seems to be not useful
const double cT_before_condenser_max 	= 99.0;      
const double cT_after_evaporator_min 	= -7.0;      	// working evaporation presure ~= -10, it is constant due to large evaporator volume     // waterhouse v1: -12 is too high
const double cT_cold_min 		= -8.0;
const double cT_hotout_max 		= 50.0;
//const double cT_workingOK_cold_delta_min = 0.5; 	// 0.7 - 1st try, 2nd try 0.5
const double cT_workingOK_hot_delta_min	= 0.5;   
const double cT_workingOK_sump_min 	= 40.0;        	//need to be not very high to normal start after deep freeze
const double c_wattage_max 		= MAX_WATTS;   	//FUNAI: 1000W seems to be normal working wattage INCLUDING 1(one) CR25/4 at 3rd speed
							//PH165X1CY : 920 Watts, 4.2 A  
const double c_workingOK_wattage_min 	= c_wattage_max/2.5;     //

int heatpump_state    		= 0;
int hotside_circle_state  	= 0;
int coldside_circle_state 	= 0;
int sump_heater_state    	= 0;

const long poweron_pause     	= POWERON_PAUSE    ;    	//default 5 mins
const long mincycle_poweroff 	= MINCYCLE_POWEROFF;    	//default 5 mins
const long mincycle_poweron  	= MINCYCLE_POWERON ;  		//default 60 mins
int _1st_start_sleeped 		= 0;
//??? TODO: periodical start ?
//const long floor_circle_maxhalted = 6000000;  //circle NOT works max 100 minutes
const long deffered_stop_hotcircle = DEFFERED_STOP_HOTCIRCLE;
          
//main cycle vars
unsigned long millis_prev	= 0;
unsigned long millis_now	= 0;
unsigned long millis_cycle	= 5000;

unsigned long millis_last_heatpump_on  = 0;
unsigned long millis_last_heatpump_off = 0;

unsigned long millis_notification  		= 0;
unsigned long millis_notification_interval 	= 33000;

unsigned long millis_displ_update          	= 0;
unsigned long millis_displ_update_interval 	= 10000;

unsigned long millis_escinput	=  0;  
unsigned long millis_charinput 	=  0;  

unsigned long millis_lasteesave	=  0;  

int skipchars = 0;

#define ERR_HZ 2500


char inData[50];      // Allocate some space for the string, do not change that size!
char inChar= -1;      // space to store the character read
byte index = 0;       // Index into array; where to store the character

//-------------temporary variables
char temp[10];
int i	= 0;
int z	= 0;
int x	= 0;
double tempdouble	= 0.0;
int tempint       	= 0;

String outString;
//-------------EEPROM
int eeprom_magic_read = 0x00;
int eeprom_addr       = 0x00;      
//initial values, saved to EEPROM and can be modified later
//CHANGE eeprom_magic after correction!
const int eeprom_magic = MAGIC;

//-------------ERROR states
#define ERR_OK       	0
#define ERR_T_SENSOR   	1
#define ERR_HOT_PUMP 	2
#define ERR_COLD_PUMP 	3
#define ERR_HEATPUMP 	4
#define ERR_WATTAGE  	5

int errorcode 		= 0;

//--------------------------- for wattage 
#define ADC_BITS 10                      //10 fo regular arduino
#define ADC_COUNTS (1<<ADC_BITS)
int em_calibration  	= 62.5;
int em_samplesnum   	= 2960;   // Calculate Irms only 1480 == full 14 periods for 50Hz
//double Irms       	= 0;      //for tests with original procedure
int supply_voltage   	= 0;
int em_i            	= 0;
//phase 1
int sampleI_1       	= 0;
double filteredI_1  	= 0;
double offsetI_1    	= ADC_COUNTS>>1;             //Low-pass filter output
double sqI_1,sumI_1 	= 0;            //sq = squared, sum = Sum, inst = instantaneous
double async_Irms_1 	= 0;
double async_wattage 	= 0;
//--------------------------- for wattage END

//!!!
#include <Stepper.h>
const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
Stepper myStepper(stepsPerRevolution, 2, 3, 4, 5);
//!!!

//--------------------------- functions
long ReadVcc() {
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
		ADMUX = _BV(MUX5) | _BV(MUX0);
	#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
		ADMUX = _BV(MUX3) | _BV(MUX2);
	#else
		ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#endif  

	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA,ADSC)); // measuring
	
	uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
	uint8_t high = ADCH; // unlocks both
	
	long result = (high<<8) | low;
	//constant NOT same as in battery controller!
	result = 1126400L / result; // Calculate Vcc (in mV); (me: !!) 1125300  (!!) = 1.1*1023*1000
	return result; // Vcc in millivolts
}

char CheckAddrExists(void) { 
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Tae.addr[i]) break;	}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Tbe.addr[i]) break;	}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Ttarget.addr[i]) break;}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Tsump.addr[i]) break;}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Tci.addr[i]) break;}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Tco.addr[i]) break;}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Thi.addr[i]) break;}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Tho.addr[i]) break;}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Tbc.addr[i]) break;}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Tac.addr[i]) break;}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Touter.addr[i]) break;}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Ts1.addr[i]) break;}
	if (i == 8) return 1;
	for (i = 0; i < 8; i++) {	if (dev_addr[i] != Ts2.addr[i]) break;}
	if (i == 8) return 1;
	return 0;	
}

void InitS_and_D(void) {
	#ifdef  DISPLAY_096
		Wire.begin();
		oled.begin(&Adafruit128x64, I2C_ADDRESS);
		oled.setFont(Adafruit5x7);
	#endif
	#ifdef   DISPLAY_1602
		lcd.init();            // initialize the lcd 
		lcd.backlight();       // not really needed
	#endif
	RS485Serial.begin(9600);
}

void PrintS_and_D (String str, int printSerial = 1) {
	char *outChar=&str[0];  
	//#ifdef RS485_HUMAN
	if (ishuman != 0) {
		if (printSerial == 1) {
			digitalWrite(SerialTxControl, RS485Transmit);
			delay(1);    
			RS485Serial.print(outChar);
			RS485Serial.println();
			RS485Serial.flush();
			digitalWrite(SerialTxControl, RS485Receive);  
		}
	}
	//#endif
	if (str == "") {
		return;
	}
	#ifdef  DISPLAY_096
		oled.clear();
		oled.println(str);
	#endif
	#ifdef   DISPLAY_1602
		lcd.backlight();     
		lcd.clear();
		lcd.print(str);
	#endif
}

void _PrintHelp(void) {   
	//sprintf(tweetMsg, "first variable = %d, 2nd variable = %ul", var1, var2) 
	PrintS_and_D( "CHPC, https://github.com/gonzho000/chpc/ fw: " + fw_version  + " board: "+ hw_version);
	PrintS_and_D(F("Commands:"));
	PrintS_and_D(F("(?) help"));
	PrintS_and_D(F("(+) increase aim T"));
	PrintS_and_D(F("(-) decrease aim T"));
	PrintS_and_D(F("(G) get stats"));
}

void PrintS_and_D_double (double double_to_print) {
	dtostrf(double_to_print,1,2,temp);
	PrintS_and_D(temp);
}

int Inc_T (void) {
	if (T_setpoint + 0.5 > cT_setpoint_max) {
		PrintS_and_D(F("ERR: Max T!"));	
		delay (200);
		return 0;
	}
	T_setpoint += 0.5;
	PrintS_and_D_double(T_setpoint);	
	return 1;	
}

int Dec_T (void) {
	if (T_setpoint - 0.5 < 1.0) {
		PrintS_and_D(F("ERR: Min T!"));	
		delay (200);
		return 0;
	}
	T_setpoint -= 0.5;
	PrintS_and_D_double(T_setpoint);	
	return 1;
}


void print_Serial_SaD (double num) {	//global string + double
	RS485Serial.print(outString);
	RS485Serial.println(num);
}

void PrintStats_Serial (void) {
	#ifdef RS485_HUMAN
		digitalWrite(SerialTxControl, RS485Transmit);
		delay(1);    
		if (Tae.e == 1)		{outString = "Tae: "	; print_Serial_SaD(Tae.T);	}
		if (Tbe.e == 1) 	{outString= "Tbe: "	; print_Serial_SaD(Tbe.T);   	}
		if (Ttarget.e == 1) 	{outString = "Ttarget: "; print_Serial_SaD(Ttarget.T);	} 
		if (Tsump.e == 1)  	{outString = "Tsump: "	; print_Serial_SaD(Tsump.T);   	}
		if (Tci.e == 1)  	{outString = "Tci: "	; print_Serial_SaD(Tci.T);  	}
		if (Tco.e == 1)  	{outString = "Tco: "	; print_Serial_SaD(Tco.T);   	}
		if (Thi.e == 1)  	{outString = "Thi: "	; print_Serial_SaD(Thi.T);   	}
		if (Tho.e == 1)  	{outString = "Tho: "	; print_Serial_SaD(Tho.T);   	}
		if (Tbc.e == 1)  	{outString = "Tbc: "	; print_Serial_SaD(Tbc.T);   	}
		if (Tac.e == 1)  	{outString = "Tac: "	; print_Serial_SaD(Tac.T);   	}
		if (Touter.e == 1)  	{outString = "Touter: "	; print_Serial_SaD(Touter.T);  	}
		if (Ts1.e == 1)  	{outString = "Ts1: "	; print_Serial_SaD(Ts1.T);   	}
		if (Ts2.e == 1)  	{outString = "Ts2: "	; print_Serial_SaD(Ts2.T);   	}
		outString = "Err: " + String(errorcode) + "\n\rWatts:" + String(async_wattage) + "\n\rAim: ";	print_Serial_SaD(T_setpoint); 
		RS485Serial.println();
		RS485Serial.flush();
		digitalWrite(SerialTxControl, RS485Receive);  
	#endif
}

void ReadEECheckAddr(unsigned char *to_addr) {
	for (i=0 ; i<8 ; i++) { 
		to_addr[i] = EEPROM.read(eeprom_addr);       
		eeprom_addr++;
	}
	i = 0;
	CheckIsInvalidCRCAddr(to_addr);
	if (i != 0) {
		while (1) { 
			PrintS_and_D(F("Err:EEPROM, reinit!"));
			delay(1000);
		}
	}
}

void CheckIsInvalidCRCAddr(unsigned char *addr) {
	if (OneWire::crc8( addr, 7) != addr[7] ) {
		i+= 1;
	}
}

void CopyAddrStoreEE(unsigned char *addr_to, int bit_offset) {  //get result from dev_addr, autoincrement eeprom_addr    
	//dev_addr and z from globals used
	for (i=0 ; i<8 ; i++) {       //no matter
		if (z == 0) {
			dev_addr[i] = 0x00;
		}
		addr_to[i] = dev_addr[i];       
		EEPROM.write(eeprom_addr, dev_addr[i]);         
		eeprom_addr++;
	}
	bitWrite(used_sensors, bit_offset, z);
}

void WriteFloatEEPROM(int addr, float val) { 
	byte *x = (byte *)&val;
	for(byte u = 0; u < 4; u++) EEPROM.write(u+addr, x[u]);
}
 
float ReadFloatEEPROM(int addr) {   
	byte x[4];
	for(byte u = 0; u < 4; u++) x[u] = EEPROM.read(u+addr);
	float *y = (float *)&x;
	return y[0];
}

void SaveSetpointEE(void) {
	if(  	(T_setpoint_lastsaved != T_setpoint) && 
		( ((unsigned long)(millis_now - millis_lasteesave) > 15*60*1000 )  ||  (millis_lasteesave == 0) )  ) {
			eeprom_addr = 1;
			WriteFloatEEPROM(eeprom_addr, T_setpoint);
			millis_lasteesave = millis_now;
			T_setpoint_lastsaved = T_setpoint;
			//PrintS_and_D("Deb: EEsave!"); //!!!
		}
}

void PrintAddr(unsigned char *str) {
	outString = "";
	for (i = 0; i < 8; i++) {
		if (str[i] < 0x10) outString += "0";
		outString += String(str[i], HEX);
	}
	PrintS_and_D(outString);
}

unsigned char FindAddr(String what, int required = 0) {
	i = 1;
	while (RS485Serial.available() > 0) {
		inChar = RS485Serial.read();
		delay(1);
	}
	inChar = 0x00;
	while (1) {
		while (!s_allTsensors.getAddress(dev_addr, 0)) {
			if (required == 0) {
				PrintS_and_D(F("Press > to skip"));
				delay(500);
				while (RS485Serial.available() > 0) {
					inChar = RS485Serial.read();
					if (inChar == 0x3E) {
						PrintS_and_D("Skipped: " + what);
						return 0;
					}
				}
				#ifdef INPUTS_AS_BUTTONS
					i = digitalRead(BUT_RIGHT);
					if (i == 1) {
						PrintS_and_D("Skipped: " + what);
						delay(4000);
						return 0;
					}
				#endif
			}
			PrintS_and_D("Insert " + what);
			delay(1000);
		}
		if ( OneWire::crc8( dev_addr, 7) != dev_addr[7]) {
			RS485Serial.print("Invalid CRC!\n");
			delay(200);
			continue;
		} else if (CheckAddrExists() == 1) {
			PrintS_and_D(F("USED! Remove!"));
			delay(1000);
			continue;
		}  else {
			break;
		}
	}
	while (1) {
		PrintAddr(dev_addr);
		delay(1000);
		if (s_allTsensors.getAddress(dev_addr, 0)) {
			PrintS_and_D("Remove " + what);
			delay(1000);
		} else {
			delay(100);
			break;
		}
	}
	return i;
}

double GetT (unsigned char *str) {
	tempdouble = -127.0;
	for ( i = 0; i < 8; i++) {
		#ifdef WATCHDOG
			wdt_reset();
		#endif
		if ( (tempdouble == 85.0) || (tempdouble == -127.0) ) {
			if ( tempdouble == 85.0 ) {    //initial value in dallas register after poweron
				delay (375);              //375 actual for 11 bits resolution, 2-3 retries OK for 12-bits resolution
			} else {
				delay (37);
			}
			tempdouble = s_allTsensors.getTempC(str);
		} else {
			break;
		}
	}
	return tempdouble;
}

void Get_Temperatures(void) {
	if (Tae.e) 	Tae.T 	= GetT(Tae.addr);
	if (Tbe.e) 	Tbe.T 	= GetT(Tbe.addr);
	Ttarget.T 	= GetT(Ttarget.addr);
	if (Tsump.e) 	Tsump.T = GetT(Tsump.addr);
	if (Tci.e) 	Tci.T 	= GetT(Tci.addr);
	if (Tco.e) 	Tco.T 	= GetT(Tco.addr);
	if (Thi.e) 	Thi.T 	= GetT(Thi.addr);
	if (Tho.e) 	Tho.T 	= GetT(Tho.addr);
	if (Tbc.e) 	Tbc.T 	= GetT(Tbc.addr);
	if (Tac.e) 	Tac.T 	= GetT(Tac.addr);
	if (Touter.e) 	Touter.T = GetT(Touter.addr);
	if (Ts1.e) 	Ts1.T 	= GetT(Ts1.addr);
	if (Ts2.e) 	Ts2.T 	= GetT(Ts2.addr);
	s_allTsensors.requestTemperatures();  //global request
	PrintStats_Serial();	//!!! debug
	//---------DEBUG and self-test !!!--------  
	/*PrintS_and_D("");
	PrintS_and_D_double(Tae.T);
	PrintS_and_D_double(Tbe.T);
	PrintS_and_D_double(Ttarget.T);
	PrintS_and_D_double(Tsump.T);
	PrintS_and_D("");*/
	/*
	PrintS_and_D("Sensor 1 ");
	PrintS_and_D_double(tr_sens_1);
	PrintS_and_D(",\tci ");
	PrintS_and_D_double(tr_cold_in);
	PrintS_and_D(",\tcout ");
	PrintS_and_D_double(tr_cold_out);
	PrintS_and_D(",\thin ");
	PrintS_and_D_double(tr_hot_in);    
	PrintS_and_D(",\tho ");
	PrintS_and_D_double(tr_hot_out);
	PrintS_and_D(",\tbcond ");
	PrintS_and_D_double(tr_before_condenser);    
	PrintS_and_D(",\t outer ");
	PrintS_and_D_double(tr_outer);
	PrintS_and_D(",\t Sensor 2 ");
	PrintS_and_D_double(tr_sens_2);
	*/
	
	//---------DEBUG END--------  
}

//--------------------------- functions END


void setup(void) {
	pinMode	(RELAY_HEATPUMP, 	OUTPUT);
	pinMode	(RELAY_HOTSIDE_CIRCLE, 	OUTPUT);
	pinMode	(RELAY_COLDSIDE_CIRCLE, OUTPUT);
	pinMode	(RELAY_SUMP_HEATER, 	OUTPUT);
	
	digitalWrite	(RELAY_HEATPUMP,	LOW);
	digitalWrite	(RELAY_HOTSIDE_CIRCLE,	LOW);
	digitalWrite	(RELAY_COLDSIDE_CIRCLE,	LOW);
	digitalWrite	(RELAY_SUMP_HEATER,	LOW);
	
	#ifdef WATCHDOG
		wdt_disable();
		delay(2000);
		wdt_enable (WDTO_8S);
	#endif
	
	InitS_and_D();
	pinMode(SerialTxControl, OUTPUT);    
	digitalWrite(SerialTxControl, RS485Receive);
	//digitalWrite(SerialTxControl, RS485Transmit);
	//RS485Serial.println("starting...");	//!!!debug
	delay(100);
	PrintS_and_D("ID: 0x" + String(devID, HEX));
	//Print_Lomem(C_ID);
	delay(2000);
	//PrintS_and_D("setpoint (C):");
	//PrintS_and_D(setpoint);
	
	//PrintS_and_D(String(freeMemory()));  //!!! debug
	
	s_allTsensors.begin();
	s_allTsensors.setWaitForConversion(false);  //ASYNC mode, request before get, see Dallas library for details
	
	eeprom_magic_read = EEPROM.read(eeprom_addr);
	#ifdef INPUTS_AS_BUTTONS
		pinMode		(BUT_RIGHT, INPUT);
		//digitalWrite	(BUT_RIGHT, LOW);
		pinMode		(BUT_LEFT, INPUT);
		//digitalWrite	(BUT_LEFT, LOW);       
	#endif
	//EEPROM content: 
	//0x00 - magic,   
	//0x01  .. 0x04 Target value, 
	//0x05 and 0x06 if sensor enabled or not, used_sensors HI and LO
	//0x07  .. 0x0e 1st addr, etc..
	
	// tr_after_evaporator(0);       tr_before_evaporator(1);     tr_target(2);             tr_sump(3);
	// tr_cold_in(4);                tr_cold_out(5);              tr_hot_in(6);             tr_hot_out(7);
	// tr_before_condenser(8);       tr_after_condenser(9);       tr_outer(10);              tr_sens_1(11);
	// tr_sens_2(12);
	
	eeprom_addr = 0x00;
	if (eeprom_magic_read == eeprom_magic) {
		eeprom_addr += 1;
		T_setpoint = ReadFloatEEPROM(eeprom_addr);
		eeprom_addr += 4;
		PrintS_and_D("EEPROM->T " + String(T_setpoint));
		
		z = EEPROM.read(eeprom_addr);  //high
		eeprom_addr += 1;
		i = EEPROM.read(eeprom_addr);  //lo
		eeprom_addr += 1;
		used_sensors= word (z,i);
	
		Tae.e     = bitRead(used_sensors, BIT_Tae	);
		Tbe.e     = bitRead(used_sensors, BIT_Tbe	);
		Ttarget.e = bitRead(used_sensors, BIT_Ttarget	);
		Tsump.e   = bitRead(used_sensors, BIT_Tsump	);
		Tci.e     = bitRead(used_sensors, BIT_Tci	);
		Tco.e     = bitRead(used_sensors, BIT_Tco	);
		Thi.e     = bitRead(used_sensors, BIT_Thi	);
		Tho.e     = bitRead(used_sensors, BIT_Tho	);
		Tbc.e     = bitRead(used_sensors, BIT_Tbc	);
		Tac.e     = bitRead(used_sensors, BIT_Tac	);
		Touter.e  = bitRead(used_sensors, BIT_Touter	);
		Ts1.e     = bitRead(used_sensors, BIT_Ts1	);
		Ts2.e     = bitRead(used_sensors, BIT_Ts2	);
		#ifdef EEV_SUPPORT
			if (Tae.e != 1 || Tbe.e != 1) {
				while (1) {
					PrintS_and_D("ERR: no Tae or Tbe for EEV!");
					delay (1000);
				}
			}
		#endif
	
		ReadEECheckAddr(Tae.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Tbe.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Ttarget.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Tsump.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Tci.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Tco.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Thi.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Tho.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Tbc.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Tac.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Touter.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Ts1.addr);  //eeprom_addr incremeneted here
		ReadEECheckAddr(Ts2.addr);  //eeprom_addr incremeneted here
		
		i = 0;
		if (Tae.e == 1) 	CheckIsInvalidCRCAddr(Tae.addr	);
		if (Tbe.e == 1) 	CheckIsInvalidCRCAddr(Tbe.addr	);
		if (Ttarget.e == 1) 	CheckIsInvalidCRCAddr(Ttarget.addr);
		if (Tsump.e == 1) 	CheckIsInvalidCRCAddr(Tsump.addr);
		if (Tci.e == 1) 	CheckIsInvalidCRCAddr(Tci.addr	);
		if (Tco.e == 1) 	CheckIsInvalidCRCAddr(Tco.addr	);
		if (Thi.e == 1) 	CheckIsInvalidCRCAddr(Thi.addr	);
		if (Tho.e == 1) 	CheckIsInvalidCRCAddr(Tho.addr	);
		if (Tbc.e == 1) 	CheckIsInvalidCRCAddr(Tbc.addr	);
		if (Tac.e == 1) 	CheckIsInvalidCRCAddr(Tac.addr	);
		if (Touter.e == 1) 	CheckIsInvalidCRCAddr(Touter.addr);
		if (Ts1.e == 1) 	CheckIsInvalidCRCAddr(Ts1.addr	);
		if (Ts2.e == 1) 	CheckIsInvalidCRCAddr(Ts2.addr	);
		if (i != 0) {
			while ( 1 ) { PrintS_and_D(F("EEPROM err1!")); delay (1000); }
		}
	} else {
		eeprom_addr += 1;
		ishuman += 1;
		WriteFloatEEPROM(eeprom_addr, T_setpoint);
		PrintS_and_D(F("init EEPROM"));
		eeprom_addr += 4;
		eeprom_addr += 2; //used sensors, skip
		//Ttarget -needed, other - optional
		#ifdef EEV_SUPPORT
			z = FindAddr("Tae", 1);       //holds result in dev_addr, returns "is used"
		#else 
			z = FindAddr("Tae");          //holds result in dev_addr, returns "is used"
		#endif
		Tae.e = z;
		CopyAddrStoreEE (Tae.addr, BIT_Tae);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		#ifdef EEV_SUPPORT        
			z = FindAddr("Tbe", 1);
		#else
			z = FindAddr("Tbe");
		#endif		
		Tbe.e = z;
		CopyAddrStoreEE (Tbe.addr, BIT_Tbe);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
		
		z = FindAddr("Ttarget", 1);
		Ttarget.e = z;
		CopyAddrStoreEE (Ttarget.addr, BIT_Ttarget);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		z = FindAddr("Tsump");
		Tsump.e = z;
		CopyAddrStoreEE (Tsump.addr, BIT_Tsump);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		z = FindAddr("Tci");
		Tci.e = z;
		CopyAddrStoreEE (Tci.addr, BIT_Tci);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		z = FindAddr("Tco");
		Tco.e = z;
		CopyAddrStoreEE (Tco.addr, BIT_Tco);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		z = FindAddr("Thi");
		Thi.e = z;
		CopyAddrStoreEE (Thi.addr, BIT_Thi);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		z = FindAddr("Tho");
		Tho.e = z;
		CopyAddrStoreEE (Tho.addr, BIT_Tho);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		z = FindAddr("Tbc");
		Tbc.e = z;
		CopyAddrStoreEE (Tbc.addr, BIT_Tbc);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		z = FindAddr("Tac");
		Tac.e = z;
		CopyAddrStoreEE (Tac.addr, BIT_Tac);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		z = FindAddr("Touter");
		Touter.e = z;
		CopyAddrStoreEE (Touter.addr, BIT_Touter);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		z = FindAddr("Ts1");
		Ts1.e = z;
		CopyAddrStoreEE (Ts1.addr, BIT_Ts1);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		z = FindAddr("Ts2");
		Ts2.e = z;
		CopyAddrStoreEE (Ts2.addr, BIT_Ts2);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit
	
		//
		//final, off-the-sequence
		EEPROM.write(0+1+4+0,highByte(used_sensors)); 
		EEPROM.write(0+1+4+1,lowByte(used_sensors));  
		EEPROM.write(0x00, eeprom_magic);
		ishuman -= 1;
	}
	T_setpoint_lastsaved = T_setpoint;
	//s_allTsensors.setResolution(ad_Tae, 12);
	/*PrintAddr(Tae.addr);
	PrintAddr(Tbe.addr);
	PrintAddr(Ttarget.addr);
	PrintAddr(Tsump.addr);
	PrintAddr(Tci.addr);
	PrintAddr(Tco.addr);
	PrintAddr(Thi.addr);
	PrintAddr(Tho.addr);
	PrintAddr(Tbc.addr);
	PrintAddr(Tac.addr);
	PrintAddr(Touter.addr);
	PrintAddr(Ts1.addr);
	PrintAddr(Ts2.addr);*/
	
	Get_Temperatures();
	
	tone(speakerOut, 2250);
	delay (1500); // like ups power on
	noTone(speakerOut);
	
	//!!!
	myStepper.setSpeed(40);
	Serial.begin(9600);
	//!!!
	
	outString.reserve(200);
	//PrintS_and_D(String(freeMemory()));  //!!! debug
}

 
void loop(void) {  
	digitalWrite(SerialTxControl, RS485Receive);
	millis_now = millis();

	//----------------------------- self-test !!!
	
	digitalWrite(RELAY_HEATPUMP,HIGH);
	//delay(300);
	digitalWrite(RELAY_HOTSIDE_CIRCLE,HIGH);
	//delay(300);
	digitalWrite(RELAY_COLDSIDE_CIRCLE,HIGH);
	//delay(300);
	digitalWrite(RELAY_SUMP_HEATER,HIGH);
	/*delay(2000);
	digitalWrite(RELAY_HEATPUMP,LOW);
	delay(300);
	digitalWrite(RELAY_HOTSIDE_CIRCLE,LOW);
	delay(300);
	digitalWrite(RELAY_COLDSIDE_CIRCLE,LOW);
	delay(300);
	digitalWrite(RELAY_SUMP_HEATER,LOW);
	*/

	// step one revolution  in one direction:
	Serial.println("clockwise");
	myStepper.step(1000);
	delay(500);

	// step one revolution in the other direction:
	Serial.println("counterclockwise");
	myStepper.step(-1000);
	delay(500);
	//----------------------------- self-test END
	
	
	//--------------------async fuction start
	if (em_i == 0) {  
		supply_voltage = ReadVcc();
	}
	if (em_i < em_samplesnum) {
		sampleI_1 = analogRead(em_pin1);
	
		// Digital low pass filter extracts the 2.5 V or 1.65 V dc offset, then subtract this - signal is now centered on 0 counts.
		offsetI_1 = (offsetI_1 + (sampleI_1-offsetI_1)/1024);
		filteredI_1 = sampleI_1 - offsetI_1;
	
		// Root-mean-square method current
		// 1) square current values
		sqI_1 = filteredI_1 * filteredI_1;
		// 2) sum
		sumI_1 += sqI_1;
		
		em_i += 1;
	} else {
		em_i = 0;
		double I_RATIO = em_calibration *((supply_voltage/1000.0) / (ADC_COUNTS));
		async_Irms_1 = I_RATIO * sqrt(sumI_1 / em_samplesnum);
		async_wattage = async_Irms_1*220.0;
	
		//Reset accumulators
		sumI_1 = 0;
		
		//----------------------------- self-test !!!
		/*
		PrintS_and_D("Async impl. results 1:  ");
		PrintS_and_D(async_wattage);	           // Apparent power
		PrintS_and_D(" ");
		PrintS_and_D(async_Irms_1);	           // Irms
		PrintS_and_D(" voltage: ");
		PrintS_and_D(supply_voltage);
		*/
		//----------------------------- self-test END
		
	}
	//--------------------async fuction END    
	
	if (  heatpump_state == 1   &&  async_wattage > c_wattage_max   ){
		PrintS_and_D(F("Overload stop."));
		millis_last_heatpump_on = millis_now;
		heatpump_state = 0;
		digitalWrite(RELAY_HEATPUMP, heatpump_state);
	}
	
	//-------------------buttons processing
	#ifdef INPUTS_AS_BUTTONS
		z = digitalRead(BUT_LEFT);
		i = digitalRead(BUT_RIGHT);
		if ( (z == 1) && ( i == 1) ) {
			//
		} else if ( (z == 1) || ( i == 1) ) {
			if ( z == 1 ) {
				x = Dec_T();
			} 
			if ( i == 1 ) {
				x = Inc_T();
			}
			if (x == 1) {
				PrintS_and_D("New aim: " + String(T_setpoint));
				delay(100);
			}
		}
	#endif
	//-------------------buttons processing END
	
	//-------------------display
	#if (DISPLAY == 2) || (DISPLAY == 1)
		if(  ((unsigned long)(millis_now - millis_displ_update) > millis_displ_update_interval )  ||  (millis_displ_update == 0) ) {
			outString = "A:" + String(T_setpoint, 1) + " Real:";
			if (Ttarget.T != -127.0){
				outString +=  String(Ttarget.T, 1);
			} else {
				outString +=  "ERR";
			}
			PrintS_and_D(outString, 1);    //do not print serial
			millis_displ_update = millis_now;
		}
	#endif
	//-------------------display END
	
	//-------------------check cycle
	if(  ((unsigned long)(millis_now - millis_prev) > millis_cycle )  ||  (millis_prev == 0) ) {
		millis_prev = millis_now;
		Get_Temperatures();  //      wdt_reset here due to 85.0'C filtration
		SaveSetpointEE();
	
		//--------------------important logic
		//check T sensors
		if ( ( errorcode == ERR_OK )     && (   (Tae.e == 1 && Tae.T == -127 )  || 
							(Tbe.e == 1 && Tbe.T == -127 )	|| 
							Ttarget.T == -127 		||
							(Tsump.e == 1 && Tsump.T == -127 ) || 
							(Tci.e == 1 && Tci.T == -127 )	|| 
							(Tco.e == 1 && Tco.T == -127 )	|| 
							(Thi.e == 1 && Thi.T == -127 )	|| 
							(Tho.e == 1 && Tho.T == -127 )	|| 
							(Tbc.e == 1 && Tbc.T == -127 )	|| 
							(Tac.e == 1 && Tac.T == -127 )	|| 
							(Touter.e == 1 && Touter.T == -127 ) || 
							(Ts1.e == 1 && Ts1.T == -127 )	|| 
							(Ts2.e == 1 && Ts2.T == -127 )             ) ) {
										errorcode = ERR_T_SENSOR;
										PrintS_and_D("ERR:T.sens." + String(errorcode));
		}
		//auto-clean sensor error on sensor appear	
		// add 1xor enable here!
		if ( ( errorcode == ERR_T_SENSOR ) && ( ( (Tae.e == 1 	&& Tae.T != -127 )	||(Tae.e	^1) )	&&
							( (Tbe.e == 1 	&& Tbe.T != -127 )	||(Tbe.e	^1) )	&&
							Ttarget.T != -127 				&&
							( (Tsump.e == 1 && Tsump.T != -127 ) 	||(Tsump.e	^1) )	&&
							( (Tci.e == 1 	&& Tci.T != -127 ) 	||(Tci.e	^1) )	&&
							( (Tco.e == 1 	&& Tco.T != -127 )	||(Tco.e	^1) )	&& 
							( (Thi.e == 1 	&& Thi.T != -127 )	||(Thi.e	^1) )	&& 
							( (Tho.e == 1 	&& Tho.T != -127 )	||(Tho.e	^1) )	&& 
							( (Tbc.e == 1 	&& Tbc.T != -127 )	||(Tbc.e	^1) )	&& 
							( (Tac.e == 1 	&& Tac.T != -127 )	||(Tac.e	^1) )	&& 
							( (Touter.e == 1 && Touter.T != -127 ) 	||(Touter.e	^1) )	&&
							( (Ts1.e == 1 	&& Ts1.T != -127 )	||(Ts1.e	^1) )	&&
							( (Ts2.e == 1 	&& Ts2.T != -127 )        ||(Ts2.e	^1) )     ) ) {
										errorcode = ERR_OK;
		}
			
		//process errors
		//beep N times error
		if ( errorcode != ERR_OK ) {
			if (  ((unsigned long)(millis_now - millis_notification) > millis_notification_interval)  ||  millis_notification == 0 ) {
				millis_notification = millis_now;
				PrintS_and_D("Error:" + String(errorcode));
				for ( i = 0; i < errorcode; i++) {
					tone(speakerOut, ERR_HZ);  	delay (500);      
					noTone(speakerOut);      	delay (500);
				}
			}
		}
		
		//process heatpump sump heater
		if (Tsump.e == 1) {
			if ( Tsump.T < cT_sump_heat_threshold   &&   sump_heater_state == 0   &&   Tsump.T != -127) {
				sump_heater_state = 1;
			} else if (Tsump.T >= cT_sump_heat_threshold  && sump_heater_state == 1) {
				sump_heater_state = 0;
			} else if (Tsump.T == -127) {
				sump_heater_state = 0;
			}
			digitalWrite(RELAY_SUMP_HEATER, sump_heater_state);
		}



		//main logic
		if (_1st_start_sleeped == 0) {
			PrintS_and_D("!!!!sleep disabled!!!!");
			_1st_start_sleeped = 1;
			if ( (millis_now < poweron_pause) && (_1st_start_sleeped == 0) ) {
				PrintS_and_D("Wait: " + String(((poweron_pause-millis_now))/1000) + " s.");
				return;
			} else {
				_1st_start_sleeped = 1;
			}
		}
		
		//process_heatpump:
		//start if
		//    (last_on > N or not_started_yet)
		//    and (no errors)
		//    and (t hot out < t target + heat_delta_min)
		//    and (sump t > min'C)
		//    and (sump t < max'C)
		//    and (t watertank < target)
		//    and (t after evaporator > after evaporator min)
		//    and (t cold in > cold min)
		//    and (t cold out > cold min)
		if ( 	heatpump_state == 0     		&&
			(((unsigned long)(millis_now - millis_last_heatpump_on) > mincycle_poweroff)   ||   (millis_last_heatpump_on == 0)  )  &&
			//( tr_hot_out < (tr_sens_1 + cT_heat_delta_min)  )    &&
			errorcode == 0         							&&
			( (Tsump.e == 1 	&& Tsump.T > cT_sump_min)  	|| (Tsump.e^1))	&&
			( (Tsump.e == 1 	&& Tsump.T < cT_sump_max)  	|| (Tsump.e^1))	&&
			Ttarget.T < T_setpoint  						&&    //was room here, change to advanced algo with room temperature
			( (Tae.e == 1 	&& Tae.T > cT_after_evaporator_min)	|| (Tae.e^1))	&&
			( (Tbc.e == 1 	&& Tbc.T < cT_before_condenser_max) 	|| (Tbc.e^1))	&&
			( (Tci.e == 1 	&& Tci.T > cT_cold_min)  		|| (Tci.e^1))	&&
			( (Tco.e == 1 	&& Tco.T > cT_cold_min) 		|| (Tco.e^1))	) {
				PrintS_and_D(F("Start"));
				millis_last_heatpump_off = millis_now;
				heatpump_state = 1;
		}
		
		//stop if
		//    ( (last_off > N) and (t watertank > target) )
		if ( heatpump_state == 1     &&     ((unsigned long)(millis_now - millis_last_heatpump_off) > mincycle_poweron)    &&    (Ttarget.T > T_setpoint)) {
			PrintS_and_D(F("Normal stop"));
			millis_last_heatpump_on = millis_now;
			heatpump_state = 0;
		}
	
		//process_hot_side_pump:
		//start if (heatpump_enabled)
		//stop if (heatpump_disabled and (t hot out or in < t target + heat delta min) )
		if (  (heatpump_state == 1)   &&  (hotside_circle_state  == 0)  ) {    
			PrintS_and_D(F("Hot WP ON"));
			hotside_circle_state  = 1;
		}
		
		if (  (heatpump_state == 0)        &&    (hotside_circle_state  == 1) ) {
			if (  (deffered_stop_hotcircle != 0 	&& 	((unsigned long)(millis_now - millis_last_heatpump_on) > deffered_stop_hotcircle)   )	) {
				if ( 	(Tho.e == 1 && Tho.T < (Ttarget.T + cT_heat_delta_min))	||
					(Thi.e == 1 && Thi.T < (Ttarget.T + cT_heat_delta_min))	) {
					PrintS_and_D(F("Hot WP OFF 1"));
					hotside_circle_state  = 0;
				} else {
					PrintS_and_D(F("Hot WP OFF 2"));
					hotside_circle_state  = 0;
				}
			}
		}
		
		//heat if we can, just in case, ex. if lost power
		if ( (hotside_circle_state  == 0) && 
			( Tho.e == 1 && Tho.T > (Ttarget.T + cT_heat_delta_min)  )  	||
			( Thi.e == 1 && Thi.T > (Ttarget.T + cT_heat_delta_min)  )  ) {
				PrintS_and_D(F("Hot WP ON"));
				hotside_circle_state  = 1;
		}
		
		//process_cold_side_pump:
		//start if (heatpump_enabled)
		//stop if (heatpump_disbled)
		if (  (heatpump_state == 1)   &&  (coldside_circle_state  == 0)  ) {
			PrintS_and_D(F("Cold WP ON"));
			coldside_circle_state  = 1;
		}
		
		if (  (heatpump_state == 0)   &&  (coldside_circle_state  == 1)  ) {
			PrintS_and_D(F("Cold WP OFF"));
			coldside_circle_state  = 0;
		}
		
		
		//protective_cycle:
		//stop if
		//      (error)
		//      (t hot out > hot out max)
		//      (sump t > max'C)
		//      or (t after evaporator < after evaporator min)
		//      or (t cold in < cold min)
		//      or (t cold out < cold min)
		//          
		if (  heatpump_state == 1   &&  
			(	errorcode != 0                             		||
				(Tho.e 	== 1 	&&	Tho.T 	> cT_hotout_max)      	||
				(Tsump.e == 1 	&&	Tsump.T > cT_sump_max)   	||
				(Tae.e 	== 1 	&& 	Tae.T 	< cT_after_evaporator_min) ||
				(Tbc.e 	== 1 	&& 	Tbc.T 	> cT_before_condenser_max) ||
				(Tci.e 	== 1 	&& 	Tci.T 	< cT_cold_min )       	||
				(Tco.e 	== 1 	&& 	Tco.T 	< cT_cold_min) )     ) {
					PrintS_and_D(F("Protective stop"));
					millis_last_heatpump_on = millis_now;
					heatpump_state = 0;
					digitalWrite(RELAY_HEATPUMP, heatpump_state);
		}
		
		//alive_check_cycle_after_5_mins:
		//error if
		//      or (t cold in - t cold out < t workingok min)
		//      or (t hot out - t hot in < t workingok min)
		//      or (sump t < 25'C)
		//      or wattage too low
		/*
		if (  heatpump_state == 1   &&  ((unsigned long)(millis_now - millis_last_heatpump_off) > 300000)  ) {
			//cold side processing simetimes works incorrectly, after long period of inactivity, due to T inertia on cold tube sensor, commented out
			//if ( ( errorcode == ERR_OK )     &&   (  tr_cold_in - tr_cold_out < cT_workingOK_cold_delta_min ) ) {
			//    errorcode = ERR_COLD_PUMP;
			//}
			//if ( ( errorcode == ERR_OK )     &&   (  Tho.e == 1 && Thi.e == 1 && (Tho.T - Thi.T < cT_workingOK_hot_delta_min )) ) {
			//	errorcode = ERR_HOT_PUMP;
			//}
			if ( ( errorcode == ERR_OK )     &&   (  Tsump.e == 1 && Tsump.T < cT_workingOK_sump_min )  ) {
				errorcode = ERR_HEATPUMP;
			}
			if ( ( errorcode == ERR_OK )     &&   ( async_wattage < c_workingOK_wattage_min )  ) {
				errorcode = ERR_WATTAGE;
			}
		}*/
			
		//disable pump by error
		if ( errorcode != ERR_OK ) {
			millis_last_heatpump_on = millis_now;
			heatpump_state = 0;
			digitalWrite(RELAY_HEATPUMP, heatpump_state);
			//PrintS_and_D("Error stop: " + String(errorcode, HEX));
		}
		
		//!!! self-test
		///heatpump_state = 1;
		
		//write states to relays
		digitalWrite	(RELAY_HEATPUMP, 	heatpump_state);
		digitalWrite	(RELAY_HOTSIDE_CIRCLE,	hotside_circle_state);
		digitalWrite	(RELAY_COLDSIDE_CIRCLE,	coldside_circle_state);
		digitalWrite	(RELAY_SUMP_HEATER,	sump_heater_state);
	}
	
	if (RS485Serial.available() > 0) {
		//RS485Serial.println("some on serial..");	//!!!debug
		#ifdef RS485_HUMAN
			if (RS485Serial.available()) {
				inChar = RS485Serial.read();
				//RS485Serial.print(inChar);	//!!!debug
				if ( inChar == 0x1B ) {
					skipchars += 3;
					inChar = 0x00;
					millis_escinput = millis();
				}
				if ( skipchars != 0 ) {
					millis_charinput = millis();
					//if (millis_escinput + 2 > millis_charinput)
					if ((unsigned long)(millis_charinput - millis_escinput) < 16*2 ) {	//2 chars for 2400
						if (inChar != 0x7e) {
							skipchars -= 1;
						}
						if (inChar == 0x7e) {
							skipchars = 0;
						}
						if (inChar >= 0x30 && inChar <= 0x35) {
							skipchars += 1;
						}
						inChar = 0x00;
					} else {
						skipchars = 0;
					}
				}
			
				//- RS485_HUMAN: remote commands +,-,G,0x20/?/Enter
				switch (inChar) {
					case 0x00:
						break;
					case 0x20:
						_PrintHelp();
						break;
					case 0x3F:
						_PrintHelp();
						break;
					case 0x0D:
						_PrintHelp();
						break;
					case 0x2B:
						Inc_T();
						break;
					case 0x2D:
						Dec_T();
						break;
					case 0x47:
						PrintStats_Serial();
						break;
					case 0x67:
						PrintStats_Serial();
						break;
					}
			}
		#endif
        
		#ifdef RS485_PYTHON
			index = 0;
			while (RS485Serial.available() > 0) { // Don't read unless you know there is data
				if(index < 49) {   //  size of the array minus 1
					inChar = RS485Serial.read(); 	// Read a character
					inData[index] = inChar;      	// Store it
					index++;                     	// Increment where to write next
					inData[index] = '\0';        	// clear next symbol, null terminate the string
					delayMicroseconds(80);       	//80 microseconds - the best choice at 9600, "no answer"disappeared
									//40(20??) microseconds seems to be good, 9600, 49 symbols
									//
				} else {            //too long message! read it to nowhere
					inChar = RS485Serial.read();
					delayMicroseconds(80);
					//break;    //do not break if symbols!!
				}
			}
		
			//!!!debug, be carefull, can cause strange results
			
			/*if (inData[0] != 0x00) {
			RS485Serial.println("-");
			RS485Serial.println(inData);
			RS485Serial.println("-");
			}*/
			//or this debug
			/*
			digitalWrite(SerialTxControl, RS485Transmit);
			delay(10);
			RS485Serial.println(inData);
			RS485Serial.flush();
			RS485Serial.println(index);
			*/
			
			//ALL lines must be terminated with \n!
			if ( (inData[0] == hostID) && (inData[1] == devID) ) {             
				//  COMMANDS:
				// G (0x47): (G)et main data
				// TNN.NN (0x54): set aim (T)emperature
				digitalWrite(SerialTxControl, RS485Transmit);
				delay(1);
				//PrintS_and_D(freeMemory());
				outString = "";
				outString += devID;
				outString += hostID;
				outString +=  "A ";  //where A is Answer, space after header
			
				if ( (inData[2] == 0x47 ) ) {
					//PrintS_and_D("G");
					//WARNING: this procedure can cause "NO answer" effect if no or few T sensors connected
					outString += "{";
					outString += "\"A1\":" + String(T_setpoint);  //(A)im (target)
					if (Ts1.e == 1) {
						outString += "\"TS1\":" + String(Ts1.T);
					}
					if (Tsump.e == 1) {
						outString += ",\"TS\":" + String(Tsump.T);
					}
					if (Tho.e == 1) {
						outString += ",\"THO\":" + String(Tho.T);
					}
					if (Tae.e == 1) {
						outString += ",\"TAE\":" + String(Tae.T);
					}
					char *outChar=&outString[0];
					RS485Serial.write(outChar);                        	//dirty hack to transfer long string
					RS485Serial.flush();
					delay (1);                                      //lot of errors without delay
					outString = "";
					if (Tbe.e == 1) {
						outString += ",\"TBE\":" + String(Tbe.T);                
					}
					if (Touter.e == 1) {
						outString += ",\"TO\":" + String(Touter.T);
					}
					if (Tco.e == 1) {
						outString += ",\"TCO\":" + String(Tco.T);
					}
					outString += ",\"W1\":" + String(async_wattage);
					outString += ",\"RP\":" + String(heatpump_state*RELAY_HEATPUMP);  
					if (Tci.e == 1) {
						outString += ",\"TCI\":" + String(Tci.T);
					}
					RS485Serial.write(outChar);                        //dirty hack to transfer long string
					RS485Serial.flush();
					delay (1);                                        //lot of errors without delay
					outString = "";
					if (Thi.e == 1) {
						outString += ",\"THI\":" + String(Thi.T);
					}
					outString += ",\"RSH\":" + String(sump_heater_state*RELAY_SUMP_HEATER);                  
					outString += ",\"RH\":" + String(hotside_circle_state*RELAY_HOTSIDE_CIRCLE);                  
					outString += ",\"RC\":" + String(coldside_circle_state*RELAY_COLDSIDE_CIRCLE);  
					if (Tbc.e == 1) {
						outString += ",\"TBC\":" + String(Tbc.T);
					}
					RS485Serial.write(outChar);                        //dirty hack to transfer long string
					RS485Serial.flush();
					delay (1);                                        //lot of errors without delay
					outString = "";
					if (Ts2.e == 1) {
						outString += ",\"TS2\":" + String(Ts2.T);
					}
					if (Tac.e == 1) {
						outString += ",\"TAC\":" + String(Tac.T);
					}
					outString += ",\"TT\":" + String(Ttarget.T);                                
					outString += ",\"E1\":" + String(errorcode);  
					outString += "}";
				} else if ( (inData[2] == 0x54 ) ) {  //format NN.NN, text
					if ( isDigit(inData[ 3 ]) && isDigit(inData[ 4 ]) && (inData[ 5 ] == 0x2e)  && isDigit(inData[ 6 ]) && isDigit(inData[ 7 ]) && ( ! isDigit(inData[ 8 ])) ) {
						
						tone(speakerOut, 2250);
						delay (100); // like ups power on
						noTone(speakerOut);
						
						char * carray = &inData[ 3 ];
						tempdouble = atof(carray);                
						if (tempdouble > cT_setpoint_max) {
							outString += "{\"err\":\"too hot!\"}";
						} else if (tempdouble < 1.0) {
							outString += "{\"err\":\"too cold!\"}";
						} else {
							T_setpoint = tempdouble;
							outString += "{\"result\":\"ok, new value is: ";
							outString += String(T_setpoint);
							outString += "\"}";
						}
					} else {
						outString += "{\"err\":\"NaN, format: NN.NN\"}";
					}
				} else {
					//default, just for example
					outString += "{\"result\":\"no_command\"}";
				}
				//crc.integer = CRC16.xmodem((uint8_t& *) outString, outString.length());
				//outString += (crc, HEX);
				outString += "\n";
				char *outChar=&outString[0];
				RS485Serial.write(outChar);
			}
			
			index = 0;
			for (i=0;i<49;i++) {  //clear buffer
				inData[i]=0;
			}
			RS485Serial.flush();
			digitalWrite(SerialTxControl, RS485Receive);
			delay(1);
		#endif
	}
    
}
