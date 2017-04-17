/*
	Thermoduino
	
	Arduino based thermal regulator for renewable energy systems

	(c)yann jautard <bricofoy@free.fr> 2017
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <yasm.h>
#include "btn.h"
#include <DS1307.h>
#include <SdFat.h>
#include <Streaming.h>

//sensor addresses EEPROM storage adresses
#define EEPROM_BASE_ADR 0
#define SENSOR_NBR 		10 
// arrays to hold sensor addresses and temperatures
DeviceAddress sensorAddress[SENSOR_NBR];
float T[SENSOR_NBR];
float Offset[SENSOR_NBR];
 
#define TEMPERATURE_RESOLUTION 9 //0,5°C sensor acccuracy.

#define PIN_ONE_WIRE_BUS A3

//encoder pins
#define PIN_A	6
#define PIN_B	7
#define PIN_BTN 8
char Counter=0;


//SD card CS pin on the robotdyn shield is pin 9
#define PIN_SD_CS	9

//outputs
#define PIN_PWM1	10	//solar pump speed signal
#define PIN_R1		5	//solar pump
#define PIN_R2		4	//heating pump
#define PIN_R3		3	//heating water temp servo +/-
#define PIN_R4		2	//heating water temp servo on
#define PIN_R5		A0	//boiler pump

byte Outputs=0;
#define OUT_R1	1
#define OUT_R2	2
#define OUT_R3	4
#define OUT_R4	8
#define OUT_R5	16
byte Pwm1=0;

char S1[4];


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(PIN_ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


//Values for the chinese GY-IICLCD backpack used
LiquidCrystal_I2C lcd(0x20, 4, 5, 6, 0, 1, 2, 3, 7, NEGATIVE);

YASM menu;
YASM gettemp;
YASM solar;
YASM heat;
YASM boiler;
BTN btn;

DS1307_t DateTime;

void setup(void)
{
  Serial.begin(9600); 
  
  lcd.begin(4,20);
  lcd.backlight();
  lcd.clear();
  lcd.print(F("Initialisation..."));
  
  RTC.start();
  
  pinMode(PIN_A,INPUT_PULLUP);  //encoder input
  pinMode(PIN_B,INPUT_PULLUP);  
  pinMode(PIN_BTN,INPUT_PULLUP);
  
  pinMode(PIN_R1,OUTPUT);
  pinMode(PIN_R2,OUTPUT);
  pinMode(PIN_R3,OUTPUT);
  pinMode(PIN_R4,OUTPUT);
  pinMode(PIN_R5,OUTPUT);
  pinMode(PIN_PWM1,OUTPUT);
  outputs();
  
  sensors.begin(); // Start up the dallas library
  sensors.setWaitForConversion(false); //don't wait !
  
  loadSensorsAddresses(); //fill the addresses array with the values in EEPROM
  setSensorsResolution();
  
  menu.next(menu_start); 
  gettemp.next(gettemp_request);
}

void loop(void) 
{
	RTC.get(&DateTime,true);
	btn.update(!digitalRead(PIN_BTN)); // ! because btn switch to gnd
	encoderRead();
	gettemp.run();
	menu.run();
	solar.run();
	heat.run();
	boiler.run();
	outputs();
}

void outputs()
{
	digitalWrite(PIN_R1,!(Outputs&OUT_R1));
	digitalWrite(PIN_R2,!(Outputs&OUT_R2));
	digitalWrite(PIN_R3,!(Outputs&OUT_R3));
	digitalWrite(PIN_R4,!(Outputs&OUT_R4));
	digitalWrite(PIN_R5,!(Outputs&OUT_R5));
	analogWrite(PIN_PWM1,Pwm1);
}

void loadSensorsAddresses()
{
	EEPROM.get(EEPROM_BASE_ADR,sensorAddress);
}

void setSensorsResolution()
{
	for (byte i=0;i<SENSOR_NBR; i++)
	{
		sensors.setResolution(sensorAddress[i], TEMPERATURE_RESOLUTION);
	}
}

void encoderRead()
{
	static byte old_states=0;
	static char state_sub=0;
	
	byte pin_states = (digitalRead(PIN_A)<<1);
	pin_states += digitalRead(PIN_B);
	
	// state is now 0, 1, 2 or 3.
	if (pin_states!=old_states)
	{
		// Exor the old & new states to determine the rotation direction.
		char inc = ((pin_states>>1)^old_states)&0x01;
		if (inc==0) inc = -1;
		old_states = pin_states;
		
		// Reset on change of direction.
		if ((inc<0 && state_sub>0) || (inc>0 && state_sub<0)) state_sub = 0;
		
		state_sub += inc;
		if (state_sub<=-4 || state_sub>=4)
		{
			state_sub -= (inc<<2);
			Counter += inc;
		}
	}
}

char encoderCount()
{
	char retval=Counter;
	Counter=0;
	return retval;
}

///////////gettemp state machine/////////
//
void gettemp_request()
{
	sensors.requestTemperatures();
	gettemp.next(gettemp_wait);
}

void gettemp_wait()
{	//we need to wait that time for conversion to finish
	if (gettemp.elapsed(750/(1<<(12-TEMPERATURE_RESOLUTION))))
		gettemp.next(gettemp_read);
}

void gettemp_read()
{
	byte i=gettemp.runCount();
	if (i<SENSOR_NBR)
		T[i] = sensors.getTempC(sensorAddress[i]);
	else
		gettemp.next(gettemp_request);
}


///////////menu state machine////////////
char Pos;

void menu_start()
{
	if (menu.isFirstRun())
	{
		lcd.clear();
		lcd.print(F("Ballon"));
		lcd.setCursor(0,1); lcd.print(F("Capteur            %"));
		lcd.setCursor(0,2); lcd.print(F("Int       Ext"));
		lcd.setCursor(0,3); lcd.print(F("Chauffage"));
	}
 	
	if(menu.periodic(1500)) 
	{
		lcd.setCursor(7,0); lcd.print(F("             "));
		lcd.setCursor(8,0); lcd.print(T[1],1);
		lcd.setCursor(14,0); lcd.print(T[3],1);

		lcd.setCursor(8,1); lcd.print(F("     "));
		lcd.setCursor(8,1); lcd.print(T[0],1);
		lcd.setCursor(16,1); lcd.print("100");
		
		lcd.setCursor(4,2); lcd.print(F("     "));
		lcd.setCursor(4,2); lcd.print(T[7],1);
		lcd.setCursor(14,2); lcd.print(F("     "));
		lcd.setCursor(14,2); lcd.print(T[6],1);
	}
 
	if(encoderCount()>0)
		menu.next(menu_start2);
	
	if(btn.state(BTN_LONGCLICK)) 
		menu.next(menu_param);
}

void menu_start2()
{
	if (menu.isFirstRun())
	{
		lcd.clear();
		lcd.print(F("T1:      T10:"));
		lcd.setCursor(0,1); lcd.print(F("T2:       T3:"));
		lcd.setCursor(0,2); lcd.print(F("R1:"));
		lcd.setCursor(0,3); lcd.print(F("R1:"));
	}	
	
	if(menu.periodic(1500))
	{
		lcd.setCursor(3,0); lcd.print(T[0],1); lcd.setCursor(13,0); lcd.print(T[9],1);
		lcd.setCursor(3,1); lcd.print(T[1],1); lcd.setCursor(13,1); lcd.print(T[2],1); 
		lcd.setCursor(3,2); lcd.print(T[3],1);
		lcd.setCursor(3,3); lcd<<(Outputs&OUT_R1)<<" "<<Pwm1<<"%"; 
	}
	
	if(Counter<0 || menu.elapsed(15E3))
		menu.next(menu_start);
	if(encoderCount()>0)
		menu.next(menu_start3);

}

void menu_start3()
{
	if (menu.isFirstRun())
	{
		lcd.clear();
		lcd.print(F("T5:      T6:"));
		lcd.setCursor(0,1); lcd.print(F("T7:      T8:"));
		lcd.setCursor(0,2); lcd.print(F("R2:  R3:  R4:"));
		lcd.setCursor(0,3); lcd.print(F("T9:       R5:"));
	}
	
	if(menu.periodic(1500))
	{
		lcd.setCursor(3,0); lcd.print(T[4],1); lcd.setCursor(13,0); lcd.print(T[5],1);
		lcd.setCursor(3,1); lcd.print(T[6],1); lcd.setCursor(13,1); lcd.print(T[7],1);
		lcd.setCursor(3,2); lcd<<(Outputs&OUT_R2); 
		lcd.setCursor(8,2); lcd<<(Outputs&OUT_R3); 
		lcd.setCursor(13,2); lcd<<(Outputs&OUT_R4);
		lcd.setCursor(3,3); lcd.print(T[8],1); 
		lcd.setCursor(13,3); lcd<<(Outputs&OUT_R5);
	}
	
	if(Counter<0)
		menu.next(menu_start2);	
	if(menu.elapsed(15E3))
		menu.next(menu_start);
}

	
void menu_param()
{	
	if (menu.isFirstRun()) 
	{
		lcd.clear();
		lcd.print(F(">Reglages horloge"));
		lcd.setCursor(1,1); lcd.print(F("Reglages sondes"));
		lcd.setCursor(1,2); lcd.print(F("Reglages solaire"));
		lcd.setCursor(1,3); lcd.print(F("Reglages chauffage"));
		Pos=0;
	}


	if (Counter!=0)
	{
		lcd.setCursor(0,Pos); lcd.print(F(" "));
		Pos+=encoderCount();
		if (Pos>3) Pos=0;
		if (Pos<0) Pos=3;
		lcd.setCursor(0,Pos); lcd.print(F(">"));
	}
	if(btn.state(BTN_CLICK))
		switch (Pos)
		{
			case 0 : { menu.next(menu_setclock); break; }
			case 1 : { menu.next(menu_setsensors); break; }
			case 2 : { menu.next(menu_setsolar); break; }
			case 3 : { menu.next(menu_setheat); break; }
		}
				
	if(btn.state(BTN_LONGCLICK) || menu.elapsed(15E3))
		menu.next(menu_start);
}

void timeprintset()
{
	lcd.setCursor(6,1); 
	if(DateTime.hour<10) lcd << "0";
	lcd << DateTime.hour << ":";
	if(DateTime.minute<10) lcd << "0";
	lcd<<DateTime.minute<<":";
	if(DateTime.second<10) lcd << "0";
	lcd<<DateTime.second;
	
	lcd.setCursor(5,2);
	if(DateTime.day<10) lcd << "0";
	lcd<<DateTime.day<<"/";
	if(DateTime.month<10) lcd << "0";
	lcd<<DateTime.month<<"/"<<DateTime.year;
	
	switch (Pos)
	{
		case 0 : { 	
			lcd.setCursor(6,1); 
			RTC.set(DS1307_HR,(DateTime.hour+=encoderCount())); 
			break; }
		case 1 : { 
			lcd.setCursor(9,1); 
			RTC.set(DS1307_MIN,(DateTime.minute+=encoderCount()));
			break; }
		case 2 : { 
			lcd.setCursor(12,1); 
			RTC.set(DS1307_SEC,(DateTime.second+=encoderCount()));
			break; }
		case 3 : { 
			lcd.setCursor(5,2); 
			RTC.set(DS1307_DOM,(DateTime.day+=encoderCount())); 
			break; }
		case 4 : { 
			lcd.setCursor(8,2); 
			RTC.set(DS1307_MTH,(DateTime.month+=encoderCount())); 
			break; }	
		case 5 : { 
			lcd.setCursor(13,2); 
			RTC.set(DS1307_YR, ((DateTime.year+=encoderCount())-2000) );
			break; }
	}
}

void menu_setclock()
{
	if (menu.isFirstRun()) 
	{
		lcd.clear();
		lcd.print(F("Reglage de l'heure :"));
		Pos=0;
		encoderCount();
		lcd.blink();
		timeprintset();
	}
	
	//refresh display when no user input only each 4sec because faster redraw
	//takes too much time and then encoder is not responsive
	if (menu.periodic(4E3)) timeprintset();
	
	if(btn.state(BTN_LONGCLICK))
	{
		lcd.noBlink();
		menu.next(menu_param);
	}
	
	if(btn.state(BTN_CLICK))
	{
		Pos++;
		if (Pos>5) Pos=0;
		timeprintset();
	}
	
	//redraw twice to get correct value on screen, because value is changed after
	//redraw in timeprintset()
	if(Counter!=0) {timeprintset();timeprintset();}
}

void menu_setsensors()
{
	if (menu.isFirstRun()) lcd.clear();
	if(btn.state(BTN_LONGCLICK))
		menu.next(menu_param);	
}

void menu_setsolar()
{
	if (menu.isFirstRun()) lcd.clear();
	if(btn.state(BTN_LONGCLICK))
		menu.next(menu_param);	
}

void menu_setheat()
{
	if (menu.isFirstRun()) lcd.clear();
	if(btn.state(BTN_LONGCLICK))
		menu.next(menu_param);	
}
