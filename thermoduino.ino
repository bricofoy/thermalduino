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
#include <btn.h>
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
bool TForce[SENSOR_NBR];
 
#define TEMPERATURE_RESOLUTION 9 //0,5°C sensor acccuracy.

#define PIN_ONE_WIRE_BUS A3

#define DELAY_MENU_BACK	120E3	//2 minutes

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
byte OutputsForce=0;
#define BIT_R1	1
#define BIT_R2	2
#define BIT_R3	4
#define BIT_R4	8
#define BIT_R5	16
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
  outputsWrite();
  
  sensors.begin(); // Start up the dallas library
  sensors.setWaitForConversion(false); //don't wait !
  
  loadSensorsAddresses(); //fill the addresses array with the values in EEPROM
  setSensorsResolution();
  
  menu.next(menu_start); 
  gettemp.next(gettemp_request);
}

void loop(void) 
{
	Outputs &= OutputsForce; //erase all output bits exept forced ones
	RTC.get(&DateTime,true);
	btn.update(!digitalRead(PIN_BTN)); // ! because btn switch to gnd
	encoderRead();
	gettemp.run();
	menu.run();
	solar.run();
	heat.run();
	boiler.run();
	outputsWrite();
}

void setOutput(byte pin, bool state)
{
	if(!(OutputsForce&pin))
		if(state) Outputs |= pin; //set the <pin> bit
		else Outputs &= ~pin; //unset the <pin> bit		
}


void outputsWrite()
{
	digitalWrite(PIN_R1,!(Outputs&BIT_R1));
	digitalWrite(PIN_R2,!(Outputs&BIT_R2));
	digitalWrite(PIN_R3,!(Outputs&BIT_R3));
	digitalWrite(PIN_R4,!(Outputs&BIT_R4));
	digitalWrite(PIN_R5,!(Outputs&BIT_R5));
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

void printT(byte i)
{
	if(TForce[i]) lcd<<"F";
	else 
		if(T[i]==-127) 
		{
			lcd<<" --.-";
			return; 
		}
	if(T[i]>=0) 
	{
		if(!TForce[i]&&T[i]<100) lcd<<F(" ");
		if(T[i]<10) lcd<<F(" ");
	}
	else if(T[i]>-10) lcd<<F(" "); 
	
	lcd.print(T[i],1);
}

void printR(byte bit)
{
	if(OutputsForce&bit) lcd<<F("F");
	else lcd<<F(" ");
	lcd<<(Outputs&bit);
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
	{
		if(!TForce[i])
			T[i] = sensors.getTempC(sensorAddress[i]);
	}
	else gettemp.next(gettemp_request);
}


///////////menu state machine////////////
char Pos;

void menu_start()
{
	if (menu.isFirstRun())
	{
		lcd.clear();
		lcd.print(F("Ballon"));
		lcd.setCursor(0,1); lcd.print(F("Capteur"));
		lcd.setCursor(0,2); lcd.print(F("Int       Ext"));
		lcd.setCursor(0,3); lcd.print(F("Chauffage"));
	}
 	
	if(menu.periodic(1500)) 
	{
		lcd.setCursor(7,0); printT(1);
		lcd.setCursor(14,0); printT(3);

		lcd.setCursor(7,1); printT(0);
		lcd.setCursor(15,1); if(OutputsForce&BIT_R1) lcd<<F("F"); lcd<<Pwm1<<F("%");
		
		lcd.setCursor(3,2); printT(7);
		lcd.setCursor(13,2); printT(6);
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
		lcd.setCursor(0,2); lcd.print(F("T4:"));
		lcd.setCursor(0,3); lcd.print(F("R1:"));
	}	
	
	if(menu.periodic(1500))
	{
		lcd.setCursor(3,0); printT(0); lcd.setCursor(13,0); printT(9);
		lcd.setCursor(3,1); printT(1); lcd.setCursor(13,1); printT(2); 
		lcd.setCursor(3,2); printT(3);
		lcd.setCursor(3,3); printR(BIT_R1); lcd<<" "<<Pwm1<<"%"; 
	}
	
	if(Counter<0 || menu.elapsed(DELAY_MENU_BACK))
		menu.next(menu_start);
	if(encoderCount()>0)
		menu.next(menu_start3);	
	if(btn.state(BTN_LONGCLICK)) 
		menu.next(menu_forceT);

}

void menu_start3()
{
	if (menu.isFirstRun())
	{
		lcd.clear();
		lcd.print(F("T5:       T6:"));
		lcd.setCursor(0,1); lcd.print(F("T7:       T8:"));
		lcd.setCursor(0,2); lcd.print(F("R2:   R3:   R4:"));
		lcd.setCursor(0,3); lcd.print(F("T9:         R5:"));
	}
	
	if(menu.periodic(1500))
	{
		lcd.setCursor(3,0); printT(4); lcd.setCursor(13,0); printT(5);
		lcd.setCursor(3,1); printT(6); lcd.setCursor(13,1); printT(7);
		lcd.setCursor(3,2); printR(BIT_R2); 
		lcd.setCursor(9,2); printR(BIT_R3); 
		lcd.setCursor(15,2); printR(BIT_R4);
		lcd.setCursor(3,3); printT(8); 
		lcd.setCursor(15,3); printR(BIT_R5);
	}
	
	if(encoderCount()<0)
		menu.next(menu_start2);	

	if(menu.elapsed(DELAY_MENU_BACK))
		menu.next(menu_start);	
	if(btn.state(BTN_LONGCLICK)) 
		menu.next(menu_forceoutputs);
}

	
void menu_param()
{	
	if (menu.isFirstRun()) 
	{
		lcd.clear();
		lcd<<(char)126<<F("Reglages horloge");
		lcd.setCursor(1,1); lcd<<F("Reglages sondes");
		lcd.setCursor(1,2); lcd<<F("Reglages solaire");
		lcd.setCursor(1,3); lcd<<F("Reglages chauffage");
		Pos=0;
	}


	if (Counter!=0)
	{
		lcd.setCursor(0,Pos); lcd.print(F(" "));
		Pos+=encoderCount();
		if (Pos>3) Pos=0;
		if (Pos<0) Pos=3;
		lcd.setCursor(0,Pos); lcd<<(char)126;
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

void outset(byte bit, char count)
{
	if(count==0) return;
	
	char mode;
	mode=(((OutputsForce&bit)&&1)<<1)|((Outputs&bit)&&1);//0=auto,0 1=auto,1 2=forced,0 3=forced,1
	
	if(mode==0) mode=1;
	mode+=count;
	if(mode>3) mode=1;
	if(mode<1) mode=3;

	switch (mode)
	{
		case 1 : { OutputsForce &= ~bit; break;}
		case 2 : { OutputsForce |= bit; Outputs &= ~bit; break;}
		case 3 : { OutputsForce |= bit; Outputs |= bit; break;}
	}
}

void outprintset()
{
	lcd.setCursor(3,1); 
	if(OutputsForce&BIT_R1) {lcd<<F("    ");lcd.setCursor(3,1); lcd<<(Outputs&BIT_R1);}
	else lcd << F("AUTO");
	lcd.setCursor(8,1); lcd<<Pwm1<<F("%  ");
	lcd.setCursor(3,2); 
	if(OutputsForce&BIT_R2) {lcd<<F("    ");lcd.setCursor(3,2); lcd<<((Outputs&BIT_R2)&&1);}
	else lcd << F("AUTO");
	lcd.setCursor(3,3); 
	if(OutputsForce&BIT_R3) {lcd<<F("    ");lcd.setCursor(3,3); lcd<<((Outputs&BIT_R3)&&1);}
	else lcd << F("AUTO");
	lcd.setCursor(15,2); 
	if(OutputsForce&BIT_R4) {lcd<<F("    ");lcd.setCursor(15,2); lcd<<((Outputs&BIT_R4)&&1);}
	else lcd << F("AUTO");
	lcd.setCursor(15,3); 
	if(OutputsForce&BIT_R5) {lcd<<F("    ");lcd.setCursor(15,3); lcd<<((Outputs&BIT_R5)&&1);}
	else lcd << F("AUTO");
	
	switch (Pos)
	{
		case 0 : { lcd.setCursor(3,1); outset(BIT_R1, encoderCount()); break; }
		case 1 : { 
			lcd.setCursor(8,1); 
			if(OutputsForce&BIT_R1) 
			{
				Pwm1+=(encoderCount()*5);
				if(Pwm1<0) Pwm1=0;
				if(Pwm1>100) Pwm1=100;
			}
			else encoderCount();
			break; }
		case 2 : { lcd.setCursor(3,2); outset(BIT_R2, encoderCount()); break; }
		case 3 : { lcd.setCursor(3,3); outset(BIT_R3, encoderCount()); break; }
		case 4 : { lcd.setCursor(15,2); outset(BIT_R4, encoderCount()); break; }
		case 5 : { lcd.setCursor(15,3); outset(BIT_R5, encoderCount()); break; }
	}
}
void menu_forceoutputs()
{
 	if (menu.isFirstRun()) 
	{
		lcd.clear();
		Pos=0;
		encoderCount();
		lcd<<F("Forcage relais");
		lcd.setCursor(0,1); lcd<<F("R1:");
		lcd.setCursor(0,2); lcd<<F("R2:         R4:");
		lcd.setCursor(0,3); lcd<<F("R3:         R5:");
		lcd.blink();
		outprintset();
	}
	
	//refresh display when no user input only each 4sec because faster redraw
	//takes too much time and then encoder is not responsive
	//if (menu.periodic(4E3)) outprintset();
	
	if(btn.state(BTN_LONGCLICK))
	{
		lcd.noBlink();
		menu.next(menu_start3);
	}
	
	if(btn.state(BTN_CLICK))
	{
		Pos++;
		if (Pos>5) Pos=0;
		outprintset();
	}

	//redraw twice to get correct value on screen, because value is changed after
	//redraw in timeprintset()
	if(Counter!=0) {outprintset();outprintset();} 
}

void Tprintset(bool dblclick=false)
{
	lcd.clear();
	lcd<<F("Forcage temp.(")<<(char)-33<<F("C)1/2"); //-33 is ° on this lcd

	if(dblclick) TForce[Pos] ^= 1; //toggle the first bit
	
	if(TForce[Pos]) T[Pos]+=(encoderCount()*0.5);
	else encoderCount();
	
	for(byte i=0;i<6;i++)
	{
		if(i<3) lcd.setCursor(0,i+1); 
		else lcd.setCursor(10,i-2);
		lcd<<F("T")<<i+1<<F(":"); 
		if(!TForce[i]) lcd<<F("AUTO "); 
		else printT(i);
	}
	
	if(Pos<3) lcd.setCursor(3,Pos+1);
	else lcd.setCursor(13,Pos-2);

}
void menu_forceT()
{
 	if (menu.isFirstRun()) 
	{
		Pos=0;
		encoderCount();
		lcd.blink();
		Tprintset();
	}
	
	if(btn.state(BTN_LONGCLICK))
	{
		lcd.noBlink();
		menu.next(menu_start3);
	}
	
	if(btn.state(BTN_CLICK))
	{
		Pos++;
		if (Pos>5) Pos=0;
		Tprintset();
	}
	
	if(btn.state(BTN_DOUBLECLICK)) Tprintset(true);

	if(Counter!=0) Tprintset();
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
	static char truc;
	if (menu.isFirstRun()) {
		lcd.clear(); lcd.blink();
	}
	
	if(Counter!=0) 
	{
		truc+=encoderCount();
		lcd<<_DEC(truc)<<" "<<truc;
	}
	
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
