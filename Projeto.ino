// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*
					.::. LCD Pin Function .::.
                    
                     |Display: LCD JHD 162A|
___________________________________________________________________
Pin 	- 		Function
____________________________________________________________________
1 		- 		GND
2 		- 		Vcc (+5V)
3 		- 		Vee (contrast voltage)
4 		- 		Rs (Register Select: 0=instruction register ; 1=data register)
5 		- 		R/W (read/write: 0=write mode; 1=read mode)
6 		- 		E (enable: 0=start to latch data to LCD character; 1=disable)
7 		- 		DB0 (databit 0; LSB)
8 		- 		DB1 (databit 1)
9 		- 		DB2 (databit 2)
10 		- 		DB3 (databit 3)
11 		- 		DB4 (databit 4)
12 		- 		DB5 (databit 5)
13 		- 		DB6 (databit 6)
14	 	- 		DB7 (databit 7; MSB)
15 		- 		BPL (backplane light: +5V or lower - optional)
16 		- 		GND (optional)
____________________________________________________________________
*/
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*					Periféricos Alternativos:
(i) Pitot: MPX 2050 DP 
[Datasheet: https://pdf1.alldatasheet.com/datasheet-pdf/view/273607/FREESCALE/MPX2050DP.html]

(ii) Altimeter: BPM 180
[Datasheet: https://pdf1.alldatasheet.com/datasheet-pdf/view/1132068/BOSCH/BMP180.html]


(iii) Inclinometer: SCA61T
[Datasheet: https://pdf1.alldatasheet.com/datasheet-pdf/view/168811/ETC2/SCA61T.html]
*/
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Begin.

//-------		BIBLIOTECAS:
#include <LiquidCrystal.h>
#include <Servo.h>

//-------		CONSTANTES UTILIZADAS:
#define pi 3.1415926535897932384626433832795
#define rho 1.225
#define msToknots 1.94384
#define delayStrobe 100
#define delayBeacon 150
#define meToft 3.28084
#define Cl 1.225
#define Weight 280
#define WingArea 6.25

/*	
	Criando um objeto da classe LiquidCrystal e 
	 inicializando com os pinos da interface:	
*/
const int rs = 9, en = 8, d4 = 5, d5 = 12, d6 = 11, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//-------		DEFINIÇÃO DOS PINOS:
const int PinPitot = A0; //Pino analógico Pitot.
const int PinDensity = A1; //Pino analógico Densidade
const int Nav = 2; //Pino digital navigation lights.
const int Beacon = 3; //Pino digital beacon light.
const int Strobe = 4; //Pino digital strobe lights.
const int PinVerifyNav = 7; //Pino digital verificação de acionamento Nav
bool StateNav = HIGH; //Estado de excitação Nav
float VerifyNav;
const int PinInclinometer = A3; //Pino digital do sensor de inclinação
const int PinLandingGear = 6;
  
//-------		VARIÁVEIS PITOT:
float Airspeed; //velocidade do fluxo (m/s)
float AirspeedKnots; //velocidade do fluxo (knots)
float PitotDiffPressureRange = 50e+03; //Máximo valor de leitura do sensor (Pa)
float ConversionFactorPitot = ((40.00/1000.00)/5.00); //Fator de conversão Pitot: 0 ~ 40mv(sensor) em 0 ~ 5V(Arduino)
float DigitalValuePitot;
float DigitalValuePitotReal;
float DigitalToDiffPressure;
float DigitalValueDensity;
float DigitalToDensity;
float DeltaDens = ( (9.2517e-04) - rho); //diferença de densidade entre 9e+03m e 0m (considerando valores ISA numa faixa de variação linear)
float CoefDensity = DeltaDens/1023; //coeficiente da curva Dens.Vs.bits

//-------		VARIÁVEIS ALTÍMETRO:
float Po = 1013.25e+02; //pressão atmosférica nível do mar (Pa)
float Pdin; //pressão dinâmica
float k2 = 1/5.255; //termo expoente eq. alt.
float Pe; //pressão estática
float kp; //razão (Pe/Po) 
float alt; //altitude (m)
float altreal; //correção de altitude (m) sem usar equação do sensor
float altrealft;

//-------		VARIÁVEIS ILUMINAÇÃO:
bool NavCond = HIGH;
int BeaconFading; //estado de brilho para o efeito "fading"
char *StateLights[] = {"LIGHTS ON", "LIGHTS OFF"}; //estados ON/OFF para Nav. Lights

//-------		VARIÁVEIS ÂNGULO DE ATAQUE: 
float offset = 2.5; //offset (V)
float sensitivity = 4.00; //sensibilidade (V/g)
float Vmin = 0.00; //voltagem mínima 
float Vmax = 5.00; //voltagem máxima
float AlphaMin = (180.00*asin((Vmin-offset)/sensitivity))/pi; //valor de ângulo máximo medido no sensor 
float AlphaMax = (180.00*asin((Vmax-offset)/sensitivity))/pi; //valor de ângulo mínimo medido no sensor
float CoeffAlpha = (AlphaMax - AlphaMin)/1023; //coeficiente da reta Alpha Vs. Dados transmitidos (bits)
float DigitalValueAlpha;
float DigitalToAlpha;

//------- 		VARIÁVEIS TREM DE POUSO:	
Servo servo1;
float Vstall;
float VstallKnots;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup()
{
 
Serial.begin(9600);

//-------		INICIALIZANDO O LCD:
lcd.begin(16, 2); // informa o tamanho de 16 colunas e 2 linhas
lcd.setCursor(0, 0);
lcd.print("Avionics Master");
lcd.setCursor(6, 1);
lcd.print("ON");
delay(800);
lcd.clear();
  
//-------		SETUP ILUMINAÇÃO:
pinMode(Nav, OUTPUT);
pinMode(Beacon, OUTPUT); 
pinMode(Strobe, OUTPUT);  
pinMode(VerifyNav, INPUT);
  
//-------		SETUP TREM DE POUSO:
servo1.attach(PinLandingGear);
  
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop()
{
lcd.clear();
//-------		LOOP ILUMINAÇÃO:  
		//Nav
digitalWrite(Nav, StateNav); 
VerifyNav=digitalRead(PinVerifyNav);
  if(VerifyNav == true)
  {
    lcd.setCursor(0, 0);
	lcd.print(StateLights[0]);
  }
  else if(VerifyNav == false)
  {
    lcd.setCursor(0, 0);
	lcd.print(StateLights[1]);
  }

		//Beacon  
for (BeaconFading = 0; BeaconFading<=255; BeaconFading +=50) //BeaconFading = BeaconFading + 5
{
analogWrite(Beacon, BeaconFading);
delay(delayBeacon);
}

for (BeaconFading = 255; BeaconFading>=0; BeaconFading -=50) //BeaconFading = BeaconFading - 5
{
analogWrite(Beacon, BeaconFading);
delay(delayBeacon);
}

		//Strobe  
digitalWrite(Strobe, HIGH);
delay(delayStrobe);  
digitalWrite(Strobe, LOW); 
delay(delayStrobe);
digitalWrite(Strobe, HIGH);
delay(delayStrobe);
digitalWrite(Strobe, LOW);  
delay(delayStrobe);
digitalWrite(Strobe, HIGH);
delay(delayStrobe);
digitalWrite(Strobe, LOW);  
delay(300);  
 
//-------		LOOP PITOT:
DigitalValuePitot = analogRead(PinPitot); //Retorna os valores digitais da leitura analógica do potenciômetro
DigitalValuePitotReal = ConversionFactorPitot * DigitalValuePitot;
DigitalToDiffPressure = (DigitalValuePitotReal*PitotDiffPressureRange)/(ConversionFactorPitot*1023); //faz a leitura do diferencial de pressão (Pa) de acordo com a faixa digital gerada no sensor
DigitalValueDensity = analogRead(PinDensity); //Retorna os valores digitais da leitura analógica do potenciômetro
DigitalToDensity = (CoefDensity*DigitalValueDensity)+rho; //Densidade [kg/m³]
Airspeed = sqrt((2*DigitalToDiffPressure)/DigitalToDensity); //velocidade de fluxo (m/s)  
AirspeedKnots = msToknots*Airspeed;

//-------		LOOP ALTÍMETRO:  
Pdin = 0.5*DigitalToDensity*pow(Airspeed,2); //pressão dinâmica (Pa)
Pe = DigitalToDiffPressure - Pdin; //
kp = abs((Pe/Po));
alt = 44330 * (1-pow(kp,k2));
altreal = (DigitalToDensity - rho)*(9000/DeltaDens); //correção de altitude (m) sem usar equação do sensor   
altrealft = (altreal*meToft); //conversão da altura (m)->(ft)

//-------		LOOP ÂNGULO DE ATAQUE:  
DigitalValueAlpha = analogRead(PinInclinometer); //leitura digital do sensor de inclinação 
DigitalToAlpha = (CoeffAlpha*DigitalValueAlpha)+AlphaMin; //conversão dos valores digitais em ângulo de ataque
  
//-------		LOOP TREM DE POUSO:
Vstall = sqrt((2*Weight)/(WingArea*DigitalToDensity*Cl));
VstallKnots = msToknots*Vstall;   
/*
Acionamento automático dada situação de aproximação de pouso:  
V <= 15% acima da velocidade de Stall   
*/  
if(altrealft<=500 && AirspeedKnots<=(1.15*VstallKnots))  
{
servo1.write(90);
lcd.setCursor(0,1);  //posiciona o cursor na coluna 0 linha 1 do LCD.
lcd.print("LG: Extended");
}    
  
else if(altrealft>500 && AirspeedKnots>(1.15*VstallKnots))
{
servo1.write(0);
lcd.setCursor(0,1);  //posiciona o cursor na coluna 0 linha 1 do LCD.
lcd.print("LG: Retracted");
}    
  
Serial.print("\t");  
Serial.print("Airspeed (knots):");
Serial.println(AirspeedKnots,4);
Serial.print("\t");   
Serial.print("Altitude (ft):"); 
Serial.println(altrealft,4); 
Serial.print("\t");   
Serial.print("Angle of Attack (dg):"); 
Serial.println(DigitalToAlpha,4); 

delay(50);   
}  
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//End.