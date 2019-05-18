// var. 2 cu tastatura si display

// include the library code:
#include <LiquidCrystal.h>

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

int lcd_key     = 0;
int adc_key_in  = 0;

// read the buttons
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);
  // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  if (adc_key_in < 50)  return btnRIGHT;
  if (adc_key_in < 195) return btnUP;
  if (adc_key_in < 380) return btnDOWN;
  if (adc_key_in < 555) return btnLEFT;
  if (adc_key_in < 790) return btnSELECT;
  return btnNONE;   // when all other fail return this
}

// achizitie date
int AO_Pin = 3;     // PWM pin
int AI_Pin = A1;    // select the input pin for the analog input

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
/////////////////////////////////
// Proiect - begin
class Interval
{
protected:
  float minim,maxim;
public:
  void setLimite(float,float);
  void setMin(float);
  void setMax(float);
  float getMin();
  float getMax();
};

void Interval :: setLimite(float imin,float imax)
{
  minim = imin;
  maxim = imax;
}

void Interval :: setMin(float imin)
{
  minim = imin;
}

void Interval :: setMax(float imax)
{
  maxim = imax;
}

float Interval :: getMin()
{
  return minim;
}

float Interval :: getMax()
{
  return maxim;
}

class algPID
{
private:
  float Kr,Ti,Td,Te;
  float q0,q1,q2;
  float rk,yk;
  float uk,uk_1;
  float ek,ek_1,ek_2;
public:
  void setParametrii(float,float,float,float);
  void setParametriiq(float,float,float);
  void setParametriiek(float,float,float);
  void setParametriiuk(float,float);
  void setrk(float);
  void setyk(float);
  void setek(float);
  void setek_1(float);
  void setek_2(float);
  void setuk(float);
  void setuk_1(float);
  float getyk();
  float getrk();
  float getek();
  float getek_1();
  float getek_2();
  float getuk();
  float getuk_1();
  void calcq();
  void calcuk();
  void actualizareMemorie();
};

void algPID :: setParametrii(float iKr, float iTi, float iTd, float iTe)
{
  Kr = iKr;
  Ti = iTi;
  Td = iTd;
  Te = iTe;
}

void algPID :: setParametriiq(float iq0, float iq1, float iq2)
{
  q0 = iq0;
  q1 = iq1;
  q2 = iq2;
}

void algPID :: setParametriiek(float iek,float iek_1, float iek_2)
{
  ek = iek;
  ek_1 = iek_1;
  ek_2 = iek_2;
}

void algPID :: setParametriiuk(float iuk, float iuk_1)
{
  uk = iuk;
  uk_1 = iuk_1;
}

void algPID :: setrk(float irk)
{
  rk = irk;
}

void algPID :: setyk(float iyk)
{
  yk = iyk;
}

void algPID :: setek(float iek)
{
  ek = iek;
}

void algPID :: setek_1(float iek_1)
{
  ek_1 = iek_1;
}

void algPID :: setek_2(float iek_2)
{
  ek_2 = iek_2;
}

void algPID :: setuk(float iuk)
{
  uk = iuk;
}

void algPID :: setuk_1(float iuk_1)
{
  uk_1 = iuk_1;
}

float algPID :: getrk()
{
  return rk;
}

float algPID :: getyk()
{
  return yk;
}

float algPID :: getek()
{
  return ek;
}

float algPID :: getek_1()
{
  return ek_1;
}

float algPID :: getek_2()
{
  return ek_2;
}

float algPID :: getuk()
{
  return uk;
}

float algPID :: getuk_1()
{
  return uk_1;
}

void algPID :: calcq()
{
  q0 = Kr*(1 + Te/Ti + Td/Te);
  q1 = -Kr*(1 + 2*Td/Te);
  q2 = Kr*Td/Te;
}

void algPID :: calcuk()
{
  uk = uk_1 + q0*ek + q1*ek_1 + q2*ek_2;
}

void algPID :: actualizareMemorie()
{
  uk_1 = uk;
  ek_2 = ek_1;
  ek_1 = ek;
}

class Afisare
{
public:
  void initLCD();
  void afisareLCD();
};

void Afisare :: initLCD()
{
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Start PID Alg");
  lcd.setCursor(0,1);
  lcd.print("Init. OK");
}

class Comunicatie
{
public:
  void setPIN();
  void setComspeed(float);
  void printSerial();
};

void Comunicatie :: setPIN()
{
  pinMode(AO_Pin, OUTPUT);
  pinMode(AI_Pin, INPUT);
}

void Comunicatie :: setComspeed(float ispeed)
{
  Serial.begin(ispeed);
}

class Regulator : public algPID,public Afisare,public Comunicatie
{
private:
  bool regim;
  Interval referinta;
  Interval comanda;
public:
  void setRegim(bool);
  void setLimiteref(float,float);
  void setLimitecom(float,float);
  void incrementeazaReferinta();
  void decrementeazaReferinta();
  void convertMasura();
  void convertComanda();
  float getrefMin();
  float getrefMax();
  float getcomMin();
  float getcomMax();
};

void Regulator :: setRegim(bool iRegim)
{
  regim = iRegim;
}

void Regulator :: setLimiteref(float imin,float imax)
{
  referinta.setLimite(imin,imax);
}

void Regulator :: setLimitecom(float imin, float imax)
{
  comanda.setLimite(imin,imax);
}

void Regulator :: incrementeazaReferinta()
{
  setrk(getrk() + 0.5);
}

void Regulator :: decrementeazaReferinta()
{
  setrk(getrk() - 0.5);
}

void Regulator :: convertMasura()
{
  setyk(getyk() * (5.0 / 1023.0));
}

void Regulator :: convertComanda()
{
  setuk(getuk() *(255.0/5.0));
}

float Regulator :: getrefMin()
{
  return referinta.getMin();
}

float Regulator :: getrefMax()
{
  return referinta.getMax();
}

float Regulator :: getcomMin()
{
  return comanda.getMin();
}

float Regulator :: getcomMax()
{
  return comanda.getMax();
}

Regulator LPID;

void Comunicatie :: printSerial()
{
  Serial.print("rk = ");
  Serial.print(LPID.getrk()*20.0);
  Serial.print("  yk = ");
  Serial.print(LPID.getyk()*20.0);
  Serial.print("  uk = ");
  Serial.println(LPID.getuk()*20.0);
}

void Afisare :: afisareLCD()
{
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Ref: Mas: Com:");
  lcd.setCursor(0,1);
  lcd.print(LPID.getrk());
  lcd.setCursor(5,1);
  lcd.print(LPID.getyk());
  lcd.setCursor(10,1);
  lcd.print(LPID.getuk());
}

    
// Proiect - end
/////////////////////////////////

  
void setup() {
  
  LPID.initLCD();
  LPID.setPIN();
  LPID.setRegim(true);
  LPID.setComspeed(9600);  // initialize serial communication at 9600 bits per second:
  LPID.setParametrii(1.0,1.5,0,1.0);
  LPID.setParametriiq(0.0,0.0,0.0);
  LPID.setParametriiek(0.0,0.0,0.0);
  LPID.setParametriiuk(0.0,0.0);
  LPID.setrk(2.5);
  LPID.setLimiteref(0.0,5.0);
  LPID.setLimitecom(0.0,5.0);
  LPID.calcq();
  delay(1000);
}

void loop() {
  // citire masura
  // read the value from the AI:
  int int_AI = analogRead(AI_Pin);
  LPID.setyk(int_AI);
  LPID.convertMasura(); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  
  // citire referinta
  lcd_key = read_LCD_buttons(); // read the buttons
  
  switch (lcd_key) // depending on which button was pushed, we perform an action
  {
    case btnUP:
    {
      LPID.incrementeazaReferinta();
      if(LPID.getrk() > LPID.getrefMax())
        LPID.setrk(5.0);
      break;
    }
    case btnDOWN:
    {
      LPID.decrementeazaReferinta();
      if(LPID.getrk() < LPID.getrefMin())
        LPID.setrk(0.0);
      break;
    }
  }
  
  // calcul comanda - facuta de algoritmul de reglare ce va fi dezvoltat
  LPID.setek(LPID.getrk() - LPID.getyk());
  LPID.calcuk();
  
  if(LPID.getuk() > LPID.getcomMax())
    LPID.setuk(5.0);
  if(LPID.getuk() < LPID.getcomMin())
    LPID.setuk(0.0);
    
  LPID.actualizareMemorie(); // pregatire pas urmator
  LPID.afisareLCD();
  LPID.printSerial();
  LPID.convertComanda();   // aplicare comanda
  analogWrite(AO_Pin,LPID.getuk());
  delay(1000);

  }
