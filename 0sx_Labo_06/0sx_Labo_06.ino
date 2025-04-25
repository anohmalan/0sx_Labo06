#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <LCD_I2C.h>
#include <HCSR04.h>
#include <AccelStepper.h>

#define MOTOR_INTERFACE_TYPE 4

#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define LED_RED 2
#define LED_BLUE 4

#define IN_1 53
#define IN_2 51
#define IN_3 49
#define IN_4 47

#define BUZZER 5

#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32

unsigned long lastToneTime = 0;
unsigned long lastLEDTime = 0;
unsigned long lastCycleTime = 0;

unsigned long lastDetectionTime = 0;

const int toneInterval = 10;     // temps entre chaque changement de fréquence

const int ledInterval = 200;     // temps entre chaque clignotement LED
const int cycleInterval = 2000;  // temps total pour une montée/descente de sirène

int toneFreq = 400;
bool goingUp = true;
bool ledState = false;

AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);

U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R0, CLK_PIN, DIN_PIN, CS_PIN,
  U8X8_PIN_NONE, U8X8_PIN_NONE);

// États de la porte
enum State { FERME,
             OUVERTURE,
             OUVERT,
             FERMETURE };
State etatPorte = FERME;

// Variables globales
long distance;
unsigned long previousTime = 0;
unsigned long dernierSerial = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastStartupDisplay = 0;
bool startupDisplayed = false;

long int closed = 10, opened = 170;
int distanceOpen = 30, distanceClose = 60;
int nullValue = 0;
long uneTour = 2038;

int distAlarme = 15;

unsigned long rateMatrix = 3000;

static const unsigned char limiteError[] U8X8_PROGMEM = {
  0b00111100,
  0b01000010,
  0b10100001,
  0b10010001,
  0b10001001,
  0b10000101,
  0b01000010,
  0b00111100
};

static const unsigned char commandOk[] U8X8_PROGMEM = {
  0b00000000,
  0b10000000,
  0b01000000,
  0b00100000,
  0b00010001,
  0b00001010,
  0b00000100,
  0b00000000
};

static const unsigned char commandInconnu[] U8X8_PROGMEM = {
  0b10000001,
  0b01000010,
  0b00100100,
  0b00011000,
  0b00011000,
  0b00100100,
  0b01000010,
  0b10000001
};

void setup() {
  u8g2.begin();
  Serial.begin(9600);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lcd.begin();
  lcd.backlight();

  myStepper.setMaxSpeed(255.0);      // Ajustement pour 2 secondes
  myStepper.setAcceleration(127.5);  // Ajustement pour 2 secondes
  myStepper.setSpeed(255);
  myStepper.setCurrentPosition(closed);

  lastStartupDisplay = millis();

  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
}

void updateStartupDisplay() {
  unsigned long rate = 2000;
  String da = "6334158     ";
  if (!startupDisplayed && millis() - lastStartupDisplay < rate) {
    lcd.setCursor(0, 0);
    lcd.print(da);
    lcd.setCursor(0, 1);
    lcd.print("Labo 4A           ");
  } else if (!startupDisplayed) {
    lcd.clear();
    startupDisplayed = true;
  }
}

void updateDistance() {
  unsigned long rate = 50;
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= rate) {
    distance = mesurerDistance();
    previousTime = currentTime;
  }
}

long mesurerDistance() {
  long duration;
  long distanceMesure = hc.dist();

  return distanceMesure;
}

void alarme() {
  unsigned long currentTime = millis();
  unsigned long stopDelay = 3000;


  if (distance <= distAlarme) {
    // Gérer la sirène
    if (currentTime - lastToneTime >= toneInterval) {
      tone(BUZZER, toneFreq);

      if (goingUp) {
        toneFreq += 25;
        if (toneFreq >= 1000) {
          goingUp = false;
        }
      } else {
        toneFreq -= 25;
        if (toneFreq <= 400) {
          goingUp = true;
        }
      }
      lastToneTime = currentTime;
    }

    // Gérer les LEDs
    if (currentTime - lastLEDTime >= ledInterval) {
      ledState = !ledState;
      digitalWrite(LED_RED, ledState ? HIGH : LOW);
      digitalWrite(LED_BLUE, ledState ? LOW : HIGH);
      lastLEDTime = currentTime;
    }
  } else {
    if (currentTime - lastDetectionTime >= stopDelay) {
      noTone(BUZZER);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, LOW);
      toneFreq = 400;
      goingUp = true;
      lastDetectionTime = currentTime;
    }
  }
}

void gererPorte() {
  switch (etatPorte) {
    case FERME:
      if (distance < distanceOpen) {
        etatPorte = OUVERTURE;
        myStepper.enableOutputs();
        myStepper.moveTo(opened);
      }
      break;
    case OUVERTURE:
      if (myStepper.distanceToGo() == nullValue) {
        etatPorte = OUVERT;
        myStepper.disableOutputs();
      }
      break;
    case OUVERT:
      if (distance > distanceClose) {
        etatPorte = FERMETURE;
        myStepper.enableOutputs();
        myStepper.moveTo(closed);
      }
      break;
    case FERMETURE:
      if (myStepper.distanceToGo() == nullValue) {
        etatPorte = FERME;
        myStepper.disableOutputs();
      }
      break;
  }
  myStepper.run();
}

void afficherLCD() {
  unsigned long rate = 500;
  long mapIndex = map(myStepper.distanceToGo(), nullValue, uneTour, closed, opened);
  if (millis() - lastLCDUpdate >= rate) {  // Mise à jour toutes les 500ms
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print(" cm  ");

    lcd.setCursor(0, 1);
    lcd.print("Porte: ");
    if (etatPorte == FERME) {
      lcd.print("Fermee  ");
    } else if (etatPorte == OUVERT) {
      lcd.print("Ouverte  ");
    } else {
      lcd.print(mapIndex);
      lcd.print(" deg ");
    }
    lastLCDUpdate = millis();
  }
}

void limiteErreur() {

  u8g2.drawXBMP(0, 0, 8, 8, limiteError);  // Affiche le 🚫
  u8g2.sendBuffer();
}

void commandeOk() {

  u8g2.drawXBMP(0, 0, 8, 8, commandOk);  // Affiche le ✔️
  u8g2.sendBuffer();
}

void commandeInconnu() {
  u8g2.drawXBMP(0, 0, 8, 8, commandInconnu);  // Affiche le ❌

  u8g2.sendBuffer();
}


void analyserCommande(const String& tampon, String& commande, String& arg1, String& arg2) {
  commande = "";
  arg1 = "";
  arg2 = "";

  int firstSep = tampon.indexOf(';');
  int secondSep = tampon.indexOf(';', firstSep + 1);

  if (firstSep == -1) {
    // Pas de point-virgule, c'est peut-être "stop" ou autre commande sans paramètre
    commande = tampon;
    return;
  }

  // Extraire la commande
  commande = tampon.substring(0, firstSep);

  // Extraire arg1
  if (secondSep != -1) {
    arg1 = tampon.substring(firstSep + 1, secondSep);
    arg2 = tampon.substring(secondSep + 1);
  } else {
    // Il y a une seule valeur après la commande
    arg1 = tampon.substring(firstSep + 1);
  }
}

void serialEvent() {
  String tampon = Serial.readStringUntil('\n');

  String commande;
  String arg1, arg2;

  analyserCommande(tampon, commande, arg1, arg2);

  if (commande == "gDist") {
    Serial.println("PC : " + tampon);
    Serial.print("Arduino : ");
    Serial.println(distance);
  }

  else if (commande == "cfg") {

    if (arg1 == "lim_inf" && arg2.toInt() < distanceClose && arg2.toInt() >= 0) {
      distanceOpen = arg2.toInt();
      Serial.println("PC : " + tampon);
      Serial.print("Arduino : Il configure la limite inférieure du moteur à " + arg2);
      Serial.println("cm");
      commandeOk();
    } else if (arg1 == "lim_sup" && arg2.toInt() > distanceOpen) {
      distanceClose = arg2.toInt();
      Serial.println("PC : " + tampon);
      Serial.print("Arduino : Il configure la limite supérieure du moteur à " + arg2);
      Serial.println("cm");
      commandeOk();
    } else if (arg1 == "alm") {
      distAlarme = arg2.toInt();
      Serial.println("PC : " + tampon);
      Serial.print("Arduino : Configure la distance de détection de l’alarme à " + arg2);
      Serial.println("cm");
      commandeOk();
    } else if (arg1 == "lim_inf" && arg2.toInt() >= distanceClose) {
      Serial.println("PC : " + tampon);
      Serial.println("Arduino : Erreur – Limite inférieure plus grande que limite supérieure");
      limiteErreur();     
      delay(rateMatrix);   
      u8g2.clearBuffer();  
      u8g2.sendBuffer();   
    } else {
      Serial.println("PC : " + tampon);
      limiteErreur();      
      delay(rateMatrix);  
      u8g2.clearBuffer();  
      u8g2.sendBuffer();   
    }
  } else {

    commandeInconnu();

    u8g2.clearBuffer();
    delay(rateMatrix);   
    u8g2.clearBuffer(); 
    u8g2.sendBuffer();   
  }
}

void loop() {
  updateStartupDisplay();
  updateDistance();
  gererPorte();
  afficherLCD();
  alarme();
}