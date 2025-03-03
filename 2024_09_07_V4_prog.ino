/*
  # moteur 1
  1. et 2 position (8bitsX2=16bits) +
  3. vitesse du mouvement
  # moteur 2
  4. et 5 position (16bits)
  6. vitesse
  7. et 8 offset moteur 1 (16bits)
  9. et 10 offset moteur 2 (16bits)
11. au dessus de 200 lance la calibration du potentiomètre du moteur 1
12. au dessus de 200 lance la calibration du potentiomètre du moteur 2
  */
//librairies
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <bitset>
#include <string>
#include <Arduino.h>
#include "DmxInput.h"
DmxInput dmxInput;

//define value
#define MODULE 2
#define NB_CHANNELS 255
#define COLOR_ORDER GRB

#define ACCELERATION1 50
#define DECELERATION1 50
#define ACCELERATION2 50
#define DECELERATION2 50  //+= plus d’amorti
#define MOYENNEVITESSE1 20
#define MOYENNEVITESSE2 20
#define MOYENNEPOT1 15
#define MOYENNEPOT2 55

#define multipleDifference1 3  // accélération
#define multipleDifference2 4.5
#define Multoffset1 int(17 / multipleDifference1)  //multiple de l’offset (gère l’amplitude disponible)
#define Multoffset2 int(17 / multipleDifference2)
#define precisionM1 1.4  //précision avant laquelle les moteurs s’arrêtent
#define precisionM2 1.2  //1.5
#define POINTFIXETIMING 100
#define TOLERANCEPOINTFIXE1 90  //800 puis 90 valeur X2 à partir de quand sa bouge (en fonction de l’entrée DMX-valeur du poto)
#define TOLERANCEPOINTFIXE2 150


 //2200 puis 150
#define TOLERANCEPOINTFIXE1REDUCETIMING 400
#define TOLERANCEPOINTFIXE2REDUCETIMING 50

#define CALIBTIME 40000  //milliseconde
#define VITESSECALIB1 65536
#define VITESSECALIB2 29000

#define VITESSEMIN1 28500
#define VITESSEMIN2 20000

#define VITESSEMAX1 65536  // la vitessemax1 est égale à VITESSEMAX2+VITESSEMIN1
#define VITESSEMAX2 9500   // la vitessemax2 est égale à VITESSEMAX2+VITESSEMIN2

#define BITS_DMX 16     // résolution d'informations du signal DMX (1 ou 2 channels = 16 bits)
#define BITS_ANALOG 12  // résolution max du lecteur du potentiomettre moteur

#define FREQUENCE 35000  //fréquence de commutation du driver moteur (jusqu’à 20 000 HZ c’est audible)

// adressage DMX
  #define DMXpositionM1a 1  //(8bitsx2=16bits)
  #define DMXpositionM1b 2  //bis
  #define DMXvitesseM1 3    // 8 bits
  #define DMXpositionM2a 4  //(8bitsx2=16bits)
  #define DMXpositionM2b 5  //bis
  #define DMXvitesseM2 6    // 8 bits
  #define DMXoffsetM1a 7    //(8bitsx2=16bits)
  #define DMXoffsetM1b 8    //bis
  #define DMXoffsetM2a 9    //(8bitsx2=16bits)
  #define DMXoffsetM2b 10   //bis
  #define lanceCalibM1 11   //À 255 lance la calibration du moteur 1
  #define lanceCalibM2 12   //À 255 lance la calibration du moteur 2
// define paramètre rapide
  #define affichageMoniteurSerie 3  //(1: moteur 1 ; 2: moteur 2 ; 3 moteur 1 et 2; 4 DMX)
  #define tPause 140                // timing MAX avant reset des secondes

  #define nbrEchantillion1 75
  #define nbrEchantillion2 200
  #define tempsEchantillion 0.2

// adressage du pinout
int pot1 = 27;  // adressage de potentiometre de position moteur 1 cad le moteur latéral (gauche-droit)
int pot2 = 28;  // adressage de potentiometre de position moteur 2 cad le moteur longitudinal (devant-derrière)

//adressage RSB moteur 1
int pwm_out_1_R = 15;  // signal vitesse moteur longitudinal analogique vers l’extérieur du verrin
int pwm_out_1_L = 14;  // signal vitesse moteur longitudinal analogique vers l’intérieur du verrin
int dir_out_1_R = 12;  // activation de la direction du moteur latéral vers l’extérieur du verrin
int enable_1_R = 11;   // activation du moteur longitudinal vers l’extérieur
int enable_1_L = 10;   // activation du moteur longitudinal vers l’extérieur //
int dir_out_1_L = 9;   // activation de la direction du moteur vers l’extérieur

//adressage RSB moteur 2
int pwm_out_2_R = 16;  // signal vitesse moteur latéral analogique vers l’extérieur du verrin
int pwm_out_2_L = 17;  // signal vitesse moteur latéral analogique vers l’intérieur du verrin
int dir_out_2_R = 19;  // activation de la direction du moteur latéral vers l’extérieur du verrin
int enable_2_R = 20;   // activation du moteur latéral vers l’extérieur
int enable_2_L = 21;   // activation du moteur latéral vers l’extérieur
int dir_out_2_L = 22;  // activation de la direction du moteur vers l’extérieur
                       //int button = 25;

// adressage pin rsb pour l'adressage DMX
int canalDMX = 0;  //canal dmx
int dmx1 = 2;
int dmx2 = 3;
int dmx3 = 4;
int dmx4 = 5;
int dmx5 = 6;
int dmx6 = 7;
int dmx7 = 8;
int dmx8 = 13;

//variables
  int pwm1m[MOYENNEVITESSE1];
  int pwm2m[MOYENNEVITESSE2];
  int pwm1 = 0;
  int pre1pwm1 = 0;
  int pre2pwm1 = 0;
  int pwm2 = 0;
  int pre1pwm2 = 0;
  int pre2pwm2 = 0;
  int stay1 = -POINTFIXETIMING - 1;  // temporisation avant désactivation moteur si le moteur à atteint un point fixe
  int stay2 = -POINTFIXETIMING - 1;  // temporisation avant désactivation moteur si le moteur à atteint un point fixe
  int pwmIn1 = -1;                   //valeur signal vitesse moteur sans temporisation ni accélération
  int pwmOut1 = -1;                  //entrée signal vitesse moteur avec temporisation et accélération
  int pwmIn2 = -1;                   //valeur signal vitesse moteur sans temporisation ni accélération
  int pwmOut2 = -1;                  //entrée signal vitesse moteur avec temporisation et accélération
  int cLoop = 0;                     //compte loop
  int s = 0;                         //’presque seconde’
  bool pot1HS = 0;                   //état de fonctionnement du potentiometre 2
  bool pot2HS = 0;                   //état de fonctionnement du potentiometre 2
  int offset1 = -1;                  //valeur 16bits de l’offset du moteur 1 dicté par le DMX
  int offset2 = -1;                  //valeur 16bits de l’offset du moteur 2 dicté par le DMX
  int bits1;                         //variable temporaire pour concaténation des deux DMX 8bits du moteur 1
  int bits2;                         //variable temporaire pour concaténation des deux DMX 8bits du moteur 2
  bool pot1Inv = 0;                  //état d’invertion du potentiomètre 1 (+ sur le - et - sur le + : Vrai ou Faux)
  bool pot2Inv = 0;                  //état d’invertion du potentiomètre 1 (+ sur le - et - sur le + : Vrai ou Faux)
  int potR1 = 0;                     // valeur initiale du potentiometre moteur 1 cad le moteur latéral (gauche-droit)
  int pot1m[MOYENNEPOT1];
  int pot2m[MOYENNEPOT2];
  int moins1prePot1 = 0;
  int potR2 = 0;  // valeur initiale du potentiometre moteur 2 cad le moteur longitudinal (devant-derrière)
  int moins1prePot2 = 0;
  int vitesseMoteur1 = 0;
  int vitesseMoteur2 = 0;
  int tempsPositionFixe1 = 0;
  int tempsPositionFixe2 = 0;


// calibrations
// structure pour le stockage des variables de calibration
struct vecteurMaxMin2axes {
  int potMin1;
  int potMax1;
  int potMin2;
  int potMax2;
};
vecteurMaxMin2axes calib1;  // déclaration de la variable matrice de deux valeurs qui stocke celle de la calibration1()
vecteurMaxMin2axes calib2;  // déclaration de la variable en question calib2

vecteurMaxMin2axes calibration1() {
  // arrêt moteur 2
  digitalWrite(dir_out_2_L, LOW);
  digitalWrite(dir_out_2_R, LOW);
  analogWrite(pwm_out_2_R, 0);
  analogWrite(pwm_out_2_L, 0);
  // mise à 0 des sommes de mesures
  int sommeCalib1 = 0;
  // lecture et affichage du pot 1
  potR1 = analogRead(pot1);
  Serial.printf("\n avant calib p1 : %d ", potR1);
  // réglage de la vitesse de calibration 1
  pwm1 = VITESSECALIB1;
  // droite ON vitesse pwm1
  digitalWrite(dir_out_1_L, LOW);
  digitalWrite(dir_out_1_R, HIGH);
  analogWrite(pwm_out_1_R, pwm1);
  analogWrite(pwm_out_1_L, 0);
  delay(CALIBTIME / 3);
  // arrêt moteur 1, 1sc pour éviter les vibrations
  digitalWrite(dir_out_1_L, LOW);
  digitalWrite(dir_out_1_R, LOW);
  analogWrite(pwm_out_1_R, 0);
  analogWrite(pwm_out_1_L, 0);
  delay(1000);
  // gauche ON vitesse pwm1
  digitalWrite(dir_out_1_L, HIGH);
  digitalWrite(dir_out_1_R, LOW);
  analogWrite(pwm_out_1_R, 0);
  analogWrite(pwm_out_1_L, pwm1);
  delay(CALIBTIME / 2);
  // lecture pot et affichage mi-parcour
  potR1 = analogRead(pot1);
  Serial.printf(" \n mid calib p1 : %d ", potR1);

  delay(CALIBTIME / 2);
  // prise multiple de valeur de calib max et affichage
  for (int i = 0; i < nbrEchantillion1; i++) {
    potR1 = analogRead(pot1);
    Serial.printf("\n valeur1 : %d ", potR1);
    sommeCalib1 = sommeCalib1 + potR1;
    delay(tempsEchantillion);
  }
  calib1.potMax1 = int(sommeCalib1 / nbrEchantillion1);
  Serial.printf("\n sommeCalib1 : %d ", calib1.potMax1);
  sommeCalib1 = 0;
  // droite vitesse pwm1
  digitalWrite(dir_out_1_L, LOW);
  digitalWrite(dir_out_1_R, HIGH);
  analogWrite(pwm_out_1_R, pwm1);
  analogWrite(pwm_out_1_L, 0);

  delay(CALIBTIME);
  // prise multiple de valeur de calib min et affichage
  for (int i = 0; i < nbrEchantillion1; i++) {
    potR1 = analogRead(pot1);
    Serial.printf("\n valeurCalib1 : %d ", potR1);
    sommeCalib1 = sommeCalib1 + potR1;
    delay(tempsEchantillion);
  }
  calib1.potMin1 = int(sommeCalib1 / nbrEchantillion1);
  Serial.printf("\n somme Calib1 : %d ", calib1.potMin1);

  // vérification existence d’un potentiomètre et sens du moteur
  if (abs(calib1.potMax1 - calib1.potMin1) < 100) {
    pot1HS = 1;
    Serial.printf("\n POT1HS max: %d ", calib1.potMax1);
    Serial.printf("\n min : %d ", calib1.potMin1);
  } else {
    pot1HS = 0;
    // vérification sens du moteur 1
    if (calib1.potMin1 > calib1.potMax1) {
      calib1.potMin1 = calib1.potMax1;
      calib1.potMax1 = int(sommeCalib1 / nbrEchantillion1);
      Serial.printf("\n moteur 1 inverser \n maxCalib : %d ", calib1.potMax1);
      pot1Inv = 1;
    } else {
      Serial.printf("\n moteur 1 (OK) minCalib : %d ", calib1.potMin1);
      pot1Inv = 0;
    }
  }
  //delay(2000); // lors du DEV temps pour lire les commentaires de calibration
  return calib1;
}
vecteurMaxMin2axes calibration2() {
  // arrêt moteur 1
  digitalWrite(dir_out_1_L, LOW);
  digitalWrite(dir_out_1_R, LOW);
  analogWrite(pwm_out_1_R, 0);
  analogWrite(pwm_out_1_L, 0);
  // mise à 0 des sommes de mesure
  int sommeCalib2 = 0;
  // lecture et affichage du pot 2
  potR2 = analogRead(pot2);
  Serial.printf(" \n avant calib p2 : %d ", potR2);
  // réglage de la vitesse de calibration 2
  pwm2 = VITESSECALIB2;
  // droite ON vitesse pwm2
  digitalWrite(dir_out_2_L, LOW);
  digitalWrite(dir_out_2_R, HIGH);
  analogWrite(pwm_out_2_R, pwm2);
  analogWrite(pwm_out_2_L, 0);
  delay(CALIBTIME / 3);
  // arrêt moteur 1, 1sc pour éviter les vibrations
  digitalWrite(dir_out_2_L, LOW);
  digitalWrite(dir_out_2_R, LOW);
  analogWrite(pwm_out_2_R, 0);
  analogWrite(pwm_out_2_L, 0);
  delay(1000);
  // gauche ON vitesse pwm2
  digitalWrite(dir_out_2_L, HIGH);
  digitalWrite(dir_out_2_R, LOW);
  analogWrite(pwm_out_2_R, 0);
  analogWrite(pwm_out_2_L, pwm2);

  delay(CALIBTIME / 2);
  // lecture pot et affichage mi-parcour
  potR2 = analogRead(pot2);
  Serial.printf(" \n milieu p2 : %d ", potR2);
  Serial.printf(" p2 : %d \n", potR2);
  delay(CALIBTIME / 2);
  // prise multiple de valeur de calib max et affichage
  for (int i = 0; i < nbrEchantillion2; i++) {
    potR2 = analogRead(pot2);
    Serial.printf("\n valeurCalib2 : %d ", potR2);
    sommeCalib2 = sommeCalib2 + potR2;
    delay(tempsEchantillion);
  }
  calib2.potMax2 = int(sommeCalib2 / nbrEchantillion2);
  Serial.printf(" \n valeur Calib2 : %d ", calib2.potMax2);
  sommeCalib2 = 0;
  // droite vitesse pwm2
  digitalWrite(dir_out_2_L, LOW);
  digitalWrite(dir_out_2_R, HIGH);
  analogWrite(pwm_out_2_R, pwm2);
  analogWrite(pwm_out_2_L, 0);
  delay(CALIBTIME);
  // prise multiple de valeur de calib min et affichage
  for (int i = 0; i < nbrEchantillion2; i++) {
    potR2 = analogRead(pot2);
    Serial.printf("\n valeurCalib2 : %d ", potR2);
    sommeCalib2 = sommeCalib2 + potR2;
    delay(tempsEchantillion);
  }
  calib2.potMin2 = int(sommeCalib2 / nbrEchantillion2);
  // vérification existence d’un potentiomètre et sens du moteur
  if (abs(calib2.potMax2 - calib2.potMin2) < 100) {
    Serial.printf("\n POT2HS max: %d ", calib2.potMax2);
    Serial.printf("\n min : %d ", calib2.potMin2);
    pot2HS = 1;
  } else {
    pot2HS = 0;
    if (calib2.potMin2 > calib2.potMax2) {
      calib2.potMin2 = calib2.potMax2;
      calib2.potMax2 = int(sommeCalib2 / nbrEchantillion2);
      pot2Inv = 1;
      Serial.printf(" INV maxCalib2 : %d ", calib2.potMax2);
    } else {
      pot2Inv = 0;
      Serial.printf(" minCalib2 : %d ", calib2.potMin2);
      Serial.printf(" maxCalib2 : %d ", calib2.potMax2);
    }
  }
  //delay(2000); // lors du DEV temps pour lire les commentaires de calibration
  return calib2;
}
// processeur 1 : gestion DMX
volatile uint8_t buffer[DMXINPUT_BUFFER_SIZE(canalDMX, NB_CHANNELS)];
void setup1() {
  // adressage DMX
  canalDMX = digitalRead(dmx1) + digitalRead(dmx2) * 2 + digitalRead(dmx3) * 4 + digitalRead(dmx4) * 8 + digitalRead(dmx5) * 16 + digitalRead(dmx6) * 32 + digitalRead(dmx7) * 64 + digitalRead(dmx8) * 128;
  if (canalDMX == 0) {
    canalDMX = 1;
  }
  canalDMX = canalDMX - 1;
  // Setup our DMX Input to read on GPIO 0, from channel 1 to 3
  dmxInput.begin(0, 0, NB_CHANNELS);
  dmxInput.read_async(buffer);  // lecteur dmx asynchrone (c.a.d. dès qu’un packet arrive)
}

void setup() {
  //parametrage
  Serial.begin(115200);
  analogWriteResolution(BITS_DMX);
  analogWriteFreq(FREQUENCE);         //fréquence de commutation du driver moteur (jusqu’à 20 000 HZ c’est audible)
  analogReadResolution(BITS_ANALOG);  // résolution max du lecteur du potentiometre moteur

  pinMode(pot1, INPUT_PULLDOWN);
  pinMode(pot2, INPUT_PULLDOWN);

  pinMode(pwm_out_1_R, OUTPUT);
  pinMode(pwm_out_1_L, OUTPUT);
  pinMode(dir_out_1_R, OUTPUT);
  pinMode(dir_out_1_L, OUTPUT);

  pinMode(pwm_out_2_R, OUTPUT);
  pinMode(pwm_out_2_L, OUTPUT);
  pinMode(dir_out_2_R, OUTPUT);
  pinMode(dir_out_2_L, OUTPUT);

  pinMode(enable_1_R, OUTPUT);
  pinMode(enable_1_L, OUTPUT);
  pinMode(enable_2_R, OUTPUT);
  pinMode(enable_2_L, OUTPUT);

  digitalWrite(enable_1_R, HIGH);
  digitalWrite(enable_1_L, HIGH);
  digitalWrite(enable_2_R, HIGH);
  digitalWrite(enable_2_L, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);  // initialisation LED

  //calib1 = calibration1(); //calibration1 au démarage
  //calib2 = calibration2(); //calibration2 au démarage

  // valeur de calib par défaut
  //precalibration
  canalDMX = (-1) + digitalRead(dmx1) + digitalRead(dmx2) * 2 + digitalRead(dmx3) * 4 + digitalRead(dmx4) * 8 + digitalRead(dmx5) * 16 + digitalRead(dmx6) * 32 + digitalRead(dmx7) * 64 + digitalRead(dmx8) * 128;
  if (canalDMX == 0)  // c’est le canal DMX-1 à cause de la lecture de la matrice qui stock les valeurs du signal DMX
  {                   // module 1
    calib1.potMin1 = 16;
    calib1.potMax1 = 4094;
    calib2.potMin2 = 369;
    calib2.potMax2 = 2383;
  } else if (canalDMX == 12)  // module 2
  {
    calib1.potMin1 = 258;
    calib1.potMax1 = 4024;
    calib2.potMin2 = 383;
    calib2.potMax2 = 2286;
  } else if (canalDMX == 24)  // module 3
  {
    calib1.potMin1 = 173; 
    calib1.potMax1 = 4024;//4003
    calib2.potMin2 = 205;
    calib2.potMax2 = 2276;
  } else if (canalDMX == 36)  // module 4
  {
    calib1.potMin1 = 204;
    calib1.potMax1 = 4061;
    calib2.potMin2 = 244;
    calib2.potMax2 = 2252;
  }
}
void gestionTemporelle() {
  cLoop = cLoop + 1;
  s = (cLoop / (3000));
  if (s > tPause) {
    cLoop = 0;
  }
}
void loop() {
  // TIMING SECONDE
  gestionTemporelle();
  // gestion moteur 1
  // lancement DMX calibration1
  if (buffer[canalDMX + lanceCalibM1] > 200) {
    calib1 = calibration1();
  }
  // gestion potentiomètre 1
    potR1 = analogRead(pot1);
    int sommePot1 = 0;
    pot1m[0] = potR1;
    for (int i = (MOYENNEPOT1-1); i >= 0; i--) {
      sommePot1 = sommePot1 + pot1m[i];
      if (i > 0) {
        pot1m[i] = pot1m[i - 1];
      }
    }
    potR1 = ((sommePot1) / MOYENNEPOT1);
    /*
    potR1 = (potR1 + moins1prePot1) / 2;
    moins1prePot1 = potR1;*/
    if (potR1 > calib1.potMax1) {
      potR1 = calib1.potMax1;
    } else if (potR1 < calib1.potMin1) {
      potR1 = calib1.potMin1;
    }
  // aquisition + conversion DMX 8bits -> décimal 16 bits
  std::bitset<16> bitsM1(std::bitset<8>(buffer[canalDMX + DMXpositionM1a]).to_string() + std::bitset<8>(buffer[canalDMX + DMXpositionM1b]).to_string());
  bits1 = bitsM1.to_ulong();
  std::bitset<16> offsetM1(std::bitset<8>(buffer[canalDMX + DMXoffsetM1a]).to_string() + std::bitset<8>(buffer[canalDMX + DMXoffsetM1b]).to_string());
  offset1 = offsetM1.to_ulong();
  // symetrie offset
  if (offset1 > 32768) {
    offset1 = 32768 - offset1;
  }
  // différence position moteur 1 vs DMX + offset
  if (pot1HS == 0) {
    pwmIn1 = (bits1 - (((potR1 - calib1.potMin1) * 65535) / (calib1.potMax1 - calib1.potMin1))) * multipleDifference1 + offset1 * Multoffset1;  //
  } else {
    pwmIn1 = bits1 - (65535 / 2) + offset1;
  }
  // vitesse entrée
  vitesseMoteur1 = ((buffer[canalDMX + DMXvitesseM1]) * VITESSEMAX1) / 255 + VITESSEMIN1;
  // acceleration
  if ((pwmIn1 < pwmOut1) and (pwmOut1 > (-pow(2, BITS_DMX) - ACCELERATION1)) and pwmOut1 > -(vitesseMoteur1 - VITESSEMIN1)) {
    pwmOut1 = pwmOut1 - DECELERATION1;
  } else if ((pwmIn1 >= pwmOut1) and (pwmOut1 < (pow(2, BITS_DMX) + ACCELERATION1)) and (pwmOut1 < (vitesseMoteur1 - VITESSEMIN1))) {
    pwmOut1 = pwmOut1 + ACCELERATION1;
  }
  // vitesse sortie 1
    pwm1 = abs(pwmOut1) + VITESSEMIN1;
    //Serial.printf("\n 1: %d", pwm1);
    int sommepwm1 = 0;
    pwm1m[0] = pwm1;
    for (int i = (MOYENNEVITESSE1-1); i >= 0; i--) {
      sommepwm1 = sommepwm1 + pwm1m[i];
      if (i > 0) {
        pwm1m[i] = pwm1m[i - 1];
      }
    }
    pwm1 = ((sommepwm1) / MOYENNEVITESSE1);
    //Serial.printf("\n 2: %d", pwm1);
    /*pwm1 = (pre1pwm1 + pre2pwm1 + pwm1)/3;
      pre2pwm1=pre1pwm1;
      pre1pwm1 = pwm1;*/
    if (pwm1 > vitesseMoteur1) {
      pwm1 = vitesseMoteur1;
    } else if (pwm1 < vitesseMoteur1) {
      pwm1 = VITESSEMIN1;
    }
  // driver 1
    if ((pwmIn1 < TOLERANCEPOINTFIXE1 and pwmIn1 > -TOLERANCEPOINTFIXE1) or ((bits1 > 65440 or bits1 < 10) and (pot1HS == 1)))  // point fixe -> coupe allimentation moteur
    {
      if (tempsPositionFixe1 < 1000) {
        tempsPositionFixe1++;
      }
      if ((pwmIn1 < TOLERANCEPOINTFIXE1 / precisionM1) and (pwmIn1 > -TOLERANCEPOINTFIXE1 / precisionM1)) {
        digitalWrite(dir_out_1_L, LOW);
        digitalWrite(dir_out_1_R, LOW);
        analogWrite(pwm_out_1_R, 0);
        analogWrite(pwm_out_1_L, 0);
        pwm1 = 0;
      }

    } else if (pwmOut1 >= 0 or (pot1Inv == 1 and pwmOut1 < 0))  // Gauche intérieur 1
    {
      if (tempsPositionFixe1 >= 0) {
        tempsPositionFixe1--;
      }
      if (tempsPositionFixe1 < 100) {
        digitalWrite(dir_out_1_L, HIGH);
        digitalWrite(dir_out_1_R, LOW);
        analogWrite(pwm_out_1_R, 0);
        analogWrite(pwm_out_1_L, pwm1);
      }
    } else  // Droite extérieur 1
    {
      if (tempsPositionFixe1 >= 0) {
        tempsPositionFixe1--;
      }
      if (tempsPositionFixe1 < 100) {
        digitalWrite(dir_out_1_L, LOW);
        digitalWrite(dir_out_1_R, HIGH);
        analogWrite(pwm_out_1_R, pwm1);
        analogWrite(pwm_out_1_L, 0);
      }
   }

  // gestion moteur 2
  // lancement DMX calibration 2
  if (buffer[canalDMX + lanceCalibM2] > 200) {
    calib2 = calibration2();
  }
  // gestion potentiomètre 2
   potR2 = analogRead(pot2);
      int sommePot2 = 0;
    pot2m[0] = potR2;
    for (int i = (MOYENNEPOT2-1); i >= 0; i--) {
      sommePot2 = sommePot2 + pot2m[i];
      if (i > 0) {
        pot2m[i] = pot2m[i - 1];
      }
    }
    potR2 = ((sommePot2) / MOYENNEPOT2);
    /*potR2 = (potR2 + moins1prePot2) / 2;
    moins1prePot2 = potR2; */
    if (potR2 > calib2.potMax2) {
      potR2 = calib2.potMax2;
    } else if (potR2 < calib2.potMin2) {
      potR2 = calib2.potMin2;
    }
  // aquisition + conversion 16 bits
  std::bitset<16> bitsM2(std::bitset<8>(buffer[canalDMX + DMXpositionM2a]).to_string() + std::bitset<8>(buffer[canalDMX + DMXpositionM2b]).to_string());
  bits2 = bitsM2.to_ulong();
  std::bitset<16> offsetM2(std::bitset<8>(buffer[canalDMX + DMXoffsetM2a]).to_string() + std::bitset<8>(buffer[canalDMX + DMXoffsetM2b]).to_string());
  offset2 = offsetM2.to_ulong();
  // symetrie offset
  if (offset2 > 32768) {
    offset2 = 32768 - offset2;
  }
  // différence position moteur 2 vs DMX + offset
  if (pot2HS == 0)  // Si moteur n’a pas de potentiomètre calcul de la position DMX et ajout de l’offset
  {
    pwmIn2 = (bits2 - (((potR2 - calib2.potMin2) * 65535) / (calib2.potMax2 - calib2.potMin2))) * multipleDifference2 + offset2 * Multoffset2;  //DMX-position+offset si >0 alors descent si <0 monte
  } else                                                                                                                                        //différence potition moteur et position DMX et ajout de l’offset
  {
    pwmIn2 = bits2 - (65535 / 2) + offset2;
  }
  // vitesse entrée
  vitesseMoteur2 = ((buffer[canalDMX + DMXvitesseM2]) * VITESSEMAX2) / 255 + VITESSEMIN2;
  // accélération 2
  if ((pwmIn2 < pwmOut2) and (pwmOut2 > (-pow(2, BITS_DMX) - ACCELERATION2)) and pwmOut2 > -(vitesseMoteur2 - VITESSEMIN2)) {
    pwmOut2 = pwmOut2 - DECELERATION2;
  } else if ((pwmIn2 >= pwmOut2) and (pwmOut2 < (pow(2, BITS_DMX) + ACCELERATION2)) and pwmOut2 < (vitesseMoteur2 - VITESSEMIN2)) {
    pwmOut2 = pwmOut2 + ACCELERATION2;
  }
  // vitesse sortie 2
    pwm2 = abs(pwmOut2) + VITESSEMIN2;
    int sommepwm2 = 0;
    pwm2m[0] = pwm2;
    for (int i = (MOYENNEVITESSE2 -1); i >= 0; i--) {
      sommepwm2 = sommepwm2 + pwm2m[i];
      if (i > 0) {
        pwm2m[i] = pwm2m[i - 1];
      }
    }
    pwm2 = ((sommepwm2) / MOYENNEVITESSE2);
  /*
    pwm2 = (pre1pwm2 + pre2pwm2 + pwm2) / 3;
    pre2pwm2 = pre1pwm2;
    pre1pwm2 = pwm2;*/
    if (pwm2 > vitesseMoteur2) {
      pwm2 = vitesseMoteur2;
    } else if (pwm2 < vitesseMoteur2) {
      pwm2 = VITESSEMIN2;
    }
  // driver 2
    if (((pwmIn2 < TOLERANCEPOINTFIXE2) and (pwmIn2 > -TOLERANCEPOINTFIXE2)) or ((bits2 > 65440 or bits2 < 10) and (pot2HS == 1))) {
      if (tempsPositionFixe2 <= 1000) {
        tempsPositionFixe2++;
      }
      if ((pwmIn2 < TOLERANCEPOINTFIXE2 / precisionM2) and (pwmIn2 > -TOLERANCEPOINTFIXE2 / precisionM2)) {
        digitalWrite(dir_out_2_L, LOW);
        digitalWrite(dir_out_2_R, LOW);
        analogWrite(pwm_out_2_R, 0);
        analogWrite(pwm_out_2_L, 0);
        pwm2 = 0;
      }
    } else if (pwmOut2 > 0 or (pot2Inv == 1 and pwmOut2 < 0)) {
      if (tempsPositionFixe2 > 0) {
        tempsPositionFixe2--;
      }
      if (tempsPositionFixe2 < 400) {
        digitalWrite(dir_out_2_L, HIGH);
        digitalWrite(dir_out_2_R, LOW);
        analogWrite(pwm_out_2_R, 0);
        analogWrite(pwm_out_2_L, pwm2);
      }
    } else {
      if (tempsPositionFixe2 > 0) {
        tempsPositionFixe2--;
      }
      if (tempsPositionFixe2 < 400) {
        digitalWrite(dir_out_2_L, LOW);
        digitalWrite(dir_out_2_R, HIGH);
        analogWrite(pwm_out_2_R, pwm2);
        analogWrite(pwm_out_2_L, 0);
      }
  }
  // affichage des variables
    if (cLoop % 1000 == 1) {
      Serial.printf("\n");
      //potR2 = analogRead(pot2);
      // affichage moteur 1
      if (affichageMoniteurSerie == 1 or affichageMoniteurSerie == 3) {
        //Serial.printf(" stay1 %d ", stay1);
        Serial.printf(" DMX %d ", canalDMX);
        Serial.printf(" R1 %d ", potR1);
        Serial.printf(" Max %d ", calib1.potMax1);
        Serial.printf(" Min %d ", calib1.potMin1);
        Serial.printf(" pwmIn %d ", pwmIn1);
        Serial.printf(" pwmOut %d ", pwmOut1);
        Serial.printf(" pwm %d ", pwm1);
        Serial.printf(" MX %d ", bits1);
        Serial.printf(" HS %d", pot1HS);
        Serial.printf(" Inv %d", pot1Inv);
        Serial.printf(" vit %d", vitesseMoteur1);
        Serial.printf(" pFix %d", tempsPositionFixe1);
      }
      // affichage moteur 2
      if (affichageMoniteurSerie == 2 or affichageMoniteurSerie == 3) {
        //Serial.printf(" stay2 %d ", stay2);
        Serial.printf(" R2 %d ", potR2);
        Serial.printf(" Min %d ", calib2.potMin2);
        Serial.printf(" Max %d ", calib2.potMax2);
        Serial.printf(" pwmIn %d", pwmIn2);
        Serial.printf(" pwmOut %d ", pwmOut2);
        Serial.printf(" pwm %d ", pwm2);
        Serial.printf(" MX %d", bits2);
        Serial.printf(" HS %d", pot2HS);
        Serial.printf(" Inv %d", pot2Inv);
        Serial.printf(" vit %d", vitesseMoteur2);
        Serial.printf(" pFix %d", tempsPositionFixe2);
      }
      // affichage dmx
      if (affichageMoniteurSerie == 4) {
        Serial.printf("1:", DMXpositionM1a);
        Serial.printf("2:", DMXpositionM1b);
        Serial.printf("4:", DMXpositionM2a);
        Serial.printf("5:", DMXpositionM2b);
        Serial.printf("7:", DMXoffsetM1a);
        Serial.printf("8:", DMXoffsetM1b);
        Serial.printf("9:", DMXoffsetM2a);
        Serial.printf("10:", DMXoffsetM2b);
        Serial.printf("11:", lanceCalibM1);
        Serial.printf("12:", lanceCalibM2);
      }
  }
}