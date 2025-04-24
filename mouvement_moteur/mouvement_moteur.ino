#include <Wire.h>
#include <SoftwareSerial.h>



// ====================================== VARIABLES ======================================
#define MD25ADDRESS 0x58 // Adresse de la MD25
#define SPEEDENGINE1 0x00 // Registre de la vitesse du moteur 1
#define SPEEDENGINE2 0x01 // Registre de la vitesse du moteur 2
#define ENC1A 0x02  // Premier registre de l'encodeur du moteur 1
#define ENC2A 0x06  // Premier registre de l'encodeur du moteur 2
#define RESETENCODERS 0x20 // Commande pour réinitialiser les encodeurs
#define WAITDELAY2SEC 0x32 // Commande pour désactiver le délai d'attente de 2 secondes des moteurs
#define ACCELERATION 0x0E // Registre de l'accélération des moteurs (temps nécessaire aux moteurs pour atteindre la
                          // vitesse souhaitée)
SoftwareSerial BTSerial(2, 3); // RX | TX du module Bluetooth

// Variables de mouvement
bool avancer = false;
bool gauche = false;
bool reculer = false;
bool droite = false;

float coef = 0.5; // Coefficient de vitesse (permet à l'utilisateur d'aller plus ou moins vite)

unsigned long previousMillis = 0; // Stocke le dernier moment où on a lu les encodeurs. On définit sur 0 car le
                                  // programme commence.
long interval = 57.8 * (1.73 / coef); // Intervalle en millisecondes. Le calcul est expliqué tout en bas.

long enc1 = 0; // Stockera le nombre d'impulsions effectuées par le moteur droit
long enc2 = 0; // Stockera le nombre d'impulsions effectuées par le moteur gauche
long enc1Prec = 0; // Stockera le nombre précédemment stocké dans enc1 afin de pouvoir faire la différence entre la
                   // nouvelle valeur de enc1 et son ancienne valeur.
long enc2Prec = 0; // Même fonction que enc1Prec, pour enc2.



// ====================================== MODE ======================================
// 1 = mode manuel
// 2 = mode autonome
int mode = 1;



// ====================================== FONCTIONS ======================================

void changerVitesseGauche(int vitesse){
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEEDENGINE2);
  Wire.write(vitesse);
  Wire.endTransmission();
}

void changerVitesseDroite(int vitesse){
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEEDENGINE1);
  Wire.write(vitesse);
  Wire.endTransmission();
}

void resetEncodeurs() {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(0x10);  // Registre Command
  Wire.write(0x20);  // Commande pour reset les encodeurs
  Wire.endTransmission();
}

long lireEncodeur(byte registre) {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(registre);  // Demande le premier octet de l'encodeur (Enc1a ou Enc2a)
  Wire.endTransmission();
  
  Wire.requestFrom(MD25ADDRESS, 4);  // Demande 4 octets

  long position = 0;
  if (Wire.available() == 4) {  // Vérifie que 4 octets sont bien reçus
    position |= (long)Wire.read() << 24;  // Byte 1 (MSB - poids fort)
    position |= (long)Wire.read() << 16;  // Byte 2
    position |= (long)Wire.read() << 8;   // Byte 3
    position |= (long)Wire.read();        // Byte 4 (LSB - poids faible)
  }
  
  return position;
}


int detectionObstacleProche(int avG, int av, int avD, int d, int arD, int ar, int arG, int g) {
    int valeurs[8] = {avG, av, avD, d, arD, ar, arG, g};
    int seuil = 80; // cm

    int minIndex = 0;

    for (int i = 1; i < 8; ++i) {
        if (valeurs[i] < valeurs[minIndex]) {
            minIndex = i;
        }
    }

    // Si la valeur minimale est en dessous du seuil ET est unique → retour du capteur concerné
    if (valeurs[minIndex] < seuil)
    {
        return minIndex + 1;
    }
    return 0; // Pas de danger détecté
}


void changementDirection(int statut) {
  switch(statut) {
    case 0:
      avancer = true;
      gauche = false;
      reculer = false;
      droite = false;
    return;

    // Si obstacle en avant à gauche
    case 1:
      // Si on va avant gauche
      if (avancer && gauche && !reculer && !droite) {
        gauche = false;
        droite = true;
      }

      // Si on va en avant
      else if (avancer && !gauche && !reculer && !droite) {
        droite = true;
      }
      // Autres cas, on ne change pas
    return;



    // Si obstacle avant
    case 2:
      // Si obstacle en avant à gauche
      if (avancer && gauche && !reculer && !droite) {
        avancer = false;
      }

      // Si on va en avant
      else if (avancer && !gauche && !reculer && !droite) {
        avancer = false;
        gauche = true;
      }

      // Si on va en avant à droite
      else if (avancer && !gauche && !reculer && droite) {
        avancer = false;
      }
      // Autres cas, on ne change pas
    return;



    // Si obstacle en avant à droite
    case 3:
      // Si on va en avant
      if (avancer && !gauche && !reculer && !droite) {
        gauche = true;
      }

      // Si on va en avant à droite
      else if (avancer && !gauche && !reculer && droite) {
        gauche = true;
        droite = false;
      }
      // Autres cas, on ne change pas
    return;



    // Si obstacle à droite
    case 4:
      // Si on va en avant à droite
      if (avancer && !gauche && !reculer && droite) {
        droite = false;
      }

      // Si on va en arrière à droite
      else if (!avancer && !gauche && reculer && droite) {
        droite = false;
      }
    // Autres cas, on ne change pas
    return;



    // Si obstacle en arrière à droite
    case 5:
      // Si on va en arrière à droite
      if (!avancer && !gauche && reculer && droite) {
        droite = false;
        gauche = true;
      }

      // Si on va en arrière
      else if (!avancer && !gauche && reculer && !droite) {
        gauche = true;
      }
      // Autres cas, on ne change pas
    return;



    // Si obstacle en arrière
    case 6:
      // Si on va en arrière à droite
      if (!avancer && !gauche && reculer && droite) {
        reculer = false;
        droite = false;
        gauche = true;
      }

      // Si on va en arrière
      else if (!avancer && !gauche && reculer && !droite) {
        reculer = false;
        gauche = true;
      }

      // Si on va en arrière à gauche
      else if (!avancer && gauche && reculer && !droite) {
        gauche = false;
        reculer = false;
        droite = true;
      }
    // Autres cas, on ne change pas
    return;



    // Si obstacle en arrière à gauche
    case 7:
      // Si on va en arrière
      if (!avancer && !gauche && reculer && !droite) {
        droite = true;
      }

      // Si on va en arrière à gauche
      else if (!avancer && gauche && reculer && !droite) {
        gauche = false;
        droite = true;
      }
    // Autres cas, on ne change pas
    return;



    // Si obstacle à gauche
    case 8:
      // Si on va en avant à gauche
      if (avancer && gauche && !reculer && !droite) {
        gauche = false;
      }

      // Si on va en arrière à gauche
      else if (!avancer && gauche && reculer && !droite) {
        gauche = false;
      }
    // Autres cas, on ne change pas
  }
}


void mettreAJourMoteurs(float coef){

  // Avancer à gauche
  if (avancer && gauche && !reculer && !droite) {
    changerVitesseGauche(-32 * coef);
    changerVitesseDroite(-64 * coef);
  }

  // Avancer à droite
  else if (avancer && !gauche && !reculer && droite) {
    changerVitesseGauche(-64 * coef);
    changerVitesseDroite(-32 * coef);
  }

  // Reculer à gauche
  else if (!avancer && gauche && reculer && !droite) {
    changerVitesseGauche(32 * coef);
    changerVitesseDroite(64 * coef);
  }

  // Reculer à droite
  else if (!avancer && !gauche && reculer && droite) {
    changerVitesseGauche(64 * coef);
    changerVitesseDroite(32 * coef);
  }

  // Avancer
  else if ((avancer && !gauche && !reculer && !droite) || (avancer && gauche && !reculer && droite)) {
    changerVitesseGauche(-64 * coef);
    changerVitesseDroite(-64 * coef);
  }

  // Reculer
  else if ((!avancer && !gauche && reculer && !droite) || (avancer && !gauche && !reculer && droite)) {
    changerVitesseGauche(64 * coef);
    changerVitesseDroite(64 * coef);
  }

  // Gauche
  else if ((!avancer && gauche && !reculer && !droite) || (avancer && gauche && reculer && !droite)) {
    changerVitesseGauche(32 * coef);
    changerVitesseDroite(-32 * coef);
  }

  // Droite
  else if ((!avancer && !gauche && !reculer && droite) || (avancer && !gauche && reculer && droite)) {
    changerVitesseGauche(-32 * coef);
    changerVitesseDroite(32 * coef);
  }

  // Arrêt
  else { 
    changerVitesseGauche(0);
    changerVitesseDroite(0);
  }
}

void mettreAJourMoteursDerive(float coef, long enc1, long enc2, long enc1Prec, long enc2Prec){

  // Avancer à gauche
  if (avancer && gauche && !reculer && !droite) {
    long ecart1 = enc1 - enc1Prec;
    long ecart2 = enc2 - enc2Prec;

    if (ecart1 < 0) ecart1 = -ecart1;
    if (ecart2 < 0) ecart2 = -ecart2;

    float derive = ((float)ecart1 / (float)ecart2) / 10;

    changerVitesseGauche((-16 + (derive * -16)) * coef);
    changerVitesseDroite((-80 - (derive * -80)) * coef);
  }

  // Avancer à droite
  else if (avancer && !gauche && !reculer && droite) {
    long ecart1 = enc1 - enc1Prec;
    long ecart2 = enc2 - enc2Prec;

    if (ecart1 < 0) ecart1 = -ecart1;
    if (ecart2 < 0) ecart2 = -ecart2;

    float derive = ((float)ecart2 / (float)ecart1) / 10;

    changerVitesseGauche((-80 - (derive * -80)) * coef);
    changerVitesseDroite((-16 + (derive * -16)) * coef);
  }

  // Reculer à gauche
  else if (!avancer && gauche && reculer && !droite) {
    changerVitesseGauche(32 * coef);
    changerVitesseDroite(64 * coef);
  }

  // Reculer à droite
  else if (!avancer && !gauche && reculer && droite) {
    changerVitesseGauche(64 * coef);
    changerVitesseDroite(32 * coef);
  }

  // Avancer
  else if (avancer && !gauche && !reculer && !droite) {
    // Calcule l’erreur entre les mises à jours
    long ecart1 = abs(enc1  - enc1Prec);
    long ecart2 = abs(enc2  - enc2Prec);
    float error = float(ecart1 - ecart2);  // + si gauche plus rapide, – si droite plus rapide

    // Gain proportionnel, plus il est élevé, plus la force de correction est grande
    const float Kp = 0.2f;

    float vitesseBase = -64.0f * coef;

    // Calcul de la correction
    float corr = Kp * error;

    // Application de la correction
    float vitesseGauche = vitesseBase - corr;
    float vitesseDroite = vitesseBase + corr;

    changerVitesseGauche(vitesseGauche);
    changerVitesseDroite(vitesseDroite);
  }

  // Reculer
  else if ((!avancer && !gauche && reculer && !droite) || (avancer && !gauche && !reculer && droite)) {
    changerVitesseGauche(64 * coef);
    changerVitesseDroite(64 * coef);
  }

  // Gauche
  else if ((!avancer && gauche && !reculer && !droite) || (avancer && gauche && reculer && !droite)) {
    changerVitesseGauche(32 * coef);
    changerVitesseDroite(-32 * coef);
  }

  // Droite
  else if ((!avancer && !gauche && !reculer && droite) || (avancer && !gauche && reculer && droite)) {
    changerVitesseGauche(-32 * coef);
    changerVitesseDroite(32 * coef);
  }

  // Arrêt
  else { 
    changerVitesseGauche(0);
    changerVitesseDroite(0);
  }
}

void gestionTouches() {
  // Si une touche est entrée par BLuetooth
  if (BTSerial.available()) {
    char key = BTSerial.read();
    Serial.print("Reçu via Bluetooth : ");
    Serial.println(key);

    // Gestion des appuis
    if (key == 'z') avancer = true;
    if (key == 's') reculer = true;
    if (key == 'q') gauche = true;
    if (key == 'd') droite = true;

    // Gestion des relâchements
    if (key == 'Z') avancer = false;
    if (key == 'S') reculer = false;
    if (key == 'Q') gauche = false;
    if (key == 'D') droite = false;

    // Gestion des vitesses (accélération ou ralentissement)
    // Accélération (si vitesse max. autorisée ne va pas être dépassée)
    if (key == 'r'){
      if (coef < 1.48){
        coef = coef + 0.25;
        interval = 57.8 * (1.73 / coef);
      }
    }
    // Ralentissement (si vitesse min. autorisée ne va pas être dépassée)
    if (key == 'f'){
      if (coef > 0.25){
        coef = coef - 0.25;
        interval = 57.8 * (1.73 / coef);
      }
    }
  }

  // Si une touche est entrée en filaire (via le PC)
  if (Serial.available() > 0) {
    char key = Serial.read();
    Serial.print("Touche reçue : ");
    Serial.println(key);

    // Gestion des appuis
    if (key == 'z') avancer = true;
    if (key == 's') reculer = true;
    if (key == 'q') gauche = true;
    if (key == 'd') droite = true;

    // Gestion des relâchements
    if (key == 'Z') avancer = false;
    if (key == 'S') reculer = false;
    if (key == 'Q') gauche = false;
    if (key == 'D') droite = false;

    // Gestion des vitesses (accélération ou ralentissement)
    // Accélération (si vitesse max. autorisée ne va pas être dépassée)
    if (key == 'r'){
      if (coef < 1.48){
        coef = coef + 0.25;
        interval = 57.8 * (1.73 / coef);
      }
    }
    // Ralentissement (si vitesse min. autorisée ne va pas être dépassée)
    if (key == 'f'){
      if (coef > 0.25){
        coef = coef - 0.25;
        interval = 57.8 * (1.73 / coef);
      }
    }
  }
}

void setup(){
  Wire.begin();
  Wire.setClock(100000); // Défini la vitesse d'échange avec les modules en I2C à 100kHz
  Serial.begin(9600); // défini la vitesse d'échange entre Arduino <-> PC à 115200 bauds = 115200 bits par secondes
  BTSerial.begin(9600);  // Communication série avec le Bluetooth

  // Changer l'accélération (1-10)
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ACCELERATION);
  Wire.write(5); // Je définis sur 5 afin d'éviter le patinage au démarrage
  Wire.endTransmission();

  // Désactive le minuteur de 2 secondes qui stoppait les moteurs après 2 secondes sans mise à jour
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(0x10);
  Wire.write(0x32);
  Wire.endTransmission();

  // Mettre la MD25 en mode 1 (plage de puissance -> -128 - 127)
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(0x0F);
  Wire.write(0x01);
  Wire.endTransmission();

  // Remettre les encodeurs à 0
  resetEncodeurs();
}


void loop() {
  // Si le mode est en manuel
  if (mode == 1) {
    gestionTouches();
  }

  // Mise à jour des moteurs (ajustement de la vitesse des deux moteurs afin de changer la direction, la vitesse du
  // robot, ou corriger la dérive)
  //
  // L'intervalle de rafraichissement est définie par la variable "interval", calculée par 57.8 * (1.73 / coef) où :
  // - coef est le coéficient de la vitesse du robot. C'est lui qui va permettre au robot d'aller plus ou moins vite.
  // Mais si ce même coef est haut, plus la vitesse est haute, et plus il faut faire de mise à jour pour corriger la
  // dérive ou éviter des obstacles.
  //
  // C'est donc pour cela que je l'inclus dans le calcul de l'intervalle
  // 57.8 est une constante que j'ai choisis arbitrairement afin de faire en sorte que quand coef = 1 (vitesse moyenne),
  // cela donne 100 ms d'intervalle entre chaque MAJ des moteurs.
  //
  // Exemple : coef = 1, alors interval = 57.8 * (1.73 / 1) = 57.8 * 1.73 ~= 99.994 (ms), soit environ 10 MAJ/sec
  // Autre exemple : coef = 1.73 (vitesse maximale), alors interval = 57.8 * (1.73 / 1.73) = 57.8 * 1 = 57.8 (ms), soit
  // environ 17 MAJ/sec
  //
  // On voit bien que plus le coef est haut, moins l'intervalle est grande, et donc plus il y a de mises à jour par
  // seconde.
  unsigned long currentMillis = millis(); // Temps actuel
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Met à jour le dernier temps de lecture
    enc1Prec = enc1;
    enc2Prec = enc2;
    enc1 = lireEncodeur(ENC1A);
    enc2 = lireEncodeur(ENC2A);
    mettreAJourMoteursDerive(coef, enc1, enc2, enc1Prec, enc2Prec);
  }

}
