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
#define SLAVE_ADDR 0x42 // Adresse de l'Arduino Mega 2560 des capteurs
SoftwareSerial BTSerial(2, 3); // RX | TX du module Bluetooth

// Variables de mouvement actuellement adoptés par le robot
bool avancer = false;
bool gauche = false;
bool reculer = false;
bool droite = false;

// Variables de mouvement prise par le robot avant détection d'obstacle. C'est donc les mouvements que le robot faisait
// avant de rencontrer un obstacle et de modifier sa direction afin de l'éviter. Pour faire cela, il va modifier les
// mouvements par les variables juste au dessus afin de garder en mémoire sa direction initiale.
bool avancerOrig = false;
bool gaucheOrig = false;
bool reculerOrig = false;
bool droiteOrig = false;

// Variables direction obstacles.
// J'utilise le type "byte" qui est disponible en Arduino. L'équivalent en C++ est unsigned char. Tout deux peuvent
// stocker des valeurs entre 0 et 255. Cela permet de n'utiliser qu'un seul octet plutôt que 4 comme les int.
// Cela convient parfaitement aux données renvoyées par les capteurs du radar qui sont comprises entre 10 et 80.
byte avG = 80;
byte av = 80;
byte avD = 80;
byte d = 80;
byte arD = 80;
byte ar = 80;
byte arG = 80;
byte g = 80;
byte distances[8] = {avG, av, avD, d, arD, ar, arG, g};
bool arret = true;

float coef = 0.5f; // Coefficient de vitesse (permet à l'utilisateur d'aller plus ou moins vite)

unsigned long previousMillis = 0; // Stocke le dernier moment où on a lu les encodeurs. On définit sur 0 car le
                                  // programme commence.
float interval = 28.9f * (1.73f / coef); // Intervalle en millisecondes. Le calcul est expliqué tout en bas.

long enc1 = 0; // Stockera le nombre d'impulsions effectuées par le moteur droit
long enc2 = 0; // Stockera le nombre d'impulsions effectuées par le moteur gauche
long enc1Prec = 0; // Stockera le nombre précédemment stocké dans enc1 afin de pouvoir faire la différence entre la
                   // nouvelle valeur de enc1 et son ancienne valeur.
long enc2Prec = 0; // Même fonction que enc1Prec, pour enc2.

// ====================================== MODE ======================================
// 1 = mode manuel
// 2 = mode autonome
// 3 = mode obstacle manuel (utilisé pour tester le programme de détection sans le radar LiDAR)
int mode = 1;

// ====================================== FONCTIONS ======================================

// Change la vitesse du moteur gauche du robot
void changerVitesseGauche(int vitesse){
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEEDENGINE2);
  Wire.write(vitesse);
  Wire.endTransmission();
}

// Change la vitesse du moteur droit du robot
void changerVitesseDroite(int vitesse){
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEEDENGINE1);
  Wire.write(vitesse);
  Wire.endTransmission();
}

// Réinitialise les encodeurs du robot. On le fait dès le lancement du programme, par sécurité
void resetEncodeurs() {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(0x10);  // Registre Command
  Wire.write(0x20);  // Commande pour reset les encodeurs
  Wire.endTransmission();
}

// Je récupère le nombre d’impulsions cumulées depuis le démarrage des encodeurs
// Note : j’ai choisi une plage de vitesses de –128 à +127, donc quand les moteurs
// tournent en avant (–1…–128), le compteur décroît. Ça n’affecte pas le comportement général.
long lireEncodeur(byte registre) {
  // J’envoie l’adresse de la MD25 et le registre d’encodeur que je veux lire
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(registre);  // registre 2 = MSB de l’encodeur 1, registre 6 = MSB de l’encodeur 2
  Wire.endTransmission();

  // Je demande les 4 octets du compteur (32 bits)
  Wire.requestFrom(MD25ADDRESS, 4);

  long position = 0;
  // Si j’ai bien reçu mes 4 octets, je les assemble
  if (Wire.available() == 4) {
    // Je déplace chaque octet à sa place dans le long (MSB → LSB)
    position |= (long)Wire.read() << 24;  // octet 1 (poids fort)
    position |= (long)Wire.read() << 16;  // octet 2
    position |= (long)Wire.read() << 8;   // octet 3
    position |= (long)Wire.read();        // octet 4 (poids faible)
  }

  // Je renvoie la valeur 32 bits signée
  return position;
}

// Cette fonction va recevoir les données des capteurs LiDAR et va déterminer quel capteur a capté l'obstacle le plus
// proche. Chaque capteur enverra ses données aux directions qui leur correspond.
// Exemple : le capteur qui est pointé vers l'avant enverra ses données au paramètre "av" pour avant.
// Autre exemple : le capteur qui est pointé vers l'avant à droite enverra ses données au paramètres "avD" pour avant
// droit.
// Note : les valeurs envoyés par les capteurs sont compris entre 10 et 80 (cm)
int detectionObstacleProche(byte avG, byte av, byte avD, byte d, byte arD, byte ar, byte arG, byte g) {
    int valeurs[8] = {avG, av, avD, d, arD, ar, arG, g};
    int seuil = 30;

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
      avancer = avancerOrig;
      gauche = gaucheOrig;
      reculer = reculerOrig;
      droite = droiteOrig;
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

      // Si on va déjà en avant à droite
      else if (avancer && !gauche && !reculer && droite){
        return;
      }
      else{
        avancer = avancerOrig;
        gauche = gaucheOrig;
        reculer = reculerOrig;
        droite = droiteOrig;
      }
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

      // Si on va à gauche
      else if (!avancer && gauche && !reculer && !droite) {
        return;
      }

      // Si on va à droite
      else if (!avancer && !gauche && !reculer && droite) {
        return;
      }

      else{
        avancer = avancerOrig;
        gauche = gaucheOrig;
        reculer = reculerOrig;
        droite = droiteOrig;
      }
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

      // Si on va en avant à gauche
      else if (avancer && gauche && !reculer && !droite) {
        return;
      }
      else{
        avancer = avancerOrig;
        gauche = gaucheOrig;
        reculer = reculerOrig;
        droite = droiteOrig;
      }
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
      else{
        avancer = avancerOrig;
        gauche = gaucheOrig;
        reculer = reculerOrig;
        droite = droiteOrig;
      }
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
      else{
        avancer = avancerOrig;
        gauche = gaucheOrig;
        reculer = reculerOrig;
        droite = droiteOrig;
      }
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
      else{
        avancer = avancerOrig;
        gauche = gaucheOrig;
        reculer = reculerOrig;
        droite = droiteOrig;
      }
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
      else{
        avancer = avancerOrig;
        gauche = gaucheOrig;
        reculer = reculerOrig;
        droite = droiteOrig;
      }
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
      else{
        avancer = avancerOrig;
        gauche = gaucheOrig;
        reculer = reculerOrig;
        droite = droiteOrig;
      }
    return;
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

// On prend en compte la dérive seulement quand on va en avant. On pourrait très bien l'appliquer quand on va en marche arrière mais c'est plutôt rare donc pas vraiment utile.
void mettreAJourMoteursDerive(float coef, long enc1, long enc2, long enc1Prec, long enc2Prec){

  // Avancer à gauche
  if (avancer && gauche && !reculer && !droite) {
    // Calcul des écarts d'encodeur (valeurs absolues)
    long ecart1 = enc1 - enc1Prec;
    long ecart2 = enc2 - enc2Prec;

    if (ecart1 < 0) ecart1 = -ecart1;
    if (ecart2 < 0) ecart2 = -ecart2;

    // Calcul de la dérive pour ajuster la trajectoire
    float derive = ((float)ecart1 / (float)ecart2) / 10;

    // Ajustement des vitesses des roues en fonction de la dérive
    changerVitesseGauche((-16 + (derive * -16)) * coef);
    changerVitesseDroite((-80 - (derive * -80)) * coef);
  }

  // Avancer à droite
  else if (avancer && !gauche && !reculer && droite) {
    // Calcul des écarts d'encodeur (valeurs absolues)
    long ecart1 = enc1 - enc1Prec;
    long ecart2 = enc2 - enc2Prec;

    if (ecart1 < 0) ecart1 = -ecart1;
    if (ecart2 < 0) ecart2 = -ecart2;

    // Calcul de la dérive pour ajuster la trajectoire
    float derive = ((float)ecart2 / (float)ecart1) / 10;

    // Ajustement des vitesses des roues en fonction de la dérive
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


void gestionMode(char entree) {  
  if (entree == '&'){
    mode = 1; // Mode manuel
    // On passe toutes les valeurs de directions à false par sécurité, afin que l'utilisateur ne soit pas prit par
    // surprise
    avancer = false;
    avancerOrig = false;
    gauche = false;
    gaucheOrig = false;
    reculer = false;
    reculerOrig = false;
    droite = false;
    droiteOrig = false;
  }
  else if (entree == 'a') mode = 2; // Mode autonome
  else if (entree == '"') mode = 3; // Mode obstacle manuel
}

void gestionVitesse(char entree) {
  // Gestion des vitesses (accélération ou ralentissement)
  // Accélération (si vitesse max. autorisée ne va pas être dépassée)
  if (entree == 'r'){
    if (coef < 1.48){
      coef = coef + 0.25;
      interval = 28.9 * (1.73 / coef);
    }
  }
  // Ralentissement (si vitesse min. autorisée ne va pas être dépassée)
  if (entree == 'f'){
    if (coef > 0.25){
      coef = coef - 0.25;
      interval = 28.9 * (1.73 / coef);
    }
  }
}

void gestionMouvement(char entree) {
  // Gestion des appuis
  if (entree == 'z'){
    avancer = true;
    avancerOrig = true;
  }
  if (entree == 's'){
    reculer = true;
    reculerOrig = true;
  }
  if (entree == 'q'){
    gauche = true;
    gaucheOrig = true;
  }
  if (entree == 'd'){
    droite = true;
    droiteOrig = true;
  }

  // Gestion des relâchements
  if (entree == 'Z'){
    avancer = false;
    avancerOrig = false;
  }
  if (entree == 'S'){
    reculer = false;
    reculerOrig = false;
  }
  if (entree == 'Q'){
    gauche = false;
    gaucheOrig = false;
  }
  if (entree == 'D'){
    droite = false;
    droiteOrig = false;
  }
}

void gestionObstacleAutonome(char entree) {
  // Touche d'arrêt complet ()
  if (entree == 'c'){
    arret = true;
    avancer = false;
    avancerOrig = false;
    gauche = false;
    gaucheOrig = false;
    droite = false;
    droiteOrig = false;
    reculer = false;
    reculerOrig = false;
  }

  // Touche de reprise
  if (entree == 'x'){
    arret = false;
    avancer = true;
    avancerOrig = true;
  }
}

void gestionObstacleManuel(char entree) {
  // Touche d'arrêt complet ()
  if (entree == 'c'){
    arret = true;
    avancer = false;
    avancerOrig = false;
    gauche = false;
    gaucheOrig = false;
    droite = false;
    droiteOrig = false;
    reculer = false;
    reculerOrig = false;
  }

  // Touche de reprise
  if (entree == 'x'){
    arret = false;
    avancer = true;
    avancerOrig = true;
  }

  // Gestion des appuis
  if (entree == 't') avG = 50;
  if (entree == 'y') av = 50;
  if (entree == 'u') avD = 50;
  if (entree == 'j') d = 50;
  if (entree == 'n') arD = 50;
  if (entree == 'h') ar = 50;
  if (entree == 'b') arG = 50;
  if (entree == 'g') g = 50;

  // Gestion des relâchements
  if (entree == 'T') avG = 80;
  if (entree == 'Y') av = 80;
  if (entree == 'U') avD = 80;
  if (entree == 'J') d = 80;
  if (entree == 'N') arD = 80;
  if (entree == 'H') ar = 80;
  if (entree == 'B') arG = 80;
  if (entree == 'G') g = 80;
}

void modeManuel(){
  // Si une touche est entrée en filaire (via le PC)
  if (Serial.available() > 0) {
    char key = Serial.read();
    Serial.print("Touche reçue : ");
    Serial.println(key);

    gestionMode(key);
    gestionVitesse(key);
    gestionMouvement(key);
  }
  // Si une touche est entrée en bluetooth (via l'application)
  if (BTSerial.available() > 0) {
    char key = (char)BTSerial.read();
    Serial.print("Touche reçue : ");
    Serial.println(key);

    gestionMode(key);
    gestionVitesse(key);
    gestionMouvement(key);
  }
}

void modeAutonome(){
  // Si une touche est entrée en filaire (via le PC)
  if (Serial.available() > 0) {
    char key = Serial.read();
    Serial.print("Touche reçue :");
    Serial.println(key);

    gestionMode(key);
    gestionVitesse(key);
    gestionObstacleAutonome(key);
  }
  // Si une touche est entrée en bluetooth (via l'application)
  if (BTSerial.available() > 0) {
    char key = (char)BTSerial.read();
    Serial.print("Touche reçue : ");
    Serial.println(key);

    gestionMode(key);
    gestionVitesse(key);
    gestionObstacleAutonome(key);
  }
  int obstacleProche = detectionObstacleProche(avG, av, avD, d, arD, ar, arG, g);
  Serial.print("Direction obstacle : ");
  Serial.println(obstacleProche);
  // Si on a décidé d'arrêter le robot avec la touche 'c' (voir plus haut), on ne fait pas le changement de direction
  // car sinon le robot continuerai de bouger.
  if (!arret) {
    changementDirection(obstacleProche);
    Serial.print("Entrée");
  }
}

void modeObstacleManuel() {
  // Si une touche est entrée en filaire (via le PC)
  if (Serial.available() > 0) {
    char key = Serial.read();
    Serial.print("Touche reçue : ");
    Serial.println(key);

    gestionMode(key);
    gestionVitesse(key);
    gestionObstacleManuel(key);
  }
  // Si une touche est entrée en bluetooth (via l'application)
  if (BTSerial.available() > 0) {
    char key = (char)BTSerial.read();
    Serial.print("Touche reçue : ");
    Serial.println(key);

    gestionMode(key);
    gestionVitesse(key);
    gestionObstacleManuel(key);
  }
  int obstacleProche = detectionObstacleProche(avG, av, avD, d, arD, ar, arG, g);

  // Si on a décidé d'arrêter le robot avec la touche 'c' (voir plus haut), on ne fait pas le changement de direction
  // car sinon le robot continuerai de bouger.
  if (!arret) {
      changementDirection(obstacleProche);
  }
}

void setup(){
  Wire.begin();
  Wire.setClock(100000); // Défini la vitesse d'échange avec les modules en I2C à 100kHz
  Serial.begin(9600); // défini la vitesse d'échange entre Arduino <-> PC à 9600 bauds = 9600 bits par secondes
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
    modeManuel();
  }

  else if (mode == 2){
    modeAutonome();
  }

  // Si le mode est en obstacle manuel
  else if (mode == 3) {
    modeObstacleManuel();
  }

  // Mise à jour des moteurs (ajustement de la vitesse des deux moteurs afin de changer la direction, la vitesse du
  // robot, ou corriger la dérive)
  //
  // L'intervalle de rafraichissement est définie par la variable "interval", calculée par 28.9 * (1.73 / coef) où :
  // - coef est le coéficient de la vitesse du robot. C'est lui qui va permettre au robot d'aller plus ou moins vite.
  // Mais si ce même coef est haut, plus la vitesse est haute, et plus il faut faire de mise à jour pour corriger la
  // dérive ou éviter des obstacles.
  //
  // C'est donc pour cela que je l'inclus dans le calcul de l'intervalle
  // 28.9 est une constante que j'ai choisis arbitrairement afin de faire en sorte que quand coef = 1 (vitesse moyenne),
  // cela donne 100 ms d'intervalle entre chaque MAJ des moteurs.
  //
  // Exemple : coef = 1, alors interval = 28.9 * (1.73 / 1) = 28.9 * 1.73 ~= 49.997 (ms), soit environ 20 MAJ/sec
  // Autre exemple : coef = 1.73 (vitesse maximale), alors interval = 28.9 * (1.73 / 1.73) = 28.9 * 1 = 28.9 (ms), soit
  // environ 34.6 MAJ/sec
  //
  // On voit bien que plus le coef est haut, moins l'intervalle est grande, et donc plus il y a de mises à jour par
  // seconde.
  unsigned long currentMillis = millis(); // Temps actuel
  if (currentMillis - previousMillis >= interval) {
    // On demande 16 octets (8 × 2)
    Wire.requestFrom(SLAVE_ADDR, (uint8_t)8);

    for (uint8_t i = 0; i < 8; i++) {
      if (Wire.available()) {
        distances[i] = Wire.read();
      }
    }
    avG = distances[0];
    av = distances[1];
    avD = distances[2];

    // Mise à 80 pour prendre en compte seulement les capteurs de devant
    // d = 80;
    // arD = 80;
    // ar = 80;
    // arG = 80;
    // g = 80;
    d = distances[3];
    arD = distances[4];
    ar = distances[5];
    arG = distances[6];
    g = distances[7];

    // Affichage des valeurs des capteurs
    // Serial.print(distances[0]);
    // Serial.print(" ");
    // Serial.print(distances[1]);
    // Serial.print(" ");
    // Serial.print(distances[2]);
    // Serial.print(" ");
    // Serial.print(distances[3]);
    // Serial.print(" ");
    // Serial.print(distances[4]);
    // Serial.print(" ");
    // Serial.print(distances[5]);
    // Serial.println(" ");
    // Serial.print(distances[6]);
    // Serial.println(" ");
    // Serial.print(distances[7]);
    // Serial.println(" ");

    previousMillis = currentMillis;  // Met à jour le dernier temps de lecture
    enc1Prec = enc1;
    enc2Prec = enc2;
    enc1 = lireEncodeur(ENC1A);
    enc2 = lireEncodeur(ENC2A);
    mettreAJourMoteursDerive(coef, enc1, enc2, enc1Prec, enc2Prec);
  }

}