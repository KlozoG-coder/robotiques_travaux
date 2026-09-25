#include <MeAuriga.h>

// LED
#define LEDNUM 12
#define LEDPIN 44

MeRGBLed led(PORT0, LEDNUM);

// SONAR
MeUltrasonicSensor sonar(PORT_10);

int distance = 150;

unsigned long tempsSonar = 0;
unsigned long tempsAffichage = 0;

const unsigned long intervalleSonar = 100;
const unsigned long intervalleAffichage = 250;

// Moteur gauche
const int m1_pwm = 11;
const int m1_in1 = 48;
const int m1_in2 = 49;

// Moteur droit
const int m2_pwm = 10;
const int m2_in1 = 47;
const int m2_in2 = 46;

const int vitesseCroisiere = 180;

const int vitesseLente = 90;

const int vitessePivot = 150;

const int distanceRalentir = 100;
const int distanceDanger = 30;

const unsigned long tempsArret = 2000;
const unsigned long tempsRecul = 1000;

const unsigned long tempsPivot = 800;

// ETATS
enum Etat {
  AVANCE,
  ARRET,
  RECULE,
  PIVOTE
};

Etat etat = AVANCE;

unsigned long debutEtat = 0;

void setup() {

  Serial.begin(115200);

  led.setpin(LEDPIN);

  // Moteur gauche
  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);

  // Moteur droit
  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);

  Stop();
}

void loop() {

  unsigned long maintenant = millis();

  lireSonar(maintenant);

  afficherDistance(maintenant);

  gererRobot(maintenant);
}

// LECTURE DU SONAR toutes les 100 ms
void lireSonar(unsigned long maintenant) {

  if (maintenant - tempsSonar < intervalleSonar) {
    return;
  }

  tempsSonar = maintenant;

  int nouvelleDistance = sonar.distanceCm();

  // Conserver uniquement une mesure valide
  if (nouvelleDistance > 0 &&
      nouvelleDistance <= 400) {

    distance = nouvelleDistance;
  }
}

// AFFICHAGE DISTANCE toutes les 250 ms
void afficherDistance(unsigned long maintenant) {

  if (maintenant - tempsAffichage < intervalleAffichage) {
    return;
  }

  tempsAffichage = maintenant;

  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.println(" cm");
}

// GESTION DU ROBOT
void gererRobot(unsigned long maintenant) {

  switch (etat) {

    case AVANCE:

      // Objet à moins de 30 cm
      if (distance < distanceDanger) {

        Stop();
        rouge();

        etat = ARRET;
        debutEtat = maintenant;
      }
      // Objet entre 30 et 100 cm
      else if (distance < distanceRalentir) {

        Forward(vitesseLente);
        jaune();
      }
      // Aucun objet à moins de 100 cm
      else {

        Forward(vitesseCroisiere);
        vert();
      }

      break;

    case ARRET:

      Stop();
      rouge();

      if (maintenant - debutEtat >= tempsArret) {

        etat = RECULE;
        debutEtat = maintenant;
      }

      break;

    case RECULE:

      Backward(vitesseLente);
      rouge();

      if (maintenant - debutEtat >= tempsRecul) {

        Stop();

        etat = PIVOTE;
        debutEtat = maintenant;
      }

      break;
      
    case PIVOTE:

      TurnRight(vitessePivot);
      rouge();

      if (maintenant - debutEtat >= tempsPivot) {

        Stop();

        etat = AVANCE;
        debutEtat = maintenant;
      }

      break;
  }
}

// AVANCER
void Forward(int vitesse) {

  // Moteur gauche
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, vitesse);

  // Moteur droit
  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, vitesse);
}

// RECULER
void Backward(int vitesse) {

  // Moteur gauche
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, vitesse);

  // Moteur droit
  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, vitesse);
}

// ARRETER
void Stop() {

  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);
}

// PIVOTER À DROITE Sens horaire
void TurnRight(int vitesse) {

  // Moteur gauche vers l'avant
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, vitesse);

  // Moteur droit vers l'arrière
  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, vitesse);
}

// VERT : moitié avant
void vert() {

  // Éteindre toutes les DEL
  led.setColor(0, 0, 0);

  // DEL 1 à 6 vertes
  for (int i = 1; i <= 6; i++) {
    led.setColor(i, 0, 10, 0);
  }

  led.show();
}

// JAUNE : moitié arrière
void jaune() {

  // Éteindre toutes les DEL
  led.setColor(0, 0, 0);

  // DEL 7 à 12 jaunes
  for (int i = 7; i <= 12; i++) {
    led.setColor(i, 10, 10, 0);
  }

  led.show();
}

// ROUGE : anneau complet
void rouge() {

  led.setColor(10, 0, 0);

  led.show();
}