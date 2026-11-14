
#include "SuiviLigne.h"

static const float SEUIL_BRUIT_NOIR      = 100.0f;
static const float SEUIL_INTENSITE_LIGNE = 150.0f;
static const float SEUIL_POIDS_LIGNE     = 20.0f;

SuiviLigne::SuiviLigne() {
    calibration = nullptr;

    kp = 0.026f;
    ki = 0.0f;
    kd = 0.00012f;

    integrale = 0;
    erreurPrecedente = 0;
    dernierPID = 0;

    positionLigne = 0;
    correction = 0;

    positionFiltree = 0;
    deriveeFiltree = 0;
    premiereLecture = true;

    ligneDetectee = false;
}

void SuiviLigne::initialiser(Calibration* capteur) {
    calibration = capteur;
}

void SuiviLigne::commencer() {
    integrale = 0;
    erreurPrecedente = 0;
    dernierPID = millis();

    positionLigne = 0;
    correction = 0;

    positionFiltree = 0;
    deriveeFiltree = 0;
    premiereLecture = true;

    ligneDetectee = false;
}

void SuiviLigne::actualiser() {
    if (calibration == nullptr || !calibration->estCalibre()) {
        correction = 0;
        ligneDetectee = false;
        return;
    }

    // Lire les cinq capteurs
    calibration->lireCapteurs();

    float numerateur = 0;
    float denominateur = 0;

    float intensiteMax = 0;

    for (int i = 0; i < NB_IR; i++) {
        int minimum = calibration->getMin(i);
        int maximum = calibration->getMax(i);

        // Ignorer une calibration insuffisante
        if (maximum - minimum < 50) {
            continue;
        }

        float valeur = calibration->getValeur(i);

        // Normalisation entre 0 et 1000
        float normalisee =
            (valeur - minimum) * 1000.0f /
            (maximum - minimum);

        normalisee = constrain(
            normalisee, 0.0f, 1000.0f
        );

        float noir = inverserNoir ? normalisee : (1000.0f - normalisee);

        if (noir < SEUIL_BRUIT_NOIR) {
            noir = 0;
        }

        if (noir > intensiteMax) {
            intensiteMax = noir;
        }

        float poids = noir * noir / 1000.0f;
        numerateur += poids * (i - 2) * 1000.0f;
        denominateur += poids;
    }

    if (intensiteMax < SEUIL_INTENSITE_LIGNE ||
        denominateur < SEUIL_POIDS_LIGNE) {

        ligneDetectee = false;
        correction = 0;

        integrale = 0;
        deriveeFiltree = 0;
        premiereLecture = true;
        dernierPID = millis();

        return;
    }

    ligneDetectee = true;


    float positionBrute =
        numerateur / denominateur;

    const float alpha = 0.95f;

    if (premiereLecture) {
        positionFiltree = positionBrute;
    } else {
        positionFiltree =
            alpha * positionBrute +
            (1.0f - alpha) * positionFiltree;
    }

    positionLigne = positionFiltree;

    unsigned long maintenant = millis();

    float dt =
        (maintenant - dernierPID) / 1000.0f;

    if (dt <= 0.0f) {
        return;
    }

  
    float erreur = positionLigne;

    integrale += erreur * dt;

    integrale = constrain(
        integrale, -1000.0f, 1000.0f
    );

    float derivee = 0;

    if (!premiereLecture) {
        derivee =
            (erreur - erreurPrecedente) / dt;
    }

    const float alphaD = 0.3f;

    deriveeFiltree =
        alphaD * derivee +
        (1.0f - alphaD) * deriveeFiltree;

    correction =
        kp * erreur +
        ki * integrale +
        kd * deriveeFiltree;

    correction = constrain(
        correction, -50.0f, 50.0f
    );

    erreurPrecedente = erreur;
    dernierPID = maintenant;
    premiereLecture = false;
}

float SuiviLigne::getPosition() {
    return positionLigne;
}

float SuiviLigne::getCorrection() {
    return correction;
}

bool SuiviLigne::estLigneDetectee() {
    return ligneDetectee;
}

void SuiviLigne::inverserNoirBlanc() { inverserNoir = !inverserNoir; }
