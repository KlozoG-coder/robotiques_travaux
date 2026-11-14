# Labo 04 - PID, Bluetooth et capteur de ligne

Robot Makeblock Auriga (Arduino Mega 2560). Bibliotheques : MeAuriga, Adafruit_seesaw.

## Fichiers
| Fichier | Role |
|---|---|
| `labo_04.ino` | `setup()` / `loop()` |
| `Robot.h/.cpp` | Machine a etats (BOOT, ATTENTE, LAB3, CALIB, SUIVRE_LIGNE, CMD_INVALIDE, NON_CALIB) et commandes |
| `Lab3.h/.cpp` | Materiel (moteurs, gyro, ultrason, DEL) + parcours du labo_03 (etat LAB3) |
| `Calibration.h/.cpp` | Lecture du LyneTracker, min/max, drapeau de calibration |
| `SuiviLigne.h/.cpp` | Position de la ligne, PID, detection de la ligne |
| `Configuration.h` | Tous les reglages (vitesses, PID, seuils, parcours du labo_03) |

## Commandes
`stop`, `lab3`, `cal`, `ligne` (toute autre commande : `cmd INCONNUE`, DEL rouges 3 s).
`ligne` sans calibration : DEL jaunes 3 s (NON_CALIB), puis retour en ATTENTE.

## Reglages utiles (Configuration.h)
- `COMMANDES_DEBUG` : 1 = commandes de mise au point (sensg, sensd, pidinv, noirinv, diag, avance). 0 pour la remise.
- `LAB3_DELAI_DEPART_MS` : attente de 3 s au debut de lab3 (comme le labo_03).
- `SUIVI_*` : vitesse, fin de ligne, recherche de ligne.
