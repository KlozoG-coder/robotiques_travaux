Labo 03 

Description

Ce laboratoire consiste à programmer le robot Makeblock Ranger pour effectuer un parcours autonome avec des déplacements précis.

Matériel utilisé

- Robot Makeblock Ranger
- Encodeurs des moteurs
- Gyroscope
- Capteur ultrasonique
- DEL RGB

Fonctionnement

Le robot doit :

1. Attendre 3 secondes avant de démarrer.
2. Avancer d'une distance déterminée.
3. Effectuer un pivot de 90° à gauche.
4. Avancer jusqu'à 30 cm d'un obstacle.
5. Effectuer un pivot de 90° à droite.
6. Avancer de nouveau.
7. Effectuer un pivot de 180°.
8. Faire clignoter les DEL pour signaler la fin.

Programmation

Le programme utilise :

- Une machine à états (FSM).
- "millis()" pour gérer le temps.
- Les encodeurs pour mesurer les déplacements.
- Le gyroscope pour corriger la trajectoire.

Avancement

- [ ] Création du projet
- [ ] Machine à états
- [ ] Calcul des ticks
- [ ] Déplacements et pivots
- [ ] Gestion des DEL
- [ ] Tests du parcours complet


