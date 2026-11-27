# Robot SUMO sur FPGA (DE0-Nano, Cyclone IV)

Projet du cours « Circuits logiques programmables (FPGA) / VHDL », MKX 3A, IMT Mines Alès.

Le FPGA Cyclone IV E (EP4CE22F17C6N) d'une carte Terasic DE0-Nano pilote un robot mini-sumo dans deux modes, choisis depuis un PC par liaison série :

| Commande | Mode |
|---|---|
| `c` | Combat SUMO : attente réglementaire de 5 s, recherche de l'adversaire, attaque, évitement de la bordure du Dohyo |
| `s` | Suiveur de ligne : course sur la piste « EMA », 2 allers-retours puis arrêt au point de départ |

À la mise sous tension (et après un appui sur BP1), le robot envoie `Robot PRET !` au PC puis attend une commande.

## Fonctionnalités

- Circuit de reset : `rst_b` à '0' pendant 1 ms à la mise sous tension et après un appui sur BP1.
- Horloge système de 500 kHz obtenue par division de l'oscillateur 50 MHz.
- Lecture en boucle des 8 voies de l'ADC128S022 (batterie et 5 capteurs de sol) par l'IP « ADC Controller for DE-series Boards ».
- Machine à états du combat conforme au diagramme du cahier des charges (6 états).
- Machine à états du suivi de ligne : correction par paliers, gestion des virages serrés, demi-tour aux extrémités, arrêt après 4 extrémités.
- Commande des moteurs par PWM 5 kHz sur les entrées ENABLE du pont en H L293D.
- Surveillance de la batterie : LED7 en PWM proportionnelle à la tension (10 % à 7,2 V, 100 % à 9 V) ; sous 7,2 V, LED7 clignote à 1 Hz et les moteurs sont arrêtés.
- Liaison série 9600 bauds, 8 bits, sans parité, 1 bit de stop (modules UART fournis, commentés).

## Architecture

```
CLOCK_50 ─► circuit_reset ─► rst_b ─────────────────────────────► tous les modules
         └► diviseur_frequence ─► clk_500KHz ───────────────────► domaine 500 kHz
ADC ◄──► adc_ctrl (IP) ─► detection_capteurs ─► sol, bordure, adversaire ─► fsm_combat / suiveur_ligne
                                     └► code batterie ─► batterie_monitor ─► LED7, batt_vide
PC ─► UART_RX ─► selection_mode ─► mode ────────────────► aiguillage ─► pwm_gen x2 ─► moteurs
PC ◄─ UART_TX ◄─ message_demarrage
```

| Module | Horloge | Rôle |
|---|---|---|
| `circuit_reset` | 50 MHz | Reset général de 1 ms, BP1 synchronisé par 2 bascules |
| `diviseur_frequence` | 50 MHz | Horloge système 500 kHz (division par 100) |
| `adc_ctrl` (IP) | 50 MHz | Lecture SPI des 8 voies de l'ADC (SCLK 3,125 MHz) |
| `detection_capteurs` | 500 kHz | Seuil noir / clair, signal de bordure, détection de l'adversaire (capteurs IR synchronisés) |
| `batterie_monitor` | 500 kHz | LED7 (PWM ou clignotement), détection de batterie vide |
| `UART_RX`, `UART_TX` | 500 kHz | Réception et émission série |
| `message_demarrage` | 500 kHz | Envoi de `Robot PRET !\n\r` après chaque reset |
| `selection_mode` | 500 kHz | Choix du mode par les caractères `c` et `s` |
| `fsm_combat` | 500 kHz | Machine à états du combat SUMO |
| `suiveur_ligne` | 500 kHz | Machine à états du suivi de ligne |
| `pwm_gen` | 500 kHz | PWM 5 kHz (moteurs et LED7) |
| `robot_sumo_top` | - | Assemblage structurel, aiguillage des commandes moteurs |

Le détail (schéma des blocs, machines à états, ressources) est dans [`livrables/architecture.pdf`](livrables/architecture.pdf).

## Structure du dépôt

| Dossier | Contenu |
|---|---|
| `src/` | Sources VHDL synthétisables (VHDL-93) |
| `ip/` | IP de l'ADC : configuration (`adc_ctrl.qsys`) et fichiers générés |
| `tb/` | Bancs de test auto-vérifiants et modèle de simulation de l'ADC128S022 |
| `sim/` | Script de simulation ModelSim (`simuler.do`) |
| `constraints/` | Contraintes temporelles (`robot_sumo.sdc`) |
| `quartus/` | Projet Quartus (`robot_sumo.qpf`, `robot_sumo.qsf`) et fichiers de programmation `.sof` / `.jic` |
| `livrables/` | Document d'architecture (HTML et PDF) |

## Prérequis

- Intel Quartus Prime Lite **20.1** avec le support Cyclone IV et ModelSim-Intel FPGA Starter Edition.
- Pilote USB-Blaster installé (`<quartus>/drivers`).
- Un terminal série (PuTTY par exemple).

## Compiler

1. Ouvrir `quartus/robot_sumo.qpf` dans Quartus Prime Lite 20.1.
2. Lancer la compilation complète (Processing > Start Compilation).
3. Résultat attendu : 0 erreur, aucun latch, toutes les marges de timing positives. Les broches sont visibles dans Assignments > Pin Planner.

Ressources utilisées : 753 éléments logiques sur 22 320 (3 %), 495 registres, 22 broches.

## Simuler

1. Ouvrir ModelSim.
2. File > Change Directory vers le dossier `sim/`.
3. Dans la console : `do simuler.do`

Le script compile l'IP de l'ADC, les sources et les 11 bancs de test, les exécute et affiche un bilan (OK / ECHEC). Chaque banc vérifie lui-même ses résultats ; le banc `tb_robot_sumo_top` simule le robot complet (message de démarrage, combat, bordure, batterie vide, reset par BP1, suivi de ligne).

## Programmer et utiliser

1. Tester d'abord le robot avec l'outil de test fourni (`SumoTests.jic`).
2. Programmer la DE0-Nano (Tools > Programmer, mode JTAG) avec `quartus/output_files/robot_sumo.jic` pour une configuration persistante, ou `robot_sumo.sof` pour un essai.
3. Ouvrir le terminal série sur le port COM du module USB/série : 9600 bauds, 8N1, sans contrôle de flux.
4. Mettre le robot sous tension : `Robot PRET !` s'affiche.
5. Envoyer `c` (combat, départ après 5 s) ou `s` (suivi de ligne). BP1 relance le robot en attente.

### LED de la DE0-Nano

| LED | Signification |
|---|---|
| LED7 | Niveau de batterie (luminosité), clignotement 1 Hz = batterie vide |
| LED6, LED5 | Mode : `00` attente, `01` combat, `10` suiveur de ligne |
| LED4 à LED0 | Capteurs de sol voyant du noir (gauche à droite), aide au réglage du seuil |

## Calibration

Chaque module garde ses propres constantes, documentées en tête de fichier (pas de paquetage commun). Les réglages propres à chaque robot sont (recompiler après modification) :

| Constante (fichier) | Valeur par défaut | Réglage |
|---|---|---|
| `SEUIL_NOIR_G` (`detection_capteurs`) | 2048 | LED0 à LED4 allumées sur noir et sur le gris des lignes de départ, éteintes sur la bordure beige et sur le blanc |
| `DIR_AVANT_GAUCHE`, `DIR_AVANT_DROIT` (`robot_sumo_top`) | '1', '1' | En recherche, la roue gauche doit avancer et la droite reculer ; inverser la constante de la roue fautive |
| `VITESSE_*` (`fsm_combat`, `suiveur_ligne`) | 100 / 40 / 70 et 60 / 25 % | Attaque, recherche, recul ; suivi, roue intérieure en virage |
| `BORDURE_MAINTIEN_CYCLES_G` (`robot_sumo_top`) | 150 000 (300 ms) | Durée du recul après détection de la bordure |
| `FIN_LIGNE_CYCLES_G` (`robot_sumo_top`) | 100 000 (200 ms) | Durée de blanc considérée comme une extrémité de ligne, à ajuster sur la piste EMA |

## Matériel

- Carte Terasic DE0-Nano (Cyclone IV E EP4CE22F17C6N, oscillateur 50 MHz).
- Robot mini-sumo Sumovore et sa carte d'interface FPGA (connecteur 2x13 JP3).
- ADC128S022 (8 voies, 12 bits, SPI) intégré à la DE0-Nano.
- Pont en H L293D, capteurs de sol QRD1114, capteurs d'obstacle PNA4602, module USB/série FT232R.
- Alimentation : 6 piles AA (9 V nominal).
