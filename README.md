# contenu du git
- lien schematique et PCB :
  - [https://oshwlab.com/sulli.nagler/dmx-rp2040-pico-board-for-2-motors](https://oshwlab.com/sulli.nagler/dmx-rp2040-pico-board-for-2-motors)
- histoirique des modifications physique et logiciel
- modèle 3D en 3MF du potentiomètre de retour
- code arduino .ino
# librairie et gestion de carte sur arduino IDE (à suivre dans l’ordre) 
## librairie
- Pico-DMX : https://github.com/jostlowe/Pico-DMX
- ArduinoDMX : https://github.com/arduino-libraries/ArduinoDMX
## gestion des cartes
- paramètre -> url de gestionnaire de carte supplémentaire
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
- gestionnaire de cartes :
  - https://github.com/earlephilhower/arduino-pico
# Hardware
### Module potentiomètre :
https://fr.rs-online.com/web/p/codeurs/8427197P
https://docs.rs-online.com/c4ad/0900766b8169acb1.pdf

## Interupteurs modification allimentations
1. interrupteur modification allimentation rasberry pico
  - position haute, allimentation 3.3V
  - position basse, allimentation 5V
2. interrupteur modification allimentation déconnection de l’allimentation sans interruption (batterie li-ions)
  - position haute passage, passage de l’allimentation par l’allimentation sans interruption, régulateur batterie->3.3V
  - position basse, ne passe pas par celle ci et utilise le régulateur 5V->3.3V du circuit imprimé (et non celui du RSB PICO)

### liste des problèmes du 13_03_2025
1. moteur 1  fait un bruit anormal (comme une difficulté 
  1. face jardin  (avec boitier 5 unité de remplacement)
  2. loin jardin module 1 (boitié corrrespondant)
3. moteur 1 face cours (gros) : bruit différent de l’original mais pas anormal (avec boitier 4 face cours)
4. changer l’allimentation 24V 3A du kit de dépanage pour une 24V 6A aliexpress (ou 24V 4A de meanwell)
#### liste des cartes flashé avec le code
##### flashé d’origine
- 1
 DMX -1  R1 0  Max 0  Min 0  pwmIn 0  pwmOut 49  pwm 0  MX 0  HS 0 Inv 0 vit 28500 pFix 1000 R2 0  Min 0  Max 0  pwmIn 0 pwmOut 49  pwm 0  MX 0 HS 0 Inv 0 vit 20000 pFix 1001
- 2
 DMX -1  R1 0  Max 0  Min 0  pwmIn 0  pwmOut 49  pwm 0  MX 0  HS 0 Inv 0 vit 28500 pFix 1000 R2 0  Min 0  Max 0  pwmIn 0 pwmOut 49  pwm 0  MX 0 HS 0 Inv 0 vit 20000 pFix 1001
- 3
 DMX -1  R1 0  Max 0  Min 0  pwmIn 0  pwmOut 49  pwm 0  MX 0  HS 0 Inv 0 vit 28500 pFix 1000 R2 0  Min 0  Max 0  pwmIn 0 pwmOut 49  pwm 0  MX 0 HS 0 Inv 0 vit 20000 pFix 1001
- 4
 DMX -1  R1 0  Max 0  Min 0  pwmIn 0  pwmOut 49  pwm 0  MX 0  HS 0 Inv 0 vit 28500 pFix 1000 R2 0  Min 0  Max 0  pwmIn 0 pwmOut 49  pwm 0  MX 0 HS 0 Inv 0 vit 20000 pFix 1001
- 5
 DMX -1  R1 0  Max 0  Min 0  pwmIn 0  pwmOut 49  pwm 0  MX 0  HS 0 Inv 0 vit 28500 pFix 1000 R2 0  Min 0  Max 0  pwmIn 0 pwmOut 49  pwm 0  MX 0 HS 0 Inv 0 vit 20000 pFix 1001
- A
 DMX -1  R1 0  Max 0  Min 0  pwmIn 0  pwmOut 49  pwm 0  MX 0  HS 0 Inv 0 vit 28500 pFix 1000 R2 0  Min 0  Max 0  pwmIn 0 pwmOut 49  pwm 0  MX 0 HS 0 Inv 0 vit 20000 pFix 1001
##### flashé le 13 03 2025
- 6
- 7
- 8
- 9
- B
# Heures travail Sullivan NAGLER 
arrivée 10h25 le 13_03_2025
pause midi à 12h
