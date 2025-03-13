# librairie arduino :
- https://github.com/jostlowe/Pico-DMX
- https://github.com/arduino-libraries/ArduinoDMX
paramètre -> url de gestionnaire de carte supplémentaire
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
modèle 3D en 3MF et code arduino ci-joint

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

# Heures travail Sullivan NAGLER 
arrivée 10h25 le 13_03_2025
