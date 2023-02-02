# Utilisation des actionneurs du véhicule :
Le robot est équipé de deux actionneurs : 
- Un **moteur DC** 10A pour la propulsion de la voiture 
- Un **servomoteur** pour sa direction   
  
Une carte électronique analogique de commande en tension des actionneurs a été réalisée. Son utilisation est décrite ci-dessous. 

### Commande en tension du moteur : 
Cette carte convertit un signal PWM issu d'une carte Arduino pour créer une tension continue suffisemment puissante pour pouvoir piloter le moteur. La carte se branche selon le schéma suivant : 
<img src="https://github.com/Intelligent-Systems-MSc/cva-su/blob/main/VehicleControl/ESC_Pinout.png" width="600">  
  
Le bouton poussoir permet de couper l'alimentation du moteur et ainsi préserver la batterie et protéger les composants lors des phases d'inutilisation du véhicule. Si le circuit est sous-tension, la LED est allumée. 
  
Note :  Attention, cette carte est assez fragile : prière de couper l'alimentation (batterie 7.2V) dès que le véhicule n'est pas utilisé. De plus, le transistor de puissance peut être amené à chauffer ... attention à ne pas se brûler !   
   
On passe en entrée de ce module un signal PWM généré par une carte **Arduino**. Le code qui tourne sur la carte Arduino souscrit aux topics **/SpeedCommand** et **/AngleCommand** dans lesquels sont publiés depuis le raspberry pi des commandes de vitesse et de direction (*control_vehicle.ino*).   
   
Note :  Pour utiliser ce circuit de commande, les données publiées depuis le raspberry pi vers ces deux topics doivent être comprises entre -1 et 1 : 
  
$\Rightarrow$ **/SpeedCommand** : 
- $-1$ = vitesse max en marche arrière 
- $1$ = vitesse max en marche avant 
- $0$ = vitesse nulle -> véhicule à l'arrêt  
    

$\Rightarrow$ **/AngleCommand** : 
- $-1$ = Direction des roues au max à gauche 
- $1$ = Direction des roues au max à droite 
  
Pour démarrer l'Arduino, il suffit de taper la commande '**arduino_ros**' dans un terminal du raspberry pi (l'Arduino doit être relié à ce dernier via un cable USB, sur l'un des 4 ports USB de la carte) :
