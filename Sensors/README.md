# Utilisation des capteurs : 
Le robot est équipé de trois types de capteurs : 
**TOFs** : Des capteurs de distance TOFs (VL53L1X) sont disposés sur la carosserie du véhicule   
<img src="https://github.com/Intelligent-Systems-MSc/cva-su/blob/main/Sensors/TOFS.png" width="600">  
  
**Caméra** : Une caméra (raspicam V2) est placée à l'avant du robot  
<img src="https://github.com/Intelligent-Systems-MSc/cva-su/blob/main/Sensors/CAMERA.png" width="600">  
  
**LIDAR** : (RPLidar A2)  
<img src="https://github.com/Intelligent-Systems-MSc/cva-su/blob/main/Sensors/LIDAR.png" width="600">  



### Branchement des capteurs : 
Avant toute manipulation informatique des capteurs, il faut vérifier leur branchement :   
  
**Caméra** : La caméra est reliée via une nappe au port CSI du raspberry PI. Attention au sens de branchement de la nappe (les pins conducteurs des deux extremités de la nappe doivent être dans le bon sens, c'est à dire en contact avec les pins conducteurs de la caméra et du raspberry respectivement).   
  
**Lidar** : Le LIDAR dispose d'un module de connexion spécial. Deux cables usb sont disponibles. L'un d'eux se connecte à un des 4 ports USB du Raspberry Pi, et l'autre à une source d'alimentation exterieure (batterie portable). L'identité de ces câbles est renseignée sur cette interface de connexion.   
  
**TOFs** : Les TOFs utilisent une connectique I2C pour se lier au GPIO du rasberry pi. Pour ce faire, les TOFs se connectent selon le schéma suivant : 
<img src="https://github.com/Intelligent-Systems-MSc/cva-su/blob/main/Sensors/TOFS_branchements.png" width="600">  
  
Les pins SDA et SCL sont **obligatoirement** reliés aux ports GPIO2 et 3 du raspberrypi (ils sont par défauts réservés à cette usage). 

### Lancement des acquisitions : 
Si tous les capteurs sont correctement installés, il est possible d'utiliser le launchfile **main_pub.launch** pour démarrer l'acquisition et la publication des données issue des différents capeturs dans l'environnement ROS du robot. Pour ce faire, il faut utiliser la commande : $>>$ roslaunch main_publisher main_pub.launch
  
Une fois ce script lancé, un roscore est généré et **les capteurs s'initialisent puis commencent leurs acquisitions**. Pour la caméra, des paramètres de résolution d'image et de vitesse d'acquisition oeuvent être réglés dans le fichier *main_pub.launch*.   
   
Une fois le script main_pub.launch lancé, il est possible d'entrer différentes commandes : 
- '**quit**+ENTRÉE' ou '**q**+ENTRÉE' : arrête le véhicule et la publication des données issues des capteurs
- '**stop**+ENTRÉE' ou '**s**+ENTRÉE' : arrête le véhicule seulement
- '**start**+ENTRÉE' ou '**d**+ENTRÉE' : redémarre le véhicule   
  
Des exemples de noeuds ROS souscrivant à ces topics sont disponibles : [subscribers](https://github.com/Intelligent-Systems-MSc/cva-su/tree/main/Sensors/main_publisher). 
