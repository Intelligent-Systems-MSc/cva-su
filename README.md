# Course de Voiture autonome - Sorbonne Université

Ce dépot à pour but de partager les avancées du groupe de sorbonne université sur la course de voiure autonome. Le modèle 3D du robot, les vidéos et les documentations sont disponibles dans ce Google Drive : https://drive.google.com/drive/folders/1IN_bX1qWm7_AuP1AxILJaJX2wpmhlquH?usp=sharing  
  
<img src="https://github.com/Intelligent-Systems-MSc/cva-su/blob/main/vehicle.png" width="600">  
  

## Contexte
Le but de ce projet est de réaliser un robot autonome compétitif à partir d'un châssis de voiture 
télécommandée au format 1/10ème. Ce robot sera engagé dans une **compétition de véhicules 
autonomes** organisée par une équipe enseignante de l'**ENS Paris Saclay**. Ce concours aura lieu le
**15 Avril 2023** et réunira plusieurs équipes d'étudiants issus de différentes formations (écoles 
d'ingénieurs, facultés…).  
  
Notre équipe de projet est constituée de 4 étudiants du M2 SAR (**Justin BRUGIERE**, **Alexandre 
BOTTRAUD**, **Omar EL OUAFI** et **Montacir DRISSI**) et 2 étudiants du M2 ISI (**Yuwang CHEN** et **Tom 
DA SILVA-FARIA**). Ce projet est encadré par **Mr. Sylvain ARGENTIERI**.   
  
## Utilisation du dépôt Git : 
Ce dépôt regroupe l'ensemble des travaux effectués par les différents membres du groupe au cours du projet. Ces travaux sont regroupés par thématiques dans différents dossiers (simulateur, commande des actionneurs, utlisation des capteurs ...). 

## Utilisation générale du Raspberry Pi : 
Le robot est équipé d'un raspberry Pi 3B+ en tant qu'ordinateur de bord. Un raspbian Buster tourne dessus, avec ROS Noetic installé et prêt à l'emploi.    
    
**Important** : Pour utiliser le robot, vous aurez besoin d'une machine sous Ubuntu 20.04 avec ROS noetic correctement installé (http://wiki.ros.org/noetic/Installation/Ubuntu). Il est également possible d'utiliser une machine virtuelle, mais cette solution n'est pas conseillée (car vraiment pas otimale), donc elle ne sera pas détailléé dans ce document (pour le moment).   
  
**Connexion au Rapsberry Pi** : Pour pouvoir manipuler les fichiers (fichiers de configuraton, scripts pythons, packages ROS...), deux options sont possibles : 
- 1 : connecter le raspberry pi à un écran des salles de TP (+ clavier et souris) et l'utiliser comme un ordinateur ordinaire (Attention : Pour ce faire, il faudra d'abord relier l'écran à un des ports HDMI de la carte avant de la mise sous tension). 
- 2 : **se connecter en ssh** au raspberry pi (ce sera évidemment cette option qui sera utilisée lors de l'utilisation du véhicule...). Pour ce faire, le raspberry pi a été configuré de sorte à générer un **réseau Wi-Fi Hotpot** (ssid : **RPIHOTSPOT** – mot de passe : **autonome**). **Sur une machine connectée à ce réseau**, il est possible d'entrer, depuis un terminal, la commande suivante : $>>$ **ssh pi@192.168.50.5** pour se connecter en ssh au raspberry pi (mot de passe : **autonome**)
  
En ssh, pour transférer un fichier depuis le PC vers le raspberry pi : **dans un terminal du PC**, entrer la commande :  
$>>$ **scp /Chemin/Dans/PC/Fichier.txt pi@192.168.50.5:/Chemin/Vers/Fichier.txt**   
  
De même, en ssh,  pour transferer un fichier depuis le raspberry pi vers le PC : entrer la commande :   
$>>$**scp pi@192.168.50.5:/Chemin/Vers/Fichier.txt /Chemin/Dans/PC/Fichier.txt**
  
Pour transferer un dossier entier, il suffit d'ajouter l'option "-r" juste aorès le mot clef "scp" 

## Simulateur : 
Nous avons mis en place en environnement de simulation sous WeBots ([simu-webots](https://github.com/Intelligent-Systems-MSc/cva-su/tree/main/SimuWebots)). Ce simulateur permet de tester des algorithmes de navigation dans un environnement ROS similaire à celui du robot réel (mêmes capteurs avec leurs limitations et bruits, mêmes topics ROS, mêmes fréquences de publication des données ...). 

## Docker : 
Des images docker ont été créées afin de lancer WeBots ainsi que ROS sur n'importe quelle machine. Pour le moment, ces fichiers sont disponibles sur le dépot suivant: [ros-webots-docker](https://github.com/Teiwin/ros-webots-docker)

## Commande en tension : 
Une carte électronique de commande en tension des actionneurs du véhicule a été réalisée et est documentée ici :[control-vehicle](https://github.com/Intelligent-Systems-MSc/cva-su/tree/main/VehicleControl). Une carte Arduino est utilisée pour contrôler les actionneurs et son code est également disponible dans ce dépôt. 

## Capteurs : 
Le robot est équipé de différents capteurs. Ces capteurs publient leurs données dans l'environnement ROS du robot, et sont documentés ici : [sensors](https://github.com/Intelligent-Systems-MSc/cva-su/tree/main/Sensors)
