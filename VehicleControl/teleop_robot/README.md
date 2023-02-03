# Télé-opération : 

**Pré-requis** : Vous devrez disposer d'une installation fonctionelle de **ROS Noetic** sur votre machine. Le package ROS de télé-opération est disponible sur le Github ([teleop_robot](https://github.com/Intelligent-Systems-MSc/cva-su/tree/main/VehicleControl/teleop_robot))  
  
Téléchargez ce package ROS et placez le dans votre workspace ROS (dans le sous dossier /src), puis compilez votre workspace (si vous ne savez pas comment faire, référez-vous aux tutoriels ROS (http://wiki.ros.org/fr/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
pour prendre en main son fonctionnement).

## Installation du véhicule : 
Pour télé-opérer le robot, aucun capteur ne sera nécéssaire. Les branchements suivants doivent être effectués :   
<img src="https://github.com/Intelligent-Systems-MSc/cva-su/blob/main/VehicleControl/teleop_robot/teleop.png" width="600">  
  
Une fois que tous ces branchements sont effectués, mettez le raspberry pi sous tension (en branchant la prise d'alimentation du Pi à la batterie portable – il faut attendre environ 30s que le système d'exploitation démarre).  ATTENTION : Pour le moment, ne branchez pas la batterie 7.2V du moteur (la LED d'alimentation doit être éteinte). 

## Connexion au véhicule : 
Une fois que le raspberry pi est allumé, un nouveau réseau wifi devrait apparaître dans la liste des réseaux disponibles sur votre machine (si ce n'est pas le cas, redémarrez le wifi sur votre PC). Connectez votre machine a ce nouveau réseau : 
- nom du réseau (SSID) : **RPIHOTSPOT**
- mot de passe : **autonome**
  
Pour la téléopération, vous aurez également besoin de connecter l'environnement ROS de votre machine à celui de robot. Pour ce faire, éditez le fichier *__.bashrc__* de votre PC de la manière suivante : 
$>>$ **cd**
$>>$ **nano .bashrc**
Ajoutez à la fin du fichier les deux lignes suivantes : 
*export ROS_MASTER_URI=http://192.168.50.5:11311*    
*export ROS_HOSTNAME=192.168.50.175*    
  
Le champ ROS_HOSTNAME doit correspondre à l'adresse IpV4 de votre PC connecté au réseau Hotspot du raspberry Pi. Vous pouvez vérifier cette adresse IP dans le menu de configuration du réseau wifi de votre machine.
  
**Fermez votre terminal et ouvrez en un nouveau** (sinon, les changements ne seront pas effectués). Votre PC se connectera automatiquement au ROS master du robot, à condition que vous ayez au préalable connecté votre machine à son réseau Wifi. Pour cesser d'utiliser le ROS master du robot, retirez les deux lignes précédemment ajoutées au fichier *.bashrc*. 
    
Pour finir, connectez vous en ssh à deux terminaux du raspberry pi depuis votre PC. Pour ce faire, dans un terminal de votre machine, tapez la commande suivante : 
$>>$ **ssh pi@192.168.50.5**  
  
Le code PIN du raspberry pi vous sera demandé : ce code est le même que celui du réseau hotspot (autonome). Répetez cette opération dans un nouveau terminal pour disposer d'un second terminal dans le raspberry pi. 

Dans ces deux terminaux, vous aurez accès à tout fichier ou package présent sur le raspberry pi (attention à ne rien supprimer/modifier au risque de déteriorer l'OS). 

## Lancement des noeuds ROS : 
Nous aurons ici besoin de deux noeud ROS. Le premier est déjà installé sur le raspberry pi et s'occupera de communiquer les informations de commande des actionneurs à l'Arduino.   
  
Le second peut être téléchargé au lien mentionné plus haut (teleop_robot). Une fois téléchargé et placé dans le bon répertoire de votre PC (~/catkin_ws/src), entrez les commandes suivantes :   
$>>$ **cd catkin_ws/**  
$>>$ **catkin_make**   
$>>$ **source devel/setup.bash**  

$1$ - Dans un premier temps, lancez le **rosmaster** ($>>$ **roscore**) depuis l'un des deux terminaux que vous avez ouvert dans le raspberry pi en ssh.   
  
$2$ - Vous devriez à présent pouvoir lancer le script de télé-opération sur votre machine avec la commande suivante :   
$>>$ **roslaunch teleop_robot teleop_robot.launch**  
  
L'interface graphique de teleop devrait alors s'ouvrir sur votre PC. Une fois le second noeud ROS lancé (voir ci dessous) sur le raspberry Pi, vous pourrez utiliser les flèches de votre clavier pour envoyer des commandes de vitesse (flèches verticales) et de direction (flèches horizontales) au robot.   
  
$3$ – Assurez-vous une nouvelle fois que l'Arduino est bien connecté en USB au raspberry pi. Pour ce faire, vous pouvez par  exemple utiliser la commande $>>$ __ls -l /dev/ttyUSB*__  
  
Si l'Arduino est bien connecté, vous pouvez alors utiliser la commande suivante (toujours dans le même terminal en ssh) : $>>$ **arduino_ros**. L'Arduino souscrit aux topics */SpeedCommand* et */AngleCommand* (ici, */PausePub* n'a pas d'importance) dans lesquels sont publiées les instructions de téléopération.  
  
## Mise sous tension : 
Vous pouvez à présent connecter la batterie du moteur et commander le véhicule depuis votre clavier d'ordinateur (à l'aide de l'interupteur sur le véhicule).   
  
Si le véhicule va trop vite/lentement, vous pouvez modifier sa vitesse maximale dans le fichier *teleop_robot.launch*. De même, vous pouvez augmenter l'angle de braquage maximal depuis ce même fichier. 
  
**ATTENTION** : Prière de ne pas âbimer le véhicule... Il faut à tout prix éviter de bloquer les roues du robot tout en envoyant des commandes d'accélération, au risque de brûler la carte analogique ESC ou le transistor de puissance... 
