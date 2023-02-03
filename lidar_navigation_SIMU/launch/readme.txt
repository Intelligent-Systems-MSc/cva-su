explication des paramètres utilisés
<param name="servo_zero" value="0" /> ## Valeur pour le servo : aller à droit
<param name="servo_left" value="-1" />  #Valeur pour le servo : max turn left
<param name="servo_right" value="1" /> # Valeur pour le servo : max turn right
<param name="left_limite" value="-0.6" />  #Limites d'angle au delà desquelles on décide de braquer à fond 
<param name="right_limite" value="0.6" />
<param name="esc_vmax" value="1" /> #Max pour vitesse 
<param name="esc_vmin" value="0.2" /> #Min pour vitesse
<param name="seuil_distance" value="2.0" /> #Limites de la distance au dessus desquelles on a la vitesse maximale
<param name="seuil_obstacle" value="0.2" /> #Limites de la distance au dessous desquelles on a une correction de l'angle pour éviter l'obstacle
<param name="seuil_voisin" value="1" />  #Limites de la distance au dessous desquelles on n'a pas assez espace pour passer le véhicule
<param name="frequence_nav" value="1" />
<param name="topic_data" value="/ScannerData"/>
