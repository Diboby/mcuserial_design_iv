ROS <--> MCU Design IV


Voici les commandes pour compiler le code.

catkin_make
catkin_make install


Lorsqu'un nouveau terminal est créé pour exécuter le code, il faut toujours exécuter ce script avant.
source devel/setup.bash



Le code pertinent est dans "src/mcu_interface/mcuserial_lib/mcuserial_pyhon/" et "src/mcu_interface/mcuserial_lib/mcuserial_msgs/".

Plus particulièrement, "mcuserial_node.py" qui est le fichier contenant le noeud démarrant le Service "alim_serial_com". La routine de gestion des requêtes y est également (callback).
"comm_protocol_def.py" défini des variables importantes pour la gestion de la communication série avec le microcontrôleur, notamment pour le protocole de communication orienté registres.
"mainTranslator.py" permet de convertir des requêtes du service, définis dans "alim_serial_com_srv.srv", en bytearray concordant au protocole de communication série orienté registres. Il permet également de faire l'opération inverse.
"SerialInterface.py" permet d'envoyer des messages et d'en recevoir au/du microcontrôleur, avec la bonne routine pour être robuste au bruit.

Les tests unitaires, décrits dans le manuel technique, sont disponibles dans "src/mcu_interface/mcuserial_lib/mcuserial_pyhon/test/".
