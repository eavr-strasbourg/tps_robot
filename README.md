tps_robot
=========

TP de robotique pour Télécom Physique Strasbourg


Infos :
------

Les TPs sont sur un github géré par Olivier Kermorgant et Bernard Bayle.
Pour les récupérer : "git clone https://github.com/eavr-strasbourg/tps_robot.git" depuis un répertoire quelconque.

Prendre soin d'effacer ou d'archiver le répertoire ~/ros. Alors, le script qui se trouve dans tps_robot/scripts/setup/setup_ros.py permet d'installer tout ce qu'il faut (sauf Linux lui-même...) sur une ubuntu 14.04 (a priori aussi sur 16.04 à vérifier). Pour le lancer depuis le répertoire : "python setup_ros.py"

Une fois l'installation faite, il est créé un répertoire ~/ros contenant le sujet de tp... et c'est parti.

Remarques :
-----------

Remarque importante : le script tps_robot/scripts/init_tp.py réinitialise lui l'environnement, idéal pour refaire une virginité au code en début de TP, par contre à ne pas faire si on ne veut pas écraser ses sources...

Autre remarque : Si pb pendant le TP on ne suit pas les consignes avec qtcreator, on s'expose à des soucis avec les CMakeLists. En cas de pb : effacer le CMakeCache du répertoire de compilation indiqué (normalement ros/build).

