# Tennis Ball Collector

[![collecte-balle-ros-foxy](https://github.com/FLEDJ-Inge/CollecteBalle/actions/workflows/collecte-balle-ros-foxy.yml/badge.svg)](https://github.com/FLEDJ-Inge/CollecteBalle/actions/workflows/collecte-balle-ros-foxy.yml)

## Lancer la simulation

### Dépendences

###### A compléter avec la/les dépendences.


### Démarrer la simulation
Dans la racine du workspace (ws), lancer:
```bash
    colcon build
    source ./install/setup.sh
    ros2 launch tennis_court tennis_court.launch.py
```


## Groupe

### Membres

- Florian
- Laurent
- Ermance
- Danut
- Jakub

### Gestion de projet

- [Taiga](https://tree.taiga.io/project/xqubics-fledj-inge/)



## Structure du dépôt

Ce dépôt doit être cloné dans le dossier `src` d'un workspace ROS 2.
Le dossier ws/src contient trois packages : tennis_court, top_camera et robot_control

### Package `tennis_court`

Le dossier `ws/src/tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié.
Consulter le [README](src/tennis_court/README.md) du package pour plus d'informations.

### Package `top_camera`

Le dossier `ws/src/top_camera` est un package ROS contenant le processing des données de la `zenith_camera` - la caméra placée au dessus du terrain de tennis.
Les codes qu'elles contient permettent donc, en s'abonnant à la caméra, de détecter le robot et les balles, et ainsi de detreminer le chemin à suivre.
Consulter le [README](top_camera/README.md) du package pour plus d'informations.

### Package 'robot_control'

Le dossier 'ws/src/robot_control' est un package ROS contenant les codes de control du robot.
Consulter le [README](robot_control/README.md) du package pour plus d'informations.

### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow_fork.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.

_________

### Installs
for `top_camera` pkg:
```sh
    sudo apt install ros-foxy-vision-opencv ros-foxy-cv-bridge
    rosdep install -i --from-path src --rosdistro foxy -y -r
```
<!-- sudo apt install ros-humble-ament-pycodestyle -->


### Testing
```sh
    make test
```
*Note: You should be in the root folder, where the Makefile is.*
