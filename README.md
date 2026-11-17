# Suivi de trajectoire sur TurtleBot3 Burger réel — ROS 2 Humble

Cette version cible le **Burger standard avec ses XL430-W250 et le firmware
ROBOTIS**, sans Gazebo. Elle réalise le suivi non linéaire **en vitesse** d'un
cercle ou d'un huit. Le contrôleur dynamique robuste **en couple** du papier de
Zidani et al. n'est pas implémenté sur les moteurs : cette interface n'existe pas
sur les XL430 standard. C'est une adaptation cinématique, pas une reproduction
complète du papier. Ce périmètre correspond au choix de conserver le matériel.

Les XL430 proposent les modes vitesse, position et PWM (tension). `Torque Enable`
active l'actionneur ; ce n'est pas une consigne de couple. `Present Load` est une
estimation interne, pas une mesure de couple. Un calcul de couple converti en
`cmd_vel` ou en PWM ne constituerait pas une régulation de couple démontrée.
Voir le [manuel ROBOTIS](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/).

La [note mathématique](docs/controller.md) précise les équations, les hypothèses
et ce que les diagnostics de Lyapunov permettent réellement de conclure.

## Architecture sur le robot

`tracking_node` lit l'odométrie, calcule le suivi à 20 Hz et publie une commande
`TwistStamped` sur `tracking_cmd_vel`. `command_guard`, **processus séparé à
exécuter sur le calculateur du robot**, vérifie fraîcheur, repère et limites,
puis alimente `cmd_vel` au format attendu par le pilote ROBOTIS. Ce dernier
conserve la régulation de vitesse des moteurs et la connexion OpenCR.

Les deux processus démarrent désactivés. Une commande nulle est publiée tant
qu'ils ne sont pas activés. Ils doivent être les seuls à commander la base :
arrêter téléop, Nav2 et tout autre producteur de `cmd_vel` pendant cet essai.
Aucun port série ni registre moteur n'est écrit directement par ce projet.

Le relais s'arrête si le contrôleur cesse de publier ou si ses commandes arrivent
trop tard. Après ce défaut, un `enable` explicite est requis. Le contrôleur
s'arrête si l'odométrie est absente, ancienne, invalide, rétrograde, si la boucle
prend trop de retard ou si l'erreur dépasse le domaine expérimental configuré.
Une odométrie redevenue valide **ne redémarre pas** le suivi.

Ce relais logiciel ne remplace pas un arrêt matériel. S'il est lui-même tué,
ou si le système/pilote se bloque, il ne peut plus envoyer zéro. Le firmware
OpenCR consulté surveille le heartbeat du pilote ROS : un pilote encore vivant
peut continuer ce heartbeat après la disparition du contrôleur. Il faut vérifier
sur la version de firmware installée le comportement lors de la perte des
commandes et de la liaison. Un watchdog embarqué des commandes serait nécessaire
pour couvrir cette défaillance indépendamment du calculateur.

## Construire

Sur le calculateur avec ROS 2 Humble et le bringup ROBOTIS déjà installés, depuis
la racine de ce dépôt :

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Les paramètres sont dans
[`src/trajectory_controller/config/burger.yaml`](src/trajectory_controller/config/burger.yaml).
Ils sont en lecture seule pendant l'exécution : modifier le YAML puis relancer
les nœuds pour changer les gains ou les limites. Avec `--symlink-install`, vérifier
que le fichier installé reflète bien les changements ; reconstruire si nécessaire.
Les gains proposés sont des valeurs initiales de mise au point, pas des gains
validés expérimentalement sur votre robot.

## Préparer l'interface réelle

Lancer le bringup ROBOTIS habituel dans un premier terminal, par exemple :

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

Dans un second terminal où les environnements ROS et du dépôt sont chargés :

```bash
ros2 topic type /cmd_vel
ros2 topic echo /odom --once
ros2 topic hz /odom
```

Vérifier `header.frame_id=odom` et `child_frame_id=base_footprint`, ou configurer
les noms exacts dans le YAML. Les repères doivent représenter les mêmes points
et axes, pas seulement porter des noms compatibles. Les paramètres officiels du
Burger utilisent une séparation de roues de 0,160 m et un rayon de 0,033 m.
`wheel_radius` ne sert ici qu'aux diagnostics de vitesse angulaire des roues.
Synchroniser les horloges si le contrôleur et le pilote tournent sur des machines
différentes ; `use_sim_time` doit rester à `false`.

Le format dépend de la version ROBOTIS :

| Résultat de `ros2 topic type /cmd_vel` | Argument du lancement |
| --- | --- |
| `geometry_msgs/msg/Twist` | `stamped_cmd_vel:=false` |
| `geometry_msgs/msg/TwistStamped` | `stamped_cmd_vel:=true` |

Le fichier ROBOTIS actuel de la branche Humble active les commandes estampillées ;
des installations plus anciennes attendent `Twist`. Ne choisir le type qu'après
vérification du pilote installé. Le relais refuse son activation si aucun
abonné de type compatible n'est détecté ; cette vérification ne prouve pas à elle
seule que l'abonné est le pilote matériel.

## Lancer, démarrer et arrêter

Exemple pour un pilote attendant `Twist` :

```bash
ros2 launch trajectory_controller hardware.launch.py trajectory:=circle stamped_cmd_vel:=false
```

Pour un huit, utiliser `trajectory:=eight`. Pour une configuration personnelle,
ajouter `params_file:=/chemin/absolu/burger.yaml`. Le lancement utilise
`trajectory:=circle` par défaut : préciser explicitement `trajectory:=eight`
même si un autre type figure dans un YAML personnalisé.

À ce stade **aucun mouvement n'est demandé**. Le robot doit être immobile avant
le démarrage. Dans un autre terminal :

```bash
ros2 service call /command_guard/enable std_srvs/srv/Trigger '{}'
ros2 service call /tracking_node/start std_srvs/srv/Trigger '{}'
```

Lire `success` et `message` pour chaque réponse. Le départ est la pose courante,
sans décalage artificiel. La référence accélère progressivement, et la commande
est bornée en vitesse linéaire, angulaire et vitesse de chaque roue. La marche
arrière reste possible si le correcteur la demande ; elle n'est plus supprimée
silencieusement. Par défaut, l'essai dure 90 s puis s'arrête.

Arrêt demandé par l'opérateur :

```bash
ros2 service call /tracking_node/stop std_srvs/srv/Trigger '{}'
ros2 service call /command_guard/disable std_srvs/srv/Trigger '{}'
```

L'arrêt envoie zéro immédiatement et contourne la rampe d'accélération. Cela
n'implique pas un arrêt mécanique instantané. Après un défaut du relais : appeler
`stop`, résoudre la cause, attendre une odométrie fraîche et le robot immobile,
puis refaire `enable` et `start`. Ne pas activer le relais avec un contrôleur qui
émet déjà une commande non nulle : il refuse volontairement cette situation.

## Observer et qualifier les essais

```bash
ros2 topic echo /tracking_diagnostics
ros2 bag record /odom /tracking_cmd_vel /cmd_vel /tracking_diagnostics /reference_path /actual_path
```

Les diagnostics à 5 Hz contiennent les erreurs `e1_m`, `e2_m`, `e3_rad`, les
vitesses de référence, la commande nominale `*_raw_*`, la commande limitée,
les vitesses mesurées, les indicateurs `speed_limited` et `slew_limited`, l'âge
de l'odométrie, la période de commande, `L`, `Ldot_ideal` et
`Ldot_kinematic_measured`. Les diagnostics sous-échantillonnent la boucle à
20 Hz : utiliser les topics de commande et d'odométrie enregistrés pour une
analyse temporelle à leur fréquence complète. Les chemins sont limités à
2000 poses.

La commande publiée n'est pas forcément la vitesse réalisée. La version Humble
du pilote consultée transmet notamment les consignes linéaire et angulaire à
OpenCR sous forme d'entiers après multiplication par 100 : les petites consignes
peuvent être quantifiées. La friction, les délais et les limites du moteur
s'ajoutent à cette différence. Les diagnostics n'estiment **aucun couple**.

L'odométrie des roues ne constitue pas une vérité terrain : en présence de
patinage, un bon suivi dans `odom` peut masquer une erreur réelle. Pour annoncer
une robustesse expérimentale, mesurer aussi la trajectoire avec une référence
indépendante, puis comparer erreurs RMS/maximales, vitesses, saturation et retards
pour plusieurs essais reproductibles (conditions initiales, charge, sol).

Avant l'essai au sol, vérifier le sens des roues et les services d'arrêt avec les
roues dégagées. Vérifier aussi la coupure du seul contrôleur, puis la perte des
liaisons selon un protocole maîtrisé, avec un moyen d'arrêt matériel accessible.
Le lancement n'implémente ni évitement d'obstacles ni protection anticollision.

## Tests et état de validation

Tests du cœur Python, sans ROS ni robot :

```bash
python3 -m pip install pytest ruff
python3 -m pytest -q
ruff check src
```

Avec ROS 2 et l'overlay compilé, les tests `test_ros_hardware_nodes.py` exercent de
vrais publishers, timers et services, dans un espace de noms aléatoire. Ils
publient de l'odométrie synthétique ; ils ne déplacent pas un robot et n'utilisent
pas de simulateur physique. Pour les isoler du réseau du robot :

```bash
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q -m 'not linter'
```

La CI comprend les tests Python 3.10/3.12 et un job de compilation et de transport
ROS Humble. Le Dockerfile fournit aussi un environnement de compilation Humble,
sans Gazebo (`docker build -f docker/Dockerfile -t tb3-tracking .`). Les contrôles
ament sont disponibles dans l'environnement ROS ; le contrôle copyright initial
reste désactivé car le dépôt ne précise pas encore sa licence.

**Validation effectuée dans l'environnement de développement :** tests unitaires
Python et contrôles statiques. **Non effectués ici :** compilation/exécution ROS,
exécution Docker/CI et essais sur le Burger réel. Les tests ROS sont ignorés quand
`rclpy` est absent. Aucune performance physique ni garantie de robustesse du
robot n'est donc annoncée par ces résultats logiciels.

Il n'est pas nécessaire de porter ce calcul cinématique en C++ à ce stade.
Mesurer d'abord retards, gigue et charge CPU sur le robot. Changer de langage ne
crée ni interface de couple sur les XL430, ni garantie de temps réel.

## Références

- G. Zidani et al., *Robust Nonlinear Control of a Mobile Robot*, JEET 11(4),
  1012–1019 (2016), [DOI](https://doi.org/10.5370/JEET.2016.11.4.1012).
- [Manuel XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/).
- [Paramètres Burger ROBOTIS, branche Humble](https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble/turtlebot3_bringup/param/burger.yaml).
- [Pilote ROS 2 ROBOTIS](https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble/turtlebot3_node/src/turtlebot3.cpp).
- [Firmware OpenCR ROS 2](https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp).
