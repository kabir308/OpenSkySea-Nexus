# Dépannage - OpenSkySea-Nexus

Guide de résolution des problèmes courants.

## Table des Matières

1. [Installation et Configuration](#installation-et-configuration)
2. [Compilation](#compilation)
3. [Simulation](#simulation)
4. [Docker](#docker)
5. [ROS 2](#ros-2)
6. [GPU et Graphiques](#gpu-et-graphiques)

## Installation et Configuration

### Problème : Docker build échoue

**Symptômes :**
```
ERROR: failed to solve: process "/bin/bash -c apt-get update..."
```

**Solutions :**

1. Vérifier la connexion internet :
```bash
ping google.com
```

2. Nettoyer le cache Docker :
```bash
docker system prune -a
docker build --no-cache -t openskysea-nexus .
```

3. Vérifier l'espace disque :
```bash
df -h
# Besoin d'au moins 20 GB libres
```

### Problème : Permission denied lors du build

**Symptômes :**
```
Got permission denied while trying to connect to the Docker daemon socket
```

**Solutions :**

```bash
# Ajouter votre utilisateur au groupe docker
sudo usermod -aG docker $USER

# Se déconnecter et reconnecter, ou
newgrp docker

# Vérifier
docker ps
```

### Problème : rosdep init échoue

**Symptômes :**
```
ERROR: cannot download default sources list from...
```

**Solutions :**

```bash
# Dans le conteneur Docker
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
```

## Compilation

### Problème : colcon build échoue avec erreur de mémoire

**Symptômes :**
```
c++: fatal error: Killed signal terminated program cc1plus
```

**Solutions :**

1. Réduire le parallélisme :
```bash
colcon build --parallel-workers 1
```

2. Ajouter du swap :
```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

3. Compiler package par package :
```bash
colcon build --packages-select package_name
```

### Problème : Package X not found

**Symptômes :**
```
Could not find a package configuration file provided by "X"
```

**Solutions :**

1. Installer les dépendances :
```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

2. Vérifier que le package existe :
```bash
apt search ros-humble-X
```

3. Compiler les dépendances d'abord :
```bash
colcon build --packages-up-to dependency_package
```

### Problème : Compilation très lente

**Symptômes :**
Compilation prend plus de 30 minutes

**Solutions :**

1. Utiliser ccache :
```bash
sudo apt install ccache
export CC="ccache gcc"
export CXX="ccache g++"
colcon build
```

2. Augmenter workers (si vous avez assez de RAM) :
```bash
colcon build --parallel-workers $(nproc)
```

3. Désactiver les tests :
```bash
colcon build --cmake-args -DBUILD_TESTING=OFF
```

## Simulation

### Problème : Gazebo ne démarre pas

**Symptômes :**
```
[Err] [REST.cc:205] Error in REST request
```

**Solutions :**

1. Mode headless :
```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch simulation_bringup simulation.launch.py gui:=false
```

2. Vérifier l'affichage X11 (Linux) :
```bash
echo $DISPLAY
xhost +local:docker
```

3. Désactiver les plugins problématiques :
Éditer le fichier world et commenter les plugins non essentiels

### Problème : Gazebo est très lent

**Symptômes :**
FPS < 10, simulation en temps réel < 0.5x

**Solutions :**

1. Réduire la qualité graphique :
```bash
# Dans Gazebo GUI : View → Quality → Low
```

2. Désactiver les ombres :
Éditer le fichier .world :
```xml
<scene>
  <shadows>false</shadows>
</scene>
```

3. Réduire le nombre de robots :
```bash
ros2 launch simulation_bringup simulation.launch.py num_robots:=1
```

4. Vérifier que le GPU est utilisé :
```bash
nvidia-smi  # Doit montrer Gazebo dans la liste
```

### Problème : Robots ne bougent pas

**Symptômes :**
Les robots sont immobiles dans Gazebo

**Solutions :**

1. Vérifier que les topics sont publiés :
```bash
ros2 topic list
ros2 topic echo /robot/cmd_vel
```

2. Vérifier les namespaces :
```bash
ros2 node list
# Les nodes doivent correspondre aux noms des robots
```

3. Relancer les contrôleurs :
```bash
ros2 service call /controller_manager/load_controller ...
```

### Problème : No transform between frames

**Symptômes :**
```
[ERROR] Transform from X to Y failed
```

**Solutions :**

1. Vérifier TF tree :
```bash
ros2 run tf2_tools view_frames
# Génère frames.pdf
```

2. Lancer static transforms :
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world base_link
```

3. Vérifier que robot_state_publisher tourne :
```bash
ros2 node list | grep robot_state_publisher
```

## Docker

### Problème : Cannot connect to Docker daemon

**Symptômes :**
```
Cannot connect to the Docker daemon at unix:///var/run/docker.sock
```

**Solutions :**

1. Démarrer Docker :
```bash
sudo systemctl start docker
sudo systemctl enable docker
```

2. Vérifier le statut :
```bash
sudo systemctl status docker
```

### Problème : Container exits immediately

**Symptômes :**
Le conteneur se ferme dès qu'il est lancé

**Solutions :**

1. Vérifier les logs :
```bash
docker logs container_name
```

2. Lancer en mode interactif :
```bash
docker run -it --entrypoint /bin/bash openskysea-nexus
```

3. Vérifier l'entrypoint :
```bash
# Dans Dockerfile, vérifier ENTRYPOINT et CMD
```

### Problème : Cannot remove container

**Symptômes :**
```
Error response from daemon: container is running
```

**Solutions :**

```bash
# Forcer l'arrêt
docker stop -f container_id

# Supprimer
docker rm container_id

# Tout nettoyer
docker system prune -a
```

### Problème : Disk space issues

**Symptômes :**
```
no space left on device
```

**Solutions :**

```bash
# Vérifier l'utilisation
docker system df

# Nettoyer images inutilisées
docker image prune -a

# Nettoyer tout
docker system prune -a --volumes

# Vérifier espace disque
df -h
```

## ROS 2

### Problème : Nodes cannot communicate

**Symptômes :**
`ros2 topic list` ne montre pas les topics attendus

**Solutions :**

1. Vérifier DDS :
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Ou
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

2. Vérifier le domaine :
```bash
export ROS_DOMAIN_ID=0
```

3. Vérifier le réseau :
```bash
# Désactiver firewall temporairement
sudo ufw disable
```

### Problème : Topic hz is 0

**Symptômes :**
```bash
ros2 topic hz /topic
# average rate: 0.000
```

**Solutions :**

1. Vérifier que le publisher tourne :
```bash
ros2 node list
ros2 node info /publisher_node
```

2. Vérifier le type de message :
```bash
ros2 topic info /topic
ros2 interface show MsgType
```

3. Echo directement :
```bash
ros2 topic echo /topic
```

### Problème : Launch file not found

**Symptômes :**
```
Package 'X' not found
```

**Solutions :**

1. Source l'environnement :
```bash
source install/setup.bash
```

2. Vérifier que le package est compilé :
```bash
ls install/
colcon list
```

3. Recompiler le package :
```bash
colcon build --packages-select package_name
```

## GPU et Graphiques

### Problème : No GPU detected in Docker

**Symptômes :**
`nvidia-smi` ne fonctionne pas dans le conteneur

**Solutions :**

1. Installer nvidia-docker :
```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

2. Lancer avec GPU :
```bash
docker run --gpus all -it openskysea-nexus
```

### Problème : OpenGL renderer is llvmpipe

**Symptômes :**
```bash
glxinfo | grep "OpenGL renderer"
# OpenGL renderer string: llvmpipe
```

**Solutions :**

1. Vérifier drivers NVIDIA :
```bash
nvidia-smi
# Si erreur, installer drivers
sudo ubuntu-drivers autoinstall
```

2. Forcer GPU :
```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

3. Dans Docker :
```bash
docker run --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all ...
```

### Problème : X11 forwarding not working

**Symptômes :**
Cannot open display

**Solutions :**

```bash
# Sur la machine hôte
xhost +local:docker

# Lancer Docker avec X11
docker run -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  openskysea-nexus
```

## Problèmes Spécifiques aux Packages

### ArduPilot SITL

**Problème : SITL ne démarre pas**

```bash
# Vérifier installation
which sim_vehicle.py

# Réinstaller si nécessaire
cd /ardupilot
git pull
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### Gazebo Plugins

**Problème : Plugin failed to load**

```bash
# Vérifier chemin plugins
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_ws/install/lib

# Lister plugins disponibles
gzserver --verbose
```

## Obtenir de l'Aide

### Informations à Fournir

Lors de l'ouverture d'une issue, incluez :

```bash
# Informations système
uname -a
lsb_release -a

# Version ROS
ros2 --version

# Version Gazebo
gazebo --version

# Logs pertinents
ros2 launch ... 2>&1 | tee output.log

# Packages installés
colcon list

# Configuration Docker
docker version
docker info
```

### Template Issue GitHub

```markdown
**Environnement:**
- OS: Ubuntu 22.04
- ROS 2: Humble
- Docker: Yes/No
- GPU: NVIDIA GTX 1660

**Problème:**
Description claire du problème

**Étapes pour reproduire:**
1. ...
2. ...

**Logs:**
```
Coller les logs ici
```

**Ce que j'ai essayé:**
- Solution 1: résultat
- Solution 2: résultat
```

### Ressources Utiles

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [GitHub Discussions](https://github.com/kabir308/OpenSkySea-Nexus/discussions)
- [Stack Overflow - ROS](https://stackoverflow.com/questions/tagged/ros2)

### Contact

- **GitHub Issues :** [Issues](https://github.com/kabir308/OpenSkySea-Nexus/issues)
- **Discussions :** [Discussions](https://github.com/kabir308/OpenSkySea-Nexus/discussions)

## Checklist de Diagnostic

Avant de demander de l'aide :

- [ ] Ai-je vérifié ce guide de dépannage ?
- [ ] Ai-je consulté la documentation ?
- [ ] Ai-je sourcé l'environnement ROS 2 ?
- [ ] Ai-je vérifié les logs pour des erreurs ?
- [ ] Ai-je essayé de recompiler ?
- [ ] Ai-je vérifié l'espace disque/mémoire ?
- [ ] Ai-je mis à jour mon système/Docker ?
- [ ] Ai-je cherché des issues similaires sur GitHub ?
