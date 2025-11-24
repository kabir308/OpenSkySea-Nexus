# Guide d'Installation - OpenSkySea-Nexus

Guide détaillé pour installer OpenSkySea-Nexus selon votre configuration.

## Table des Matières

1. [Installation Docker (Recommandée)](#installation-docker)
2. [Installation Native Ubuntu](#installation-native-ubuntu)
3. [Installation Légère](#installation-légère)
4. [Installation Windows (WSL2)](#installation-windows-wsl2)
5. [Installation macOS](#installation-macos)
6. [Vérification de l'Installation](#vérification-installation)

## Installation Docker

### Prérequis

- Docker installé ([Guide officiel](https://docs.docker.com/get-docker/))
- 20 GB d'espace disque libre
- Connexion internet

### Étapes

```bash
# 1. Cloner le repository
git clone https://github.com/kabir308/OpenSkySea-Nexus.git
cd OpenSkySea-Nexus

# 2. Construire l'image Docker
docker build -t openskysea-nexus .

# 3. Lancer le conteneur
docker run -it --rm -v .:/ros2_ws/src openskysea-nexus

# 4. Dans le conteneur, compiler
colcon build

# 5. Sourcer l'environnement
source install/setup.bash
```

### Support GPU (Linux uniquement)

```bash
# Installer nvidia-docker
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# Lancer avec GPU
docker run --gpus all -it --rm -v .:/ros2_ws/src openskysea-nexus
```

### Support GUI (Linux)

```bash
# Autoriser X11
xhost +local:docker

# Lancer avec affichage
docker run -it --rm \
  -v .:/ros2_ws/src \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  openskysea-nexus
```

## Installation Native Ubuntu

### Prérequis

- Ubuntu 22.04 LTS
- 50 GB d'espace disque libre
- Connexion internet

### Étape 1 : Installer ROS 2 Humble

```bash
# Configurer locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Ajouter le repository ROS 2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Installer ROS 2 Humble Desktop Full
sudo apt update
sudo apt install ros-humble-desktop-full
```

### Étape 2 : Installer les Outils de Développement

```bash
# Installer colcon
sudo apt install python3-colcon-common-extensions

# Installer rosdep
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# Installer autres outils
sudo apt install python3-pip git
```

### Étape 3 : Installer ArduPilot SITL (Optionnel)

```bash
# Cloner ArduPilot
cd ~
git clone --depth 1 https://github.com/ArduPilot/ardupilot.git
cd ardupilot/Tools/environment_install
./install-prereqs-ubuntu.sh -y

# Sourcer l'environnement
. ~/.profile
```

### Étape 4 : Cloner et Compiler OpenSkySea-Nexus

```bash
# Créer workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Cloner le projet
git clone https://github.com/kabir308/OpenSkySea-Nexus.git .

# Installer les dépendances
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Compiler
colcon build

# Sourcer
source install/setup.bash
```

### Étape 5 : Configurer l'Environnement

```bash
# Ajouter au .bashrc pour auto-sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Recharger
source ~/.bashrc
```

## Installation Légère

Pour machines limitées (8 GB RAM, pas de GPU).

### Dockerfile Léger

Créer `Dockerfile.lite` :

```dockerfile
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

WORKDIR /ros2_ws
COPY . ./src

RUN . /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

CMD ["bash"]
```

```bash
# Build léger
docker build -f Dockerfile.lite -t openskysea-nexus-lite .

# Lancer
docker run -it --rm -v .:/ros2_ws/src openskysea-nexus-lite
```

### Compiler en Mode Léger

```bash
# Désactiver tests et debug symbols
colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=MinSizeRel -DBUILD_TESTING=OFF \
  --parallel-workers 1
```

### Simulation Headless

```bash
# Toujours en mode headless
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch simulation_bringup simulation.launch.py gui:=false
```

## Installation Windows (WSL2)

### Prérequis

- Windows 10/11 version 2004+
- WSL2 installé
- Docker Desktop pour Windows

### Étape 1 : Installer WSL2

```powershell
# Dans PowerShell Admin
wsl --install -d Ubuntu-22.04
```

### Étape 2 : Installer Docker Desktop

1. Télécharger [Docker Desktop](https://www.docker.com/products/docker-desktop)
2. Installer et activer l'intégration WSL2
3. Redémarrer

### Étape 3 : Dans WSL2 Ubuntu

```bash
# Ouvrir terminal WSL2
wsl

# Suivre les instructions Docker ci-dessus
git clone https://github.com/kabir308/OpenSkySea-Nexus.git
cd OpenSkySea-Nexus
docker build -t openskysea-nexus .
docker run -it --rm -v .:/ros2_ws/src openskysea-nexus
```

### Limitations Windows

- ❌ Pas de support GPU natif (pour l'instant)
- ⚠️ Performances réduites vs Linux natif
- ⚠️ GUI Gazebo limitée (mode headless recommandé)

## Installation macOS

### Prérequis

- macOS 12+
- Docker Desktop pour Mac
- 20 GB d'espace disque

### Installation

```bash
# Installer Docker Desktop pour Mac
# https://docs.docker.com/desktop/mac/install/

# Cloner et utiliser Docker
git clone https://github.com/kabir308/OpenSkySea-Nexus.git
cd OpenSkySea-Nexus
docker build -t openskysea-nexus .
docker run -it --rm -v .:/ros2_ws/src openskysea-nexus
```

### Limitations macOS

- ❌ Pas de support GPU
- ⚠️ Performances limitées (émulation x86 sur ARM)
- ⚠️ Mode headless uniquement recommandé

### Alternative : Lima + QEMU (Avancé)

Pour de meilleures performances sur Apple Silicon :

```bash
# Installer Lima
brew install lima

# Créer VM Ubuntu
limactl start --name=ros2 template://ubuntu-lts

# Utiliser la VM
limactl shell ros2
# Puis suivre instructions native Ubuntu
```

## Vérification Installation

### Vérifier ROS 2

```bash
# Version
ros2 --version

# Nodes de test
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener

# Nettoyer
pkill -f demo_nodes
```

### Vérifier Gazebo

```bash
# Lancer Gazebo (si GUI disponible)
gazebo --version
gazebo --verbose

# Ou headless
gzserver --version
```

### Vérifier Compilation

```bash
# Dans workspace
cd ~/ros2_ws  # ou /ros2_ws dans Docker
colcon build --packages-select simple_package
source install/setup.bash
```

### Test Complet

```bash
# Lancer simulation
ros2 launch simulation_bringup simulation.launch.py gui:=false &

# Vérifier topics (dans nouveau terminal)
ros2 topic list
ros2 topic hz /clock

# Arrêter
pkill -f launch
```

## Désinstallation

### Docker

```bash
# Supprimer image
docker rmi openskysea-nexus

# Nettoyer tout Docker
docker system prune -a
```

### Native Ubuntu

```bash
# Supprimer workspace
rm -rf ~/ros2_ws

# Supprimer ROS 2 (optionnel)
sudo apt remove ros-humble-* -y
sudo apt autoremove -y

# Supprimer ArduPilot (optionnel)
rm -rf ~/ardupilot
```

## Mise à Jour

### Docker

```bash
# Pull dernières modifications
cd OpenSkySea-Nexus
git pull

# Rebuild image
docker build -t openskysea-nexus .
```

### Native

```bash
# Pull modifications
cd ~/ros2_ws/src
git pull

# Recompiler
cd ~/ros2_ws
colcon build
```

## Dépannage

### Docker build échoue

```bash
# Nettoyer cache
docker system prune -a

# Rebuild sans cache
docker build --no-cache -t openskysea-nexus .
```

### Compilation échoue

```bash
# Nettoyer build
rm -rf build install log

# Mettre à jour dépendances
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Recompiler
colcon build --cmake-clean-cache
```

### Manque de RAM

```bash
# Compiler séquentiellement
colcon build --parallel-workers 1

# Ajouter swap (Linux)
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

Voir [TROUBLESHOOTING.md](./TROUBLESHOOTING.md) pour plus de solutions.

## Support

**Besoin d'aide ?**

- 📚 [Documentation complète](.)
- ❓ [FAQ](./FAQ.md)
- 🔧 [Troubleshooting](./TROUBLESHOOTING.md)
- 💬 [GitHub Discussions](https://github.com/kabir308/OpenSkySea-Nexus/discussions)
- 🐛 [GitHub Issues](https://github.com/kabir308/OpenSkySea-Nexus/issues)

## Prochaines Étapes

Une fois installé :

1. ✅ Lancer votre [première simulation](./QUICK_START.md#première-simulation)
2. 📖 Explorer la [documentation](.)
3. 🤝 [Contribuer](../CONTRIBUTING.md) au projet
