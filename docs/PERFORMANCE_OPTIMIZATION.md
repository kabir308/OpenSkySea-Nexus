# Optimisation des Performances - OpenSkySea-Nexus

Guide pour optimiser les performances sur différentes configurations matérielles.

## Table des Matières

1. [Optimisation de la Compilation](#optimisation-de-la-compilation)
2. [Optimisation de la Simulation](#optimisation-de-la-simulation)
3. [Optimisation Mémoire](#optimisation-mémoire)
4. [Optimisation GPU](#optimisation-gpu)
5. [Mode Léger](#mode-léger)

## Optimisation de la Compilation

### Utilisation des Cores CPU

```bash
# Utiliser tous les cores (rapide mais consomme beaucoup de RAM)
colcon build --parallel-workers $(nproc)

# Limiter le nombre de workers (recommandé si <16 GB RAM)
colcon build --parallel-workers 4

# Compilation séquentielle (lent mais minimal en RAM)
colcon build --parallel-workers 1
```

### Compilation Incrémentale

```bash
# Ne recompiler que les packages modifiés
colcon build --packages-select package_name

# Recompiler un package et ses dépendants
colcon build --packages-up-to package_name

# Compilation sans tests (plus rapide)
colcon build --cmake-args -DBUILD_TESTING=OFF
```

### Cache de Compilation

```bash
# Installer ccache pour accélérer les recompilations
sudo apt install ccache

# Configurer colcon pour utiliser ccache
export CC="ccache gcc"
export CXX="ccache g++"
colcon build
```

### Réduire l'Espace Disque

```bash
# Supprimer les fichiers de build intermédiaires
colcon build --cmake-clean-cache

# Nettoyer complètement build et install
rm -rf build install log

# Configurer pour builds plus petits
colcon build --cmake-args -DCMAKE_BUILD_TYPE=MinSizeRel
```

## Optimisation de la Simulation

### Mode Headless (Sans GUI)

Pour machines sans GPU ou via SSH :

```bash
# Lancer simulation sans interface Gazebo
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch simulation_bringup simulation.launch.py gui:=false

# Ou définir dans le launch file
ros2 launch simulation_bringup simulation.launch.py headless:=true
```

### Réduire la Fréquence de Simulation

```bash
# Dans votre monde Gazebo (.world), réduire le real_time_factor
<physics type="ode">
  <real_time_update_rate>100</real_time_update_rate>  <!-- au lieu de 1000 -->
  <max_step_size>0.01</max_step_size>
</physics>
```

### Simplification des Modèles

```xml
<!-- Dans vos fichiers .sdf/.urdf, réduire la complexité -->

<!-- Utiliser des collision simples -->
<collision>
  <geometry>
    <box><size>1 1 1</size></box>  <!-- Au lieu de mesh complexe -->
  </geometry>
</collision>

<!-- Réduire la qualité visuelle -->
<visual>
  <geometry>
    <mesh>
      <uri>model://simple_drone.dae</uri>  <!-- Modèle simplifié -->
    </mesh>
  </geometry>
</visual>
```

### Limiter le Nombre de Robots

```python
# Dans simulation.launch.py
num_robots = 2  # Au lieu de 10

# Ou via argument
ros2 launch simulation_bringup simulation.launch.py num_robots:=2
```

## Optimisation Mémoire

### Surveiller l'Utilisation

```bash
# Installer htop pour monitoring
sudo apt install htop
htop

# Ou utiliser free
watch -n 1 free -h

# Vérifier utilisation par processus
ps aux --sort=-%mem | head -10
```

### Réduire la Consommation ROS 2

```bash
# Limiter la taille des buffers DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain>
    <Internal>
      <MaxMessageSize>65500</MaxMessageSize>
    </Internal>
  </Domain>
</CycloneDDS>'

# Ou utiliser FastDDS avec configuration légère
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Désactiver les Logs Verbeux

```bash
# Réduire le niveau de log
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN

# Désactiver complètement les logs (debug)
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=ERROR
```

### Swap et Out-of-Memory

Si vous manquez de RAM :

```bash
# Créer un fichier swap (16 GB)
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Rendre permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Ajuster swappiness (0-100, plus bas = moins de swap)
sudo sysctl vm.swappiness=10
```

## Optimisation GPU

### Vérifier l'Utilisation GPU

```bash
# Installer nvidia-smi
nvidia-smi

# Monitoring continu
watch -n 1 nvidia-smi

# Vérifier quel processus utilise le GPU
nvidia-smi pmon
```

### Configuration Gazebo pour GPU

```bash
# Forcer l'utilisation du GPU NVIDIA
export __GL_SYNC_TO_VBLANK=0
export __GL_SHADER_DISK_CACHE=1

# Vérifier que Gazebo utilise le bon GPU
export OGRE_RTShader_CACHE_PATH=/tmp/ogre_cache
```

### Multi-GPU

Si vous avez plusieurs GPUs :

```bash
# Sélectionner un GPU spécifique
export CUDA_VISIBLE_DEVICES=0  # Premier GPU

# Répartir la charge
# Terminal 1
export CUDA_VISIBLE_DEVICES=0
ros2 launch sim1.launch.py

# Terminal 2
export CUDA_VISIBLE_DEVICES=1
ros2 launch sim2.launch.py
```

### Réduire la Qualité Graphique

Dans Gazebo GUI : View → Quality → Low

Ou via code :

```xml
<!-- Dans world file -->
<scene>
  <shadows>false</shadows>  <!-- Désactiver les ombres -->
  <sky>false</sky>          <!-- Pas de ciel -->
</scene>
```

## Mode Léger

### Configuration Docker Légère

```dockerfile
# Alternative au Dockerfile principal pour machines limitées
FROM ros:humble-ros-base  # Au lieu de desktop-full

# Installer uniquement les dépendances essentielles
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    # ... minimal dependencies
```

```bash
# Build avec moins de cache
docker build --no-cache=false -t openskysea-nexus-lite .

# Limiter la mémoire Docker
docker run --memory="4g" --memory-swap="6g" -it openskysea-nexus-lite
```

### Lancer Simulations Légères

```bash
# Exemple de simulation minimale
ros2 run simple_simulator basic_drone

# Au lieu de
ros2 launch simulation_bringup simulation.launch.py  # Lourd
```

### Développement Sans Simulation

```bash
# Tester uniquement la logique, sans Gazebo
ros2 run your_package your_node

# Utiliser des bags ROS 2 enregistrés
ros2 bag play recorded_simulation.bag

# Mock des capteurs
ros2 run mock_sensors camera_publisher
```

## Benchmarks et Mesures

### Script de Benchmark

Créer `benchmark.sh` :

```bash
#!/bin/bash

echo "=== CPU ==="
lscpu | grep "Model name\|CPU(s)\|MHz"

echo "=== RAM ==="
free -h

echo "=== GPU ==="
nvidia-smi --query-gpu=name,memory.total --format=csv 2>/dev/null || echo "No NVIDIA GPU"

echo "=== Stockage ==="
df -h | grep -E "/$|/home"

echo "=== Compilation Test ==="
time colcon build --packages-select simple_package

echo "=== Simulation Test ==="
timeout 30s ros2 launch simulation_bringup simulation.launch.py gui:=false &
sleep 25
ros2 topic hz /robot/pose
```

### Profiling ROS 2

```bash
# Installer ros2 performance tools
sudo apt install ros-humble-performance-test

# Profiler un node
ros2 run perf_test perf_test -c pub_sub_test

# Analyser latence
ros2 topic delay /topic_name
```

## Recommandations par Configuration

### Machine avec 8 GB RAM

```bash
# Compilation
colcon build --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF

# Simulation
ros2 launch simulation_bringup simulation.launch.py gui:=false num_robots:=1

# Swap recommandé
sudo swapon /swapfile  # 8-16 GB
```

### Machine avec 16 GB RAM

```bash
# Compilation
colcon build --parallel-workers 4

# Simulation
ros2 launch simulation_bringup simulation.launch.py num_robots:=3
```

### Machine avec 32+ GB RAM

```bash
# Compilation
colcon build --parallel-workers $(nproc)

# Simulation
ros2 launch simulation_bringup simulation.launch.py num_robots:=10
```

### Sans GPU

```bash
# Toujours en headless
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch simulation_bringup simulation.launch.py gui:=false

# Utiliser RViz2 au lieu de Gazebo pour visualisation légère
ros2 run rviz2 rviz2
```

## Outils de Diagnostic

### Installation

```bash
sudo apt install -y \
    htop \
    iotop \
    nethogs \
    sysstat \
    nvtop  # Pour GPU NVIDIA
```

### Utilisation

```bash
# CPU et RAM
htop

# I/O disque
sudo iotop

# Réseau
sudo nethogs

# GPU (NVIDIA)
nvtop

# Statistiques système
sar -u 1 10  # CPU
sar -r 1 10  # RAM
```

## Troubleshooting Performance

### Gazebo très lent

```bash
# Vérifier si GPU est utilisé
glxinfo | grep "OpenGL renderer"  # Doit montrer votre GPU, pas "llvmpipe"

# Si llvmpipe (CPU rendering)
export LIBGL_ALWAYS_SOFTWARE=0
export SVGA_VGPU10=0
```

### Out of Memory lors de la compilation

```bash
# Réduire workers
colcon build --parallel-workers 1

# Activer swap
sudo swapon -a

# Compiler par étapes
colcon build --packages-up-to package1
colcon build --packages-up-to package2
```

### ROS 2 topics lents

```bash
# Vérifier network overhead
ros2 topic bw /topic_name

# Changer DDS implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # Plus léger
# ou
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp    # Plus rapide
```

## Ressources Supplémentaires

- [ROS 2 Performance](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)
- [Gazebo Performance](http://gazebosim.org/tutorials?tut=performance_metrics)
- [Docker Resource Limits](https://docs.docker.com/config/containers/resource_constraints/)

## Contact

Pour des questions spécifiques sur les performances :
- [GitHub Issues - Performance](https://github.com/kabir308/OpenSkySea-Nexus/issues)
