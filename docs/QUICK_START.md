# Guide de Démarrage Rapide - OpenSkySea-Nexus

Ce guide vous aidera à démarrer rapidement avec OpenSkySea-Nexus, que vous soyez débutant ou développeur expérimenté.

## Table des Matières

1. [Aperçu du Projet](#aperçu-du-projet)
2. [Options de Démarrage](#options-de-démarrage)
3. [Installation Rapide](#installation-rapide)
4. [Première Simulation](#première-simulation)
5. [Prochaines Étapes](#prochaines-étapes)

## Aperçu du Projet

OpenSkySea-Nexus est un écosystème open source pour véhicules autonomes multi-environnements :
- **Air** : Drones, quadricoptères, VTOL
- **Mer** : Bateaux autonomes, sous-marins
- **Terre** : Rovers, robots terrestres

**Cas d'usage :**
- Recherche académique en robotique autonome
- Simulation de missions multi-domaines
- Développement d'algorithmes d'IA embarquée
- Prototypage de systèmes collaboratifs

## Options de Démarrage

Choisissez l'option qui correspond le mieux à votre situation :

### Option 1 : Docker (Recommandé pour débuter)
✅ **Avantages :** Configuration automatique, environnement cohérent
⚠️ **Prérequis :** Docker installé, 20 GB d'espace disque

### Option 2 : Installation Native
✅ **Avantages :** Meilleures performances, contrôle total
⚠️ **Prérequis :** Ubuntu 22.04, ROS 2 Humble, expérience Linux

### Option 3 : Mode Léger (Documentation Seulement)
✅ **Avantages :** Aucune installation, découverte du projet
⚠️ **Limitation :** Pas d'exécution de code

## Installation Rapide

### Avec Docker (15-30 minutes)

```bash
# 1. Cloner le projet
git clone https://github.com/kabir308/OpenSkySea-Nexus.git
cd OpenSkySea-Nexus

# 2. Construire l'image Docker (patience, première fois ~20 min)
docker build -t openskysea-nexus .

# 3. Lancer le conteneur
docker run -it --rm -v .:/ros2_ws/src openskysea-nexus

# 4. Dans le conteneur, compiler le projet
colcon build

# 5. Sourcer l'environnement
source install/setup.bash
```

### Installation Native (Pour utilisateurs avancés)

Voir [docs/INSTALLATION.md](./INSTALLATION.md) pour les instructions détaillées.

## Première Simulation

Une fois dans le conteneur Docker ou avec l'installation native :

```bash
# Sourcer l'environnement ROS 2
source install/setup.bash

# Lancer une simulation simple (exemple : quadricoptère)
ros2 launch simulation_bringup simulation.launch.py

# Dans un autre terminal, vous pouvez lancer des commandes
ros2 topic list
ros2 topic echo /drone/pose
```

### Visualisation avec Gazebo

Si vous utilisez Linux avec interface graphique :

```bash
# Avant de lancer Docker, autoriser l'affichage X11
xhost +local:docker

# Lancer avec support GUI
docker run -it --rm \
  -v .:/ros2_ws/src \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  openskysea-nexus

# Puis lancer la simulation
ros2 launch simulation_bringup simulation.launch.py
```

## Prochaines Étapes

### Pour les Chercheurs
1. Consulter [docs/RESEARCH_GUIDE.md](./RESEARCH_GUIDE.md) pour les cas d'usage académiques
2. Explorer les algorithmes d'IA dans `src/collaborative_intelligence/`
3. Lire la documentation sur l'architecture multi-agents

### Pour les Développeurs
1. Lire [CONTRIBUTING.md](../CONTRIBUTING.md) pour les conventions
2. Choisir un module dans `src/` (air/mer/terre)
3. Consulter les issues GitHub pour contribuer

### Pour les Étudiants
1. Commencer par les tutoriels dans `docs/tutorials/`
2. Expérimenter avec les simulations existantes
3. Rejoindre la communauté sur GitHub Discussions

## Besoin d'Aide ?

- **Documentation complète** : [docs/](.)
- **FAQ** : [docs/FAQ.md](./FAQ.md)
- **Troubleshooting** : [docs/TROUBLESHOOTING.md](./TROUBLESHOOTING.md)
- **Issues GitHub** : [Créer une issue](https://github.com/kabir308/OpenSkySea-Nexus/issues)

## Matériel Recommandé

### Configuration Minimale (Mode Léger)
- CPU : 4 cores
- RAM : 8 GB
- Disque : 30 GB
- GPU : Non requis (simulation légère sans GUI)

### Configuration Recommandée
- CPU : 8+ cores
- RAM : 16 GB
- Disque : 50 GB SSD
- GPU : NVIDIA avec 4 GB VRAM (pour Gazebo avec GUI)

### Configuration Optimale (Recherche/Développement)
- CPU : 16+ cores
- RAM : 32 GB
- Disque : 100 GB SSD
- GPU : NVIDIA RTX avec 8+ GB VRAM

Voir [docs/HARDWARE_REQUIREMENTS.md](./HARDWARE_REQUIREMENTS.md) pour plus de détails.
