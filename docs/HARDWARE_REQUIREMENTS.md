# Configuration Matérielle - OpenSkySea-Nexus

Ce document détaille les exigences matérielles pour différents scénarios d'utilisation.

## Vue d'Ensemble

OpenSkySea-Nexus est un projet de recherche ambitieux qui peut fonctionner sur différentes configurations matérielles en fonction de vos besoins.

## Configurations par Cas d'Usage

### 1. Exploration et Apprentissage

**Objectif :** Découvrir le projet, lire le code, exécuter des exemples simples

**Configuration Minimale :**
- **CPU :** 4 cores (Intel i5/AMD Ryzen 5 ou équivalent)
- **RAM :** 8 GB
- **Stockage :** 30 GB disponible
- **GPU :** Optionnel (peut utiliser le mode headless)
- **OS :** Ubuntu 22.04 LTS (recommandé) ou Windows avec WSL2

**Limitations :**
- Simulations simplifiées uniquement
- Pas de visualisation 3D en temps réel
- Un seul robot à la fois
- Compilation plus lente

### 2. Développement Standard

**Objectif :** Développer des fonctionnalités, tester des algorithmes, simulations multi-robots basiques

**Configuration Recommandée :**
- **CPU :** 8+ cores (Intel i7/AMD Ryzen 7 ou supérieur)
- **RAM :** 16 GB (32 GB pour multi-robots)
- **Stockage :** 50 GB SSD
- **GPU :** NVIDIA GTX 1660 ou équivalent (4 GB VRAM)
- **OS :** Ubuntu 22.04 LTS

**Capacités :**
- Visualisation Gazebo fluide
- 2-4 robots simultanés
- Compilation rapide avec `colcon build --parallel-workers 4`
- Développement confortable

### 3. Recherche Avancée

**Objectif :** Recherche académique, développement d'IA, simulations complexes, multi-agents

**Configuration Optimale :**
- **CPU :** 16+ cores (Intel i9/AMD Ryzen 9/Threadripper)
- **RAM :** 32-64 GB
- **Stockage :** 100+ GB NVMe SSD
- **GPU :** NVIDIA RTX 3070/4070 ou supérieur (8+ GB VRAM)
- **OS :** Ubuntu 22.04 LTS

**Capacités :**
- 10+ robots simultanés
- Apprentissage machine embarqué
- Simulations réalistes avec physique avancée
- Compilation parallèle ultra-rapide
- Entraînement de modèles d'IA

### 4. Mode Cloud/Serveur

**Objectif :** Exécution headless, CI/CD, simulations batch

**Configuration Serveur :**
- **CPU :** 8-16 cores
- **RAM :** 16-32 GB
- **Stockage :** 50 GB
- **GPU :** Optionnel (Tesla/A100 pour IA)
- **OS :** Ubuntu 22.04 LTS Server

**Avantages :**
- Pas de GUI nécessaire
- Peut utiliser des instances cloud (AWS, Azure, GCP)
- Automatisation complète

## Détails GPU

### Pourquoi un GPU ?

Le GPU accélère :
1. **Gazebo** : Rendu 3D de la simulation
2. **Perception** : Traitement d'images (caméras, LiDAR)
3. **IA** : Entraînement et inférence de réseaux de neurones

### GPU Recommandés

**Entrée de Gamme (Développement basique):**
- NVIDIA GTX 1650 (4 GB)
- NVIDIA GTX 1660 (6 GB)

**Milieu de Gamme (Recommandé):**
- NVIDIA RTX 3060 (12 GB)
- NVIDIA RTX 3070 (8 GB)
- NVIDIA RTX 4060 (8 GB)

**Haut de Gamme (Recherche):**
- NVIDIA RTX 3080/3090
- NVIDIA RTX 4080/4090
- NVIDIA A4000/A5000 (Workstation)

**Serveur/Cloud:**
- NVIDIA Tesla T4
- NVIDIA A100
- NVIDIA H100

### Mode Sans GPU

Il est possible d'exécuter OpenSkySea-Nexus sans GPU :

```bash
# Désactiver le rendu Gazebo (mode headless)
export LIBGL_ALWAYS_SOFTWARE=1

# Ou utiliser le backend CPU
ros2 launch simulation_bringup simulation.launch.py gui:=false
```

**Limitations :**
- Pas de visualisation 3D
- Simulations plus lentes
- Certaines fonctionnalités de perception désactivées

## Stockage

### Espace Disque Requis

- **Code source :** ~2 GB
- **Docker image :** ~10 GB
- **Build artifacts (colcon):** ~5-10 GB
- **Datasets (optionnels) :** 10-100 GB
- **Logs et résultats :** 5-20 GB

**Total recommandé :** 50-100 GB

### Type de Stockage

- **SSD/NVMe** : Fortement recommandé pour compilation et I/O
- **HDD** : Possible mais compilation 3-5x plus lente

## RAM

### Utilisation Mémoire Typique

- **Base ROS 2 + Gazebo :** 4-6 GB
- **1 robot simple :** +1-2 GB
- **Chaque robot additionnel :** +0.5-1 GB
- **Algorithmes d'IA :** +2-8 GB (selon modèle)
- **Compilation colcon :** +2-4 GB

### Recommandations

- **8 GB :** Limite, 1-2 robots max, pas d'IA lourde
- **16 GB :** Confortable, 3-4 robots, IA basique
- **32 GB :** Idéal, 10+ robots, IA avancée, multi-tâches
- **64 GB :** Recherche, datasets massifs, simulations complexes

## CPU

### Cores Recommandés

ROS 2 et Gazebo sont multi-threadés :

- **4 cores :** Minimum, simulations simples
- **8 cores :** Recommandé, bon équilibre
- **16+ cores :** Optimal pour compilation et multi-robots

### Optimisation Compilation

```bash
# Utiliser tous les cores disponibles
colcon build --parallel-workers $(nproc)

# Ou limiter pour éviter de saturer
colcon build --parallel-workers 4
```

## Réseau

### Simulations Distribuées

Pour exécuter des simulations sur plusieurs machines :

- **Bande passante :** 100 Mbps minimum, 1 Gbps recommandé
- **Latence :** <10 ms sur LAN
- **Configuration :** Voir [docs/DISTRIBUTED_SIMULATION.md](./DISTRIBUTED_SIMULATION.md)

## Alternatives Économiques

### Cloud Computing

Si vous n'avez pas le matériel :

1. **AWS EC2** : Instances g4dn.xlarge (~0.50$/h)
2. **Google Cloud** : Instances n1-standard-8 + GPU
3. **Azure** : NC-series
4. **Paperspace** : GPU Cloud à partir de 0.50$/h

### Laboratoires Universitaires

- Accès à des serveurs de calcul
- Clusters avec GPUs
- Crédits cloud académiques (AWS Educate, GCP for Education)

### Optimisation pour Machines Limitées

Voir [docs/PERFORMANCE_OPTIMIZATION.md](./PERFORMANCE_OPTIMIZATION.md) pour :
- Réduire l'utilisation mémoire
- Accélérer la compilation
- Simulations légères
- Mode headless

## Tests de Performance

Pour tester votre configuration :

```bash
# Vérifier CPU
lscpu

# Vérifier RAM
free -h

# Vérifier GPU (si NVIDIA)
nvidia-smi

# Vérifier stockage
df -h

# Test de compilation (benchmark)
time colcon build --cmake-clean-cache
```

## Tableau Récapitulatif

| Cas d'Usage | CPU | RAM | GPU | Stockage | Coût Estimé |
|-------------|-----|-----|-----|----------|-------------|
| Exploration | 4 cores | 8 GB | Aucun | 30 GB | ~500€ (ordinateur d'occasion) |
| Développement | 8 cores | 16 GB | GTX 1660 | 50 GB SSD | ~1000€ |
| Recherche | 16 cores | 32 GB | RTX 3070 | 100 GB NVMe | ~2000€ |
| Serveur | 8-16 cores | 16-32 GB | Optionnel | 50 GB | 0.50-2$/h (cloud) |

## Questions Fréquentes

**Q: Puis-je utiliser un Mac ?**
R: Possible avec Docker, mais performances limitées (pas de GPU passthrough natif). Linux recommandé.

**Q: Windows est-il supporté ?**
R: Via WSL2 + Docker. Performances réduites. Ubuntu natif recommandé.

**Q: Mon GPU AMD fonctionnera-t-il ?**
R: Partiellement. ROS 2 supporte AMD, mais certains outils sont optimisés NVIDIA (CUDA).

**Q: Combien de temps dure la compilation ?**
R: 
- 4 cores, HDD : 15-20 min
- 8 cores, SSD : 5-10 min
- 16 cores, NVMe : 2-5 min

## Support

Pour des questions spécifiques sur le matériel :
- [GitHub Discussions - Hardware](https://github.com/kabir308/OpenSkySea-Nexus/discussions)
- [Issues - Installation](https://github.com/kabir308/OpenSkySea-Nexus/issues)
