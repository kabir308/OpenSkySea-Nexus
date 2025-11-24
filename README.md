# OpenSkySea-Nexus

<div align="center">

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/docker-ready-brightgreen)](https://www.docker.com/)
[![Gazebo](https://img.shields.io/badge/gazebo-classic-orange)](http://gazebosim.org/)

**Un écosystème open source unifié pour véhicules autonomes multi-environnements**

*Air • Mer • Terre | Simulation • IA Embarquée • Recherche Collaborative*

[🚀 Démarrage Rapide](#démarrage-rapide) •
[📚 Documentation](#documentation) •
[🤝 Contribuer](#contribuer) •
[💬 Communauté](#communauté)

</div>

---

## 🌟 À Propos

OpenSkySea-Nexus est une plateforme de recherche et développement pour véhicules autonomes multi-domaines :

- 🚁 **Air** : Drones, quadricoptères, VTOL
- 🚢 **Mer** : Bateaux autonomes, sous-marins (ROV/AUV)
- 🤖 **Terre** : Rovers, robots mobiles

### Points Forts

✅ **Reproductible** : Environnement Dockerisé prêt à l'emploi  
✅ **Standards Industriels** : ROS 2, Gazebo, colcon  
✅ **Multi-Robots** : Simulations collaboratives avancées  
✅ **Open Research** : Parfait pour la recherche académique  
✅ **Architecture Professionnelle** : Conception modulaire en couches

### Cas d'Usage

- 🎓 **Recherche académique** en robotique et systèmes autonomes
- 🏫 **Enseignement** : ROS 2, simulations, IA embarquée
- 🔬 **Prototypage** d'algorithmes et systèmes multi-agents
- 🌊 **Missions spécialisées** : océanographie, surveillance, exploration

## 📋 Prérequis

### Configuration Minimale (Exploration)
- **OS** : Ubuntu 22.04 ou Docker
- **CPU** : 4 cores
- **RAM** : 8 GB
- **Disque** : 30 GB

### Configuration Recommandée (Développement)
- **OS** : Ubuntu 22.04 LTS
- **CPU** : 8+ cores
- **RAM** : 16 GB
- **GPU** : NVIDIA GTX 1660 ou supérieur
- **Disque** : 50 GB SSD

📖 Voir [Configuration Matérielle Détaillée](docs/HARDWARE_REQUIREMENTS.md) pour tous les scénarios

## 🚀 Démarrage Rapide

### Option 1 : Docker (Recommandé pour débuter)

```bash
# 1. Cloner le projet
git clone https://github.com/kabir308/OpenSkySea-Nexus.git
cd OpenSkySea-Nexus

# 2. Construire l'image Docker (15-30 min première fois)
docker build -t openskysea-nexus .

# 3. Lancer le conteneur
docker run -it --rm -v .:/ros2_ws/src openskysea-nexus

# 4. Dans le conteneur : compiler
colcon build

# 5. Sourcer l'environnement
source install/setup.bash

# 6. Lancer une simulation !
ros2 launch simulation_bringup simulation.launch.py
```

### Option 2 : Installation Native

Pour Ubuntu 22.04 avec ROS 2 Humble déjà installé :

```bash
# Installer les dépendances
cd OpenSkySea-Nexus
rosdep install --from-paths src --ignore-src -r -y

# Compiler
colcon build

# Utiliser
source install/setup.bash
ros2 launch simulation_bringup simulation.launch.py
```

### 🎯 Premier Test

```bash
# Dans un autre terminal, vérifier les topics
ros2 topic list

# Observer la pose d'un robot
ros2 topic echo /drone/pose

# Publier une commande de mouvement
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
```

📖 Guide complet : [Démarrage Rapide](docs/QUICK_START.md)

## 📚 Documentation

### Pour Bien Démarrer
- 📘 [Guide de Démarrage Rapide](docs/QUICK_START.md) - Premier pas avec le projet
- ❓ [FAQ](docs/FAQ.md) - Questions fréquentes
- 🔧 [Dépannage](docs/TROUBLESHOOTING.md) - Solutions aux problèmes courants
- 💻 [Configuration Matérielle](docs/HARDWARE_REQUIREMENTS.md) - Choisir votre setup

### Guides Techniques
- ⚡ [Optimisation des Performances](docs/PERFORMANCE_OPTIMIZATION.md) - Optimiser pour votre machine
- 🏗️ [Architecture](docs/architecture/high_level.md) - Design du système
- 🗺️ [Roadmap](docs/ROADMAP.md) - Vision et développement futur

### Pour Contribuer
- 🤝 [Guide de Contribution](CONTRIBUTING.md) - Comment contribuer
- 📜 [Code de Conduite](CODE_OF_CONDUCT.md) - Nos valeurs

## 🛠️ Structure du Projet

```
OpenSkySea-Nexus/
├── src/                      # Code source ROS 2
│   ├── air/                  # Véhicules aériens
│   ├── sea/                  # Véhicules maritimes
│   ├── land/                 # Véhicules terrestres
│   ├── collaborative_intelligence/  # IA multi-agents
│   ├── simulation_bringup/   # Launch files simulation
│   └── ...                   # Autres modules
├── docs/                     # Documentation
├── hardware/                 # Info matériel
├── sim/                      # Fichiers simulation
├── tests/                    # Tests
├── Dockerfile                # Configuration Docker
└── README.md                 # Ce fichier
```

## ⚙️ Fonctionnalités

### Simulation Multi-Domaines
- 🚁 Quadricoptères et drones fixes
- 🛥️ Bateaux de surface autonomes
- 🤿 Véhicules sous-marins
- 🚗 Rovers terrestres
- 🔄 Missions collaboratives air-mer-terre

### Intelligence Artificielle
- 🧠 IA embarquée pour navigation autonome
- 🤝 Apprentissage collaboratif multi-agents
- 👁️ Vision par ordinateur
- 📊 Analyse de données en temps réel

### Outils de Développement
- 🐳 Environnement Docker reproductible
- 🔨 Intégration ROS 2 Humble
- 📦 Support colcon build
- 🎮 Simulation Gazebo Classic (Ignition à venir)

## 🤝 Contribuer

Nous accueillons chaleureusement les contributions ! 

### Comment Contribuer

1. 🍴 **Fork** le projet
2. 🌿 **Créer une branche** : `git checkout -b feature/ma-fonctionnalite`
3. ✍️ **Commiter** : `git commit -m 'Ajout d'une fonctionnalité'`
4. 📤 **Pousser** : `git push origin feature/ma-fonctionnalite`
5. 🎯 **Ouvrir une Pull Request**

### Domaines de Contribution

- 💻 **Code** : Nouvelles fonctionnalités, corrections de bugs
- 📖 **Documentation** : Guides, tutoriels, traductions
- 🧪 **Tests** : Tests unitaires, intégration, validation
- 🎨 **Design** : Interface, visualisation, modèles 3D
- 🐛 **Rapports de bugs** : Identifier et documenter les problèmes

📘 Voir le [Guide de Contribution](CONTRIBUTING.md) pour plus de détails

## 💬 Communauté

### Obtenir de l'Aide

- 💬 [GitHub Discussions](https://github.com/kabir308/OpenSkySea-Nexus/discussions) - Questions et discussions
- 🐛 [Issues](https://github.com/kabir308/OpenSkySea-Nexus/issues) - Bugs et fonctionnalités
- 📚 [Documentation](docs/) - Guides et références

### Rester Informé

- ⭐ **Star** le projet sur GitHub
- 👁️ **Watch** pour les notifications
- 📢 Suivre les **Releases** pour les mises à jour

## 🗺️ Roadmap

Notre vision pour l'avenir :

### Court Terme (Q1 2025)
- ✅ Documentation complète et accessible
- 🔄 Optimisations performances
- 🧪 Tests automatisés (CI/CD)
- 📹 Tutoriels vidéo

### Moyen Terme (Q2-Q3 2025)
- 🚀 Migration Gazebo Ignition
- 🤖 Plus de modèles de véhicules
- 🧠 Exemples IA/ML avancés
- ☁️ Support cloud (AWS, Azure)

### Long Terme (2026+)
- 🏢 Cas d'usage industriels
- 🎓 Programme de formation
- 🌐 Écosystème de plugins
- 🤝 Partenariats académiques

📖 Voir la [Roadmap Complète](docs/ROADMAP.md)

## 📊 Statut du Projet

🚧 **En Développement Actif** - Le projet est en phase de développement. Les contributions sont les bienvenues !

**Phase actuelle :** Fondations et Documentation  
**Prochaine phase :** Accessibilité et Performance

## 📄 Licence

Ce projet est sous licence MIT. Voir [LICENSE](LICENSE) pour plus de détails.

## 🙏 Remerciements

- Communauté [ROS 2](https://docs.ros.org/)
- Équipe [Gazebo](http://gazebosim.org/)
- Projet [ArduPilot](https://ardupilot.org/)
- Tous les [contributeurs](https://github.com/kabir308/OpenSkySea-Nexus/graphs/contributors)

## 📞 Contact

- **GitHub Issues** : [Créer une issue](https://github.com/kabir308/OpenSkySea-Nexus/issues)
- **Discussions** : [Discussions](https://github.com/kabir308/OpenSkySea-Nexus/discussions)
- **Email** : (TODO: ajouter si disponible)

---

<div align="center">

**Fait avec ❤️ par la communauté OpenSkySea-Nexus**

Si ce projet vous aide, pensez à lui donner une ⭐ !

</div>
