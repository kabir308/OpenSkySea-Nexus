# FAQ - Foire Aux Questions

Questions fréquemment posées sur OpenSkySea-Nexus.

## Général

### Qu'est-ce qu'OpenSkySea-Nexus ?

OpenSkySea-Nexus est un écosystème open source unifié pour le développement et la simulation de véhicules autonomes multi-environnements (air, mer, terre). Il combine :
- Simulation réaliste avec Gazebo
- Framework ROS 2 pour la robotique
- Support multi-pilotes automatiques (PX4, ArduPilot, etc.)
- IA embarquée et apprentissage collaboratif

### À qui s'adresse ce projet ?

- **Chercheurs** : Recherche en robotique autonome, IA, systèmes multi-agents
- **Étudiants** : Apprentissage de ROS 2, robotique, systèmes autonomes
- **Développeurs** : Prototypage d'applications robotiques
- **Passionnés** : Exploration des technologies autonomes

### Est-ce gratuit ?

Oui, complètement. OpenSkySea-Nexus est open source sous licence MIT/Apache (vérifier LICENSE).

## Installation

### Quels sont les prérequis ?

**Minimaux :**
- Ubuntu 22.04 (recommandé) ou Docker
- 8 GB RAM
- 30 GB d'espace disque
- Connexion internet

**Recommandés :**
- 16 GB RAM
- GPU NVIDIA
- 50 GB SSD
- Voir [HARDWARE_REQUIREMENTS.md](./HARDWARE_REQUIREMENTS.md)

### Dois-je utiliser Docker ?

**Docker est recommandé si :**
- Vous débutez avec ROS 2
- Vous voulez une configuration rapide et reproductible
- Vous utilisez Windows/Mac

**Installation native si :**
- Vous êtes sur Ubuntu 22.04
- Vous voulez les meilleures performances
- Vous avez de l'expérience avec ROS 2

### Puis-je utiliser Windows ou Mac ?

**Windows :**
- ✅ Via Docker Desktop + WSL2
- ⚠️ Performances réduites
- ❌ Pas de support GPU natif (pour l'instant)

**Mac :**
- ✅ Via Docker Desktop (Intel/Apple Silicon)
- ⚠️ Performances limitées
- ❌ Pas de support GPU

**Recommandation :** Ubuntu 22.04 pour la meilleure expérience.

### Combien de temps prend l'installation ?

- **Docker build :** 15-30 minutes (première fois)
- **Compilation colcon :** 5-20 minutes (selon CPU)
- **Total :** ~30-60 minutes

### Dois-je avoir un GPU ?

**Non obligatoire, mais recommandé :**
- ✅ Sans GPU : Mode headless, simulations légères
- ✅ Avec GPU : Visualisation 3D Gazebo, simulations complexes

Voir [PERFORMANCE_OPTIMIZATION.md](./PERFORMANCE_OPTIMIZATION.md#mode-sans-gpu)

## Utilisation

### Comment lancer ma première simulation ?

```bash
# Dans le conteneur Docker ou après installation native
source install/setup.bash
ros2 launch simulation_bringup simulation.launch.py
```

Voir [QUICK_START.md](./QUICK_START.md) pour un guide complet.

### Quels types de véhicules sont supportés ?

**Air :**
- Quadricoptères
- Drones fixes
- VTOL (vertical takeoff and landing)

**Mer :**
- Bateaux de surface autonomes (ASV)
- Sous-marins (ROV/AUV)

**Terre :**
- Rovers
- Robots mobiles

### Puis-je ajouter mes propres robots ?

Oui ! Le projet est extensible :
1. Créer un package ROS 2 dans `src/`
2. Définir votre modèle URDF/SDF
3. Créer un launch file
4. Compiler avec `colcon build`

Voir documentation sur l'ajout de nouveaux véhicules (TODO: lien).

### Comment puis-je contribuer ?

Voir [CONTRIBUTING.md](../CONTRIBUTING.md) pour :
- Guidelines de contribution
- Standards de code
- Processus de pull request

## Technique

### Quelle version de ROS utiliser ?

**ROS 2 Humble (LTS)** - recommandé et supporté.

ROS 1 n'est pas supporté (projet moderne ROS 2 only).

### Puis-je utiliser un autre simulateur que Gazebo ?

Actuellement, Gazebo (Classic et Ignition/Gazebo) est le simulateur principal.

**Alternatives possibles (contributions bienvenues) :**
- Unity avec ROS 2
- Webots
- Isaac Sim

### Comment changer le nombre de robots ?

```bash
ros2 launch simulation_bringup simulation.launch.py num_robots:=5
```

Ou éditer le launch file.

### Puis-je exécuter sur plusieurs machines ?

Oui ! ROS 2 supporte les systèmes distribués.

Configuration (TODO: lien vers documentation distribuée) :
```bash
# Machine 1
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Machine 2 (même configuration)
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

### Comment enregistrer une simulation ?

```bash
# Enregistrer tous les topics
ros2 bag record -a

# Enregistrer topics spécifiques
ros2 bag record /robot/pose /robot/cmd_vel

# Rejouer
ros2 bag play my_recording.db3
```

## Performance

### Pourquoi est-ce lent sur mon ordinateur ?

Plusieurs raisons possibles :
1. **Machine limitée** : Voir [HARDWARE_REQUIREMENTS.md](./HARDWARE_REQUIREMENTS.md)
2. **Pas de GPU** : Utiliser mode headless
3. **Trop de robots** : Réduire à 1-2
4. **Configuration non optimisée** : Voir [PERFORMANCE_OPTIMIZATION.md](./PERFORMANCE_OPTIMIZATION.md)

### Comment optimiser les performances ?

**Quick wins :**
```bash
# Mode headless (sans GUI)
ros2 launch simulation_bringup simulation.launch.py gui:=false

# Moins de robots
ros2 launch simulation_bringup simulation.launch.py num_robots:=1

# Compilation parallèle
colcon build --parallel-workers $(nproc)
```

Guide complet : [PERFORMANCE_OPTIMIZATION.md](./PERFORMANCE_OPTIMIZATION.md)

### Combien de robots puis-je simuler ?

Dépend de votre machine :
- **8 GB RAM, pas de GPU :** 1-2 robots
- **16 GB RAM, GPU milieu :** 3-5 robots
- **32 GB RAM, bon GPU :** 10+ robots

### La compilation est très longue, normal ?

**Oui**, première compilation peut prendre 10-30 minutes.

**Accélérer :**
- Utiliser ccache
- Augmenter `--parallel-workers`
- SSD au lieu de HDD
- Désactiver tests : `--cmake-args -DBUILD_TESTING=OFF`

## Développement

### Quels langages sont utilisés ?

- **Python** : Scripts, nodes ROS 2, IA
- **C++** : Nodes performance-critiques, plugins Gazebo
- **XML/YAML** : Configuration, launch files
- **URDF/SDF** : Description de robots

### Dois-je connaître ROS ?

**Recommandé mais pas obligatoire.**

Ressources pour apprendre :
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [The Construct](https://www.theconstructsim.com/)
- [ROS 2 Book](https://github.com/osrf/ros2multirobotbook)

### Comment débugger un robot qui ne bouge pas ?

```bash
# 1. Vérifier les topics
ros2 topic list

# 2. Vérifier publication commandes
ros2 topic echo /robot/cmd_vel

# 3. Vérifier les nodes
ros2 node list

# 4. Tester manuellement
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
```

Voir [TROUBLESHOOTING.md](./TROUBLESHOOTING.md) pour plus de solutions.

### Comment ajouter un capteur ?

1. Modifier le fichier URDF/SDF du robot
2. Ajouter le plugin Gazebo correspondant
3. Créer un node pour traiter les données
4. Recompiler

Exemple (Lidar) : TODO - lien vers tutoriel

### Puis-je utiliser du machine learning ?

Oui ! Le projet inclut :
- Support TensorFlow/PyTorch
- Modules d'apprentissage distribué
- Exemples dans `src/collaborative_intelligence/`

## Problèmes Courants

### Docker build échoue

Causes courantes :
- Connexion internet instable
- Espace disque insuffisant
- Permissions Docker

Solutions : [TROUBLESHOOTING.md#docker](./TROUBLESHOOTING.md#docker)

### Gazebo ne s'affiche pas

```bash
# Vérifier X11 (Linux)
echo $DISPLAY
xhost +local:docker

# Ou utiliser headless
ros2 launch simulation_bringup simulation.launch.py gui:=false
```

### "Package not found" lors du lancement

```bash
# Sourcer l'environnement
source install/setup.bash

# Recompiler si nécessaire
colcon build --packages-select package_name
```

### Out of memory lors de la compilation

```bash
# Réduire workers
colcon build --parallel-workers 1

# Ajouter swap
sudo fallocate -l 8G /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

Voir [TROUBLESHOOTING.md](./TROUBLESHOOTING.md) pour tous les problèmes.

## Recherche et Académique

### Puis-je utiliser ce projet pour ma thèse/recherche ?

**Absolument !** C'est encouragé.

**Si vous publiez :**
- Citez le projet OpenSkySea-Nexus
- Partagez vos résultats (optionnel mais apprécié)
- Contribuez vos améliorations si possible

### Y a-t-il des publications associées ?

TODO: Ajouter références quand disponibles.

### Comment citer ce projet ?

```bibtex
@software{openskysea_nexus,
  author = {OpenSkySea-Nexus Contributors},
  title = {OpenSkySea-Nexus: Open Source Ecosystem for Multi-Domain Autonomous Vehicles},
  year = {2024},
  url = {https://github.com/kabir308/OpenSkySea-Nexus}
}
```

### Puis-je utiliser ceci pour un projet commercial ?

Vérifier la licence (LICENSE file). Généralement :
- ✅ Utilisation commerciale autorisée
- ✅ Modifications autorisées
- ⚠️ Vérifier obligations de la licence (attribution, etc.)

## Communauté

### Comment obtenir de l'aide ?

1. **Documentation** : Lire docs/
2. **FAQ** : Ce fichier
3. **Troubleshooting** : [TROUBLESHOOTING.md](./TROUBLESHOOTING.md)
4. **GitHub Discussions** : Pour questions générales
5. **GitHub Issues** : Pour bugs/features

### Y a-t-il un forum ou chat ?

Actuellement :
- GitHub Discussions (recommandé)
- GitHub Issues

TODO: Considérer Discord/Slack si la communauté grandit.

### Comment rester informé des mises à jour ?

- ⭐ Star le projet sur GitHub
- 👁️ Watch le repository
- Suivre les releases
- Lire le CHANGELOG (TODO)

### Puis-je devenir mainteneur ?

Les contributeurs actifs et de qualité peuvent être invités à devenir mainteneurs.

**Processus :**
1. Contribuer régulièrement (PRs de qualité)
2. Aider dans issues/discussions
3. Démontrer expertise dans un domaine
4. Contact par les mainteneurs actuels

## Roadmap

### Quelles sont les prochaines fonctionnalités ?

Voir le projet GitHub pour la roadmap complète (TODO: lien).

**Priorités :**
- Support Gazebo Ignition
- Plus de modèles de véhicules
- Tutoriels vidéo
- Support cloud (AWS, Azure)
- Interface web de monitoring

### Comment proposer une fonctionnalité ?

1. Vérifier qu'elle n'existe pas déjà (issues)
2. Ouvrir une issue avec template "Feature Request"
3. Décrire clairement le besoin et cas d'usage
4. Discussion communautaire
5. Implémentation (par vous ou autres contributeurs)

### Y a-t-il un business model ?

**Non.** OpenSkySea-Nexus est un projet de recherche open source.

**Possibilités futures :**
- Support commercial (consulting)
- Formation payante
- Services cloud
- Rien de défini pour l'instant

## Divers

### Pourquoi "OpenSkySea-Nexus" ?

- **Open** : Open source
- **Sky** : Domaine aérien
- **Sea** : Domaine maritime
- **Nexus** : Point de connexion entre les domaines

### Qui développe ce projet ?

Projet open source communautaire.

Contributeurs : Voir [Contributors](https://github.com/kabir308/OpenSkySea-Nexus/graphs/contributors)

### Puis-je redistribuer ce projet ?

Oui, selon les termes de la licence (voir LICENSE).

**Bonnes pratiques :**
- Maintenir attribution originale
- Indiquer modifications
- Respecter la licence

### Ce projet est-il actif ?

Vérifier :
- Date du dernier commit
- Issues/PRs récentes
- Fréquence des releases

**Si inactif :** Vous pouvez fork et maintenir votre version !

## Questions Non Répondues ?

**Cette FAQ n'a pas répondu à votre question ?**

1. Cherchez dans [Issues existantes](https://github.com/kabir308/OpenSkySea-Nexus/issues)
2. Consultez la [documentation complète](.)
3. Posez votre question dans [GitHub Discussions](https://github.com/kabir308/OpenSkySea-Nexus/discussions)
4. Ouvrez une [nouvelle issue](https://github.com/kabir308/OpenSkySea-Nexus/issues/new)

---

**Dernière mise à jour :** Novembre 2024
**Contributeurs à cette FAQ :** [Liste]
