---
name: 🐛 Rapport de Bug
about: Signaler un bug pour nous aider à améliorer le projet
title: '[BUG] '
labels: bug
assignees: ''
---

## Description du Bug
<!-- Une description claire et concise du bug -->

## Environnement
<!-- Complétez les informations suivantes -->

- **OS** : [ex. Ubuntu 22.04]
- **ROS 2 Version** : [ex. Humble]
- **Docker** : [Oui/Non]
- **GPU** : [ex. NVIDIA GTX 1660, Aucun]
- **RAM** : [ex. 16 GB]
- **Version du projet** : [ex. commit SHA ou tag]

## Étapes pour Reproduire
<!-- Étapes pour reproduire le comportement -->

1. Lancer '...'
2. Exécuter la commande '...'
3. Observer l'erreur '...'

## Comportement Attendu
<!-- Une description claire de ce que vous attendiez -->

## Comportement Actuel
<!-- Ce qui se passe actuellement -->

## Logs et Messages d'Erreur
<!-- Coller les logs pertinents ici -->

```
Coller les logs ici
```

## Captures d'Écran
<!-- Si applicable, ajouter des captures d'écran -->

## Informations Supplémentaires

### Configuration Docker (si applicable)
```bash
docker version
docker info | grep -i memory
```

### Configuration ROS 2
```bash
ros2 --version
echo $ROS_DISTRO
```

### État du Système
```bash
# RAM
free -h

# Espace disque
df -h

# GPU (si applicable)
nvidia-smi
```

## Solutions Tentées
<!-- Qu'avez-vous déjà essayé ? -->

- [ ] Consulté [TROUBLESHOOTING.md](../docs/TROUBLESHOOTING.md)
- [ ] Consulté [FAQ.md](../docs/FAQ.md)
- [ ] Recherché dans les issues existantes
- [ ] Essayé de recompiler : `colcon build --cmake-clean-cache`
- [ ] Essayé en mode headless
- [ ] Autre : ___________

## Contexte Additionnel
<!-- Toute autre information pertinente -->
