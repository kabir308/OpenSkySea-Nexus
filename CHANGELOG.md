# Changelog

Toutes les modifications notables de ce projet seront documentées dans ce fichier.

Le format est basé sur [Keep a Changelog](https://keepachangelog.com/fr/1.0.0/),
et ce projet adhère au [Semantic Versioning](https://semver.org/lang/fr/).

## [Unreleased]

### Ajouté
- Documentation complète pour améliorer l'accessibilité du projet
  - Guide de démarrage rapide (QUICK_START.md)
  - Configuration matérielle détaillée (HARDWARE_REQUIREMENTS.md)
  - Guide d'optimisation des performances (PERFORMANCE_OPTIMIZATION.md)
  - Guide de dépannage (TROUBLESHOOTING.md)
  - FAQ exhaustive (FAQ.md)
  - Guide d'installation multi-plateforme (INSTALLATION.md)
  - Roadmap du projet (ROADMAP.md)
- Templates GitHub pour améliorer l'engagement communautaire
  - Templates d'issues (bug, feature, question, documentation)
  - Template de pull request
- Code de conduite (CODE_OF_CONDUCT.md)
- README amélioré avec badges et structure claire
- Fichier .gitignore pour exclure les artefacts de build
- Ce CHANGELOG

### Modifié
- README principal restructuré pour meilleure lisibilité et accessibilité

### Contexte
Ces améliorations visent à répondre aux faiblesses identifiées pour le déploiement immédiat :
- Réduction de la courbe d'apprentissage
- Documentation des exigences matérielles
- Optimisation pour différentes configurations
- Meilleure accessibilité pour les nouveaux contributeurs

## [0.1.0] - 2024-XX-XX

### Ajouté
- Architecture multi-domaines de base (air/mer/terre)
- Configuration Docker initiale
- Support ROS 2 Humble
- Intégration Gazebo Classic
- Simulations multi-robots basiques
- Structure de projet modulaire
- Documentation initiale

---

## Format des Entrées

### Types de Changements
- `Ajouté` pour les nouvelles fonctionnalités
- `Modifié` pour les changements de fonctionnalités existantes
- `Déprécié` pour les fonctionnalités qui seront supprimées
- `Supprimé` pour les fonctionnalités supprimées
- `Corrigé` pour les corrections de bugs
- `Sécurité` pour les vulnérabilités

### Exemple d'Entrée
```markdown
## [Version] - YYYY-MM-DD

### Ajouté
- Nouvelle fonctionnalité X qui permet Y (#123)
- Support pour Z (#456)

### Modifié
- Amélioration de la performance de A (#789)
- Refactoring du module B pour meilleure maintenabilité

### Corrigé
- Bug dans C qui causait D (#101)
- Problème de compilation sur Ubuntu 24.04 (#202)

### Sécurité
- Correction de vulnérabilité CVE-XXXX-YYYY (#303)
```

[Unreleased]: https://github.com/kabir308/OpenSkySea-Nexus/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/kabir308/OpenSkySea-Nexus/releases/tag/v0.1.0
