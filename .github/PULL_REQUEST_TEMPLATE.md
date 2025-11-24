# Pull Request

## Description
<!-- Décrivez vos modifications de manière claire et concise -->

## Type de Changement
<!-- Cochez les cases pertinentes -->

- [ ] 🐛 Bug fix (changement non-breaking qui corrige un problème)
- [ ] ✨ Nouvelle fonctionnalité (changement non-breaking qui ajoute une fonctionnalité)
- [ ] 💥 Breaking change (correction ou fonctionnalité qui causerait un dysfonctionnement des fonctionnalités existantes)
- [ ] 📚 Documentation (amélioration ou correction de la documentation)
- [ ] ♻️ Refactoring (amélioration du code sans changer la fonctionnalité)
- [ ] ⚡ Performance (amélioration des performances)
- [ ] 🧪 Tests (ajout ou correction de tests)
- [ ] 🔧 Configuration (modifications des fichiers de config, Docker, CI/CD)

## Motivation et Contexte
<!-- Pourquoi ce changement est-il nécessaire ? Quel problème résout-il ? -->

**Issue(s) liée(s) :** #(numéro)

## Comment a-t-il été testé ?
<!-- Décrivez les tests que vous avez effectués -->

- [ ] Tests unitaires
- [ ] Tests d'intégration
- [ ] Tests manuels en simulation
- [ ] Tests sur matériel réel

**Environnement de test :**
- OS : 
- ROS 2 Version : 
- Configuration : Docker / Native

**Commandes de test :**
```bash
# Commandes utilisées pour tester
colcon test --packages-select my_package
ros2 launch ...
```

## Checklist
<!-- Cochez les cases au fur et à mesure que vous complétez les étapes -->

- [ ] Mon code suit les conventions de style du projet
- [ ] J'ai effectué une auto-revue de mon code
- [ ] J'ai commenté mon code, particulièrement dans les zones complexes
- [ ] J'ai mis à jour la documentation en conséquence
- [ ] Mes modifications ne génèrent pas de nouveaux warnings
- [ ] J'ai ajouté des tests qui prouvent que ma correction est efficace ou que ma fonctionnalité fonctionne
- [ ] Les tests unitaires nouveaux et existants passent localement avec mes modifications
- [ ] Toutes les dépendances ont été mises à jour (package.xml, CMakeLists.txt, etc.)

## Compilation et Tests
<!-- Résultats de la compilation et des tests -->

```bash
# Résultats colcon build
colcon build --packages-select ...
# Status : ✅ / ❌

# Résultats colcon test
colcon test --packages-select ...
# Status : ✅ / ❌
```

## Changements Détaillés

### Fichiers Modifiés
<!-- Liste des fichiers principaux modifiés -->

- `src/.../file1.cpp` : Description des changements
- `src/.../file2.py` : Description des changements
- `docs/.../doc.md` : Documentation mise à jour

### API Changes (si applicable)
<!-- Décrivez les changements d'API -->

**Avant :**
```python
old_function(param1, param2)
```

**Après :**
```python
new_function(param1, param2, new_param3)
```

## Captures d'Écran / Vidéos (si applicable)
<!-- Ajoutez des captures d'écran ou vidéos démontrant les changements -->

## Performance Impact (si applicable)
<!-- Impact sur les performances -->

- Temps de compilation : 
- Utilisation mémoire : 
- FPS simulation : 

## Breaking Changes (si applicable)
<!-- Décrivez les breaking changes et comment migrer -->

**Migration requise :**
```bash
# Étapes pour migrer vers cette version
```

## Dépendances
<!-- Nouvelles dépendances ajoutées -->

- [ ] Aucune nouvelle dépendance
- [ ] Nouvelles dépendances ajoutées (listées ci-dessous)

**Nouvelles dépendances :**
- Package X version Y.Z
- Bibliothèque A version B.C

## Notes pour les Reviewers
<!-- Informations pour aider les reviewers -->

**Points à vérifier particulièrement :**
- 
- 

**Zones nécessitant une attention particulière :**
- 
- 

## Post-Merge Actions
<!-- Actions à effectuer après le merge -->

- [ ] Mettre à jour la documentation en ligne
- [ ] Créer une release note
- [ ] Annoncer dans Discussions
- [ ] Autre : ___________

## Contexte Additionnel
<!-- Toute autre information utile -->
