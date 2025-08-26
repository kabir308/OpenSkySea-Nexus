# OpenSkySea-Nexus
Un écosystème open source unifié pour véhicules autonomes multi-environnements (air/mer/terre) avec IA embarquée et simulation collaborative

## Environnement de Développement (Docker)

Ce projet utilise Docker pour fournir un environnement de développement cohérent et reproductible.

### Prérequis

*   [Docker](https://docs.docker.com/get-docker/)

### Mise en place

1.  **Construire l'image Docker :**
    À la racine du projet, exécutez la commande suivante. Cela peut prendre un certain temps car elle télécharge l'image de base de ROS 2 et installe toutes les dépendances.
    ```bash
    docker build -t openskysea-nexus .
    ```

2.  **Lancer le conteneur de développement :**
    Exécutez la commande suivante pour démarrer un shell interactif à l'intérieur du conteneur. Le répertoire actuel du projet sera monté dans `/ros2_ws/src`, ce qui signifie que les modifications que vous apportez à vos fichiers locaux seront reflétées à l'intérieur du conteneur.
    ```bash
    docker run -it --rm -v .:/ros2_ws/src openskysea-nexus
    ```
    *Note pour les utilisateurs de Linux :* Pour permettre à l'interface graphique de Gazebo de s'afficher, vous devrez peut-être exécuter `xhost +local:docker` avant la commande `docker run`.

## Utilisation

Une fois que vous êtes à l'intérieur du shell du conteneur (`root@...:/ros2_ws#`), vous pouvez travailler avec le projet.

### Construire le projet

Pour compiler tous les paquets ROS 2, exécutez la commande suivante :
```bash
colcon build
```

### Lancer la Simulation

Pour lancer la simulation multi-véhicules (quadricoptère et rover) dans Gazebo, exécutez :
```bash
ros2 launch simulation_bringup simulation.launch.py
```
