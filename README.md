# F1TENTH Teleoperation Visualizer

Ce projet implémente une interface de téléopération pour le simulateur F1TENTH avec visualisation en temps réel des données LIDAR. Il permet de contrôler manuellement un véhicule F1TENTH dans un environnement simulé tout en visualisant les retours des capteurs et l'état du véhicule.

# Configuration de l'environnement F1TENTH Gym

Ce guide détaille les étapes pour configurer correctement l'environnement de simulation F1TENTH Gym.

## Prérequis

- Windows 10 ou plus récent
- Python 3.8 (spécifiquement cette version)
- Git

## Installation étape par étape

1. **Créer un environnement virtuel Python**
   ```powershell
   # Créer un nouvel environnement virtuel
   py -3.8 -m venv f1tenth_env
   
   # Activer l'environnement
   .\f1tenth_env\Scripts\activate
   ```

2. **Mettre à jour pip à une version spécifique**
   ```powershell
   python -m pip install pip==21.3.1
   ```

3. **Installer les dépendances (2 méthodes possibles)**
   ```powershell
   # Installer à partir du fichier requirements.txt
   pip install -r requirements.txt
   ```

4. **Cloner et installer F1TENTH Gym**
   ```powershell
   # Cloner le dépôt
   git clone https://github.com/f1tenth/f1tenth_gym.git
   cd f1tenth_gym
   
   # Installer en mode développement
   pip install -e .
   ```

## Vérification de l'installation

Pour vérifier que tout fonctionne correctement :

1. **Tester l'exemple de base**
   ```powershell
   cd examples
   python waypoint_follow.py
   ```
   Une fenêtre devrait s'ouvrir montrant la simulation.

## Description du Projet

### Fonctionnalités
- Interface de téléopération avec visualisation LIDAR en temps réel
- Affichage des paramètres du véhicule (vitesse, direction)
- Détection des collisions
- Visualisation du scan laser avec FOV 270° et 1080 points
- Interface graphique optimisée avec PyGame

### Contrôles
- Flèches Haut/Bas : Accélération/Freinage
- Flèches Gauche/Droite : Direction
- 'R' : Reset de la position
- 'Échap' : Quitter

### Caractéristiques Techniques
- Fenêtre principale : 500x400 pixels
- Zone de scan : 200x200 pixels
- Échelle de visualisation : 40 pixels/mètre
- Limite de détection : 10 mètres
- Intégration RK4 pour la simulation physique
- FOV du LIDAR automatiquement détecté

### Structure du Projet
```
f1tenth_optimizer/
├── main.py              # Programme principal
├── navigation.py        # Classe de téléopération
├── maps/               # Dossier des cartes
│   └── example_map.png
└── README.md
```

