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

## Vérification de l'installation

