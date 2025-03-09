# Simulateur de Voiture avec Lidar

Un simulateur 2D d'une voiture équipée d'un lidar, capable de navigation manuelle et autonome.

## Installation

1. Assurez-vous d'avoir Python installé sur votre ordinateur
2. Installez les dépendances :
```bash
pip install -r requirements.txt
```

## Structure du Projet

- `car_simulator.py` : Fichier principal du simulateur
- `car.py` : Classe gérant la cinématique du véhicule
- `lidar.py` : Classe simulant le capteur lidar
- `autonomous_navigator.py` : Classe gérant la navigation autonome
- `map.yaml` : Configuration de la carte
- `TRR.bmp` : Image de la carte

## Utilisation

1. Lancez le simulateur :
```bash
python car_simulator.py
```

2. Contrôles :
- Flèches directionnelles : contrôle manuel
  - Haut/Bas : vitesse linéaire (avant/arrière)
  - Gauche/Droite : vitesse angulaire (rotation)
- Espace : activer/désactiver la navigation autonome
- Échap : quitter le simulateur

## Fonctionnalités

- Affichage en temps réel de la carte et du véhicule
- Simulation de capteur lidar
- Navigation manuelle
- Navigation autonome avec évitement d'obstacles
- Détection de collision avec arrêt d'urgence
- Visualisation du scan lidar

## Configuration

- La carte est configurée dans `map.yaml`
- Les paramètres de simulation peuvent être ajustés dans les fichiers respectifs :
  - Paramètres du véhicule dans `car.py`
  - Configuration du lidar dans `lidar.py`
  - Paramètres de navigation dans `autonomous_navigator.py`
  - Seuils de collision dans `car_simulator.py`

## Logs

Le simulateur affiche plusieurs informations en temps réel :
- Position et orientation du véhicule
- Vitesses linéaire et angulaire
- Distances aux obstacles
- État de la détection de collision
- Mode de navigation actif

## Format de la Carte

- Image en niveaux de gris (format BMP)
- Pixels noirs : obstacles
- Pixels blancs : espace libre
- Configuration dans le fichier YAML :
  - Resolution : mètres par pixel
  - Origin : position de l'origine de la carte 