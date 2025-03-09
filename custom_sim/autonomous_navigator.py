import numpy as np
import cv2
import yaml
import matplotlib.pyplot as plt
from car import Car
from lidar import Lidar


class AutonomousNavigator:
    def __init__(self, 
                 # Paramètres de détection d'obstacles
                 stop_distance=0.5,      # mètres
                 left_sector_start=315,  # angle de début du secteur gauche (-45°)
                 left_sector_end=340,    # angle de fin du secteur gauche (-20°)
                 right_sector_start=20,  # angle de début du secteur droit (20°)
                 right_sector_end=45,    # angle de fin du secteur droit (45°)
                 # Paramètres de vitesse
                 normal_linear_speed=2.0,    # vitesse linéaire normale (m/s)
                 normal_angular_speed=0.0,   # vitesse angulaire normale (rad/s)
                 obstacle_linear_speed=1.0,  # vitesse linéaire en présence d'obstacle (m/s)
                 obstacle_angular_speed=6.0,  # vitesse angulaire en présence d'obstacle (rad/s)
                 min_gap_width=0.6,      # largeur minimale d'espace libre (m)
                 max_detection_dist=3.0,  # distance maximale de détection (m)
                 safety_margin=0.3):      # marge de sécurité (m)
        # Paramètres de détection d'obstacles
        self.stop_distance = stop_distance
        self.left_sector_start = int(left_sector_start)
        self.left_sector_end = int(left_sector_end)
        self.right_sector_start = int(right_sector_start)
        self.right_sector_end = int(right_sector_end)
        
        # Paramètres de vitesse
        self.normal_linear_speed = normal_linear_speed
        self.normal_angular_speed = normal_angular_speed
        self.obstacle_linear_speed = obstacle_linear_speed
        self.obstacle_angular_speed = obstacle_angular_speed
        
        self.min_gap_width = min_gap_width
        self.max_detection_dist = max_detection_dist
        self.safety_margin = safety_margin
        
        # État de la navigation
        self.obstacle_right_detected = False
        self.obstacle_left_detected = False
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def process_scan(self, scan):
        """Analyse le scan lidar pour la détection d'obstacles"""
        ranges = np.sqrt(scan[:, 0]**2 + scan[:, 1]**2)
        
        # Vérifier les obstacles à droite et à gauche
        front_left = ranges[self.left_sector_start:self.left_sector_end]
        front_right = ranges[self.right_sector_start:self.right_sector_end]
        
        min_right = min(front_right) if len(front_right) > 0 else float('inf')
        min_left = min(front_left) if len(front_left) > 0 else float('inf')
        
        self.obstacle_right_detected = min_right < self.stop_distance
        self.obstacle_left_detected = min_left < self.stop_distance
        
        print("\n=== Navigation ===")
        print(f"Distance - droite: {min_right:.2f}m (seuil: {self.stop_distance}m)")
        print(f"Distance - gauche: {min_left:.2f}m (seuil: {self.stop_distance}m)")
        print(f"Obstacle droite: {self.obstacle_right_detected}")
        print(f"Obstacle gauche: {self.obstacle_left_detected}")
        print("================")

    def find_largest_gap(self, scan):
        """Trouve le plus grand espace libre dans le scan"""
        # Convertir le scan en distances
        ranges = np.sqrt(scan[:, 0]**2 + scan[:, 1]**2)
        angles = np.arange(len(ranges)) * (2 * np.pi / len(ranges))
        
        # Limiter la portée de détection
        ranges[ranges > self.max_detection_dist] = self.max_detection_dist
        
        # Trouver les espaces libres
        gaps = []
        gap_start = 0
        in_gap = False
        
        for i in range(len(ranges)):
            if ranges[i] > (self.stop_distance + self.safety_margin):
                if not in_gap:
                    gap_start = i
                    in_gap = True
            else:
                if in_gap:
                    gap_end = i
                    gap_width = abs(ranges[gap_start:gap_end].mean() * 
                                  (angles[gap_end] - angles[gap_start]))
                    if gap_width >= self.min_gap_width:
                        gaps.append((gap_start, gap_end, gap_width))
                    in_gap = False
        
        # Gérer le dernier gap si nécessaire
        if in_gap:
            gap_end = len(ranges)
            gap_width = abs(ranges[gap_start:gap_end].mean() * 
                          (angles[gap_end-1] - angles[gap_start]))
            if gap_width >= self.min_gap_width:
                gaps.append((gap_start, gap_end, gap_width))
        
        if not gaps:
            return None, None, 0
            
        # Trouver le plus grand gap
        largest_gap = max(gaps, key=lambda x: x[2])
        return largest_gap[0], largest_gap[1], largest_gap[2]
    
    def compute_command(self, scan):
        """Calcule les commandes de vitesse en fonction du scan"""
        # Trouver le plus grand espace libre
        gap_start, gap_end, gap_width = self.find_largest_gap(scan)
        
        if gap_start is None:
            # Aucun espace suffisant trouvé, reculer ou tourner sur place
            print("Navigation: Aucun passage trouvé - Manœuvre d'évitement")
            return -0.2, 1.0  # Reculer en tournant
            
        # Calculer l'angle vers le centre du gap
        ranges = np.sqrt(scan[:, 0]**2 + scan[:, 1]**2)
        gap_center_idx = (gap_start + gap_end) // 2
        gap_angle = gap_center_idx * (2 * np.pi / len(ranges))
        
        # Normaliser l'angle entre -pi et pi
        if gap_angle > np.pi:
            gap_angle -= 2 * np.pi
            
        # Calculer les vitesses
        # La vitesse angulaire est proportionnelle à l'angle vers le gap
        angular_vel = 2.0 * gap_angle
        
        # La vitesse linéaire diminue quand on tourne beaucoup
        linear_vel = self.normal_linear_speed * (1.0 - abs(gap_angle) / np.pi)
        linear_vel = max(0.2, linear_vel)  # Garder une vitesse minimale
        
        # Afficher les informations de navigation
        print("\n=== Navigation ===")
        print(f"Largeur du passage: {gap_width:.2f}m")
        print(f"Angle vers le passage: {np.degrees(gap_angle):.1f}°")
        print(f"Commandes: v={linear_vel:.2f} m/s, w={angular_vel:.2f} rad/s")
        print("================")
        
        return linear_vel, angular_vel 