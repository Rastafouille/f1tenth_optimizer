import numpy as np
import cv2
import yaml
import matplotlib.pyplot as plt
from car import Car
from lidar import Lidar

class CarSimulator:
    def __init__(self, map_path="TRR.bmp", yaml_path="map.yaml", headless=False):
        """Simulateur de voiture avec lidar
        
        Args:
            map_path: chemin vers l'image de la carte
            yaml_path: chemin vers le fichier de configuration
            headless: si True, désactive l'interface graphique
        """
        self.headless = headless
        
        # Charger la carte
        self.map_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        if self.map_img is None:
            raise ValueError(f"Impossible de charger la carte: {map_path}")
            
        # Charger la configuration
        with open(yaml_path, 'r') as f:
            self.map_config = yaml.safe_load(f)
            
        # Position initiale de la voiture
        self.start_x = 4.0
        self.start_y = 1.5
        self.start_theta = 0.0
            
        # Initialiser la voiture et le lidar
        self.car = Car(self.start_x, self.start_y, self.start_theta)
        self.lidar = Lidar(self.map_img, self.map_config)
        self.lidar.update(self.car.x, self.car.y, self.car.theta)
        
        # État de la simulation
        self.collision_detected = False
        self.collision_distance = 0.1  # Réduit à 10cm
        self.car_length = 0.5  # mètres
        self.car_width = 0.3   # mètres
        
        # Créer la fenêtre et configurer l'affichage seulement si pas en mode headless
        if not self.headless:
            plt.ion()
            self.fig = plt.figure(figsize=(12, 5))
            self.fig.canvas.manager.set_window_title('Simulateur de voiture avec Lidar')
            
            self.ax1 = self.fig.add_subplot(121)
            self.ax2 = self.fig.add_subplot(122)
            
            # Rectangle rouge pour l'avertissement de collision
            self.warning_rect = plt.Rectangle((0, 0), 1, 1, facecolor='red', alpha=0.3)
            self.warning_rect.set_visible(False)
            self.ax1.add_patch(self.warning_rect)
            
            self.ax1.set_title("Vue de la carte")
            self.ax2.set_title("Vue du Lidar")
            
    def reset(self):
        """Réinitialise la simulation à son état initial"""
        self.car = Car(self.start_x, self.start_y, self.start_theta)
        self.lidar.update(self.car.x, self.car.y, self.car.theta)
        self.collision_detected = False
        
    def check_collision(self, scan):
        """Vérifie s'il y a une collision en détectant si un point du lidar est dans le rectangle du véhicule"""
        if len(scan) == 0:
            return False
            
        # Filtrer les points invalides (0,0) qui sont probablement des erreurs de mesure
        valid_mask = ~(np.isclose(scan[:, 0], 0.0) & np.isclose(scan[:, 1], 0.0))
        points = scan[valid_mask]
        
        if len(points) == 0:
            return False
            
        # Dimensions du rectangle du véhicule
        half_length = self.car_length / 2
        half_width = self.car_width / 2
        
        # Pour chaque point, vérifier s'il est à l'intérieur du rectangle du véhicule
        for x, y in points:
            # Si le point est à l'intérieur du rectangle
            if abs(x) <= half_length and abs(y) <= half_width:
                print(f"Collision détectée! Point ({x:.3f}, {y:.3f}) dans le rectangle du véhicule")
                self.collision_detected = True
                return True
                
        self.collision_detected = False
        return False
        
    def step(self, cmd_vel_linear, cmd_vel_angular, dt):
        """Fait avancer la simulation d'un pas de temps
        
        Args:
            cmd_vel_linear: vitesse linéaire commandée (m/s)
            cmd_vel_angular: vitesse angulaire commandée (rad/s)
            dt: pas de temps (s)
            
        Returns:
            scan: données du scan lidar
            collision: True si collision détectée
        """
        # Stocker les vitesses courantes
        self.car.cmd_vel_linear = cmd_vel_linear
        self.car.cmd_vel_angular = cmd_vel_angular
        
        # Mettre à jour la position de la voiture
        self.car.update(cmd_vel_linear, cmd_vel_angular, dt)
        
        # Mettre à jour la position du lidar
        self.lidar.update(self.car.x, self.car.y, self.car.theta)
        
        # Obtenir le scan lidar
        scan = self.lidar.get_scan()
        
        # Vérifier les collisions
        collision = self.check_collision(scan)
        
        # Afficher si nécessaire
        if not self.headless:
            self.render(scan)
            
        return scan, collision
            
    def render(self, scan):
        """Affiche l'état de la simulation"""
        if self.headless:
            return
            
        self.ax1.clear()
        
        # Calculer les dimensions de la carte en mètres
        width_meters = self.map_img.shape[1] * self.map_config['resolution']
        height_meters = self.map_img.shape[0] * self.map_config['resolution']
        
        # Afficher l'image
        self.ax1.imshow(self.map_img, 
                       extent=[0, width_meters, 0, height_meters],
                       cmap='gray')
        
        # Dessiner le véhicule
        points = np.array([
            [-self.car_length/2, -self.car_width/2],
            [self.car_length/2, -self.car_width/2],
            [self.car_length/2, self.car_width/2],
            [-self.car_length/2, self.car_width/2]
        ])
        
        # Rotation
        theta = self.car.theta
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        points = points @ R.T
        
        # Translation
        points = points + np.array([self.car.x, self.car.y])
        
        # Dessiner le rectangle
        self.ax1.fill(points[:, 0], points[:, 1], 'r', alpha=0.5)
        
        # Configurer les axes
        self.ax1.set_xlabel('X (mètres)')
        self.ax1.set_ylabel('Y (mètres)')
        
        # Titre avec les vitesses
        title = "Vue de la carte"
        if hasattr(self.car, 'cmd_vel_linear') and hasattr(self.car, 'cmd_vel_angular'):
            title += f"\nVitesse: {self.car.cmd_vel_linear:.1f} m/s"
            title += f" | Rotation: {self.car.cmd_vel_angular:.1f} rad/s"
        if self.collision_detected:
            title += "\nCOLLISION !"
        self.ax1.set_title(title)
        
        # Afficher le scan lidar
        self.ax2.clear()
        self.ax2.plot(scan[:, 0], scan[:, 1], 'b.', markersize=1)
        self.ax2.set_title("Vue du Lidar" + (" - COLLISION !" if self.collision_detected else ""))
        self.ax2.set_xlim(-5, 5)
        self.ax2.set_ylim(-5, 5)
        self.ax2.grid(True)
        
        # Ajouter un fond rouge en cas de collision
        if self.collision_detected:
            # Créer un rectangle rouge transparent qui couvre toute la figure
            self.ax1.add_patch(plt.Rectangle((0, 0), width_meters, height_meters, 
                                           facecolor='red', alpha=0.2))
            self.ax2.add_patch(plt.Rectangle((-5, -5), 10, 10, 
                                           facecolor='red', alpha=0.2))
        
        plt.pause(0.01) 