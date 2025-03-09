import numpy as np
import cv2

class Lidar:
    def __init__(self, map_img, map_info):
        self.map_img = map_img
        self.map_info = map_info
        self.resolution = map_info['resolution']
        
        # Position du lidar
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Paramètres du lidar
        self.num_beams = 360  # nombre de rayons
        self.max_range = 5.0  # portée maximale en mètres
        self.min_range = 0.1  # portée minimale en mètres
        self.angle_increment = 2 * np.pi / self.num_beams
        
    def update(self, x, y, theta):
        """Met à jour la position du lidar"""
        self.x = x
        self.y = y
        self.theta = theta
        
    def get_scan(self):
        """
        Génère un scan lidar à partir de la position actuelle du véhicule
        """
        scan = np.zeros((self.num_beams, 2))
        
        # Convertir la position du lidar de mètres en pixels
        lidar_x_pixel = int(self.x / self.resolution)
        lidar_y_pixel = int(self.y / self.resolution)
        # Inverser Y car l'image a Y vers le bas
        lidar_y_pixel = self.map_img.shape[0] - lidar_y_pixel
        
        # Pour chaque rayon
        for i in range(self.num_beams):
            # Angle dans le repère du véhicule (0 devant, sens horaire)
            angle = self.theta + (i * self.angle_increment)
            
            # Rayon de recherche
            for r in range(int(self.min_range / self.resolution), 
                          int(self.max_range / self.resolution)):
                # Point sur le rayon en pixels
                x_pixel = int(lidar_x_pixel + r * np.cos(angle))
                y_pixel = int(lidar_y_pixel - r * np.sin(angle))  # Inverser le signe de sin pour Y
                
                # Vérifier les limites de l'image
                if (x_pixel < 0 or x_pixel >= self.map_img.shape[1] or 
                    y_pixel < 0 or y_pixel >= self.map_img.shape[0]):
                    break
                
                # Si on rencontre un obstacle
                if self.map_img[y_pixel, x_pixel] < 128:  # pixel noir = obstacle
                    # Convertir en coordonnées relatives au véhicule en mètres
                    scan[i, 0] = r * self.resolution * np.cos(angle)
                    scan[i, 1] = r * self.resolution * np.sin(angle)
                    break
        
        return scan 