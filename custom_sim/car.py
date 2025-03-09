import numpy as np

class Car:
    def __init__(self, x, y, theta):
        self.x = x      # position x en mètres
        self.y = y      # position y en mètres
        self.theta = theta  # orientation en radians
        
    def update(self, linear_vel, angular_vel, dt):
        """
        Met à jour la position du véhicule en utilisant le modèle de cinématique différentielle
        """
        # Mise à jour de l'orientation
        self.theta += angular_vel * dt
        
        # Mise à jour de la position
        self.x += linear_vel * np.cos(self.theta) * dt
        self.y += linear_vel * np.sin(self.theta) * dt 