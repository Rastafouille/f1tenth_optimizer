import numpy as np

class EquidistanceNavigator:
    def __init__(self, 
                 stop_distance=0.5,         # Distance minimale d'arrêt (m)
                 max_detection_dist=3.0,     # Distance maximale de détection (m)
                 max_linear_speed=1.0,      # Vitesse linéaire maximale (m/s)
                 max_angular_speed=2.0,     # Vitesse angulaire maximale (rad/s)
                 left_sector_start=60,      # Angle de début du secteur gauche
                 left_sector_end=120,       # Angle de fin du secteur gauche
                 right_sector_start=240,    # Angle de début du secteur droit
                 right_sector_end=300,      # Angle de fin du secteur droit
                 front_sector_size=30):     # Taille du secteur avant (degrés)
        
        # Paramètres de configuration
        self.stop_distance = stop_distance
        self.max_detection_dist = max_detection_dist
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        
        # Convertir les angles en indices pour le scan
        self.left_sector_start = int(left_sector_start * 360 / 360)
        self.left_sector_end = int(left_sector_end * 360 / 360)
        self.right_sector_start = int(right_sector_start * 360 / 360)
        self.right_sector_end = int(right_sector_end * 360 / 360)
        self.front_sector_size = int(front_sector_size * 360 / 360)
        
        # État interne
        self.last_error = 0.0
        self.error_integral = 0.0
        
        # Paramètres du contrôleur PID
        self.kp = 1.0  # Gain proportionnel
        self.ki = 0.1  # Gain intégral
        self.kd = 0.2  # Gain dérivé
        
    def get_sector_distance(self, scan, start_idx, end_idx):
        """Calcule la distance moyenne dans un secteur donné"""
        sector = scan[start_idx:end_idx]
        if len(sector) == 0:
            return self.max_detection_dist
            
        distances = np.sqrt(sector[:, 0]**2 + sector[:, 1]**2)
        distances = distances[distances < self.max_detection_dist]
        
        if len(distances) == 0:
            return self.max_detection_dist
            
        return np.mean(distances)
    
    def compute_command(self, scan):
        """Calcule les commandes de vitesse pour maintenir l'équidistance"""
        # Calculer les distances moyennes à gauche et à droite
        left_dist = self.get_sector_distance(scan, self.left_sector_start, self.left_sector_end)
        right_dist = self.get_sector_distance(scan, self.right_sector_start, self.right_sector_end)
        front_dist = self.get_sector_distance(scan, 
                                           360 - self.front_sector_size//2,
                                           360 + self.front_sector_size//2)
        
        # Calculer l'erreur (différence entre les distances)
        error = right_dist - left_dist
        
        # Calculer les termes du PID
        error_derivative = error - self.last_error
        self.error_integral = np.clip(self.error_integral + error, -1.0, 1.0)
        
        # Calculer la commande angulaire avec le PID
        angular_vel = (self.kp * error + 
                      self.ki * self.error_integral +
                      self.kd * error_derivative)
        
        # Limiter la vitesse angulaire
        angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
        
        # Mettre à jour l'erreur précédente
        self.last_error = error
        
        # Calculer la vitesse linéaire en fonction de la distance frontale
        linear_vel = self.max_linear_speed
        if front_dist < self.stop_distance * 2:
            linear_vel = 0.0
        elif front_dist < self.stop_distance * 4:
            linear_vel = self.max_linear_speed * (front_dist - self.stop_distance * 2) / (self.stop_distance * 2)
        
        # Afficher les informations de navigation
        print("\n=== Navigation (Equidistance) ===")
        print(f"Distance gauche: {left_dist:.2f}m")
        print(f"Distance droite: {right_dist:.2f}m")
        print(f"Distance avant: {front_dist:.2f}m")
        print(f"Erreur d'équidistance: {error:.2f}m")
        print(f"Commandes: v={linear_vel:.2f} m/s, w={angular_vel:.2f} rad/s")
        print("================================")
        
        return linear_vel, angular_vel 