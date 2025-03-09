import numpy as np
import pygame

class Teleoperation:
    def __init__(self):
        # Initialiser pygame pour la capture des touches
        pygame.init()
        # Créer une petite fenêtre invisible pour capturer les entrées clavier
        self.screen = pygame.display.set_mode((1, 1))
        pygame.display.set_caption("Teleop")
        
        # Paramètres de contrôle
        self.max_speed = 3.0  # m/s
        self.max_steer = 0.4  # rad
        self.speed = 0.0
        self.steer = 0.0
        
        # Paramètres d'accélération
        self.speed_increment = 0.1
        self.steer_increment = 0.1

    def plan(self, obs):
        """
        Retourne les actions de contrôle basées sur les entrées clavier
        Args:
            obs: observation de l'environnement (non utilisée pour la téléopération)
        Returns:
            numpy.ndarray: [angle_braquage, vitesse]
        """
        # Gestion des événements pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return None

        # Obtenir l'état des touches
        keys = pygame.key.get_pressed()
        
        # Mise à jour de la vitesse
        if keys[pygame.K_UP]:
            self.speed = min(self.speed + self.speed_increment, self.max_speed)
        elif keys[pygame.K_DOWN]:
            self.speed = max(self.speed - self.speed_increment, -self.max_speed)
        else:
            # Décélération naturelle
            self.speed = self.speed * 0.95

        # Mise à jour de la direction
        if keys[pygame.K_LEFT]:
            self.steer = min(self.steer + self.steer_increment, self.max_steer)
        elif keys[pygame.K_RIGHT]:
            self.steer = max(self.steer - self.steer_increment, -self.max_steer)
        else:
            # Retour progressif au centre
            self.steer = self.steer * 0.7

        # Arrêt d'urgence avec la touche espace
        if keys[pygame.K_SPACE]:
            self.speed = 0.0
            self.steer = 0.0

        return np.array([[self.steer, self.speed]])

    def __del__(self):
        pygame.quit()

class SimpleAutonomousController:
    def __init__(self, 
                 # Paramètres de contrôle
                 max_speed=3.0,           # m/s
                 max_steer=2.0,           # rad
                 min_front_dist=0.5,      # distance minimale frontale (m)
                 safety_margin=0.3,       # marge de sécurité (m)
                 
                 # Paramètres du scan
                 front_angle=30,          # demi-angle du secteur frontal (degrés)
                 side_angle=90):          # demi-angle des secteurs latéraux (degrés)
                 
        # Paramètres de contrôle
        self.max_speed = max_speed
        self.max_steer = max_steer
        self.min_front_dist = min_front_dist
        self.safety_margin = safety_margin
        
        # Paramètres du scan
        self.front_angle = front_angle
        self.side_angle = side_angle
        
        # Variables pour l'affichage
        self.front_dist = np.inf
        self.left_dist = np.inf
        self.right_dist = np.inf
        self.current_speed = 0.0
        self.current_steer = 0.0
        self.decision_msg = ""
        self.speed_msg = ""
        self.first_display = True
        
        # Variables pour le temps au tour et nombre de tours
        self.lap_time = 0.0
        self.lap_count = 0
        
    def move_cursor(self, line):
        """Déplace le curseur à une ligne spécifique"""
        print(f"\033[{line};1H", end="")
        
    def clear_line(self):
        """Efface la ligne courante"""
        print("\033[K", end="")
        
    def update_display(self):
        """Met à jour l'affichage dans le terminal"""
        if self.first_display:
            print("\033[2J", end="")  # Efface l'écran seulement la première fois
            self.first_display = False
            
        # Retour au début
        print("\033[H", end="")
        
        # Affichage ligne par ligne avec effacement
        lines = [
            "╔══════════════════════════════════════════════════════╗",
            "║            === État du véhicule ===                  ║",
            "║                                                      ║",
            f"║  Distance avant  : {self.front_dist:6.2f} m                    ║",
            f"║  Distance gauche : {self.left_dist:6.2f} m                    ║",
            f"║  Distance droite : {self.right_dist:6.2f} m                    ║",
            "║                                                      ║",
            "║            === Commandes actuelles ===              ║",
            "║                                                      ║",
            f"║  Vitesse        : {self.current_speed:6.2f} m/s                  ║",
            f"║  Direction      : {self.current_steer:6.2f} rad                  ║",
            "║                                                      ║",
            "║            === Décisions ===                        ║",
            "║                                                      ║",
            f"║  {self.decision_msg:<46} ║",
            f"║  {self.speed_msg:<46} ║",
            "╚══════════════════════════════════════════════════════╝"
        ]
        
        #for line in lines:
        #    self.clear_line()
        #    print(line)
        
    def process_lidar(self, scan_data):
        """Traite les données LIDAR pour détecter les obstacles"""
        if scan_data is None or len(scan_data) == 0:
            return np.inf, np.inf, np.inf
            
        # Convertir en array numpy si nécessaire
        distances = np.array(scan_data)
        
        # Nombre total de points
        num_points = len(distances)
        
        # Indices pour les différents secteurs
        center = num_points // 2
        
        # Secteur frontal (±30°)
        front_span = int(num_points * self.front_angle / 360)
        front_indices = np.arange(center - front_span, center + front_span)
        
        # Secteurs latéraux (±90°)
        side_span = int(num_points * self.side_angle / 360)
        left_indices = np.arange(center - side_span, center)
        right_indices = np.arange(center, center + side_span)
        
        # Assurer que les indices sont dans les limites
        front_indices = front_indices[(front_indices >= 0) & (front_indices < num_points)]
        left_indices = left_indices[(left_indices >= 0) & (left_indices < num_points)]
        right_indices = right_indices[(right_indices >= 0) & (right_indices < num_points)]
        
        # Calculer les distances minimales dans chaque secteur
        self.front_dist = np.min(distances[front_indices]) if len(front_indices) > 0 else np.inf
        self.left_dist = np.min(distances[left_indices]) if len(left_indices) > 0 else np.inf
        self.right_dist = np.min(distances[right_indices]) if len(right_indices) > 0 else np.inf
        
        return self.front_dist, self.left_dist, self.right_dist
        
    def find_best_direction(self, front_dist, left_dist, right_dist):
        """Détermine la meilleure direction à suivre"""
        # Si la voie est libre devant
        if front_dist > self.min_front_dist + self.safety_margin:
            # Centrage entre les obstacles
            if abs(left_dist - right_dist) > self.safety_margin:
                steer = - 0.2 * np.sign(left_dist - right_dist)
                self.decision_msg = f"Correction de centrage: {steer:.2f} rad"
                return steer
            self.decision_msg = "Continuer tout droit"
            return 0.0
            
        # Si obstacle devant, choisir la direction la plus dégagée
        if left_dist > right_dist:
            self.decision_msg = f"Tourner à gauche (G:{left_dist:.2f}m > D:{right_dist:.2f}m)"
            return -self.max_steer
        self.decision_msg = f"Tourner à droite (D:{right_dist:.2f}m >= G:{left_dist:.2f}m)"
        return +self.max_steer
        
    def compute_speed(self, front_dist):
        """Calcule la vitesse en fonction de la distance frontale"""
        if front_dist < self.min_front_dist:
            self.speed_msg = f"Arrêt - Trop proche ({front_dist:.2f}m)"
            return 0.0
            
        # Vitesse proportionnelle à la distance, avec une limite
        speed = self.max_speed * (front_dist - self.min_front_dist) / (2 * self.min_front_dist)
        speed = np.clip(speed, 0.0, self.max_speed)
        self.speed_msg = f"Vitesse adaptée à la distance ({front_dist:.2f}m)"
        return speed
        
    def plan(self, obs):
        """Planifie les actions de contrôle basées sur les observations"""
        # Extraire les données du scan
        if 'scans' not in obs or obs['scans'] is None or len(obs['scans']) == 0:
            return np.array([[0.0, 0.0]])  # Arrêt si pas de données
            
        scan_data = obs['scans'][0]
        
        # Mettre à jour le temps au tour et le nombre de tours
        if 'lap_times' in obs and 'lap_counts' in obs:
            self.lap_time = obs['lap_times'][0]
            self.lap_count = obs['lap_counts'][0]
        
        # Analyser l'environnement
        front_dist, left_dist, right_dist = self.process_lidar(scan_data)
        
        # Calculer les commandes
        self.current_steer = self.find_best_direction(front_dist, left_dist, right_dist)
        self.current_speed = self.compute_speed(front_dist)
        
        # Mettre à jour l'affichage
        self.update_display()
        
        return np.array([[self.current_steer, self.current_speed]])
