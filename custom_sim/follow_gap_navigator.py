import numpy as np

class FollowGapNavigator:
    def __init__(self, 
                 stop_distance=0.5,         # Distance minimale d'arrêt (m)
                 min_gap_width=0.6,         # Largeur minimale d'un passage (m)
                 max_detection_dist=3.0,     # Distance maximale de détection (m)
                 safety_margin=0.3,         # Marge de sécurité autour des obstacles (m)
                 max_linear_speed=2.0,      # Vitesse linéaire maximale (m/s)
                 max_angular_speed=4.0,     # Vitesse angulaire maximale (rad/s)
                 gap_weight=1.0,           # Poids pour la taille du passage
                 angle_weight=0.5):         # Poids pour l'angle du passage
        
        # Paramètres de configuration
        self.stop_distance = stop_distance
        self.min_gap_width = min_gap_width
        self.max_detection_dist = max_detection_dist
        self.safety_margin = safety_margin
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.gap_weight = gap_weight
        self.angle_weight = angle_weight
        
        # État interne
        self.current_gap = None
        self.current_direction = 0.0
        
    def find_gaps(self, scan):
        """Trouve tous les espaces libres dans le scan"""
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
                    # Calculer les caractéristiques du passage
                    gap_ranges = ranges[gap_start:gap_end]
                    gap_angles = angles[gap_start:gap_end]
                    gap_width = abs(np.mean(gap_ranges) * 
                                  (gap_angles[-1] - gap_angles[0]))
                    gap_center_idx = (gap_start + gap_end) // 2
                    gap_center_angle = gap_angles[len(gap_angles)//2]
                    gap_distance = np.mean(gap_ranges)
                    
                    if gap_width >= self.min_gap_width:
                        gaps.append({
                            'start_idx': gap_start,
                            'end_idx': gap_end,
                            'width': gap_width,
                            'center_idx': gap_center_idx,
                            'center_angle': gap_center_angle,
                            'distance': gap_distance
                        })
                    in_gap = False
        
        return gaps
    
    def select_best_gap(self, gaps):
        """Sélectionne le meilleur passage en fonction de plusieurs critères"""
        if not gaps:
            return None
            
        best_gap = None
        best_score = float('-inf')
        
        for gap in gaps:
            # Score basé sur la largeur et la proximité de l'avant du robot
            width_score = gap['width'] * self.gap_weight
            angle_score = (1.0 - abs(gap['center_angle']) / np.pi) * self.angle_weight
            
            total_score = width_score + angle_score
            
            if total_score > best_score:
                best_score = total_score
                best_gap = gap
        
        return best_gap
    
    def compute_command(self, scan):
        """Calcule les commandes de vitesse pour suivre le meilleur passage"""
        # Trouver tous les passages
        gaps = self.find_gaps(scan)
        
        # Sélectionner le meilleur passage
        best_gap = self.select_best_gap(gaps)
        
        if best_gap is None:
            print("\n=== Navigation (Follow the Gap) ===")
            print("Aucun passage trouvé - Manœuvre d'évitement")
            print("=====================================")
            return -0.2, self.max_angular_speed  # Reculer en tournant
        
        # Calculer l'angle vers le centre du passage
        gap_angle = best_gap['center_angle']
        if gap_angle > np.pi:
            gap_angle -= 2 * np.pi
            
        # Calculer les vitesses
        # La vitesse angulaire est proportionnelle à l'angle vers le passage
        angular_vel = self.max_angular_speed * (gap_angle / np.pi)
        angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
        
        # La vitesse linéaire diminue quand on tourne beaucoup
        linear_vel = self.max_linear_speed * (1.0 - abs(gap_angle) / np.pi)
        linear_vel = max(0.2, linear_vel)  # Garder une vitesse minimale
        
        # Afficher les informations de navigation
        print("\n=== Navigation (Follow the Gap) ===")
        print(f"Largeur du passage: {best_gap['width']:.2f}m")
        print(f"Distance au passage: {best_gap['distance']:.2f}m")
        print(f"Angle vers le passage: {np.degrees(gap_angle):.1f}°")
        print(f"Commandes: v={linear_vel:.2f} m/s, w={angular_vel:.2f} rad/s")
        print("=====================================")
        
        return linear_vel, angular_vel 