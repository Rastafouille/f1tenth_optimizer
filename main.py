import gym
import numpy as np
from navigation import SimpleAutonomousController
import pygame
import os
from f110_gym.envs.base_classes import Integrator  # Ajout de l'import pour RK4

class InfoDisplay:
    def __init__(self):
        pygame.init()
        # Agrandir la fenêtre pour accueillir la visualisation du scan et les temps au tour
        self.screen = pygame.display.set_mode((800, 400))  # Augmenter la largeur pour les temps au tour
        pygame.display.set_caption("Info F1TENTH")
        
        # Positionner la fenêtre à droite de la fenêtre du simulateur
        # La fenêtre du simulateur fait environ 800x800 pixels
        os.environ['SDL_VIDEO_WINDOW_POS'] = "820,100"  # Position x=820 (800 + 20 de marge), y=100
        
        self.font = pygame.font.Font(None, 36)
        self.small_font = pygame.font.Font(None, 24)  # Police plus petite pour la liste des temps
        
        # Zone pour le scan laser (carré de 200x200 pixels)
        self.scan_surface = pygame.Surface((200, 200))
        self.scan_center = (100, 100)  # Centre du scan
        self.scan_scale = 40  # Échelle ajustée pour une meilleure visualisation
        self.fov = None  # Sera initialisé avec la première observation
        
        # Variables pour le suivi des tours
        self.lap_times_history = []  # Liste des temps au tour
        self.current_lap_count = 0  # Pour détecter les nouveaux tours
        self.best_lap_time = float('inf')  # Meilleur temps au tour

    def draw_scan(self, scan_data, obs):
        """
        Dessine les données du scan laser
        """
        self.scan_surface.fill((0, 0, 0))  # Fond noir
        
        if scan_data is not None and len(scan_data) > 0:
            # Initialiser le FOV si pas encore fait
            if self.fov is None and 'lidar_param' in obs:
                self.fov = obs['lidar_param'][1]  # Récupérer le FOV depuis les paramètres du lidar
            
            # Si FOV disponible, utiliser sa valeur, sinon utiliser la valeur par défaut
            half_fov = self.fov / 2 if self.fov is not None else 2.356194490192345
            
            # Convertir les distances en coordonnées x,y
            angles = np.linspace(-half_fov, half_fov, len(scan_data))
            
            for i, distance in enumerate(scan_data):
                if not np.isinf(distance) and distance < 10:  # Limite de distance réduite à 10m
                    # Convertir coordonnées polaires en cartésiennes
                    x = distance * np.cos(angles[i])
                    y = distance * np.sin(angles[i])
                    
                    # Mettre à l'échelle et décaler au centre
                    screen_x = int(self.scan_center[0] + x * self.scan_scale)
                    screen_y = int(self.scan_center[1] - y * self.scan_scale)  # Inverser y pour l'affichage
                    
                    # Vérifier que le point est dans les limites
                    if 0 <= screen_x < 200 and 0 <= screen_y < 200:
                        pygame.draw.circle(self.scan_surface, (0, 255, 0), (screen_x, screen_y), 1)
        
        # Dessiner le robot au centre
        pygame.draw.circle(self.scan_surface, (255, 0, 0), self.scan_center, 3)
        
        # Dessiner les axes avec des couleurs différentes
        pygame.draw.line(self.scan_surface, (100, 100, 255), 
                        (self.scan_center[0], 0), 
                        (self.scan_center[0], 200), 1)  # Axe vertical en bleu
        pygame.draw.line(self.scan_surface, (255, 100, 100), 
                        (0, self.scan_center[1]), 
                        (200, self.scan_center[1]), 1)  # Axe horizontal en rouge

    def update(self, speed, steer, obs):
        """
        Met à jour l'affichage des informations avec des observations simplifiées
        """
        # Vérifier collision (les collisions sont dans une liste)
        is_colliding = False
        if obs['collisions'] is not None and len(obs['collisions']) > 0:
            is_colliding = bool(obs['collisions'][0])
        
        # Couleur de fond basée sur l'état de collision
        if is_colliding:
            self.screen.fill((100, 0, 0))  # Rouge foncé pour collision
        else:
            self.screen.fill((50, 50, 50))  # Gris foncé normal
        
        # Créer et afficher les textes
        y_offset = 20
        line_spacing = 30
        
        # Informations de contrôle
        steer_text = self.font.render(f"Direction: {steer:.2f} rad", True, (0, 255, 0))
        speed_text = self.font.render(f"Vitesse: {speed:.2f} m/s", True, (0, 255, 0))
        
        # Informations de performance
        lap_time = obs['lap_times'][0] if 'lap_times' in obs and len(obs['lap_times']) > 0 else 0.0
        lap_count = int(obs['lap_counts'][0]) if 'lap_counts' in obs and len(obs['lap_counts']) > 0 else 0
        
        # Vérifier si un nouveau tour a été complété
        if lap_count > self.current_lap_count:
            # Sauvegarder le temps du tour précédent
            if self.current_lap_count > 0:  # Ne pas sauvegarder le temps du premier tour incomplet
                self.lap_times_history.append(lap_time)
                if lap_time < self.best_lap_time:
                    self.best_lap_time = lap_time
            self.current_lap_count = lap_count
        
        # Afficher les informations de tour actuelles
        lap_time_text = self.font.render(f"Temps tour actuel: {lap_time:.2f} s", True, (255, 255, 0))
        lap_count_text = self.font.render(f"Tour: {lap_count}", True, (255, 255, 0))
        
        if self.best_lap_time < float('inf'):
            best_time_text = self.font.render(f"Meilleur tour: {self.best_lap_time:.2f} s", True, (255, 215, 0))
        
        # Afficher warning si collision
        if is_colliding:
            warning_text = self.font.render("! COLLISION !", True, (255, 0, 0))
            self.screen.blit(warning_text, (80, 90))
        
        # Positionner les textes principaux
        self.screen.blit(steer_text, (20, y_offset))
        self.screen.blit(speed_text, (20, y_offset + line_spacing))
        self.screen.blit(lap_time_text, (20, y_offset + 2 * line_spacing))
        self.screen.blit(lap_count_text, (20, y_offset + 3 * line_spacing))
        if self.best_lap_time < float('inf'):
            self.screen.blit(best_time_text, (20, y_offset + 4 * line_spacing))
        
        # Afficher l'historique des temps au tour
        if self.lap_times_history:
            history_title = self.font.render("Historique des tours:", True, (200, 200, 200))
            self.screen.blit(history_title, (500, y_offset))
            
            for i, time in enumerate(self.lap_times_history):
                color = (255, 215, 0) if time == self.best_lap_time else (200, 200, 200)
                lap_text = self.small_font.render(f"Tour {i+1}: {time:.2f} s", True, color)
                self.screen.blit(lap_text, (500, y_offset + 40 + i * 25))
        
        # Dessiner le scan laser s'il est disponible
        if 'scans' in obs and obs['scans'] is not None and len(obs['scans']) > 0:
            self.draw_scan(obs['scans'][0], obs)
        
        # Afficher la surface du scan
        self.screen.blit(self.scan_surface, (280, 20))
        
        # Mettre à jour l'affichage
        pygame.display.flip()

    def __del__(self):
        pygame.quit()

def main():
    
    
    # Création de l'environnement
    map_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps', 'example_map')
    racecar_env = gym.make('f110_gym:f110-v0',
                          map=map_path,
                          map_ext='.png',
                          num_agents=1)

    # Position initiale
    initial_pose = np.array([[0.7, 0.0, 1.37079632679]], dtype=np.float64)
    
    # Initialisation
    obs = racecar_env.reset(initial_pose)
    controller = SimpleAutonomousController(# Paramètres de contrôle
                 max_speed=3.0,           # m/s
                 max_steer=2.0,           # rad
                 min_front_dist=0.5,      # distance minimale frontale (m)
                 safety_margin=0.3,       # marge de sécurité (m)
                 
                 # Paramètres du scan
                 front_angle=30,          # demi-angle du secteur frontal (degrés)
                 side_angle=90)          # demi-angle des secteurs latéraux (degrés)
                 
    display = InfoDisplay()
    
    try:
        while True:
            # Obtenir les actions du contrôleur
            actions = controller.plan(obs)
            
            # Quitter si demandé
            if actions is None:
                break
                
            # Faire un pas de simulation
            obs, step_reward, done, info = racecar_env.step(actions)
            print(obs)
            
            # Extraire vitesse et angle de braquage des actions
            speed = actions[0][0]  # Vitesse
            steer = actions[0][1]  # Angle de braquage
            
            # Mettre à jour l'affichage
            display.update(speed, steer, obs)
            
            # Rendu de l'environnement
            racecar_env.render(mode='human')
            
            # En cas de collision, réinitialiser la position
            if done:
                obs = racecar_env.reset(initial_pose)
            
    except KeyboardInterrupt:
        print("\nSimulation interrompue par l'utilisateur")
    finally:
        racecar_env.close()

if __name__ == '__main__':
    main()