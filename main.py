import gym
import numpy as np
from navigation import Teleoperation
import pygame
import os
from f110_gym.envs.base_classes import Integrator  # Ajout de l'import pour RK4

class InfoDisplay:
    def __init__(self):
        pygame.init()
        # Agrandir la fenêtre pour accueillir la visualisation du scan
        self.screen = pygame.display.set_mode((500, 400))
        pygame.display.set_caption("Info F1TENTH")
        self.font = pygame.font.Font(None, 36)
        
        # Zone pour le scan laser (carré de 200x200 pixels)
        self.scan_surface = pygame.Surface((200, 200))
        self.scan_center = (100, 100)  # Centre du scan
        self.scan_scale = 40  # Échelle ajustée pour une meilleure visualisation
        self.fov = None  # Sera initialisé avec la première observation

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
        steer_text = self.font.render(f"Steer: {steer:.2f} rad", True, (0, 255, 0))
        speed_text = self.font.render(f"Speed: {speed:.2f} m/s", True, (0, 255, 0))
        
        # Afficher warning si collision
        if is_colliding:
            warning_text = self.font.render("! COLLISION !", True, (255, 0, 0))
            self.screen.blit(warning_text, (80, 90))
        
        # Positionner les textes (inversés)
        self.screen.blit(steer_text, (20, 20))
        self.screen.blit(speed_text, (20, 50))
        
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
    # Paramètres du véhicule
    params_dict = {
        'mu': 1.0489,
        'C_Sf': 4.718,
        'C_Sr': 5.4562,
        'mass': 3.463388126201571,
        'lf': 0.15597534362552312,
        'lr': 0.17145,
        'h': 0.074,
        'I': 0.04712,
        's_min': -0.4189,
        's_max': 0.4189,
        'sv_min': -3.2,
        'sv_max': 3.2,
        'v_switch': 7.319,
        'a_max': 9.51,
        'v_min': -5.0,
        'v_max': 20.0,
        'width': 0.31,
        'length': 0.58
    }
    
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
    controller = Teleoperation()
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
            
            # Extraire vitesse et angle de braquage des actions
            speed = actions[0][0]  # Vitesse
            steer = actions[0][1]  # Angle de braquage
            
            # Mettre à jour l'affichage
            display.update(speed, steer, obs)
            
            # Rendu de l'environnement
            racecar_env.render(mode='human')
            
            if done:
                break
            
    except KeyboardInterrupt:
        print("\nSimulation interrompue par l'utilisateur")
    finally:
        racecar_env.close()

if __name__ == '__main__':
    main()