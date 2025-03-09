import gym
import numpy as np
from navigation import SimpleAutonomousController
import pygame
import os
from f110_gym.envs.base_classes import Integrator  # Ajout de l'import pour RK4
import csv
from datetime import datetime
from parameter_tester import ParameterTester

class InfoDisplay:
    def __init__(self):
        pygame.init()
        # Couleurs modernes
        self.COLORS = {
            'background': (40, 44, 52),      # Fond sombre
            'panel': (30, 33, 39),           # Panneaux plus sombres
            'text': (220, 223, 228),         # Texte clair
            'highlight': (97, 175, 239),     # Bleu clair pour les valeurs importantes
            'warning': (224, 108, 117),      # Rouge pour les alertes
            'success': (152, 195, 121),      # Vert pour les succès
            'separator': (55, 59, 69)        # Lignes de séparation
        }
        
        # Configuration de la fenêtre
        self.screen = pygame.display.set_mode((1000, 500))
        pygame.display.set_caption("F1TENTH Telemetry")
        os.environ['SDL_VIDEO_WINDOW_POS'] = "820,100"
        
        # Chargement des polices
        try:
            self.title_font = pygame.font.SysFont('Arial', 36, bold=True)
            self.main_font = pygame.font.SysFont('Arial', 28)
            self.small_font = pygame.font.SysFont('Arial', 20)
        except:
            self.title_font = pygame.font.Font(None, 36)
            self.main_font = pygame.font.Font(None, 28)
            self.small_font = pygame.font.Font(None, 20)
        
        # Zone pour le scan laser (300x300 pixels)
        self.scan_surface = pygame.Surface((300, 300))
        self.scan_center = (150, 150)
        self.scan_scale = 30
        self.fov = None
        
        # Variables pour le suivi des tours
        self.lap_times_history = []
        self.current_lap_count = 0
        self.best_lap_time = float('inf')

    def draw_panel(self, surface, rect, title=None):
        """Dessine un panneau avec titre et bordure"""
        pygame.draw.rect(surface, self.COLORS['panel'], rect)
        pygame.draw.rect(surface, self.COLORS['separator'], rect, 1)
        if title:
            title_surf = self.title_font.render(title, True, self.COLORS['highlight'])
            surface.blit(title_surf, (rect[0] + 10, rect[1] + 5))

    def draw_scan(self, scan_data, obs):
        """Dessine les données du scan laser avec un style amélioré"""
        self.scan_surface.fill(self.COLORS['panel'])
        
        if scan_data is not None and len(scan_data) > 0:
            if self.fov is None and 'lidar_param' in obs:
                self.fov = obs['lidar_param'][1]
            half_fov = self.fov / 2 if self.fov is not None else 2.356194490192345
            angles = np.linspace(-half_fov, half_fov, len(scan_data))
            
            # Grille de fond
            for i in range(0, 301, 50):
                pygame.draw.circle(self.scan_surface, self.COLORS['separator'], self.scan_center, i, 1)
            pygame.draw.line(self.scan_surface, self.COLORS['separator'], 
                           (0, self.scan_center[1]), (300, self.scan_center[1]), 1)
            pygame.draw.line(self.scan_surface, self.COLORS['separator'], 
                           (self.scan_center[0], 0), (self.scan_center[0], 300), 1)
            
            # Points du scan
            for i, distance in enumerate(scan_data):
                if not np.isinf(distance) and distance < 10:
                    x = distance * np.cos(angles[i])
                    y = distance * np.sin(angles[i])
                    screen_x = int(self.scan_center[0] + x * self.scan_scale)
                    screen_y = int(self.scan_center[1] - y * self.scan_scale)
                    if 0 <= screen_x < 300 and 0 <= screen_y < 300:
                        pygame.draw.circle(self.scan_surface, self.COLORS['highlight'], 
                                        (screen_x, screen_y), 2)
        
        # Robot au centre
        pygame.draw.circle(self.scan_surface, self.COLORS['success'], self.scan_center, 5)

    def update(self, speed, steer, obs):
        """Met à jour l'affichage avec un design moderne"""
        # Fond principal
        self.screen.fill(self.COLORS['background'])
        
        # Vérifier collision
        is_colliding = False
        if obs['collisions'] is not None and len(obs['collisions']) > 0:
            is_colliding = bool(obs['collisions'][0])
        
        # Panneau principal (gauche)
        main_panel = pygame.Rect(20, 20, 460, 460)
        self.draw_panel(self.screen, main_panel, "État du Véhicule")
        
        # Informations de contrôle
        y_offset = 70
        line_spacing = 40
        
        # Vitesse et direction avec des barres de progression
        speed_text = self.main_font.render(f"Vitesse: {speed:.2f} m/s", True, self.COLORS['text'])
        steer_text = self.main_font.render(f"Direction: {steer:.2f} rad", True, self.COLORS['text'])
        self.screen.blit(speed_text, (40, y_offset))
        self.screen.blit(steer_text, (40, y_offset + line_spacing))
        
        # Barres de progression
        speed_bar_rect = pygame.Rect(40, y_offset + 30, 400 * (speed/3.0), 5)
        steer_bar_rect = pygame.Rect(40, y_offset + line_spacing + 30, 400 * (abs(steer)/0.4), 5)
        pygame.draw.rect(self.screen, self.COLORS['highlight'], speed_bar_rect)
        pygame.draw.rect(self.screen, self.COLORS['highlight'], steer_bar_rect)
        
        # Informations de tour
        y_offset += 3 * line_spacing
        lap_time = obs['lap_times'][0] if 'lap_times' in obs and len(obs['lap_times']) > 0 else 0.0
        lap_count = int(obs['lap_counts'][0]) if 'lap_counts' in obs and len(obs['lap_counts']) > 0 else 0
        
        if lap_count > self.current_lap_count:
            if self.current_lap_count > 0:
                self.lap_times_history.append(lap_time)
                if lap_time < self.best_lap_time:
                    self.best_lap_time = lap_time
            self.current_lap_count = lap_count
        
        # Affichage des temps
        lap_time_text = self.main_font.render(f"Temps tour: {lap_time:.2f} s", True, self.COLORS['text'])
        lap_count_text = self.main_font.render(f"Tour: {lap_count}", True, self.COLORS['text'])
        self.screen.blit(lap_time_text, (40, y_offset))
        self.screen.blit(lap_count_text, (40, y_offset + line_spacing))
        
        if self.best_lap_time < float('inf'):
            best_time_text = self.main_font.render(f"Meilleur tour: {self.best_lap_time:.2f} s", 
                                                 True, self.COLORS['success'])
            self.screen.blit(best_time_text, (40, y_offset + 2 * line_spacing))
        
        # Panneau de scan (droite)
        scan_panel = pygame.Rect(500, 20, 480, 460)
        self.draw_panel(self.screen, scan_panel, "Scan LIDAR")
        
        # Mise à jour et affichage du scan
        if 'scans' in obs and obs['scans'] is not None and len(obs['scans']) > 0:
            self.draw_scan(obs['scans'][0], obs)
        self.screen.blit(self.scan_surface, (590, 100))
        
        # Affichage des alertes
        if is_colliding:
            warning_text = self.title_font.render("! COLLISION !", True, self.COLORS['warning'])
            text_rect = warning_text.get_rect(center=(250, 400))
            self.screen.blit(warning_text, text_rect)
        
        pygame.display.flip()

    def __del__(self):
        pygame.quit()

def test_parameters(env, initial_pose, params, display=None):
    """Teste un jeu de paramètres spécifique sur un seul tour"""
    # Initialisation du contrôleur avec les paramètres
    controller = SimpleAutonomousController(**params)
    
    # Réinitialisation de l'environnement
    obs_tuple = env.reset(initial_pose)
    obs = obs_tuple[0] if isinstance(obs_tuple, tuple) else obs_tuple
    
    # Variables de suivi
    total_time = 0.0
    collision = False
    tour_complete = False
    temps_tour = 0.0
    distance_parcourue = 0.0
    
    # Variables pour détecter l'immobilité
    derniere_position = (obs['poses_x'][0], obs['poses_y'][0])
    temps_immobile = 0.0
    seuil_mouvement = 0.01  # 1cm de déplacement minimum
    
    # Pas de temps fixe
    dt = 0.01
    
    while True:
        # Obtenir les actions du contrôleur
        actions = controller.plan(obs)
        
        # Faire un pas de simulation
        obs_tuple, _, done, _ = env.step(actions)
        obs = obs_tuple[0] if isinstance(obs_tuple, tuple) else obs_tuple
        
        # Vérifier si le véhicule est immobile
        position_actuelle = (obs['poses_x'][0], obs['poses_y'][0])
        deplacement = np.sqrt((position_actuelle[0] - derniere_position[0])**2 + 
                            (position_actuelle[1] - derniere_position[1])**2)
        
        if deplacement < seuil_mouvement:
            temps_immobile += dt
        else:
            temps_immobile = 0.0
            derniere_position = position_actuelle
        
        # Arrêter si immobile depuis 5 secondes
        if temps_immobile >= 5.0:
            break
        
        # Mettre à jour les métriques
        total_time += dt
        speed = actions[0][1]  # Vitesse actuelle
        distance_parcourue += speed * dt
        
        # Mettre à jour l'affichage si disponible
        if display is not None:
            display.update(actions[0][1], actions[0][0], obs)
            env.render(mode='human_fast')
        
        # Vérifier si le tour est complet
        if int(obs['lap_counts'][0]) > 0:
            tour_complete = True
            temps_tour = total_time
            break
        
        # Vérifier collision
        if done:
            collision = bool(obs['collisions'][0])
            break
        
        # Limite de temps de sécurité (30 secondes)
        if total_time > 120.0:
            break
    
    return {
        'collision': collision,
        'total_time': total_time,
        'distance': distance_parcourue,
        'tour_complete': tour_complete,
        'temps_tour': temps_tour,
        'immobile': temps_immobile >= 5.0
    }

def main():
    # Création de l'environnement
    map_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps', 'example_map')
    racecar_env = gym.make('f110_gym:f110-v0',
                          map=map_path,
                          map_ext='.png',
                          num_agents=1)

    # Position initiale
    initial_pose = np.array([[0.7, 0.0, 1.37079632679]], dtype=np.float64)
    
    # Initialisation de l'affichage
    display = InfoDisplay()
    
    try:
        # Création du testeur de paramètres
        tester = ParameterTester()
        max_tests = tester.get_total_combinations()
        nb_tests_precedents = len(tester.tested_params)
        print(f"Début de l'optimisation - {max_tests} tests prévus")
        print(f"Tests déjà effectués: {nb_tests_precedents}")
        print("-" * 50)
        
        # Test de chaque combinaison de paramètres
        tested = nb_tests_precedents
        while tested < max_tests:
            # Obtenir les prochains paramètres à tester
            params = tester.get_next_parameters()
            
            # Afficher les paramètres avant le test
            print(f"\nTest {tested + 1}/{max_tests}")
            print("Paramètres testés:")
            for name, value in params.items():
                print(f"  {name}: {value:.3f}")
            
            # Test des paramètres avec affichage
            results = test_parameters(racecar_env, initial_pose, params, display)
            
            # Afficher les résultats
            print("\nRésultats:")
            if results['collision']:
                print("  Collision détectée")
            elif results['immobile']:
                print("  Véhicule immobile pendant 5 secondes")
            elif results['tour_complete']:
                print(f"  Tour complet en {results['temps_tour']:.2f} secondes")
            else:
                print("  Tour incomplet")
            print(f"  Distance parcourue: {results['distance']:.2f}m")
            print(f"  Temps total: {results['total_time']:.2f}s")
            print("-" * 50)
            
            # Enregistrement des résultats
            tester.save_results(
                params=params,
                total_time=results['total_time'],
                distance=results['distance'],
                collision=results['collision'],
                tour_complete=results['tour_complete'],
                temps_tour=results['temps_tour']
            )
            
            tested += 1
        
        # Une fois tous les tests terminés, afficher les meilleurs paramètres
        if tester.best_params is not None:
            print("\n\nMeilleurs paramètres trouvés:")
            for name, value in tester.best_params.items():
                print(f"  {name}: {value:.3f}")
            print(f"Score: {tester.best_score:.2f}")
            
            print("\nTest des meilleurs paramètres...")
            results = test_parameters(racecar_env, initial_pose, tester.best_params, display)
            
    except KeyboardInterrupt:
        print("\nTests interrompus par l'utilisateur")
        if tester.best_params is not None:
            print("\nMeilleurs paramètres trouvés jusqu'ici:")
            for name, value in tester.best_params.items():
                print(f"  {name}: {value:.3f}")
            print(f"Score: {tester.best_score:.2f}")
    finally:
        racecar_env.close()

if __name__ == '__main__':
    main()