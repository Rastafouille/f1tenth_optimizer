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
        self.scan_scale = 20  # Échelle ajustée pour une meilleure visualisation
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