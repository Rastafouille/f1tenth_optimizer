import numpy as np
from car_simulator import CarSimulator
from autonomous_navigator import AutonomousNavigator
import time

class NavigationOptimizer:
    def __init__(self, map_path="TRR.bmp", yaml_path="map.yaml"):
        self.map_path = map_path
        self.yaml_path = yaml_path
        self.best_params = None
        self.best_score = float('inf')
        self.results = []
        
        # Position de départ et paramètres de détection
        self.start_x = 4.0  # même que dans CarSimulator
        self.start_y = 1.5  # même que dans CarSimulator
        self.detection_radius = 0.5  # rayon de la zone de détection en mètres
        self.min_distance_for_lap = 5.0  # distance minimale à parcourir avant de pouvoir compter un tour
        
    def evaluate_params(self, params, max_time=60, collision_penalty=float('inf')):
        """Évalue un jeu de paramètres en simulant une course
        
        Args:
            params: dict avec les paramètres de navigation
            max_time: temps maximum de simulation en secondes
            collision_penalty: pénalité pour une collision (infini par défaut = élimination)
            
        Returns:
            score: temps total + pénalités (infini si collision)
            metrics: dict avec les métriques détaillées
        """
        # Créer un nouveau simulateur avec ces paramètres en mode headless
        simulator = CarSimulator(self.map_path, self.yaml_path, headless=True)
        
        # Configurer le navigateur avec les paramètres à tester
        simulator.navigator = AutonomousNavigator(**params)
        
        # Activer le mode autonome
        simulator.autonomous_mode = True
        
        # Variables pour le suivi
        start_time = time.time()
        collision_detected = False
        distance_traveled = 0
        last_pos = (simulator.car.x, simulator.car.y)
        
        # Boucle de simulation avec dt plus petit pour plus de précision
        dt = 0.05  # 50ms au lieu de 100ms
        elapsed_simulation_time = 0
        
        while elapsed_simulation_time < max_time:
            # Mettre à jour la simulation
            simulator.update(simulator.linear_vel, simulator.angular_vel, dt)
            elapsed_simulation_time += dt
            
            # Si collision, on arrête immédiatement avec un mauvais score
            if simulator.collision_detected:
                collision_detected = True
                print("❌ Collision détectée - Paramètres éliminés")
                break
            
            # Calculer la distance parcourue
            current_pos = (simulator.car.x, simulator.car.y)
            step_distance = np.sqrt((current_pos[0] - last_pos[0])**2 + 
                                  (current_pos[1] - last_pos[1])**2)
            distance_traveled += step_distance
            last_pos = current_pos
            
            # Vérifier si on a fait un tour complet
            if self._check_lap_completed(simulator, distance_traveled):
                print("✅ Tour complet sans collision!")
                break
        
        # Calculer le score final
        elapsed_time = time.time() - start_time
        final_score = float('inf') if collision_detected else elapsed_simulation_time
        
        metrics = {
            'elapsed_time': elapsed_simulation_time,
            'real_time': elapsed_time,
            'collision': collision_detected,
            'distance': distance_traveled,
            'score': final_score,
            'params': params,
            'completed': distance_traveled >= self.min_distance_for_lap and not collision_detected
        }
        
        # Mettre à jour le meilleur score seulement si pas de collision et tour complété
        if not collision_detected and metrics['completed'] and final_score < self.best_score:
            self.best_score = final_score
            self.best_params = params
            print(f"🏆 Nouveau meilleur score: {final_score:.2f}s")
            print(f"   Distance: {distance_traveled:.2f}m")
            print(f"   Temps réel d'évaluation: {elapsed_time:.2f}s")
        
        # Sauvegarder les résultats
        self.results.append(metrics)
        
        return final_score, metrics
    
    def _near_start_line(self, simulator):
        """Vérifie si le véhicule est près de la ligne de départ"""
        dist_to_start = np.sqrt((simulator.car.x - self.start_x)**2 + 
                              (simulator.car.y - self.start_y)**2)
        return dist_to_start < self.detection_radius
    
    def _check_lap_completed(self, simulator, distance_traveled):
        """Vérifie si un tour complet a été effectué"""
        # On doit avoir parcouru une distance minimale et être près de la ligne de départ
        return (distance_traveled >= self.min_distance_for_lap and 
                self._near_start_line(simulator))
    
    def optimize_grid_search(self, param_grid):
        """Effectue une recherche sur grille des meilleurs paramètres
        
        Args:
            param_grid: dict avec les plages de valeurs à tester pour chaque paramètre
            Par exemple:
            {
                'stop_distance': [0.3, 0.5, 0.7],
                'normal_linear_speed': [1.0, 1.5, 2.0],
                'obstacle_angular_speed': [2.0, 2.5, 3.0]
            }
        """
        # Générer toutes les combinaisons de paramètres
        from itertools import product
        keys = param_grid.keys()
        combinations = product(*param_grid.values())
        
        for values in combinations:
            params = dict(zip(keys, values))
            print(f"\nTest des paramètres: {params}")
            
            score, metrics = self.evaluate_params(params)
            print(f"Score: {score:.2f}")
            print(f"Temps: {metrics['elapsed_time']:.2f}s")
            print(f"Collisions: {metrics['collision']}")
            print(f"Distance: {metrics['distance']:.2f}m")
            print(f"Tour complété: {'Oui' if metrics['completed'] else 'Non'}")
        
        print("\nMeilleurs paramètres trouvés:")
        print(self.best_params)
        print(f"Meilleur score: {self.best_score:.2f}")
        
        return self.best_params, self.best_score 