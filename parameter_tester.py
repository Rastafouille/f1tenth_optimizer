import numpy as np
import csv
from datetime import datetime
from scipy.optimize import minimize
import os

class ParameterTester:
    def __init__(self):
        # Définition des plages de paramètres avec min, max et pas
        self.parameter_definitions = {
            'max_speed': {
                'min': 3.0,    # m/s
                'max': 3.0,
                'step': 0.5
            },
            'max_steer': {
                'min': .2,    # rad
                'max': .2,
                'step': 0.05
            },
            'min_front_dist': {
                'min': 0.1,    # m
                'max': 1.,
                'step': 0.1
            },
            'safety_margin': {
                'min': 0.2,    # m
                'max': 0.4,
                'step': 0.05
            },
            'front_angle': {
                'min': 10,     # degrés
                'max': 45,
                'step': 5
            },
            'side_angle': {
                'min': 10,     # degrés
                'max': 110,
                'step': 10
            }
        }
        
        # Cache pour éviter les doublons
        self.tested_combinations = set()
        
        # Création des plages de valeurs à partir des définitions
        self.parameter_ranges = {
            name: np.arange(
                params['min'],
                params['max'] + params['step']/2,  # +step/2 pour inclure la valeur max
                params['step']
            )
            for name, params in self.parameter_definitions.items()
        }
        
        # Historique des tests
        self.tested_params = []
        self.results = []
        
        # Meilleurs résultats
        self.best_params = None
        self.best_score = float('-inf')
        
        # État de l'optimisation Nelder-Mead
        self.using_nelder_mead = False
        self.nelder_mead_result = None
        self.current_params = None
        
        # Création du fichier de résultats (sans l'heure)
        self.results_file = f"parameter_test_results_{datetime.now().strftime('%Y%m%d')}.csv"
        
        # Nouveaux paramètres pour la détection de convergence
        self.score_history = []
        self.convergence_window = 10  # Nombre de tests pour vérifier la convergence
        self.convergence_threshold = 50  # Seuil de variation pour détecter la convergence
        self.exploration_mode = False
        self.exploration_count = 0
        self.max_exploration_tests = 5  # Nombre de tests aléatoires après convergence
        
        # Charger les résultats existants si le fichier existe
        self.load_existing_results()
        self.write_header()
        
        # Initialiser l'optimisation Nelder-Mead si on a assez de données
        if len(self.tested_params) >= 10:
            self.initialize_nelder_mead()
        
    def load_existing_results(self):
        """Charge les résultats existants du fichier CSV s'il existe"""
        try:
            with open(self.results_file, 'r') as f:
                reader = csv.reader(f)
                # Ignorer les lignes de commentaires
                for row in reader:
                    if not row[0].startswith('#'):
                        if row[0] != 'max_speed':  # Ignorer l'en-tête
                            # Convertir les valeurs en float
                            params = {
                                'max_speed': float(row[0]),
                                'max_steer': float(row[1]),
                                'min_front_dist': float(row[2]),
                                'safety_margin': float(row[3]),
                                'front_angle': float(row[4]),
                                'side_angle': float(row[5])
                            }
                            
                            # Recréer l'objet results
                            results = {
                                'total_time': float(row[6]),
                                'distance': float(row[7]),
                                'collision': row[8].lower() == 'true',
                                'tour_complete': row[9].lower() == 'true',
                                'temps_tour': float(row[10]) if row[10] != 'N/A' else None
                            }
                            
                            # Calculer le score
                            score = self.calculate_score(results)
                            
                            # Mettre à jour le meilleur score si nécessaire
                            if score > self.best_score:
                                self.best_score = score
                                self.best_params = params.copy()
                            
                            # Ajouter aux historiques
                            self.tested_params.append(params)
                            self.results.append(results)
        except FileNotFoundError:
            pass  # Le fichier n'existe pas encore
        
    def write_header(self):
        """Écrit l'en-tête du fichier CSV uniquement si le fichier n'existe pas"""
        # Vérifier si le fichier existe déjà
        try:
            with open(self.results_file, 'r') as f:
                return  # Si le fichier existe, ne rien faire
        except FileNotFoundError:
            # Si le fichier n'existe pas, créer l'en-tête
            with open(self.results_file, 'w', newline='') as f:
                writer = csv.writer(f)
                # Ajouter les plages de valeurs en commentaire
                writer.writerow(['# Plages de paramètres:'])
                for name, params in self.parameter_definitions.items():
                    writer.writerow([
                        f'# {name}: min={params["min"]}, max={params["max"]}, '
                        f'step={params["step"]}'
                    ])
                writer.writerow(['#'])
                # En-tête des données
                writer.writerow([
                    'max_speed', 'max_steer', 'min_front_dist', 'safety_margin',
                    'front_angle', 'side_angle', 'temps_total', 'distance_parcourue',
                    'collision', 'tour_complete', 'temps_tour', 'score'
                ])
    
    def calculate_score(self, results):
        """Calcule un score pour les résultats d'un test"""
        # Pénalité pour collision
        if results['collision']:
            return -1000
        
        # Si le tour n'est pas complet
        if not results['tour_complete']:
            # Pénalité pour tour non complété
            return -1000 + results['distance']  # On ajoute la distance pour différencier les essais incomplets
        
        # Si le tour est complet, score basé sur le temps
        # Plus le temps est court, meilleur est le score
        base_score = 10000  # Score de base plus élevé pour un tour complet
        time_penalty = results['temps_tour'] * 10  # Pénalité de temps
        return base_score - time_penalty
    
    def save_results(self, params, total_time, distance, collision, tour_complete, temps_tour):
        """Enregistre les résultats dans le fichier CSV et met à jour l'historique"""
        # Calculer le score
        results = {
            'total_time': total_time,
            'distance': distance,
            'collision': collision,
            'tour_complete': tour_complete,
            'temps_tour': temps_tour
        }
        score = self.calculate_score(results)
        
        # Mettre à jour le meilleur score
        if score > self.best_score:
            self.best_score = score
            self.best_params = params.copy()
        
        # Sauvegarder dans l'historique
        self.tested_params.append(params)
        self.results.append(results)
        
        # Écrire dans le fichier CSV avec 2 décimales
        with open(self.results_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                f"{params['max_speed']:.2f}",
                f"{params['max_steer']:.2f}",
                f"{params['min_front_dist']:.2f}",
                f"{params['safety_margin']:.2f}",
                f"{params['front_angle']:.2f}",
                f"{params['side_angle']:.2f}",
                f"{total_time:.2f}",
                f"{distance:.2f}",
                collision,
                tour_complete,
                f"{temps_tour:.2f}" if tour_complete else 'N/A',
                f"{score:.2f}"
            ])
        
        # Ajouter le score à l'historique
        self.score_history.append(score)
    
    def parameters_to_array(self, params):
        """Convertit un dictionnaire de paramètres en tableau numpy"""
        return np.array([
            params['max_speed'],
            params['max_steer'],
            params['min_front_dist'],
            params['safety_margin'],
            params['front_angle'],
            params['side_angle']
        ])
    
    def array_to_parameters(self, x):
        """Convertit un tableau numpy en dictionnaire de paramètres"""
        return {
            'max_speed': np.clip(x[0], self.parameter_definitions['max_speed']['min'], 
                               self.parameter_definitions['max_speed']['max']),
            'max_steer': np.clip(x[1], self.parameter_definitions['max_steer']['min'], 
                               self.parameter_definitions['max_steer']['max']),
            'min_front_dist': np.clip(x[2], self.parameter_definitions['min_front_dist']['min'], 
                                    self.parameter_definitions['min_front_dist']['max']),
            'safety_margin': np.clip(x[3], self.parameter_definitions['safety_margin']['min'], 
                                   self.parameter_definitions['safety_margin']['max']),
            'front_angle': np.clip(x[4], self.parameter_definitions['front_angle']['min'], 
                                 self.parameter_definitions['front_angle']['max']),
            'side_angle': np.clip(x[5], self.parameter_definitions['side_angle']['min'], 
                                self.parameter_definitions['side_angle']['max'])
        }
    
    def initialize_nelder_mead(self):
        """Initialise l'optimisation Nelder-Mead avec les meilleurs résultats précédents"""
        if self.best_params is None:
            return
        
        self.using_nelder_mead = True
        x0 = self.parameters_to_array(self.best_params)
        
        # Définir les bornes pour chaque paramètre
        bounds = [(params['min'], params['max']) for params in self.parameter_definitions.values()]
        
        # Calculer l'échelle initiale pour chaque paramètre (10% de la plage)
        initial_simplex = []
        for i, (low, high) in enumerate(bounds):
            scale = (high - low) * 0.1
            point = x0.copy()
            point[i] += scale
            initial_simplex.append(point)
        initial_simplex = [x0] + initial_simplex
        
        # Lancer l'optimisation Nelder-Mead avec des paramètres ajustés
        self.nelder_mead_result = minimize(
            lambda x: -self.estimate_score(x),  # Négation car on veut maximiser
            x0,
            method='Nelder-Mead',
            bounds=bounds,
            options={
                'maxiter': 200,      # Plus d'itérations
                'maxfev': 300,       # Plus d'évaluations de fonction
                'xatol': 1e-3,       # Tolérance plus large sur x
                'fatol': 1e-3,       # Tolérance plus large sur f
                'adaptive': True,     # Adaptation automatique du simplexe
                'initial_simplex': initial_simplex,  # Simplexe initial personnalisé
                'disp': True
            }
        )
    
    def estimate_score(self, x):
        """Estime le score pour un jeu de paramètres donné"""
        params = self.array_to_parameters(x)
        
        # Chercher les tests similaires dans l'historique
        scores = []
        weights = []
        
        for test_params, result in zip(self.tested_params, self.results):
            # Calculer la distance euclidienne entre les paramètres
            distance = np.sqrt(sum(
                ((params[k] - test_params[k]) / (self.parameter_definitions[k]['max'] - self.parameter_definitions[k]['min'])) ** 2
                for k in params.keys()
            ))
            
            if distance < 0.2:  # Seulement considérer les tests proches
                weight = 1 / (1 + distance)  # Plus le test est proche, plus il a de poids
                scores.append(self.calculate_score(result))
                weights.append(weight)
        
        if not scores:
            return self.best_score  # Retourner le meilleur score connu si pas de données similaires
        
        # Retourner la moyenne pondérée des scores
        return np.average(scores, weights=weights)
    
    def parameters_to_key(self, params):
        """Convertit un dictionnaire de paramètres en une chaîne unique pour le cache"""
        return "|".join(f"{k}:{v:.3f}" for k, v in sorted(params.items()))

    def is_similar_combination(self, params, threshold=0.01):
        """Vérifie si une combinaison similaire a déjà été testée"""
        params_array = self.parameters_to_array(params)
        for tested_params in self.tested_params:
            tested_array = self.parameters_to_array(tested_params)
            if np.allclose(params_array, tested_array, rtol=threshold, atol=threshold):
                return True
        return False

    def is_converging(self):
        """Vérifie si l'algorithme converge en analysant l'historique des scores"""
        if len(self.score_history) < self.convergence_window:
            return False
            
        recent_scores = self.score_history[-self.convergence_window:]
        score_variation = np.std(recent_scores)
        mean_score = np.mean(recent_scores)
        
        # On considère qu'il y a convergence si:
        # 1. La variation est faible (écart-type < seuil)
        # 2. Les scores sont positifs (tours complétés)
        return score_variation < self.convergence_threshold and mean_score > 0

    def get_random_parameters(self):
        """Génère des paramètres complètement aléatoires"""
        params = {}
        for name, config in self.parameter_definitions.items():
            min_val = config['min']
            max_val = config['max']
            # Utiliser une distribution uniforme pour l'exploration
            params[name] = min_val + (max_val - min_val) * np.random.random()
        return params

    def get_next_parameters(self):
        """Retourne les prochains paramètres à tester"""
        if self.exploration_mode:
            # Mode exploration : générer des paramètres aléatoires
            self.exploration_count += 1
            if self.exploration_count >= self.max_exploration_tests:
                self.exploration_mode = False
                self.exploration_count = 0
                print("Fin de la phase d'exploration, retour à l'optimisation normale")
            else:
                print(f"Mode exploration ({self.exploration_count}/{self.max_exploration_tests})")
            return self.get_random_parameters()
        
        # Vérifier la convergence
        if self.is_converging() and not self.exploration_mode:
            print("Convergence détectée, passage en mode exploration")
            self.exploration_mode = True
            self.exploration_count = 0
            return self.get_random_parameters()
        
        # Optimisation normale avec Nelder-Mead modifié
        if not self.best_params:
            # Premier test : paramètres au milieu des plages
            params = {}
            for name, config in self.parameter_definitions.items():
                params[name] = (config['min'] + config['max']) / 2
            return params
        
        # Générer de nouveaux paramètres basés sur les meilleurs résultats
        new_params = {}
        for name, config in self.parameter_definitions.items():
            current_value = self.best_params[name]
            step = config['step']
            
            # Ajout d'une petite perturbation aléatoire
            perturbation = np.random.normal(0, step/2)
            new_value = current_value + perturbation
            
            # S'assurer que la valeur reste dans les limites
            new_value = max(config['min'], min(config['max'], new_value))
            new_params[name] = new_value
        
        return new_params

    def get_total_combinations(self):
        """Calcule le nombre total de combinaisons possibles"""
        # Calculer le nombre de valeurs pour chaque paramètre
        nb_combinations = 1
        for name, values in self.parameter_ranges.items():
            nb_combinations *= len(values)
        return nb_combinations  # Retourne le produit du nombre de valeurs pour chaque paramètre 