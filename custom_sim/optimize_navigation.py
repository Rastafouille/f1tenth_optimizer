from navigation_optimizer import NavigationOptimizer
import numpy as np

def create_range(min_val, max_val, step):
    """Crée une liste de valeurs entre min et max avec un pas donné"""
    return list(np.arange(min_val, max_val + step/2, step))

def main():
    # Créer l'optimiseur
    optimizer = NavigationOptimizer(map_path="TRR.bmp", yaml_path="map.yaml")
    
    # Définir les plages de paramètres (min, max, pas)
    ranges = {
        # Distances de détection (mètres)
        'stop_distance': (0.2, 0.8, 0.2),      # [0.2, 0.4, 0.6, 0.8]
        
        # Vitesses normales
        'normal_linear_speed': (0.5, 2.5, 0.5), # [0.5, 1.0, 1.5, 2.0, 2.5]
        'normal_angular_speed': (0.0, 0.0, 1.0), # [0.0] fixe
        
        # Vitesses d'évitement d'obstacles
        'obstacle_linear_speed': (0.2, 0.8, 0.2),  # [0.2, 0.4, 0.6, 0.8]
        'obstacle_angular_speed': (1.5, 3.5, 0.5), # [1.5, 2.0, 2.5, 3.0, 3.5]
        
        # Secteurs de détection (degrés)
        'left_sector_start': (300, 330, 15),    # [300, 315, 330] (-60° à -30°)
        'left_sector_end': (335, 355, 10),      # [335, 345, 355] (-25° à -5°)
        'right_sector_start': (5, 25, 10),      # [5, 15, 25]
        'right_sector_end': (30, 60, 15)        # [30, 45, 60]
    }
    
    # Créer la grille de paramètres
    param_grid = {
        param: create_range(*range_vals)
        for param, range_vals in ranges.items()
    }
    
    # Afficher les plages de valeurs
    print("=== Plages de paramètres à tester ===")
    for param, values in param_grid.items():
        print(f"{param}:")
        print(f"  Valeurs: {values}")
        print(f"  [{len(values)} valeurs]")
    
    print("\n=== Optimisation des paramètres de navigation ===")
    print(f"Nombre total de combinaisons: {len([1 for _ in __import__('itertools').product(*param_grid.values())])}")
    print("Appuyez sur Entrée pour commencer...")
    input()
    
    # Lancer l'optimisation
    best_params, best_score = optimizer.optimize_grid_search(param_grid)
    
    # Afficher les résultats détaillés
    print("\n=== Résultats détaillés ===")
    print("Meilleurs paramètres:")
    for param, value in best_params.items():
        print(f"  {param}: {value}")
    print(f"\nMeilleur score: {best_score:.2f}")
    
    # Afficher les statistiques des essais
    completed_runs = [r for r in optimizer.results if r['completed']]
    if completed_runs:
        print("\nStatistiques des tours complétés:")
        print(f"Nombre de tours complétés: {len(completed_runs)}")
        avg_time = sum(r['elapsed_time'] for r in completed_runs) / len(completed_runs)
        avg_collisions = sum(r['collisions'] for r in completed_runs) / len(completed_runs)
        print(f"Temps moyen: {avg_time:.2f}s")
        print(f"Collisions moyennes: {avg_collisions:.1f}")
    else:
        print("\nAucun tour n'a été complété!")

if __name__ == "__main__":
    main() 