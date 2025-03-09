import time
from car_simulator import CarSimulator
from keyboard_controller import KeyboardController
from autonomous_navigator import AutonomousNavigator
from follow_gap_navigator import FollowGapNavigator
from equidistance_navigator import EquidistanceNavigator

def main():
    # Créer le simulateur
    simulator = CarSimulator(map_path="TRR.bmp", yaml_path="map.yaml")
    
    # Créer le contrôleur clavier
    keyboard = KeyboardController()
    
    # Créer les navigateurs
    navigators = {
        'basic': AutonomousNavigator(),
        'follow_gap': FollowGapNavigator(),
        'equidistance': EquidistanceNavigator()
    }
    current_navigator = 'equidistance'
    
    # Variables de contrôle
    dt = 0.01  # pas de temps (s)
    autonomous_mode = False
    running = True
    last_time = time.time()
    
    print("=== Contrôles ===")
    print("Flèches : contrôle manuel")
    print("Espace  : activer/désactiver navigation autonome")
    print("n       : changer de stratégie de navigation")
    print("r       : réinitialiser la simulation")
    print("Échap   : quitter")
    print("================")
    
    try:
        while running:
            # Gestion du temps
            current_time = time.time()
            if current_time - last_time < dt:
                continue
            last_time = current_time
            
            # Mise à jour des commandes clavier
            cmd_vel_linear, cmd_vel_angular, running = keyboard.update()
            
            # Obtenir le scan lidar et l'état des collisions
            scan, collision = simulator.step(cmd_vel_linear, cmd_vel_angular, dt)
            
            # Si collision, réinitialiser la simulation
            if collision:
                print("Collision détectée ! Réinitialisation...")
                #simulator.reset()
                continue
            
    except KeyboardInterrupt:
        print("\nArrêt demandé par l'utilisateur")
    finally:
        keyboard.close()
        
if __name__ == "__main__":
    main() 