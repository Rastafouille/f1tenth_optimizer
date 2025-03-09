import pygame
import time

class KeyboardController:
    def __init__(self, 
                 linear_speed=0.5,    # m/s - réduit pour correspondre à l'échelle
                 angular_speed=1.0):   # rad/s
        """Contrôleur clavier pour la voiture
        
        Args:
            linear_speed: vitesse linéaire maximale (m/s)
            angular_speed: vitesse angulaire maximale (rad/s)
        """
        # Initialiser pygame pour la capture des touches
        pygame.init()
        
        # Créer une fenêtre avec un titre
        self.window = pygame.display.set_mode((300, 250))
        pygame.display.set_caption("Contrôle Clavier")
        
        self.max_linear_speed = linear_speed
        self.max_angular_speed = angular_speed
        
        # Pas d'incrémentation des vitesses
        self.linear_step = 0.05  # m/s - réduit pour un contrôle plus fin
        self.angular_step = 0.2  # rad/s - réduit pour un contrôle plus fin
        
        # Délai entre les incrémentations (en secondes)
        self.increment_delay = 0.01  # Plus rapide pour plus de réactivité
        
        # Temps du dernier incrément
        self.last_increment_time = time.time()
        
        # État des commandes
        self.cmd_vel_linear = 0.0
        self.cmd_vel_angular = 0.0
        
        # État des touches
        self.last_keys = pygame.key.get_pressed()  # Initialisation avec l'état actuel
        
        # Police pour le texte
        self.font = pygame.font.Font(None, 24)
        
        # Instructions
        self.instructions = [
            "=== Contrôles ===",
            "HAUT : +vitesse avant",
            "BAS : +vitesse arrière",
            "GAUCHE : +rotation gauche",
            "DROITE : +rotation droite",
            "ESPACE : arrêt",
            "ECHAP : quitter",
            "=================="
        ]
        
    def update(self):
        """Met à jour les commandes en fonction des touches pressées
        
        Returns:
            cmd_vel_linear: vitesse linéaire commandée (m/s)
            cmd_vel_angular: vitesse angulaire commandée (rad/s)
            running: False si demande de quitter
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return 0.0, 0.0, False
                
        # Récupérer l'état des touches
        keys = pygame.key.get_pressed()
        
        # Vérifier si c'est le moment d'incrémenter pour les touches maintenues
        current_time = time.time()
        should_increment = current_time - self.last_increment_time >= self.increment_delay
            
        # Espace pour arrêt
        if keys[pygame.K_SPACE]:
            self.cmd_vel_linear = 0.0
            self.cmd_vel_angular = 0.0
            
        else:
            # Vitesse linéaire
            if keys[pygame.K_UP]:
                # Incrémenter immédiatement sur nouvel appui
                if not self.last_keys[pygame.K_UP]:
                    self.cmd_vel_linear = min(self.cmd_vel_linear + self.linear_step, self.max_linear_speed)
                    self.last_increment_time = current_time
                # Incrémenter si touche maintenue et délai écoulé
                elif should_increment:
                    self.cmd_vel_linear = min(self.cmd_vel_linear + self.linear_step, self.max_linear_speed)
                    self.last_increment_time = current_time
                    
            elif keys[pygame.K_DOWN]:
                if not self.last_keys[pygame.K_DOWN]:
                    self.cmd_vel_linear = max(self.cmd_vel_linear - self.linear_step, -self.max_linear_speed)
                    self.last_increment_time = current_time
                elif should_increment:
                    self.cmd_vel_linear = max(self.cmd_vel_linear - self.linear_step, -self.max_linear_speed)
                    self.last_increment_time = current_time
                
            # Vitesse angulaire
            if keys[pygame.K_LEFT]:
                if not self.last_keys[pygame.K_LEFT]:
                    self.cmd_vel_angular = min(self.cmd_vel_angular + self.angular_step, self.max_angular_speed)
                    self.last_increment_time = current_time
                elif should_increment:
                    self.cmd_vel_angular = min(self.cmd_vel_angular + self.angular_step, self.max_angular_speed)
                    self.last_increment_time = current_time
                    
            elif keys[pygame.K_RIGHT]:
                if not self.last_keys[pygame.K_RIGHT]:
                    self.cmd_vel_angular = max(self.cmd_vel_angular - self.angular_step, -self.max_angular_speed)
                    self.last_increment_time = current_time
                elif should_increment:
                    self.cmd_vel_angular = max(self.cmd_vel_angular - self.angular_step, -self.max_angular_speed)
                    self.last_increment_time = current_time
            
        # Quitter
        if keys[pygame.K_ESCAPE]:
            return 0.0, 0.0, False
            
        # Mettre à jour l'état des touches
        self.last_keys = keys
            
        # Effacer la fenêtre
        self.window.fill((240, 240, 240))  # Fond gris clair
        
        # Afficher les instructions
        y = 20
        for line in self.instructions:
            text = self.font.render(line, True, (0, 0, 0))
            text_rect = text.get_rect(centerx=150, y=y)
            self.window.blit(text, text_rect)
            y += 25
            
        # Afficher les vitesses actuelles
        y += 10
        status = f"Vitesse: {self.cmd_vel_linear:.1f} m/s"
        text = self.font.render(status, True, (0, 100, 0))
        text_rect = text.get_rect(centerx=150, y=y)
        self.window.blit(text, text_rect)
        
        y += 25
        status = f"Rotation: {self.cmd_vel_angular:.1f} rad/s"
        text = self.font.render(status, True, (0, 100, 0))
        text_rect = text.get_rect(centerx=150, y=y)
        self.window.blit(text, text_rect)
            
        # Mettre à jour l'affichage
        pygame.display.flip()
            
        return self.cmd_vel_linear, self.cmd_vel_angular, True
        
    def close(self):
        """Ferme proprement le contrôleur"""
        pygame.quit() 