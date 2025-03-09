import numpy as np
import pygame

class Teleoperation:
    def __init__(self):
        # Initialiser pygame pour la capture des touches
        pygame.init()
        # Créer une petite fenêtre invisible pour capturer les entrées clavier
        self.screen = pygame.display.set_mode((1, 1))
        pygame.display.set_caption("Teleop")
        
        # Paramètres de contrôle
        self.max_speed = 3.0  # m/s
        self.max_steer = 0.4  # rad
        self.speed = 0.0
        self.steer = 0.0
        
        # Paramètres d'accélération
        self.speed_increment = 0.1
        self.steer_increment = 0.1

    def plan(self, obs):
        """
        Retourne les actions de contrôle basées sur les entrées clavier
        Args:
            obs: observation de l'environnement (non utilisée pour la téléopération)
        Returns:
            numpy.ndarray: [angle_braquage, vitesse]
        """
        # Gestion des événements pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return None

        # Obtenir l'état des touches
        keys = pygame.key.get_pressed()
        
        # Mise à jour de la vitesse
        if keys[pygame.K_UP]:
            self.speed = min(self.speed + self.speed_increment, self.max_speed)
        elif keys[pygame.K_DOWN]:
            self.speed = max(self.speed - self.speed_increment, -self.max_speed)
        else:
            # Décélération naturelle
            self.speed = self.speed * 0.95

        # Mise à jour de la direction
        if keys[pygame.K_LEFT]:
            self.steer = min(self.steer + self.steer_increment, self.max_steer)
        elif keys[pygame.K_RIGHT]:
            self.steer = max(self.steer - self.steer_increment, -self.max_steer)
        else:
            # Retour progressif au centre
            self.steer = self.steer * 0.7

        # Arrêt d'urgence avec la touche espace
        if keys[pygame.K_SPACE]:
            self.speed = 0.0
            self.steer = 0.0

        return np.array([[self.steer, self.speed]])

    def __del__(self):
        pygame.quit()
