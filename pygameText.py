import pygame

class Text:
    def __init__(self, t, pos, screen):
        self.font = pygame.font.Font('freesansbold.ttf', 16)
        self.text = self.font.render(t, True, (255, 255, 255))
        self.rect = self.text.get_rect()
        self.rect.center = pos
        self.screen = screen

    def draw(self):
        self.screen.blit(self.text, self.rect)