import pygame
import numpy as np
from shapely.geometry import LineString
from shapely.geometry import Point
import math

pygame.init()

black = (0, 0, 0)
table = (194, 178, 128)
white = (255, 255, 255)
tool_radius = 100
X = 1000
Y = 600

rect = [[20, 20], [960, 480]]  # 2 meters by 1 meter

display_surface = pygame.display.set_mode((X, Y))
POOL_BALL_D = 25
clock = pygame.time.Clock()


def getAngle(a, b, c):
    ang = math.degrees(math.atan2(c.y - (b.y - 10), c.x - b.x) - math.atan2(a.y - (b.y-10), a.x - b.x))
    return ang + 360 if ang < 0 else ang


def dist_pos(pos1, pos2):
    a = np.array([pos1.x, pos1.y])
    b = np.array([pos2.x, pos2.y])
    return np.linalg.norm(a - b)


class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Velocity:
    def __int__(self, acceleration):
        fps = clock.get_fps()
        self.acceleration = acceleration
        self.x = (self.acceleration.X / 480) * (0 / fps)
        self.y = (self.acceleration.Y / 480) * (0 / fps)
        self.frame = 0
        # Input is acceleration in m/s^2
        # Output is velocity in pixels per frame

    def update(self):
        fps = clock.get_fps()
        self.frame += 1
        self.x += (self.acceleration.X / 480) * self.frame / fps
        self.y += (self.acceleration.X / 480) * self.frame / fps


class Acceleration:
    def __init__(self, force, mass):
        opposite = np.sin(force.angle) * force.magnitude
        adjacent = np.cos(force.angle) * force.magnitude
        self.y = adjacent / mass
        self.x = opposite / mass
        # Acceleration is in terms of m/s^2


class Force:
    def __int__(self, magnitude, angle):
        self.magnitude = magnitude
        self.angle = angle


class QBall:
    def __init__(self, pos, color):
        self.pos = pos
        self.color = color
        self.diameter = POOL_BALL_D
        self.radius = self.diameter // 2
        self.slider_pos = Position(self.pos.x, self.pos.y)
        self.slider_color = black
        self.slider_radius = self.diameter // 4
        self.tool_is_released = False
        self.mass = 0.17
        self.forces = []
        self.velocity = None
        self.direction = None

    def slider_move(self):
        if dist_pos(self.slider_pos, self.pos) != 0:


            angle = getAngle(self.pos,self.slider_pos,self.slider_pos)

            opposite = np.sin(angle) * 2
            adjacent = np.cos(angle) * 2


            self.slider_pos.x += int(opposite)
            self.slider_pos.y += int(adjacent)

    def draw(self):

        pygame.draw.circle(display_surface, self.color, [self.pos.x, self.pos.y], self.radius)
        pygame.draw.circle(display_surface, self.slider_color, [self.slider_pos.x, self.slider_pos.y],
                           self.slider_radius)
        pygame.draw.line(display_surface, self.slider_color, [self.slider_pos.x, self.slider_pos.y],
                         [self.pos.x, self.pos.y])

    def addForce(self, force, POC):
        a = np.array([self.pos.x, self.pos.y])
        b = np.array([self.pos.x, self.pos.y - self.radius])
        c = np.array(POC)
        diffAB = a - b
        diffBC = c - b
        cosine_angle = np.dot(diffAB, diffBC) / (np.linalg.norm(diffAB) * np.linalg.norm(diffBC))
        angle = np.arccos(cosine_angle)
        angle = np.degrees(angle)

    def check_collision(self):
        if self.tool_is_released:
            if self.radius <= dist_pos(self.pos, self.slider_pos) <= (self.radius + self.slider_radius):
                print('collision detected')
                p = Point([self.pos.x, self.pos.y])
                c = p.buffer(self.radius).boundary
                l = LineString([[self.pos.x, self.pos.y], [self.slider_pos.x, self.slider_pos.y]])
                i = c.intersection(l)
                self.addForce(1, [int(i.x), int(i.y)])
                self.slider_pos = self.pos
                self.tool_is_released = False

    def apply_forces(self):
        fps = clock.get_fps()
        for force in self.forces:
            self.velocityX = force[0]
            self.velocityY = force[1]

    def move(self):

        self.pos[0] += self.velocityX
        self.pos[1] += self.velocityY

    def update(self):

        # self.apply_forces()
        # self.move()

        self.check_collision()

        self.draw()


ball = QBall(Position(200, 200), white)

is_tool_dragged = False
clock_speed = 100
while True:
    clock.tick(clock_speed)
    mouse_pos = pygame.mouse.get_pos()
    MOUSE = Position(mouse_pos[0], mouse_pos[1])
    display_surface.fill(black)
    pygame.draw.rect(display_surface, table, rect)
    for event in pygame.event.get():
        if event.type == pygame.MOUSEBUTTONDOWN:

            if dist_pos(ball.slider_pos, MOUSE) <= ball.slider_radius:
                is_tool_dragged = True
                ball.tool_is_released = False
        elif event.type == pygame.MOUSEBUTTONUP:
            is_tool_dragged = False

        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

    if is_tool_dragged:
        if dist_pos(MOUSE, ball.pos) >= tool_radius:
            p = Point([ball.pos.x, ball.pos.y])
            c = p.buffer(tool_radius).boundary
            l = LineString([[ball.pos.x, ball.pos.y], [MOUSE.x, MOUSE.y]])
            i = c.intersection(l)

            ball.slider_pos = Position(int(i.x), int(i.y))

        else:
            ball.slider_pos = MOUSE

    else:

        ball.slider_move()

        ball.tool_is_released = True

    ball.update()

    pygame.display.update()
