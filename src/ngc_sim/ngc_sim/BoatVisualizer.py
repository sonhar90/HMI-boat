import pygame
import sys
import math

# Initialize Pygame
pygame.init()

# Screen settings
screen_width = 800
screen_height = 600
# screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Rotating Boat")

# Colors
white = (255, 255, 255)
red = (255, 0, 0)
black = (0, 0, 0)


class BoatMovementVisualizer():
    """
    This class is the overlord in the visualiser, and is supplying the
    necessary methods to update and visualise the boat movement in a 2d map

    """
    white = (255, 255, 255)
    red = (255, 0, 0)
    black = (0, 0, 0)

    def __init__(self, fps):
        self.target_spawn_radius = None
        self.fps = fps
        self.boat = None
        self.hud = None
        self.SCALE = 8
        self.clock = None
        self.observations = None
        self.screen = None
        self.VIEWPORT_W = 1000  # the width of the pygame screen
        self.VIEWPORT_H = 1000  # the height of the pygame screen
        self.all_sprites = None
        self.canvas = pygame.Surface((self.VIEWPORT_W, self.VIEWPORT_H))


    def update(self, eta, nu, info_str, target = None):
        # Initiate if it is the first time it is called
        if self.screen is None:  # if the screen is not initiatet, everythen needs to be initiated
            pygame.init()
            pygame.display.init()
            self.screen = pygame.display.set_mode((self.VIEWPORT_W, self.VIEWPORT_H))
            self.canvas = pygame.Surface((self.VIEWPORT_W, self.VIEWPORT_H))
            self.clock = pygame.time.Clock()
            self.all_sprites = pygame.sprite.LayeredUpdates()
            self.boat= Boat(self.all_sprites, radius=10, CANVAS_W=self.VIEWPORT_W,
                                        CANVAS_H=self.VIEWPORT_H, scale=self.SCALE)
            self.hud = HeadUpDisplay(group=self.all_sprites, screen_width=self.VIEWPORT_W,
                                     screen_height=self.VIEWPORT_H, layer=1, spawn_radius=self.target_spawn_radius,
                                     scale=self.SCALE)

        self.boat.set_pos(eta=[eta[0], eta[1], eta[5]])
        self.screen.fill(black)
        self.canvas.fill(white)
        if len(self.boat.trajectory) > 1:
            pygame.draw.lines(surface=self.canvas, color=black, closed=False, points=self.boat.trajectory, width=1)
        self.hud.hud_str = info_str

        if target is not None:
            pygame.draw.circle(surface=self.canvas, center=(
                target[0] * self.SCALE + self.VIEWPORT_W / 2,
                target[1] * self.SCALE + self.VIEWPORT_H / 2), radius=3,
                               color=red)
        self.all_sprites.update()
        self.all_sprites.draw(self.canvas)
        self.canvas.blit(pygame.transform.rotate(self.canvas, angle=90),
                         (0, 0))  # rotate the coordinate system fom the top left to the bottom left
        canvas_position = (0, self.VIEWPORT_H - self.VIEWPORT_H)

        self.screen.blit(self.canvas, canvas_position)  # place the coordinate system at the bottom left of the screen
        pygame.display.flip()
        if self.boat.trajectory is not None and len(self.boat.trajectory) > 1:
            pygame.draw.lines(self.canvas, black, False, self.boat.trajectory, 1)
            # draw target
            pygame.draw.circle(surface=self.canvas, center=(
                target[0] * self.SCALE + self.VIEWPORT_W / 2,
                target[1] * self.SCALE + self.VIEWPORT_H / 2), radius=3,
                               color=red)

        assert self.screen is not None
        pygame.event.pump()
        self.clock.tick(self.fps)
        pygame.display.flip()



    def clean(self):
        #clean persistent variables in the boat
        self.boat.trajectory = []

    def close(self):
        if self.screen is not None:
            import pygame
            pygame.display.quit()
            pygame.quit()
            self.isopen = False
class Boat(pygame.sprite.Sprite):
    def __init__(self, group, radius, CANVAS_H, CANVAS_W, layer=0, x=0, y=0, scale=1):
        super().__init__()
        self.angle = 0
        self.SCALE = scale
        self.HEIGHT_OFF_SET = CANVAS_H // 2
        self.WIDTH_OFF_SET = CANVAS_W // 2
        self.trajectory = []
        self.original_image = pygame.Surface((radius * 2, radius * 2))
        self.original_image.fill(white)
        pygame.draw.circle(self.original_image, red, (radius, radius), radius)
        pygame.draw.line(self.original_image, black, (radius, radius), (3 * radius, radius), 2)

        self.image = self.original_image
        self.rect = self.image.get_rect()
        self.rect.center = (x, y)
        self._layer = layer
        pygame.sprite.Sprite.__init__(self, group)

    def rotate(self, angle_change):
        self.angle += angle_change
        self.image = pygame.transform.rotate(self.original_image, math.degrees(self.angle))
        self.rect = self.image.get_rect(center=self.rect.center)

    def move_forward(self, speed):
        self.rect.x += speed * math.cos(self.angle)
        self.rect.y -= speed * math.sin(self.angle)

    def move_backward(self, speed):
        self.rect.x -= speed * math.cos(self.angle)
        self.rect.y += speed * math.sin(self.angle)

    def set_pos(self, eta: []):
        """
        Function to set a new position and orientation of the boat
        """
        assert len(eta) == 3, f"eta must be of len 3 and contain [x,y, yaw]"
        self.angle = eta[2]
        self.image = pygame.transform.rotate(self.original_image, -math.degrees(self.angle))
        self.rect.center = (eta[0] * self.SCALE, eta[1] * self.SCALE)
        self._adjust_origo()
        self._log_trajectory()


    def _log_trajectory(self):
        self.trajectory.append(self.rect.center)

    def _adjust_origo(self):
        self.rect.x = self.rect.x + self.WIDTH_OFF_SET
        self.rect.y = self.rect.y + self.HEIGHT_OFF_SET
        # self.rect.center = (self.rect.center[0]+self.WIDTH_OFF_SET, self.rect.center[1]+self.HEIGHT_OFF_SET)


class HeadUpDisplay(pygame.sprite.Sprite):
    def __init__(self, group, screen_width, screen_height, spawn_radius=50, scale=1, layer=1):
        super().__init__()
        self._layer = layer  # Set the layer attribute with the correct prefix
        self.SCALE = scale
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.image = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
        self.rect = self.image.get_rect()
        self.cross_color = (0, 0, 0)
        self.cross_thickness = 2
        self.cross_length = 20
        self.font = pygame.font.Font(None, 15)
        self.hud_str = f"THIS IS MY TEXT"
        self.spawn_radius = 50
        pygame.sprite.Sprite.__init__(self, group)

    def draw_cross(self):
        # Calculate the center of the screen
        center_x = self.screen_width // 2
        center_y = self.screen_height // 2

        # Draw horizontal line of the cross
        pygame.draw.line(self.image, self.cross_color, (center_x - self.cross_length, center_y),
                         (center_x + self.cross_length, center_y), self.cross_thickness)

        # Draw vertical line of the cross
        pygame.draw.line(self.image, self.cross_color, (center_x, center_y - self.cross_length),
                         (center_x, center_y + self.cross_length), self.cross_thickness)

        # draw the radius indicating the outer border of the spawn circle:
        pygame.draw.circle(self.image, black, (center_x, center_y), self.spawn_radius * self.SCALE,
                           5)  # (x, y), radius, width

    def draw_hud_str(self):
        y = 10
        lines = list(reversed(self.hud_str.splitlines()))
        for line in lines:
            text_surface = self.font.render(line, True, (0, 0, 0))
            self.image.blit(pygame.transform.rotate(text_surface, angle=-90), dest=(y, 10))
            y += text_surface.get_height()
        """
        text_surface = self.font.render(, True, (0, 0, 0))  # You can adjust the text color
        text_rect = text_surface.get_rect()
        text_rect.topleft = (0, 0)  # Adjust the position as needed
        """
        # self.image.blit(text_surface, text_rect)

    def update(self):
        self.image.fill((0, 0, 0, 0))  # Clear the HUD surface
        self.draw_cross()
        self.draw_hud_str()
