import numpy as np
import pygame
from pendulum_function import *



#-------Simulation Parameters-------------

g= 9.81 #m/s^2
l = 0.3 #m
d_1 = 0.1

omega = np.deg2rad(90)
omega_dot = 0
omega_ddot =0

t_tot = 0
dt = 0

#--------Pygame simulation-----------

WIDTH = 1280
HEIGHT = 720


pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
running = True

position_history = []
velocity_history =[]
accel_history =[]


graph_width = 300
graph_height = 100
max_history = graph_width

color_position_line = (77, 168, 91)
color_velocity_line = (53, 129, 184)
color_accel_line = (209, 0, 0)

position_graph_position = [20,(HEIGHT // 2)- graph_height - 40,graph_width,graph_height]
velocity_graph_position = [20,HEIGHT // 2,graph_width,graph_height]
accel_graph_position = [20,(HEIGHT // 2)+graph_height + 40,graph_width,graph_height]


pendulum_pos = pygame.Vector2(screen.get_width() / 2 + 200*np.cos(omega), screen.get_height() / 2 + 200*np.sin(omega))

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
 
    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(100) / 1000

    omega, omega_dot, omega_ddot = Pendulum_ODE(omega,omega_dot,omega_ddot,g,l,d_1,dt)
    
    position_history.append(np.rad2deg(omega))
    velocity_history.append(np.rad2deg(omega_dot))
    accel_history.append(np.rad2deg(omega_ddot))

    if len(velocity_history) > max_history:
        position_history.pop(0)
        velocity_history.pop(0)
        accel_history.pop(0)


     # fill the screen with a color to wipe away anything from last frame
    screen.fill("white")
    pendulum_pos = pygame.Vector2(screen.get_width() / 2 + 200*np.sin(omega), screen.get_height() / 2 + 200*np.cos(omega))


    pygame.draw.line(screen,"black",(screen.get_width()/2,screen.get_height()/2),pendulum_pos,4)
    pygame.draw.circle(screen, (241, 80, 37), pendulum_pos, 40)
    pygame.draw.line(screen,"black",pendulum_pos,(pendulum_pos[0],pendulum_pos[1]+40))

    #Draw position graph
    draw_graph(screen,position_graph_position,position_history,"Angle",color_position_line)

    
    # Draw velocity graph
    draw_graph(screen,velocity_graph_position,velocity_history,"Velocity",color_velocity_line)
    

    # Draw acceleration graph
    draw_graph(screen,accel_graph_position,accel_history,"Acceleration",color_accel_line)
    

    # flip() the display to put your work on screen
    pygame.display.flip()
    

pygame.quit()
