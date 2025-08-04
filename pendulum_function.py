import pygame
import numpy as np
#this file contains all the functions to be used on the reaction wheel pendulum project


# Pygame functions
integral = 0
error_prev = 0

def draw_graph_background(surface, x, y, w, h, scale, label_text, color):
    # Background
    pygame.draw.rect(surface, (25, 25, 25), (x, y, w, h))
    
    # Centerline
    pygame.draw.line(surface, (100, 100, 100), (x, y + h // 2), (x + w, y + h // 2), 1)
    
    # Horizontal grid lines
    for i in range(-2, 3):  # 5 horizontal lines
        y_line = y + h // 2 - int(i * h // 5)
        pygame.draw.line(surface, (60, 60, 60), (x, y_line), (x + w, y_line), 1)

    # Labels
    font = pygame.font.SysFont(None, 20)
    for i in range(-2, 3):
        val = i * (h // 5) / scale
        label = font.render(f"{-val:.2f}", True, color)
        y_label = y + h // 2 + int(i * h // 5) - 8
        surface.blit(label, (x + 5, y_label))
    
    # Title
    title = font.render(label_text, True, color)
    surface.blit(title, (x + 5, y - 18))

def draw_graph_line(surface, data, x, y, h, scale, color):
    for i in range(1, len(data)):
        y1 = y + h // 2 - int(data[i - 1] * scale)
        y2 = y + h // 2 - int(data[i] * scale)
        pygame.draw.line(surface, color, (x + i - 1, y1), (x + i, y2))

def draw_graph(surface,graph_layout,data,label_text,color_line,color_background=(100, 200, 255)):
    scale=0.5*(graph_layout[3]/max(data))
    draw_graph_background(surface,graph_layout[0],graph_layout[1],graph_layout[2],graph_layout[3],scale,label_text,color_line)
    draw_graph_line(surface,data,graph_layout[0],graph_layout[1],graph_layout[3],scale,color_line)



# Pendulum functions

def Pendulum_ODE(omega,omega_dot,g,l,d,dt):
    omega_ddot=-g*np.sin(omega)/l - d*omega_dot
    omega_dot+=omega_ddot*dt
    omega+=omega_dot*dt
    return omega,omega_dot,omega_ddot


def accelerate_motor(dir,power):
    torque_mod = 1*power
    if np.absolute(torque_mod)>=2:
        torque_mod = 2

    if dir == 1:
        return torque_mod
    elif dir == 0:
        return -torque_mod
    else:
        return 0
    
def PID (set_peed,real_speed,kp,ki,kd,dt):
    global integral, error_prev

    error = set_peed-real_speed

    #print(error)

    proportional = kp*error
    integral += ki*error*dt
    derivative = kd*(error-error_prev)/dt

    error_prev = error

    response = proportional + integral + derivative

    

    return response



        
    
    