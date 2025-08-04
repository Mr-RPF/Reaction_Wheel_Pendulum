
from pendulum_function import *

if __name__ == "__main__":


    #-------Simulation Parameters-------------

    g= 9.81 #m/s^2
    l = 0.3 #m
    m_p = 1 #kg
    m_w = 0.05 #kg

    d_p = 0.01
    d_w = 0.01

    omega = np.deg2rad(170)
    omega_dot = 0.001
    omega_ddot =0

    phi = np.deg2rad(0)
    phi_dot = 0
    phi_ddot = 0

    omega_setpoint=np.deg2rad(180)

    t_tot = 0
    dt = 0

    I_pendulum = (m_p/3 +m_w)*l**2 
    print(I_pendulum) 
    I_wheel = 0.005#kg * m^2

    alpha = g*l*(m_w+0.5*m_p)

    t_mot = 0

    def dynamics_reaction_wheel(y,t,m):
        o,o_dot,p,p_dot = y

        A = np.array([[I_pendulum+I_wheel,I_wheel],[I_wheel,I_wheel]])
        t_mot = m
        b1 = -1*t_mot -d_p*o_dot-alpha*np.sin(o)
        b2 = t_mot-d_w*p_dot

        b = np.array([[b1],[b2]]) 

        result = np.linalg.solve(A,b)
        
        o_ddot = result[0][0] 
        o_dot += o_ddot*t
        o += o_dot*t

        if o > 2*np.pi:
            o-=2*np.pi
        elif o<0:
            o+=2*np.pi

        p_ddot = result[1][0]
        p_dot += p_ddot*t
        p += p_dot*t

        return o,o_dot, o_ddot,p, p_dot, p_ddot


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
        dt = clock.tick(60) / 1000

        #omega, omega_dot, omega_ddot = Pendulum_ODE(omega,omega_dot,g,l,d_p,dt)

        speed_controller = PID(omega,omega_setpoint,2,0.5,0.5,dt)

        if speed_controller >=1:
            t_mot = accelerate_motor(1,speed_controller)
        elif speed_controller<=1:
            t_mot = accelerate_motor(0,speed_controller)

        x = [omega,omega_dot,phi,phi_dot]
        omega,omega_dot, omega_ddot,phi, phi_dot, phi_ddot = dynamics_reaction_wheel(x,dt,t_mot)

        position_history.append(np.rad2deg(omega))
        velocity_history.append(phi_dot)
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
        pygame.draw.line(screen,"black",pendulum_pos,(pendulum_pos[0] + 40*np.cos(phi),pendulum_pos[1]+40*np.sin(phi)),2)

        #Draw position graph
        draw_graph(screen,position_graph_position,position_history,"Angle",color_position_line)

        
        # Draw velocity graph
        draw_graph(screen,velocity_graph_position,velocity_history,"Velocity",color_velocity_line)
        

        # Draw acceleration graph
        draw_graph(screen,accel_graph_position,accel_history,"Acceleration",color_accel_line)
        

        # flip() the display to put your work on screen
        pygame.display.flip()
        

    pygame.quit()
