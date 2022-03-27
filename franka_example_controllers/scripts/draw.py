import pygame
from pygame.locals import *
 
class vec2d(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

 
gray = (100,100,100)
white = (255,255,255)
lightgray = (200,200,200)
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
yellow = (255,255,0)
X,Y,Z = 0,1,2

turn = 0



def compute_beizer_from_ends(vertices):
    ax = vertices[0][0]
    ay = vertices[0][1]

    bx = vertices[1][0]
    by = vertices[1][1]
    t = [0,0]
    if (ay < by):
        t[0] = bx
        t[1] = ay
    else:
        t[0] = ax
        t[1] = by
    v = [[0,0],[0,0],[0,0],[0,0]]
    v[0][0] = ax
    v[0][1] = ay
    v[1][0] = t[0]
    v[1][1] = t[1]
    v[2][0] = bx
    v[2][1] = by
    v[3][0] = bx
    v[3][1] = by
    
    return v


def compute_beizer_animation(vertices,t):
    p = [0,0]

    b0x = vertices[0][0]
    b0y = vertices[0][1]
    b1x = vertices[1][0]
    b1y = vertices[1][1]
    b2x = vertices[2][0]
    b2y = vertices[2][1]
    b3x = vertices[3][0]
    b3y = vertices[3][1]
    
    p[0]  = (1-t)*(1-t)*(1-t)*b0x + 3*(1-t)*(1-t)*t*b1x + 3*(1-t)*t*t*b2x + t*t*t*b3x;
    p[1] = (1-t)*(1-t)*(1-t)*b0y + 3*(1-t)*(1-t)*t*b1y + 3*(1-t)*t*t*b2y + t*t*t*b3y;

    return p


def compute_bezier_points(vertices, numPoints=None):
    if numPoints is None:
        numPoints = 60
    if numPoints < 2 or len(vertices) != 4:
        return None

    result = []

    b0x = vertices[0][0]
    b0y = vertices[0][1]
    b1x = vertices[1][0]
    b1y = vertices[1][1]
    b2x = vertices[2][0]
    b2y = vertices[2][1]
    b3x = vertices[3][0]
    b3y = vertices[3][1]

    ax = -b0x + 3 * b1x + -3 * b2x + b3x
    ay = -b0y + 3 * b1y + -3 * b2y + b3y

    bx = 3 * b0x + -6 * b1x + 3 * b2x
    by = 3 * b0y + -6 * b1y + 3 * b2y

    cx = -3 * b0x + 3 * b1x
    cy = -3 * b0y + 3 * b1y

    dx = b0x
    dy = b0y

    numSteps = numPoints - 1 
    h = 1.0 / numSteps 

    pointX = dx
    pointY = dy

    firstFDX = ax * (h * h * h) + bx * (h * h) + cx * h
    firstFDY = ay * (h * h * h) + by * (h * h) + cy * h


    secondFDX = 6 * ax * (h * h * h) + 2 * bx * (h * h)
    secondFDY = 6 * ay * (h * h * h) + 2 * by * (h * h)

    thirdFDX = 6 * ax * (h * h * h)
    thirdFDY = 6 * ay * (h * h * h)

    result.append((int(pointX), int(pointY)))

    for i in range(numSteps):
        pointX += firstFDX
        pointY += firstFDY

        firstFDX += secondFDX
        firstFDY += secondFDY

        secondFDX += thirdFDX
        secondFDY += thirdFDY

        result.append((int(pointX), int(pointY)))

    return result
    
def main():
    pygame.init()
    screen = pygame.display.set_mode((1024, 768))
 
    
    cp = compute_beizer_from_ends([[100,100],[500,150]])
    control_points = [vec2d(cp[0][0],cp[0][1]), vec2d(cp[1][0],cp[1][1]), vec2d(cp[2][0],cp[2][1]), vec2d(cp[3][0],cp[3][1])]
 
    
    clock = pygame.time.Clock()
    p1 = [100,100]
    p2 = [500,150]
    newp1 = [0,0]
    running = True

    draw1 = False

    turn = 0

    start_time = None
    go = False
    
    clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type in (QUIT, KEYDOWN):
                running = False
            elif event.type == MOUSEBUTTONDOWN and event.button == 1:
                if turn == 0:
                    newp1 = pygame.mouse.get_pos()
                    draw1 = True
                elif turn == 1:
                    p2 = pygame.mouse.get_pos()
                    p1 = newp1
                    start_time = pygame.time.get_ticks()
                    go = True
                turn += 1
                turn = turn % 2
            

        cp = compute_beizer_from_ends([p1,p2])
        control_points = [vec2d(cp[0][0],cp[0][1]), vec2d(cp[1][0],cp[1][1]), vec2d(cp[2][0],cp[2][1]), vec2d(cp[3][0],cp[3][1])]
        
        
        screen.fill(gray)
        if go:
            time_since_enter = pygame.time.get_ticks() - start_time
            t = time_since_enter / 2000
            if t > 1:
                t = 1.0
            p = compute_beizer_animation([(x.x, x.y) for x in control_points],t) 
            pygame.draw.circle(screen, yellow, (p[0],p[1]), 5)
        
        if draw1:
            pygame.draw.circle(screen, yellow, (newp1[0],newp1[1]), 5)
        
        
        for p in control_points:
            pygame.draw.circle(screen, blue, (int(p.x), int(p.y)), 4)

        
        pygame.draw.lines(screen, lightgray, False, [(x.x, x.y) for x in control_points])

        
        b_points = compute_bezier_points([(x.x, x.y) for x in control_points])

        for x in b_points:
            pygame.draw.circle(screen, yellow, x, 1)
        pygame.draw.lines(screen, pygame.Color("red"), False, b_points, 2)

        
        pygame.display.flip()
        clock.tick(100)
        
    
if __name__ == '__main__':
    main()