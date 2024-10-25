import time
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from touchScreenBasicCoordOutput import read_touch_coordinates

# Global variables to store points and their timestamps
points = []

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
    glEnable(GL_POINT_SMOOTH)
    glPointSize(5.0)  # Set the point size

def draw_rectangular_prism():
    glBegin(GL_QUADS)
    # Define vertices and draw the prism
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glEnd()

def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    
    # Draw the rectangular prism
    draw_rectangular_prism()
    
    # Draw the points on top of the rectangular prism
    current_time = time.time()
    glColor3f(1.0, 1.0, 1.0)  # Set the point color to white
    glBegin(GL_POINTS)
    for point, timestamp in points:
        if current_time - timestamp < 2:  # Display point for 2 seconds
            glVertex3f(point[0], point[1], point[2])  # Adjust coordinates as needed
    glEnd()
    
    # Draw three static points for testing
    glBegin(GL_POINTS)
    glVertex3f(0.0, 0.3, 0.0)  # Point 1
    glVertex3f(0.5, 0.3, 0.0)  # Point 2
    glVertex3f(-0.5, 0.3, 0.0) # Point 3
    glEnd()
    
    glutSwapBuffers()

def update_points():
    global points
    for x, y in read_touch_coordinates():
        # Convert touch coordinates to OpenGL coordinates
        gl_x = ((x - 250) / (3800 - 250)) * 2 - 1  # Map x from [250, 3800] to [-1, 1]
        gl_y = ((y - 150) / (3940 - 150)) * 0.4 - 0.2  # Map y from [150, 3940] to [-0.2, 0.2]
        gl_z = 0.21  # Slightly above the top of the prism to ensure visibility
        
        # Add the point with the current timestamp
        points.append(((gl_x, gl_y, gl_z), time.time()))
        
        # Post a USEREVENT to the Pygame event queue
        pygame.event.post(pygame.event.Event(pygame.USEREVENT))

def main():
    glutInit()
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
    glutInitWindowSize(800, 600)
    glutInitWindowPosition(0, 0)
    glutCreateWindow("OpenGL Rectangular Prism with Touch Points")
    
    init()
    glutDisplayFunc(display)
    glutIdleFunc(display)
    
    # Start a separate thread to update points
    import threading
    threading.Thread(target=update_points, daemon=True).start()
    
    glutMainLoop()

if __name__ == "__main__":
    main()