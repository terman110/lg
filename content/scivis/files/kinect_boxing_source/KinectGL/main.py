#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

PROGRAM_TITLE = "Perspective Boxing"

# Benoetigte Module (alle 32Bit, unabhaengig von Betriebssystem):
# Python 2.7:                       http://python.org/download/releases/2.7.3/
# setuptools:                       http://pypi.python.org/pypi/setuptools
# Windows: pywin32:                 http://sourceforge.net/projects/pywin32/
# NumPy:                            http://numpy.scipy.org/
# pygame:                           http://www.pygame.org/download.shtml
# PyOpenGL 3.x:                     http://pyopengl.sourceforge.net/
# Python Imaging Library (PIL):     http://www.pythonware.com/products/pil/
# PyKinect (startet auch ohne):     https://pytools.codeplex.com/wikipage?title=PyKinect
# fuer PyKinect:
#   Visual Studio 2010, z.B.:       https://www.microsoft.com/visualstudio/en-us/products/2010-editions/visual-cpp-express  (VISUAL C++ 2010 EXPRESS)
#   KinectSDK (32 Bit):             http://kinectforwindows.org/

# Bibliotheken laden
from numpy import *
import sys
import itertools
import ctypes
import thread
import time

# pyGame laden
import pygame
from pygame.color import THECOLORS
from pygame.locals import *
if not pygame.mixer: 
    print 'Warnung: Ton wurde deaktiviert.'

# pyKinect laden
try:
    #from pykinect import nui
    from pykinect.nui import JointId
    from pykinect.nui import _NUI_IMAGE_DEPTH_MINIMUM
    from pykinect.nui import _NUI_IMAGE_DEPTH_MAXIMUM
    from pykinect import *
except:
    print "pyKinect konnte nicht geladen werden"
    pass

# OpenGL und Toolkits laden
try:
    # For OpenGL-ctypes
    from OpenGL import platform
    gl = platform.OpenGL
except ImportError:
    try:
        # For PyOpenGL
        gl = cdll.LoadLibrary('libGL.so')
    except OSError:
        # Load for Mac
        from ctypes.util import find_library
        # finds the absolute path to the framework
        path = find_library('OpenGL')
        gl = cdll.LoadLibrary(path)
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from PIL import Image

# OpenGL Funktionen und Bibliotheken laden
import glContent

# Fenstergroess (GLUT)
WIN_SIZE = [ 1024, 768]

# Groesse des Kinect-Tiefenpuffers
DEPTH_SIZE = ( 640, 480)

# Intensitaet der Perspektivenmanipulation
cam_pos_multiplier = 20

# Cameraposition
cam = [0.0, 0.0, 0.0] #-100.0]

# Tiefeninformation
depth_arr = empty( (2*DEPTH_SIZE[0],DEPTH_SIZE[1]), uint16)

# Kinect verfuegbar
kinect_available = True

# Schalter fuer Hilfsmenue
help_menu = False

# Hilfsmenue
help = ( PROGRAM_TITLE,
         "",
         "Tastaturbelegung:",
         "  \"i\"     : Hilfe ein/ ausschalten",
         "  \"Hoch\"  : Kinect stärker neigen",
         "  \"Runter\": Kinect schwächer neigen",
         "  \"q\"     : Kameraposition weiter vom Zentrum entfernen",
         "  \"e\"     : Kameraposition weiter zum Zentrum führen",
         "  \"x\"     : Skelett(e) zeichnen",
         "  \"f\"     : Vollbildmodus umschalten",
         "  \"ESC\"   : Programm beenden" )

# pyGameEvent fuer neue Kinectdaten
KINECTEVENT = pygame.USEREVENT

# Skelettfarben
SKELETON_COLORS = [THECOLORS["red"], 
                   THECOLORS["blue"], 
                   THECOLORS["green"], 
                   THECOLORS["orange"], 
                   THECOLORS["purple"], 
                   THECOLORS["yellow"], 
                   THECOLORS["violet"]]

# Körperteile
# Linker Arm
LEFT_ARM = ( JointId.ShoulderCenter, 
             JointId.ShoulderLeft, 
             JointId.ElbowLeft, 
             JointId.WristLeft, 
             JointId.HandLeft)

# Rechter Arm
RIGHT_ARM = (JointId.ShoulderCenter, 
             JointId.ShoulderRight, 
             JointId.ElbowRight, 
             JointId.WristRight, 
             JointId.HandRight)

# Linkes Bein
LEFT_LEG = ( JointId.HipCenter, 
             JointId.HipLeft, 
             JointId.KneeLeft, 
             JointId.AnkleLeft, 
             JointId.FootLeft)

# Rechtes Bein
RIGHT_LEG = (JointId.HipCenter, 
             JointId.HipRight, 
             JointId.KneeRight, 
             JointId.AnkleRight, 
             JointId.FootRight)

# Rückrad
SPINE = (    JointId.HipCenter, 
             JointId.Spine, 
             JointId.ShoulderCenter, 
             JointId.Head)

# Tiefenpuffer info
dispInfo = 0

# Tiefenpuffer fenster
screen = 0

# Skeletthandle
skeletons = None

# Nur ein Skelett zulassen
only_one_person = True

# Skelett zeichnen
draw_skelett = True

# Tiefeninformation bei Schlag ignorieren
disable_depth = False

# Niedrigste Schlaggeschwindigkeit 
min_v = 1.

#############################################################
#                                                           #
#   OpenGL und GLUT                                         # 
#                                                           #
#############################################################

# GLUT-Callbackfunktion. Grafik rendern
def draw():
    global cam
    global help_menu, help

    # Skybox coordinates
    center = (0.0, 0.0, 100.0)

    # Matrizen des Modellraums laden
    glMatrixMode(GL_MODELVIEW)

    # Farb(pixel)puffer und Tiefenpuffer (OpenGL!) loeschen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    
    # Neue perspektivische Matrix berechnen und Matrix anwenden
    # http://pyopengl.sourceforge.net/documentation/manual/gluLookAt.3G.html
    glLoadIdentity()
    gluLookAt( -cam[0], cam[1], cam[2], center[0], center[1], center[2], 0., 1., 0.)

    # Szene rendern
    glContent.renderScene( center)

    # Hilfe ausgeben
    if help_menu:
        offset = 17
        length = len(help)
        for index, string in enumerate(help):
            glContent.renderStr( 50, (length  + 4 )*offset-(index+1)*offset, string, 1.0, 1.0, 1.0, GLUT_BITMAP_9_BY_15 )

    glContent.renderStr( 10, 10, 'Kameraposition ( {}, {}, {} )'.format( cam[0], cam[1], cam[2] ), .2, .8, .2, GLUT_BITMAP_8_BY_13)
    glContent.renderStr( 10, 25, '{} FPS'.format( int( glContent.framerate()) ), 1.0, 1.0, 1.0, GLUT_BITMAP_9_BY_15 )

    glutSwapBuffers()

# GLUT-Callbackfunktion. Wird im Leerlauf aufgerufen und fordert zum rendern auf 
def idle():
    # GLUT zum rendern auffordern (Display-Callback laden)
    glutPostRedisplay()
    # pyGame-Fenster refreshen
    if( pygame.display.get_active()):
        pygame.display.update()
    
isFS = 0    # Ist GLUT im Vollbildmodus?
# GLUT-Callbackfunktion. Taste gedruekt
def key(k, x, y):
    global cam, isFS, help_menu, WIN_SIZE, draw_skelett, disable_depth
    # w: Kamera nach rechts
    if k == 'd':
        cam[0] += 0.5
    # s: Kamera nach links
    elif k == 'a':
        cam[0] -= 0.5
    # w: Kamera nach oben
    elif k == 'w':
        cam[1] += 0.5
    # s: Kamera nach unten
    elif k == 's':
        cam[1] -= 0.5
    # q: Kamera weiter vom Zentrum entfernen
    elif k == 'q':
        cam[2] -= 0.5
    # e: Kamera weiter zum Zentrum fuehren
    elif k == 'e':
        cam[2] += 0.5
    # q: Kamera weiter vom Zentrum entfernen
    if k == 'q':
        cam[2] -= 0.5
    # e: Kamera weiter zum Zentrum fuehren
    elif k == 'e':
        cam[2] += 0.5
    # f: Vollbildmodus umschalten
    elif k == 'f':
        if isFS == 0:
            glutFullScreen()
            isFS = 1
        else:
            glutReshapeWindow( WIN_SIZE[0], WIN_SIZE[1])
            isFS = 0
    # i: Hilfe einblenden
    elif k == 'i':
        help_menu = not help_menu
        #if help_menu == True:
        #    help_menu = False
        #else:
        #    help_menu = True
    # ESC: Programm beenden
    elif ord(k) == 27: # Escape
        sys.exit(0)
    # x: Skelett ein/ ausblenden
    elif k == 'x':
        draw_skelett = not draw_skelett
        #if( draw_skelett):
        #    draw_skelett = False
        #else:
        #    draw_skelett = True
    elif k == 'z':
        disable_depth = not disable_depth
        if( disable_depth): print( 'Tiefeninformation wird ignoriert')
        else: print( 'Tiefeninformation wird berücksichtigt')
    else:
        return

    # GLUT zum rendern auffordern (Display-Callback laden)
    glutPostRedisplay()

# GLUT-Callbackfunktion. Sondertaste gedruekt
def special(k, x, y):  
    # Pfeielobentaste: Kinect staerker neigen
    if k == GLUT_KEY_UP:
        try:
            kinect.camera.elevation_angle = kinect.camera.elevation_angle + 2
        except:
            print "Kinect nicht verfügbar?!"
            pass
    # Pfeieluntentaste: Kinect schwaecher neigen
    elif k == GLUT_KEY_DOWN:
        try:
            kinect.camera.elevation_angle = kinect.camera.elevation_angle - 2
        except:
            print "Kinect nicht verfügbar?!"
            pass
    else:
        return
    # GLUT zum rendern auffordern (Display-Callback laden)
    glutPostRedisplay()

# GLUT-Callbackfunktion. Fensterdimensionen haben sich geaendert.
def reshape(width, height):
    global WIN_SIZE
    
    # Neue Fenstergroesse festlegen
    WIN_SIZE[0] = width
    WIN_SIZE[1] = height

    # Verhaeltnis uwischen Hoehe und Breite
    h = float(height)/ float(width)

    # Neuer Viewport
    glViewport(0, 0, width, height)

    # Projektionsmatrix manipulieren
    glMatrixMode(GL_PROJECTION)
    # Einheitsmatrix laden
    glLoadIdentity()
        
    # Neue Frustums-Matrix berechnen und uebergeben
    glFrustum(-1.0, 1.0, -h, h, 5.0, 3000.0)

    # Moderlierungsmatrix manipulieren
    glMatrixMode(GL_MODELVIEW)

    # Einheitsmatrix laden
    glLoadIdentity()
    
    # GLUT zum rendern auffordern (Display-Callback laden)
    glutPostRedisplay()

# GLUT-Callbackfunktion. Test ob Fenster sichtbar ist.
def visible(vis):
    if vis == GLUT_VISIBLE:
        glutIdleFunc(idle)
    else:
        glutIdleFunc(None)

# GLUT (OpenGL Utility Toolkit) initialisieren 
def initGLUT():
    global sys

    # GLUT starten
    glutInit(sys.argv)

    # Anzeigemodus definieren
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)

    # Fensterposition
    sw = glutGet( GLUT_SCREEN_WIDTH) - WIN_SIZE[0]
    sh = glutGet( GLUT_SCREEN_HEIGHT) - WIN_SIZE[1]
    glutInitWindowPosition( sw/2, sh/2)

    # Fenstergroesse
    glutInitWindowSize(WIN_SIZE[0], WIN_SIZE[1])

    # Fenster erstellen und Titel zuweisen
    glutCreateWindow( PROGRAM_TITLE)

    # Callbackfunktionen festlegen
    glutDisplayFunc(draw)
    glutReshapeFunc(reshape)
    glutKeyboardFunc(key)
    glutSpecialFunc(special)
    glutVisibilityFunc(visible)

#############################################################
#                                                           #
#   pyGame und pyKinect                                     # 
#                                                           #
#############################################################

# .wav Sounddatei laden
def playsound(soundfile):
    sound = pygame.mixer.Sound(soundfile)
    #clock = pygame.time.Clock()
    sound.play()
    #while pygame.mixer.get_busy():
    #    clock.tick(1)

def getDepth( depth_array, x, y):
    return ((( depth_array[ 2*x+1, y] << 8) + depth_array[ 2*x, y] ) >> 3) / 1000000.
        
# Berechnet neue Kamerakoordinaten aus Kinect-Tiefenpuffer und speichert sie in cam
omega = [ 0., 0., 0.]
phi = [ 0., 0., 0.]
handPosR = [0.,0.,0.]
handPosL = [0.,0.,0.]
def updateCamFromKinectEvent():
    global DEPTH_SIZE, WIN_SIZE
    global cam
    global kinect_available
    global rHandPos, lHandPos
    global depth_arr, dt, t
    global omega, phi, handPosR, handPosL
    global isHitingR, isHitingL, startR, startL
    global frame_ready
    global screen, dispInfo

    if not kinect_available:
        return

    # pyGame-Events nach Kinect-Events durchsuchen
    events = pygame.event.get()
    for i, e in enumerate(events):

        # Kinect-Eventgefunden
        if e.type == KINECTEVENT:
            # Zeitabfrage
            t0 = float( time.time())
            tmp_dt = (t0 - glContent.t)
            if( tmp_dt >= 0.01):
                glContent.dt = tmp_dt
            else:
                return

            # Skeletdaten laden
            # Alle Skeletdaten verarbeiten
            for index, data in enumerate(e.skeletons):
                # Nur erstes Skelett berücksichtigen
                if( index > 0 and only_one_person):
                    continue

                # Position des Kopfs in Fensterkoordinaten bestimmen
                HeadPos = nui.SkeletonEngine.skeleton_to_depth_image(data.SkeletonPositions[JointId.Head], DEPTH_SIZE[0], DEPTH_SIZE[1])
                rHandPos = nui.SkeletonEngine.skeleton_to_depth_image(data.SkeletonPositions[JointId.HandRight], DEPTH_SIZE[0], DEPTH_SIZE[1])
                lHandPos = nui.SkeletonEngine.skeleton_to_depth_image(data.SkeletonPositions[JointId.HandLeft], DEPTH_SIZE[0], DEPTH_SIZE[1])

                if(HeadPos[0] == 0 or HeadPos[1] == 0):
                    continue

                cam = [ ( HeadPos[0] / DEPTH_SIZE[0] * 2. - 1. ) * cam_pos_multiplier, 
                      ( HeadPos[1] / DEPTH_SIZE[1] * 2. - 1. ) * cam_pos_multiplier * float(WIN_SIZE[1]) / float(WIN_SIZE[0]),
                      cam[2]]
                HeadDepth = getDepth( depth_arr, HeadPos[0], HeadPos[1])

                # Skelett zeichnen
                if( draw_skelett):
                    head = nui.SkeletonEngine.skeleton_to_depth_image(data.SkeletonPositions[JointId.Head], dispInfo.current_w, dispInfo.current_h) 
                    pygame.draw.circle( screen, SKELETON_COLORS[index], (int(head[0]), int(head[1])), 20, 0)
                    draw_skeleton_data(data, index, SPINE)
                    draw_skeleton_data(data, index, LEFT_ARM)
                    draw_skeleton_data(data, index, RIGHT_ARM)
                    draw_skeleton_data(data, index, LEFT_LEG)
                    draw_skeleton_data(data, index, RIGHT_LEG)

                handV = [0.,0.,0.]

                posR = [ float( rHandPos[0]) / float( DEPTH_SIZE[0]), float( rHandPos[1]) / float( DEPTH_SIZE[1]), 0.]
                if( rHandPos[0] >= 0 and rHandPos[0] < DEPTH_SIZE[0] and rHandPos[1] >= 0 and rHandPos[1] < DEPTH_SIZE[1] and not disable_depth):
                        posR[2] = getDepth( depth_arr, rHandPos[0], rHandPos[1])*.5
                
                posL = [ float( lHandPos[0]) / float( DEPTH_SIZE[0]), float( lHandPos[1]) / float( DEPTH_SIZE[1]), 0.]
                if( lHandPos[0] >= 0 and lHandPos[0] < DEPTH_SIZE[0] and lHandPos[1] >= 0 and lHandPos[1] < DEPTH_SIZE[1] and not disable_depth):
                        posL[2] = getDepth( depth_arr, lHandPos[0], lHandPos[1])*.5
                
                v = [0.,0.,0.]

                if( handPosR[0] - posR[0] >= 0.05):
                    v[0] += float( posR[0]) / glContent.dt

                if( handPosL[0] - posL[0] <= -0.05):
                    v[0] += float( -posL[0]) / glContent.dt

                if( handPosR[1] - posR[1] >= 0.05):
                    v[1] += float( posR[1]) / glContent.dt

                if( handPosL[1] - posL[1] <= -0.05):
                    v[1] += float( -posL[1]) / glContent.dt

                if( handPosR[2] - posR[2] >= 0.2):
                    v[2] += float( posR[2]) / glContent.dt * 3.0

                if( handPosL[2] - posL[2] >= 0.2):
                    v[2] += float( posL[2]) / glContent.dt * 3.0

                handPosR = posR
                handPosL = posL

                alpha = 200.
                beta = 8.
                slow_down = 0.5

                omega_neu = [ omega[0] + v[0]*0.3, 
                              omega[1] + v[1]*0.3, 
                              omega[2] + v[2]*0.3]

                omega = [ omega[0] + slow_down * glContent.dt * ( -alpha * math.sin( phi[2]) - beta * omega_neu[0]),
                          omega[1] + slow_down * glContent.dt * ( -alpha * math.sin( phi[1]) - beta * omega_neu[1]),
                          omega[2] + slow_down * glContent.dt * ( -alpha * math.sin( phi[0]) - beta * omega_neu[2])]


                phi = [ phi[0] + slow_down * glContent.dt * omega_neu[2],
                        phi[1] + slow_down * glContent.dt * omega_neu[1],
                        phi[2] + slow_down * glContent.dt * omega_neu[0]]

                glContent.phi = [ phi[0] / math.pi * 180.,
                                  phi[1] / math.pi * 180.,
                                  phi[2] / math.pi * 180.]

            glContent.t = t0

# recipe to get address of surface: http://archives.seul.org/pygame/users/Apr-2008/msg00218.html
if hasattr(ctypes.pythonapi, 'Py_InitModule4'):
   Py_ssize_t = ctypes.c_int
elif hasattr(ctypes.pythonapi, 'Py_InitModule4_64'):
   Py_ssize_t = ctypes.c_int64
else:
   raise TypeError("Cannot determine type of Py_ssize_t")
_PyObject_AsWriteBuffer = ctypes.pythonapi.PyObject_AsWriteBuffer
_PyObject_AsWriteBuffer.restype = ctypes.c_int
_PyObject_AsWriteBuffer.argtypes = [ctypes.py_object,
                                  ctypes.POINTER(ctypes.c_void_p),
                                  ctypes.POINTER(Py_ssize_t)]


# Skelett zeichnen
def draw_skeleton_data(pSkelton, index, positions, width = 4):
    global dispInfo, screen
    start = pSkelton.SkeletonPositions[positions[0]]
    for position in itertools.islice(positions, 1, None):
        next = pSkelton.SkeletonPositions[position.value]
        curstart = nui.SkeletonEngine.skeleton_to_depth_image(start, dispInfo.current_w, dispInfo.current_h) 
        curend = nui.SkeletonEngine.skeleton_to_depth_image(next, dispInfo.current_w, dispInfo.current_h)
        pygame.draw.line(screen, SKELETON_COLORS[index], curstart, curend, width)
        start = next

# Wandle Bildschirmkoordinaten um
def surface_to_array(surface):
   buffer_interface = surface.get_buffer()
   address = ctypes.c_void_p()
   size = Py_ssize_t()
   _PyObject_AsWriteBuffer(buffer_interface, ctypes.byref(address), ctypes.byref(size))
   bytes = (ctypes.c_byte * size.value).from_address(address.value)
   bytes.object = buffer_interface
   return bytes

# Callback-Funktion. Wird aufgerufen, wenn neuer Tiefenpuffer von Kinect verfuegbar ist
def depth_frame_ready(frame):
    global frame_ready, dispInfo, screen

    dispInfo = pygame.display.Info()

    # Im Thread ...
    with screen_lock:
        # Bildschirmkoordinaten uebersetzen
        address = surface_to_array(screen)
        ctypes.memmove( address, frame.image.bits, len(address))
        ctypes.memmove( depth_arr.ctypes.data, frame.image.bits, len(frame.image.bits))
        del address

        # Neu Kamerakoordinaten aus Kinect-Tiefenpuffer berechnen und in cam speichern
        updateCamFromKinectEvent()

# Callback-Funktion. Wird aufgerufen, wenn neuer Frame von Kinect verfuegbar ist
def post_frame(frame):
    try:
        # Skeletdaten an KINECTEVENT uebergeben
        pygame.event.post(pygame.event.Event(KINECTEVENT, skeletons = frame.SkeletonData))
    except:
        #print(" event queue full ")
        pass

#############################################################
#                                                           #
#   Main                                                    # 
#                                                           #
#############################################################

if __name__ == '__main__':

    for h in help:
        print h    

    # GLUT initialisieren
    initGLUT()

    # OpenGL initialisieren
    glContent.initGL()
    
    # Thread fuer die Kinect-Datenerfassung
    screen_lock = thread.allocate()
    
    # Initialisiere pyGame-Fenster fuer den Kinect Tiefenpuffer
    pygame.init()
    screen = pygame.display.set_mode(DEPTH_SIZE, 0, 16)
    pygame.display.iconify()

    # Soundeffekt laden
    pygame.mixer.init()

    try:
        # Initialisiere Kinect (NUI-Runtime)
        kinect = nui.Runtime()

        # Aktiviere Skeletdaten
        kinect.skeleton_engine.enabled = True

        # Weise Callbackfunction post_frame zu, die bei jedem neuen Frame aufgerufen wird
        kinect.skeleton_frame_ready += post_frame

        # Weise Callbackfunction depth_frame_ready zu, die aufgeruden wird wenn neues Bild des Tiefenpuffers verfuegbar ist
        kinect.depth_frame_ready += depth_frame_ready    

        # Starte Tiefenpuffer-Stream
        kinect.depth_stream.open(nui.ImageStreamType.Depth, nui.SkeletonFrameQuality.UpperBodySkeleton, nui.ImageResolution.Resolution640x480, nui.ImageType.Depth)
    except:
        # Fange exception falls Kinect nicht verfuegbar ist
        kinect_available = False,
        print "#  FEHLER: Kinect kann nicht gefunden bzw. die NUI-Runtime nicht initialisiert werden."
        pass

    # Starte die "GLUT-Schleife"
    glutMainLoop()