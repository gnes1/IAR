#!/usr/bin/env python
"""
Simple 2D robot simulator in Python+Pygame (tested in versions 2.6 and 2.7,
under Windows and Ubuntu).

Need to have Python and Pygame (http://www.pygame.org) installed for it to
work. Make sure Python and Pygame versions match, including the number of
bits, e.g. both 32 or both 64.
Launch from its own directory by typing
   python pyrobosim2d_v18.py
in the console/terminal window. May have to add Python dir. to path.

Expects two image files (back2_800_600.bmp, robo2.bmp) in the same directory.

Press ESC to exit, spacebar to teleoperate (arrows for steps in each direction,
'r' to rotate clockwise, 'e' counterclockwise), 'w' to perform a random walk with
random turns when bumping into walls/obstacles, 'a' for autonomous operation
(not implemented, it's the same as W right now), and 't' to toggle the trace
visibility.

The console/terminal window displays the distance (in pixels) and color (4 values:
RGB+transparency alpha) readings of each sensor.


Send your questions to agapie@tarleton.edu



    Copyright (C) 2013 Mircea Agapie 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.



Changes in version 19:
--Sensor data is written in the console window only for those sensors that are
seeing something; the others only display "> range"
--When spinning, the size of the robot image does not change anymore
--Several "robo" images are provided, with difefrent colors for the robot:
robo1.bmp (gray), robo2.bmp (blue), robo3.bmp (green), robo4.bmp (purple).
Change r_image to the one you like, and remember that this file must be in the
same dir. as the .py file (pyRoboSim_v19.py).
"""
from sys import argv
import os, pygame
from pygame.locals import *
import math
import random
import time
import CBGTC
import numpy as np
#import courbeEtResultat #Fichier qui fait les histogrammes
import ApprentissageTabRecompense


script, k, z, x1, y1, x2, y2, typ = argv

typAlgo    = typ
numTest    = int(k)
nbTest     = int(z)
xobstacle1 = int(x1)
yobstacle1 = int(y1)
xobstacle2 = int(x2)
yobstacle2 = int(y2)
if xobstacle1 == -1 :
    xobstacle1 = random.randint(100,900)
if yobstacle1 == -1 :
    yobstacle1 = random.randint(100,900)
if xobstacle2 == -1 :
    xobstacle2 = random.randint(100,900)
if yobstacle2 == -1 :
    yobstacle2 = random.randint(100,900)


fps                 = 20                    #at most  this many frames per second
back_image          = 'photo.jpg'           #must have this file in same dir.
display_cols        = 1000
display_rows        = 1000
wall_thickness      = 5                     #thickness in pixels
wall_color          = 'black'               #le mur
food_color          = 'green'               #energie
food2_color         = 'red'                 #energie_potentiel
trace_color         = 'blue'
trace_arc           = 10                    #in degrees, shows on both sides of r.azi
trace_decrease      = -17                   #negative, subtracts from robot size to make a smaller trace
trace_width         = 1
leave_trace         = 0                     #default mode is not to leave traces

color_of_nothing    = 'white'
sim_version         = 'RoboSim v.19'

r_image          = 'robo2.bmp'              #must have this file in same dir.
r_edge           = 51                       #edge of square surrounding robot (in pixels)
r_init_azi       = 0                        #azimuth, in degrees (up is 0)
r_init_x_topleft = random.randint(10,990)   #must be >= wall_thickness
r_init_y_topleft = random.randint(10,990)
r_step_tele      = 1                        #steps for teleop, equal in x or y (in pixels)
r_step_theta     = 7.5                      #step for teleop, in azimuth
r_init_fwd_speed = 5                        #pixels per simulation cycle
r_init_spin_speed= 3                        #degrees per simulation cycle
r_transparency   = 75                       #0 is totally transp., 255 totally opaque
r_visual_range_lasers  = display_cols/2
r_visual_range_camera  = 700                #measured from robot center
r_visual_angle   = 30                       #in degrees, must divide 90 exactly!
r_visual_granularity = 5                    #must be < wall_thickness for walls to be detected correctly!
r_visual_granularity_camera = 6


######################################################### Robot Variable ########################################################
r_energie = 1                               #initialisation energie
r_energiepotentiel = 1                      #initialisation energie potentiel
r_tab_inib = np.zeros(7)
r_onEpBLob = False                          #Variable pour dire que le robot est sur l'energie potentiel
r_onEBLob = False                           #Variable pour dire que le robot est sur l'energie 
r_seeEBLob = False                          #Variable pour dire que le robot voit l'energie
r_seeEpBLob = False                         #Variable pour dire que le robot voit l'energie potentiel
r_collision = False
r_tabresultat = []


main_dir = os.path.split(os.path.abspath(__file__))[0]
screen = pygame.display.set_mode((display_cols, display_rows))
list_traces = list()
listeAction = ["Rest", "Wander", "AvoidOstacle", "ApproachE", "ApproachEp", "ReloadOnE", "ReloadOnEp"]
tabRecompense = ApprentissageTabRecompense.getTabRecompense()

class Trace():
    def __init__(self, from_rect, start_angle, stop_angle):
        self.rect       = from_rect
        self.start_angle= start_angle
        self.stop_angle = stop_angle

class Obstacle(pygame.Rect):       #for now just colored rectangles
    def __init__(self, x_topleft, y_topleft, width, height, color):
        self.x_topleft  = x_topleft
        self.y_topleft  = y_topleft
        self.width      = width
        self.height     = height
        self.color      = pygame.Color(color)
      
#cette fonction permet de generer les ressources Ep et E
class ObstacleFood(pygame.Rect):       #for now just colored rectangles
    def __init__(self, x_topleft, y_topleft, width, height, color):
        self.x_topleft  = x_topleft
        self.y_topleft  = y_topleft
        self.width      = width
        self.height     = height
        self.color      = pygame.Color(color)
        
''' Changes alpha for surfaces with per-pixel alpha; only for small surfaces!
    Sets alpha for WHITE pixels to new_alpha.
    The alpha value is an integer from 0 to 255, 0 is fully transparent and
    255 is fully opaque. '''
def change_alpha_for_white(surface,new_alpha):
    size = surface.get_size()
    if size[0]>300 or size[1]>300:
        return surface
    for y in xrange(size[1]):
	for x in xrange(size[0]):
	    r,g,b,a = surface.get_at((x,y))
	    if r==255 and g==255 and b==255:
                surface.set_at((x,y),(r,g,b,new_alpha))
    return surface

''' Changes alpha for surfaces with per-pixel alpha; only for small surfaces!
    Sets alpha for pixels with alpha == 0 to new_alpha. It is needed b/c
    transform.smoothscale pads image with alpha=0. '''
def change_alpha_for_alpha(surface,new_alpha):
    size = surface.get_size()
    for y in xrange(size[1]):
	for x in xrange(size[0]):
	    r,g,b,a = surface.get_at((x,y))
	    if a<200:
                surface.set_at((x,y),(r,g,b,new_alpha))
    return surface

def draw_traces(target_surf):
    for t in list_traces:
        pygame.draw.arc(target_surf, pygame.Color(trace_color), t.rect,\
                        t.start_angle*math.pi/180, t.stop_angle*math.pi/180, trace_width)


def load_image(name):
    path = os.path.join(main_dir, name)
    temp_image = pygame.image.load(path).convert_alpha()  #need this if using ppalpha
    return change_alpha_for_white(temp_image, r_transparency)  
################################################## Instancier les obstacles ##############################################

#Create list of obstacles (walls+others)
#First 2 args are x and y of top-left corner, next two width and height, next color
list_obstacles = []
list_obstaclescamera = []
w01 = Obstacle(0,0,display_cols,wall_thickness, wall_color)                          #top wall
list_obstacles.append(w01)
w02 = Obstacle(display_cols-wall_thickness,0,wall_thickness,display_rows,wall_color) #right wall
list_obstacles.append(w02)
w03 = Obstacle(0,display_rows-wall_thickness,display_cols,wall_thickness,wall_color) #bottom wall
list_obstacles.append(w03)
w04 = Obstacle(0,0,wall_thickness,display_rows, wall_color)                          #left wall
list_obstacles.append(w04)
f01 = ObstacleFood(xobstacle1, yobstacle1,50,50,food_color)                          #Ressource E
list_obstaclescamera.append(f01)
f02 = ObstacleFood(xobstacle2, yobstacle2,50,50,food2_color)                         #Ressource Ep
list_obstaclescamera.append(f02)


#for collision-checking (right now only in Robot.move()), only the rectangles are needed.
#so for speed a stripped-down list of rectangles is built:
list_rect_obstacles = []
for ob in list_obstacles:
    list_rect_obstacles.append(pygame.Rect(ob.x_topleft,ob.y_topleft,ob.width,ob.height))

list_rect_obstacles_camera = []
for ob in list_obstaclescamera:
    list_rect_obstacles_camera.append(pygame.Rect(ob.x_topleft,ob.y_topleft,ob.width,ob.height))
    

########################################## Class Robot ########################################################################
class Robot(pygame.sprite.Sprite):
    def __init__(self, image, x_topleft, y_topleft, azimuth, fwd_speed, spin_speed,\
                 r_visual_range_lasers,r_visual_range_camera, visual_angle,r_E,r_Ep,r_seeEBLob,r_seeEpBLob,r_onEBLob,r_onEpBLob,r_collision,r_tab_inib,r_tabRecompense):
        pygame.sprite.Sprite.__init__(self) #call Sprite initializer
        
        #Sprites must have an image and a rectangle
        self.image          = image
        self.image_original = self.image                                                             #unchanging copy, for rotations
        self.rect           = image.get_rect()
        self.rect_original  = self.rect                                                              #unchanging copy, for rotations
        self.rect.topleft   = x_topleft, y_topleft                                                   #for now used only for initial position
        self.fwd_speed      = fwd_speed
        self.spin_speed     = spin_speed
        self.azi            = azimuth                                                                #in degrees
        self.collided       = False
        self.opmode         = 0                                                                      #0=tele, 1=(random)walk 2=auto
        self.spin_angle_left= 0                                                                      #relative angle left to spin
        
        #these are the parameters of the range-sensing system
        self.visual_range_lasers   = r_visual_range_lasers                                           #Range laser
        self.visual_range_camera   = r_visual_range_camera                                           #Range camera
        self.visual_angle   = visual_angle
        self.nr_cameras     = 120*90/180+1                                                           # 1 camera de 60 degres compose de 61 lasers detectant uniquement la couleur des ressources
        self.nr_sensors     = 16                                                                     # 16 lasers
        self.nr_sensors_ba  = 0
        self.retina         = list([self.visual_range_lasers, pygame.Color(color_of_nothing)]\
                                   for i in range(self.nr_sensors))
        self.retinaBack     = list([self.visual_range_lasers, pygame.Color(color_of_nothing)]\
                                   for i in range(self.nr_sensors_ba))
        self.retinaCamera   = list([self.visual_range_camera, pygame.Color(color_of_nothing)]\
                                  for i in range(self.nr_cameras))
        self.energie = r_E                                                                           #energie
        self.energiepotentiel = r_Ep                                                                 #energie potentiel
        self.seeEBlob = r_seeEBLob                                                                   #inialisation de la variable seeEBlob
        self.seeEpBlob = r_seeEpBLob                                                                 #inialisation de la variable seeEpBlob
        self.onEBlob = r_onEBLob                                                                     #inialisation de la variable onEBlob
        self.onEpBlob = r_onEpBLob                                                                   #inialisation de la variable onEpBlob
        self.collision = r_collision                                                                 #gerer les collisions
        self.tabinib = r_tab_inib                                                                    #tableau des inhibitions
        self.CBGT  = CBGTC.CBGTC()                                                                   #instancier CBGTC
        self.tabSalience = []                                                                        #tab de salience
        self.tabresultat = r_tabresultat                                                             #tableau qui sauvegarde les donnees a chaque seconde
        self.tabRecompense = r_tabRecompense
        
    #Affiche les resultats des lasers     
    def printRetinaBack(self):
        """Prints the content of the retina list"""
        for s in self.retinaBack:
            if (s[0] == self.visual_range_lasers):                                                   #this really means >=, since sense() func. caps distances
                print 'ici'+'>'+str(self.visual_range_lasers)
            else: #obstacle detected
                print s
        print '\n'
        
    #Affiche l'energie 
    def printEnergie(self):
        print "energie" + str(self.energie)
   
                
    #Affiche les resultats des cameras  
    def printRetina(self):
        """Prints the content of the retina list"""
        for s in self.retina:
            if (s[0] == self.visual_range_lasers):                                                   #this really means >=, since sense() func. caps distances
                print '>'+str(self.visual_range_lasers)
            else: #obstacle detected
                print s
        print '\n'
        
    #Affiche l'energie potentiel
    def printEnergiePotentiel(self):
        print "energiepotentiel"+ str(self.energiepotentiel)

    #Permet de traiter les donnees de la camera et de mettre a jour les variables et les tableaux de donnnees.
    def senseCamera(self):
        n = (self.nr_cameras - 1)/2                                                                  #the "natural" sensor range is -n to +n
        granu = r_visual_granularity_camera                                                          #must be at least as large as the wall thickness!!
        for i in range(-n,n+1):                                                                      #sense with each of the 2n+1 range sensors
            ang = (self.azi - i*self.visual_angle)*math.pi/180
            for distance in range(granu,  self.visual_range_camera+granu, granu):
                x = self.rect.center[0]-distance*math.sin(ang)                                       #endpoint coordinates
                y = self.rect.center[1]-distance*math.cos(ang)
                nr_collisions = 0
                count = -1                                                                           #needed to coordinate the two lists, to extract color after loop
                for ob in list_rect_obstacles_camera:                                                #use the stripped-down list of rectangles for speed
                    count = count + 1
                    if ob.collidepoint(x,y):
                        nr_collisions = 1
                        break                                                                        #breaks out of wall loop
                if nr_collisions:                                                                    #non-zero collision
                    break                                                                            #breaks out of distance loop
            #distance now has the min. between the visual range and the first collision
            self.retinaCamera[i+n][0] = distance
            
            if nr_collisions:                                                                        #nr_collisions is 1 if a collision has occurred
                self.retinaCamera[i+n][1] = list_obstaclescamera[count].color                        #color comes form the larger list
            else:
                self.retinaCamera[i+n][1] = pygame.Color(color_of_nothing)

    #Permet de generer les lasers
    def draw_rays(self, target_surf):
        n = (self.nr_sensors -1 )/2                                                                  #the "natural" sensor range -n to +n
        for i in range(-n,n):                                                                        #draw the 2n+1 rays of the range sensors
            ang = (self.azi - i*self.visual_angle)*math.pi/180
            x = self.rect.center[0]-self.retina[i+n][0]*math.sin(ang)
            y = self.rect.center[1]-self.retina[i+n][0]*math.cos(ang)
            #use aaline for smoother (but slower) lines
            pygame.draw.line(target_surf, (0,255,0), self.rect.center, (x,y))

    #Permet de generer les lasers composant la camera
    def draw_rays_back(self, target_surf):
           n = (self.nr_sensors_ba - 1)/2                                                            #the "natural" sensor range -n to +n
           for i in range(-n,n+1):                                                                   #draw the 2n+1 rays of the range sensors
                ang = (self.azi - i*self.visual_angle)*math.pi/180
                x = self.rect.center[0]-self.retinaBack[i+n][0]*math.sin(ang)+5
                y = self.rect.center[1]-self.retinaBack[i+n][0]*math.cos(ang)+5
                #use aaline for smoother (but slower) lines
                pygame.draw.line(target_surf, (255,0,0), self.rect.center, (x,y))
                
    #Permet de generer les fichiers de sortie       
    def ecrireFichierResultat(self,filename):
        fichier = open(filename, "w")
        for i in self.tabresultat:
            fichier.write(str(i[0])+";"+str(i[1])+";"+str(i[2])+";"+str(i[3])+"\n")
        fichier.close()     
            
    def draw_camera(self, target_surf):
            n = (self.nr_cameras - 1)/2                                                             #the "natural" sensor range -n to +n
            for i in range(-n,n+1):                                                                 #draw the 2n+1 rays of the range sensors
                ang = (self.azi - i)*math.pi/180
                x = self.rect.center[0]-self.retinaCamera[i+n][0]*math.sin(ang)
                y = self.rect.center[1]-self.retinaCamera[i+n][0]*math.cos(ang)
                #use aaline for smoother (but slower) lines
                pygame.draw.line(target_surf, (0,0,255), self.rect.center, (x,y))
               
############################################################ Le comportement du robot #################################################################

    #Permet de charger l'energie grace a l'energie potentiel    
    def ReloadOnE(self,Temps):
            Temps = time.clock()
            if self.onEBlob:                                                                        #Verifier que le robot est sur une ressource E
                if self.energiepotentiel >= 0.2 and self.energie <= 0.8:
                    self.energie += 0.02
                    self.energiepotentiel -=0.02
                    if self.energiepotentiel < 0 :
                        self.energiepotentiel = 0           
            return Temps
        
    #Permet de charger l'energie potentiel  
    def ReloadOnEp(self,Temps):
           Temps = time.clock()
           if self.onEpBlob:                                                                        #Verifier que le robot est sur une ressource Ep
               if self.energiepotentiel <= 0.8:
                    self.energiepotentiel +=0.02           
           return Temps
        
    #Permet de s'approcher des ressources energies ou energies potentiel 
    #Cette fonction permet de verifier la distance minimum de detection par la camera pour se retourner dans le bon sens.
    def ApprochAE(self,Temps):
            if self.collided:                                                                       #collision in prev. cycle --> start SPIN
                self.collided = False
                walk_dazi = random.randint(-180, 180)
                if math.fabs(walk_dazi) <= self.spin_speed:                                         #can spin in 1 cycle
                    self.spin(walk_dazi)
                else:
                    if walk_dazi > 0:                                                               #calculate the angle's sign
                        sign = 1
                    else:
                        sign = -1
                    self.spin(sign*self.spin_speed)
                    self.spin_angle_left = walk_dazi-sign*self.spin_speed                           #not collided --> finish SPIN, or MOVE fwd
            xcentreRobot = self.rect.center[0]
            ycentreRobot = self.rect.center[1]
            i = 0
            mini = 700
            imin = -1
            
            #Permet de recuperer le laser de camera qui a detecte la ressource ayant la plus petite distance par rapport aux autres lasers de la camera
            for s in self.retinaCamera:
                i =  i + 1
                if s[0] < mini:
                    imin = i
                    mini = s[0]
            walk_dazi = 0
            
            #On calcule le cos par rapport au rayon du milieu pour savoir le bon angle
            result = mini/self.retinaCamera[31][0]
            if result >= -1 and result <= 1:
                walk_dazi = math.acos(mini/self.retinaCamera[31][0])
            
            #Si le resultat ne permet pas d'etre dans la plage des valeurs -1 et 1
            #Si le laser qui a detecte a un id > 50
            if imin > 50 :
                    walk_dazi = 30
                    
            #Si le laser qui a detecte a un id >= 10 et id imin < 30
            if imin >= 10 and imin < 30 :
                     walk_dazi = -30
                     
            #Apres avoir definir l angle \walk dazi\ on tourne et on avance un peu
            self.spin(walk_dazi) 
            temp_unghi = self.azi*math.pi/180
            walk_dx = -(self.fwd_speed)*math.sin(temp_unghi)
            walk_dy = -(self.fwd_speed)*math.cos(temp_unghi)
            self.move(walk_dx*4, walk_dy*4)
        
            return Temps
            
    #Cette fonction diminue l'energie 0.01
    def DiminuerEnergie (self):
        self.energie -= 0.01
        if self.energie < 0:
            self.energie = 0
        

    #Cette focntion stoppe les mouvements du robot et diminue son energie de 0.005      
    def Rest(self,Temps):
           Temps = time.clock()    
           if  self.energie >= 0.005:
               self.energie -= 0.005
               
           
           return Temps

    #Cette fonction est le comportement d'evitement les obstacles
    def AvoidObstacles (self,Temps):
        Temps = time.clock()
        n = (self.nr_sensors - 1)/2                                                                  #the "natural" sensor range is -n to +n
        granu = r_visual_granularity                                                                 #must be at least as large as the wall thickness!!
        for i in range(-n,n+1):                                                                      #sense with each of the 2n+1 range sensors
            ang = (self.azi - i*self.visual_angle)*math.pi/180
            for distance in range(granu, self.visual_range_lasers + granu, granu):
                x = self.rect.center[0]-distance*math.sin(ang)                                       #endpoint coordinates
                y = self.rect.center[1]-distance*math.cos(ang)
                nr_collisions = 0
                count = -1                                                                           #needed to coordinate the two lists, to extract color after loop
                for ob in list_rect_obstacles:                                                       #use the stripped-down list of rectangles for speed
                    count = count + 1
                    if ob.collidepoint(x,y):
                        nr_collisions = 1
                        break                                                                        #breaks out of wall loop
                if nr_collisions:                                                                    #non-zero collision
                    break                                                                            #breaks out of distance loop
            #distance now has the min. between the visual range and the first collision
            self.retina[i+n][0] = distance
            if nr_collisions:                                                                        #nr_collisions is 1 if a collision has occurred
                self.retina[i+n][1] = list_obstacles[count].color                                    #color comes form the larger list
            else:
                self.retina[i+n][1] = pygame.Color(color_of_nothing)
        avoid = False
        for s in self.retina:
            if (s[0] != self.visual_range_lasers) : 
                avoid = False
                #Verifie que le mur est a une distance de maximum 39 metres
                if s[0] < 40 and avoid == False:
                    avoid = True
                if avoid == True :   
                    walk_dazi = random.randint(-180, 180)
                    if math.fabs(walk_dazi) <= self.spin_speed:                                      #can spin in 1 cycle
                        self.spin(walk_dazi)
                    else:
                        if walk_dazi > 0:                                                            #calculate the angle's sign
                            sign = 1
                        else:
                            sign = -1
                            self.spin(sign*self.spin_speed)
                            self.spin_angle_left = walk_dazi-sign*self.spin_speed
                    temp_unghi = self.azi*math.pi/180
                    walk_dx = -self.fwd_speed*math.sin(temp_unghi)
                    walk_dy = -self.fwd_speed*math.cos(temp_unghi)
                    self.move(walk_dx, walk_dy)
        temp_unghi = self.azi*math.pi/180
        walk_dx = -self.fwd_speed*math.sin(temp_unghi)
        walk_dy = -self.fwd_speed*math.cos(temp_unghi)
        self.move(walk_dx, walk_dy)            
        return Temps
    
    #Affiche la distances et la couleur de chaque objet detecte par chaque laser de la camera
    def printRetinaCamera(self):
        """Prints the content of the retina list"""
        for s in self.retinaCamera:
            if s[1] != (255,255,255,255):
                print s[1] 
        print '\n'

    #Cette fonction fait des actions aleatoires
    def wander(self,Temps):
        Temps = time.clock()    
        i = 0
        i = random.randint(1,20)
        fwd_speed = random.randint(3,5)                                                              #Choix aleatoire du speed
        temp_unghi = self.azi*math.pi/180 
        walk_dx = -fwd_speed*math.sin(temp_unghi)
        walk_dy = -fwd_speed*math.cos(temp_unghi)
        self.move(walk_dx, walk_dy)
        return Temps

    #Cette fonction permet juste d'executer le move
    def move(self,dx,dy):
        previous_rect = self.rect                                                                    #remember in case undo is necessary
        self.rect = self.rect.move(dx,dy)
        if self.rect.collidelist(list_rect_obstacles) != -1:                                         #if collision exists
            self.rect.collidelistall(list_rect_obstacles)
            self.rect = previous_rect                   
            self.collided = True
        else:#if there was no collision
            if leave_trace:     
                tr = self.rect.inflate(trace_decrease, trace_decrease)
                list_traces.append(Trace(tr, 90+self.azi-trace_arc, 90+self.azi+trace_arc))
    
    #Cette fonction permet de faire tourner le robot
    def spin(self,dtheta):
        center = self.rect.center
        self.azi += dtheta
        if self.azi >= 360:                                                                          #keep theta between -360..360
            self.azi = self.azi-360
        if self.azi <= -360:
          self.azi = self.azi+360
        original_rect = self.image_original.get_rect()
        rotated_image = pygame.transform.rotate(self.image_original, self.azi)
        rotated_rect  = original_rect.copy()
        rotated_rect.center = rotated_image.get_rect().center
        self.image = rotated_image.subsurface(rotated_rect).copy()
        self.image = change_alpha_for_alpha(self.image, r_transparency)
        if leave_trace:     
            tr = self.rect.inflate(trace_decrease, trace_decrease)
            list_traces.append(Trace(tr, 90+self.azi-trace_arc, 90+self.azi+trace))
    
    #this function's job is to place in self.retina the range sensed by each sensor
    def sense(self):
        n = (self.nr_sensors - 1)/2                                                                  #the "natural" sensor range is -n to +n
        granu = r_visual_granularity                                                                 #must be at least as large as the wall thickness!!
        for i in range(-n,n):                                                                        #sense with each of the 2n+1 range sensors
            ang = (self.azi - i*self.visual_angle)*math.pi/180
            for distance in range(granu,self.visual_range_lasers+granu, granu):
                x = self.rect.center[0]-distance*math.sin(ang)                                       #endpoint coordinates
                y = self.rect.center[1]-distance*math.cos(ang)
                nr_collisions = 0
                count = -1                                                                           #needed to coordinate the two lists, to extract color after loop
                for ob in list_rect_obstacles:                                                       #use the stripped-down list of rectangles for speed
                    count = count + 1
                    if ob.collidepoint(x,y):
                        nr_collisions = 1
                        break                                                                        #breaks out of wall loop
                if nr_collisions:                                                                    #non-zero collision
                    break                                                                            #breaks out of distance loop
            #distance now has the min. between the visual range and the first collision
            self.retina[i+n][0] = distance
            if nr_collisions:                                                                        #nr_collisions is 1 if a collision has occurred
                self.retina[i+n][1] = list_obstacles[count].color                                    #color comes form the larger list
            else:
                self.retina[i+n][1] = pygame.Color(color_of_nothing)


##################################################################### Calculer CBGTC #####################################################################################
    # Calcule de f(x)
    def CalculFx(self,x):
        return (2 / ( 1 + math.exp ( - 4 * x ))) - 1

    # Salience du comportement Wander
    def Sw(self): 
        return 380

    # Salience du comportement Rest
    def SSl(self): 
        return 550 * self.CalculFx ( 2 * max(self.energiepotentiel * self.energie - 0.5 , 0 ))

    # Salience du comportement de ReloadOnE
    def ROE (self,xFCROE): 
        value = 0
        if self.onEBlob :
            value = 1  
        return 950* self.CalculFx ( 4 * value * self.energiepotentiel*(1 - self.energie)) +0.6 * xFCROE

    # Salience du comportement ReloadOnEp
    def ROEp (self,xFCROEp): 
        value = 0
        if self.onEpBlob :
            value = 1  
        return 750* self.CalculFx ( 4 * value *(1 - self.energiepotentiel)) +0.2 * xFCROEp

    # Salience du comportement AvoidOstacles
    def SAO (self,xFCAO,SFL,SFR): 
        return 950* self.CalculFx ( 2 * ( max (1.5 - SFL ,0) + max (1.5 - SFR, 0))) + 0.2 * xFCAO

    # Salience du comportement de Approach la ressource en energie
    def SAE (self, xFCAE): 
        value = 0
        value1 = 0
        if self.seeEBlob :
            value = 1
        if self.onEBlob :
            value1 = 1
        return 750* self.CalculFx (( value * self.energiepotentiel * ( 1 - self.energie )) * ( 1 - value1)) + 0.2 * xFCAE

    # Salience du comportement de Approch la ressource en energie potentielle
    def SAEp (self, xFCAEp): 
        value = 0
        value1 = 0
        if self.seeEpBlob :
            value = 1
        if self.onEpBlob :
            value1 = 1

        return 750* self.CalculFx (( value * (1 - self.energiepotentiel)) * ( 1 - value1 ))  + 0.2 * xFCAEp

    
    # Cette fonction permet de generer le tableau des saliences 
    def tabsalience(self):
        xFCROE = self.tabinib[5]                                                                            # Definir le x pour ReloadOnE
        xFCROEp = self.tabinib[6]                                                                           # Definir le x pour ReloadOnEp
        xFCAO = self.tabinib[2]                                                                             # Definir le x pour AvoidObstcles
        i = 0
        SFL = 0
        SFR = 0
        for s in self.retina:
            if i == 6 :                                                                                     # Recuperer le SFL
                SFL = (int) (s[0]/100)                                                                      # Definir SFL
            
            if i == 0:                                                                                      # Recuperer le SFR
                SFR = (int) (s[0] /100)                                                                     # Definir SFR
            i = i + 1
        xFCAE = self.tabinib[3]                                                                             # Definir le x pour approachE
        xFCAEp = self.tabinib[4]                                                                            # Definir le x pour approachaEP
        tab = []
        tab.append(self.SSl())
        tab.append(self.Sw())
        tab.append(self.SAO(xFCAO,SFL,SFR))
        tab.append(self.SAE(xFCAE))
        tab.append(self.SAEp(xFCAEp))
        tab.append(self.ROE(xFCROE)) 
        tab.append(self.ROEp(xFCROEp))
        return np.array(tab)


    def CBGModel(self,Temps):
        Temps = time.clock()
        inhibs = []
        MinInhib = 0
        t = 0
        salience = self.tabsalience()                                                                       # Recuperer le nouveau tableau de salience
        self.tabSalience = salience                                                                         # Mettre a jour le tableau de salience du tour de boucle precedente
        inhibs= self.CBGT.nbStepsCompute(0.001,100,self.tabSalience,'v') 
        if t == 0 :
            sali = np.zeros(7)
            inhibs2= self.CBGT.nbStepsCompute(0.001,100,sali,'v') 
            MinInhib = inhibs2[0]                                                                           # Recuperer le YGpi Rest
            t = 1
        Tab1 = []
        for z in range(len(inhibs)):
            if inhibs[z] <= MinInhib:                                                                       # Recuperer le output des inhibitions et le comparer a YGpi Rest
                Tab1.append(z)
        if len(Tab1) == 1:                                                                                  # Si un seul desinhiber le choisir
            i = Tab1[0]
        if len(Tab1) == 2 and Tab1[0] == 0 :                                                                # Si une action est desinhibe au meme niveau que le rest
            i = Tab1[1]                                                                                     # Alors on choisit l'action a la place de l'action rest
        else:
            tabdifference = []
            tabdifferencecombien = []
            for z in range(len(inhibs)):                                                                    #Verifie le niveau de desinhibition par rapport au tableau des inhibitions
                    tabdifference.append(self.tabinib[z] - inhibs[z])
            woho = np.argmax(tabdifference)
            
            for y in range(len(tabdifference)):
                if woho >= 0 and tabdifference[y] == tabdifference[woho] :
                    tabdifferencecombien.append(y)

           #Si un seul est desinhibe
            if len(tabdifferencecombien) == 1 :
                i = tabdifferencecombien[0]
            else: #Si plusieurs qui sont desinhibes "au meme niveau" alors on utilise l'equation 17 pour choisir l'action
                tabE = []
                for u in range(len(inhibs)):
                    if u != 0 :
                        tabE.append(1-self.CBGT.BG.GPi[u]/MinInhib)
                Choix = []
                
                maxi = np.argmax(tabE)                                                                      #Prendre le max de YiGPi
                maxi = tabE[maxi]
                for z in range(len(tabE)):                                                                  #Si il en existe un, alors on le prend
                    if tabE[z] == maxi:
                        Choix.append(z)
                if len(Choix) == 1:
                    i = Choix[0] + 1
                else:                                                                                       #Si il y en a plusieurs alors on choisit l'action en random
                    csisi = random.randint(0,len(Choix))
                    csisi = csisi - 1
                    i = Choix[csisi]  + 1

        if i == 0 :
            action = "Rest"
        elif i  ==  1 :
            action = "Wander"
        elif i  ==  2 :
            action = "AvoidOstacle"
        elif i  ==  3 :
            action = "ApproachE"
        elif i  ==  4 :
            action = "ApproachEp"
        elif i  ==  5 :
            action = "ReloadOnE"
        else :
            action = "ReloadOnEp"
            
        self.tabinib = inhibs                                                                             
        return Temps,action
        
############################################################################# IfElseModel ########################################################################
    def IfElseModel(self,Temps):
        Temps = time.clock()
        action = "" 
        if self.energiepotentiel < 1 and self.onEpBlob  :
            action = "ReloadOnEp"
        elif self.energie < 1 and self.energiepotentiel > 0 and self.onEBlob :
            action = "ReloadOnE"
        elif self.energie < 0.8 and self.energiepotentiel > 0 and self.seeEBlob  :
            action = "ApproachE"
        elif self.energiepotentiel < 0.8 and self.seeEpBlob  :
            action = "ApproachEp"
        elif self.energie > 0.7 and self.energiepotentiel > 0.7 :
            action = "Rest"
        elif self.collision  :
            action = "AvoidOstacle"
        else:
            action = "Wander"
        return Temps,action          
        
############################################################################# CBGAmeliore ########################################################################   
    
    
    
    
            

    def SwAmeliorer(self, actionPrec): #Salience Wander
        return 380 + self.tabRecompense[actionPrec]["Wander"]

    def SSlAmeliorer(self, actionPrec): #Salience Rest
        return (550 * self.CalculFx ( 2 * max(self.energiepotentiel * self.energie - 0.5 , 0 ))) + self.tabRecompense[actionPrec]["Rest"]

    def ROEAmeliorer (self,xFCROE, actionPrec): #Salience ReloadOnE
        value = 0
        if self.onEBlob :
            value = 1
        #print "danssalienceReloadonE"  + str(self.onEBlob)    
        return (950* self.CalculFx ( 4 * value * self.energiepotentiel*(1 - self.energie)) +0.6 * xFCROE )+ self.tabRecompense[actionPrec]["ReloadOnE"]

    def ROEpAmeliorer (self,xFCROEp, actionPrec): #Salience ReloadOnEp
        value = 0
        if self.onEpBlob :
            value = 1  
        return (750* self.CalculFx ( 4 * value *(1 - self.energiepotentiel)) +0.2 * xFCROEp) + self.tabRecompense[actionPrec]["ReloadOnEp"]

    def SAOAmeliorer (self,xFCAO,SFL,SFR, actionPrec): #Salience Avoid Obstacles
        #SFL = SFL / 100
        #SFR = SFR / 100
        return (950* self.CalculFx ( 2 * ( max (1.5 - SFL ,0) + max (1.5 - SFR, 0))) + 0.2 * xFCAO) +  self.tabRecompense[actionPrec]["AvoidOstacle"]

    def SAEAmeliorer (self, xFCAE, actionPrec): #Salience Approach ressource E
        value = 0
        value1 = 0
        if self.seeEBlob :
            value = 1
        if self.onEBlob :
            value1 = 1
        return (750* self.CalculFx (( value * self.energiepotentiel * ( 1 - self.energie )) * ( 1 - value1)) + 0.2 * xFCAE) + self.tabRecompense[actionPrec]["ApproachE"]

    def SAEpAmeliorer (self, xFCAEp, actionPrec): #Salience Approach ressource Ep
        value = 0
        value1 = 0
        if self.seeEpBlob :
            value = 1
        if self.onEpBlob :
            value1 = 1
        return (750* self.CalculFx (( value * (1 - self.energiepotentiel)) * ( 1 - value1 ))  + 0.2 * xFCAEp) + self.tabRecompense[actionPrec]["ApproachEp"]


    def tabsalienceAmeliorer(self, actionPrec):
        xFCROE = self.tabinib[5]  #definir le x pour ReloadOnE
        xFCROEp = self.tabinib[6] #definir le x pour ReloadOnEp
        xFCAO = self.tabinib[2] #definir le x pour AvoidObstcles
        i = 0
        SFL = 0
        SFR = 0
        for s in self.retina:
            if i == 6 :
                SFL = (int) (s[0]/100) #definir SFL
            
            if i == 0:
                SFR = (int) (s[0] /100)#definir SFR
            i = i + 1
        xFCAE = self.tabinib[3] #definir le x pour approachE
        xFCAEp = self.tabinib[4] #definir le x pour approachaEP
        tab = []
        tab.append(self.SSlAmeliorer(actionPrec))
        tab.append(self.SwAmeliorer(actionPrec))
        tab.append(self.SAOAmeliorer(xFCAO,SFL,SFR,actionPrec))
        tab.append(self.SAEAmeliorer(xFCAE,actionPrec))
        tab.append(self.SAEpAmeliorer(xFCAEp,actionPrec))
        tab.append(self.ROEAmeliorer(xFCROE,actionPrec))
        tab.append(self.ROEpAmeliorer(xFCROEp,actionPrec))
        return np.array(tab)
   

    def CBGModelAmeliore(self,Temps,actionpred):
        Temps = time.clock()
        inhibs = []
        MinInhib = 0
        t = 0
        salience = self.tabsalienceAmeliorer(actionpred)                                                    # Recuperer le nouveau tableau de salience
        self.tabSalience = salience                                                                         # Mettre a jour le tableau de salience du tour de boucle precedente
        inhibs= self.CBGT.nbStepsCompute(0.001,100,self.tabSalience,'v') 
        if t == 0 :
            sali = np.zeros(7)
            inhibs2= self.CBGT.nbStepsCompute(0.001,100,sali,'v') 
            MinInhib = inhibs2[0]                                                                           # Recuperer le YGpi Rest
            t = 1
        Tab1 = []
        for z in range(len(inhibs)):
            if inhibs[z] <= MinInhib:                                                                       # Recuperer le output des inhibitions et le comparer a YGpi Rest
                Tab1.append(z)
        if len(Tab1) == 1:                                                                                  # Si un seul desinhiber le choisir
            i = Tab1[0]
        if len(Tab1) == 2 and Tab1[0] == 0 :                                                                # Si une action est desinhibe au meme niveau que le rest
            i = Tab1[1]                                                                                     # Alors on choisit l'action a la place de l'action rest
        else:
            tabdifference = []
            tabdifferencecombien = []
            for z in range(len(inhibs)):                                                                    #Verifie le niveau de desinhibition par rapport au tableau des inhibitions
                    tabdifference.append(self.tabinib[z] - inhibs[z])
            woho = np.argmax(tabdifference)
            
            for y in range(len(tabdifference)):
                if woho >= 0 and tabdifference[y] == tabdifference[woho] :
                    tabdifferencecombien.append(y)

           #Si un seul est desinhibe
            if len(tabdifferencecombien) == 1 :
                i = tabdifferencecombien[0]
            else: #Si plusieurs qui sont desinhibes "au meme niveau" alors on utilise l'equation 17 pour choisir l'action
                tabE = []
                for u in range(len(inhibs)):
                    if u != 0 :
                        tabE.append(1-self.CBGT.BG.GPi[u]/MinInhib)
                Choix = []
                
                maxi = np.argmax(tabE)                                                                      #Prendre le max de YiGPi
                maxi = tabE[maxi]
                for z in range(len(tabE)):                                                                  #Si il en existe un, alors on le prend
                    if tabE[z] == maxi:
                        Choix.append(z)
                if len(Choix) == 1:
                    i = Choix[0] + 1
                else:                                                                                       #Si il y en a plusieurs alors on choisit l'action en random
                    csisi = random.randint(0,len(Choix))
                    csisi = csisi - 1
                    i = Choix[csisi]  + 1

        if i == 0 :
            action = "Rest"
        elif i  ==  1 :
            action = "Wander"
        elif i  ==  2 :
            action = "AvoidOstacle"
        elif i  ==  3 :
            action = "ApproachE"
        elif i  ==  4 :
            action = "ApproachEp"
        elif i  ==  5 :
            action = "ReloadOnE"
        else :
            action = "ReloadOnEp"
            
        self.tabinib = inhibs                                                                             
        return Temps,action
  
############################################################################# Main ###########################p#####################################################
def main():
        iteration = 0
        TempsMoyen = []
        while iteration < nbTest :
            tabAx = []
            global leave_trace, list_traces
            pygame.display.set_caption(sim_version + str(typAlgo))
            r_sprite = load_image(r_image)
            background  = load_image(back_image)
            ancienneValeur = time.clock()
            
            #prepare simulation objects
            clock = pygame.time.Clock()
            r = Robot(r_sprite, r_init_x_topleft, r_init_y_topleft,r_init_azi, r_init_fwd_speed,\
                      r_init_spin_speed,r_visual_range_lasers,r_visual_range_camera, r_visual_angle,r_energie,r_energiepotentiel,r_seeEBLob,r_seeEpBLob,r_onEBLob,r_onEpBLob,r_collision,r_tab_inib,tabRecompense)
            allsprites = pygame.sprite.RenderPlain((r))
            r.tabresultat = []
            
            #display the environment once, right before event loop
            screen.blit(background, (0, 0))
            count = -1
            for ob in list_obstacles:
                count = count + 1
                s = pygame.display.get_surface()
                s.fill(ob.color, list_rect_obstacles[count])
            count = -1    
            for ob in list_obstaclescamera:
                count = count + 1
                s = pygame.display.get_surface()
                s.fill(ob.color, list_rect_obstacles_camera[count])      
            r.draw_rays(screen)  
            r.draw_camera(screen)
            r.draw_rays_back(screen)
            pygame.display.flip()
            MoyenneTemps = []
            TempsTab = []
            going = True
            action = ""
            r.tabSalience =np.zeros(7);                                                                     # Creer un tableau de salience vide
            while going and r.energie > 0:                                                                  # Tester qu'il reste de l'energie               
                tabAx = []
                clock.tick(fps)
                Temps = time.clock()
                val = Temps + 1
                while Temps < val :
                    for s in r.retinaCamera:
                        #remettre a jour les donnees des cameras
                        r.onEpBlob = False 
                        r.onEBlob = False
                        r.seeEpBlob = False
                        r.seeEBlob = False
                        
                        if  s[0] < 80 and  s[1] == (0,255,0,255) :
                            r.seeEBlob = True
                        if  s[0] < 80 and s[1] == (255,0,0,255) :
                            r.seeEpBlob = True
                        if  s[0] <= 50 and s[1] == (0,255,0,255) :
                            r.onEBlob = True
                        if  s[0] <= 50 and s[1] == (255,0,0,255) :
                            r.onEpBlob = True
                        if  r.seeEBlob  or r.seeEpBlob  or r.onEBlob  or r.onEpBlob :
                            break
                        if s[0] > 45 :
                            r.onEpBlob = False
                            r.onEBlob = False
                            r.seeEpBlob = False
                            r.seeEBlob = False
                    
                    r.sense()
                    r.senseCamera()
                    
                    for s in r.retina:                                                                      # Detecter qu on a une colision
                        if s[0] <= 50:
                            r.collision = True
                            break
                        if s[0] > 50:
                            r.collision = False
                    
                    Temps  = 0
                    action = "Wander"
                    # Choix de l'action
                    if  typAlgo == "ITE":
                        Temps,action = r.IfElseModel(Temps)                                                # Lancer le model ifelse
                    elif typAlgo == "CBG":
                        Temps,action = r.CBGModel(Temps)                                                   # Lancer le model CBGTC
                    elif typAlgo == "CBGAmeliorer" or typAlgo == "CGBAmeliorer":
                        Temps,action = r.CBGModelAmeliore(Temps,action)                                    # Lancer le model CBGTCAmeliore
                    else:
                        print "Comportement par defaut" 
                        Temps,action = r.IfElseModel(Temps)

                    # Action choisie
                    if action == "AvoidOstacle" :
                            Temps = r.AvoidObstacles(Temps)
                    if action == "ApproachE" :
                            Temps= r.ApprochAE(Temps)
                    if action == "ApproachEp" :
                            Temps= r.ApprochAE(Temps)        
                    if action == "Wander" :
                            Temps = r.wander(Temps)
                    if action == "ReloadOnE" :
                             Temps = r.ReloadOnE(Temps) 
                    if action == "ReloadOnEp" :
                             Temps = r.ReloadOnEp(Temps) 
                    if action == "Rest" :
                            Temps = r.Rest(Temps)
                    
                    # Mettre a jour les donnees 
                    allsprites.update()
                    screen.blit(background, (0, 0))
                    count = -1
                    for ob in list_obstacles:
                        count = count + 1
                        s = pygame.display.get_surface()
                        s.fill(ob.color, list_rect_obstacles[count])
                    count = -1
                    for ob in list_obstaclescamera:
                        count = count + 1
                        s = pygame.display.get_surface()
                        s.fill(ob.color, list_rect_obstacles_camera[count])
                    r.draw_rays(screen)
                    r.draw_camera(screen)
                    r.draw_rays_back(screen)
                    draw_traces(screen)
                    allsprites.draw(screen)
                    pygame.display.flip()
                    if action == "ReloadOnE" or action == "ReloadOnEp" or action == "Rest" : 
                        while Temps < val:  
                                Temps = time.clock()
                if action == "AvoidOstacle" or action == "ApproachE" or action == "ApproachEp" or action =="Wander":
                    r.DiminuerEnergie()
                tabAx.append(Temps - ancienneValeur)
                tabAx.append(r.energie)
                tabAx.append(r.energiepotentiel)
                tabAx.append(action)
                r.tabresultat.append(tabAx)
                if(Temps - ancienneValeur > 1000 ):
                    going = False
            filename  = "TestSerie"+ str(numTest) + "Test" + str(iteration) + "_" + str(typAlgo) +".txt"
            r.ecrireFichierResultat(filename)
            r.tabresultat = []
            TempsMoyen.append(Temps-ancienneValeur)
            tabAx = []
            iteration = iteration + 1
        filename  = "TestSerie"+ str(numTest) +"Moyenne" + str(typAlgo)
        fichier = open(filename, "w")
        fichier.write(str(np.mean(TempsMoyen)))     
        pygame.quit()
    
if __name__ == '__main__':
    main()

        

