#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from rospy.core import is_shutdown, rospyinfo
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from math import sqrt, pow, atan2, atan, sin, cos

from calculos import *

class Bacteria:
    def __init__(self, id, topicPubVel):
        # Identificacao da bacteria
        self.id = id
        self.pub_vel = rospy.Publisher(topicPubVel, Float32, queue_size=0)

        # Cada bacteria tem seu erro de tolerancia angular
        self.erro_angular_pequeno = 5.00
        self.erro_angular_medio = 10.00
        self.erro_angular_grande = 180 - self.erro_angular_pequeno - self.erro_angular_medio

        # Linear
        self.position = Point()
        self.destino = Point()
        self.d_posicao = 0.0
        self.v = 0.0
        self.erro_linear = 99.0

        self.last_position = Point()

        # Angular
        self.orientation = Quaternion()
        self.yaw_atual = 0.0 # Em graus
        self.angulo_necessario = 0.0 # Em graus
        self.d_yaw = 0.0 # Em graus
        
        self.w = 0.0 # Velocidade angular (grau/s)
        self.d_w = 0.0
        self.ac_angular = 0.0 # Aceleracao angular (grau/s^2)

        self.last_yaw = 0.0 # Em graus
        self.last_w = 0.0
        self.soma_erros_w = 0.0

        # Obstaculos
        self.obstacles = [100, 100, 100] # Esquerda, frente, direita

        # Salva os comandos pros robos
        self.vel1 = 0.0
        self.vel2 = 0.0

    def update(self, dt):
        # rospy.logwarn("updating id >> {}".format(self.id))

        # Atualiza erros de tolerancia angular
        # self.

        # Parte angular
        self.angulo_necessario = AnguloNecessario(self.position, self.destino)
        self.d_yaw = self.yaw_atual - self.angulo_necessario
        
        if (dt != 0):
            self.w = (self.yaw_atual - self.last_yaw) / dt
            self.d_w = self.w - 0.0 # Isso eh usado qdo ele era pra estar sem rotacao
            self.soma_erros_w += (self.d_w * dt)
        else:
            rospy.logwarn("(calculating w) dt = 0, skipping...")
            self.w = 0.0
            self.d_w = 0.0
            self.soma_erros_w = 0.0

        # Parte linear
        self.erro_linear = hipotenusa(self.destino.x-self.position.x, self.destino.y-self.position.y, 0)
        self.d_posicao = hipotenusa(self.position.x-self.last_position.x, self.position.y-self.last_position.y, 0)
        if (dt != 0):
            self.v = self.d_posicao / dt
        else:
            rospy.logwarn("(calculating v) dt = 0, skipping...")
            self.v = 0.0

        # Atualiza valores antigos
        self.last_position = self.position
        self.last_yaw = self.yaw_atual

    def setDestination(self, x, y, z):
        self.destino = Point(x,y,z)

# Grupo de bacterias (classe do swarm)
class Bacterias:
    def __init__(self, bacList):
        self.bacDict = {}
        for bac in bacList:
            self.bacDict[bac.id] = bac
    def update(self, dt):
        for bac in self.bacList():
            bac.update(dt)
            if bac.id == 2:
                self.updateObstacles(bac)
    def bacteria(self, id):
        return self.bacDict[id]
    def bacList(self):
        l = []
        for id in self.bacDict:
            bacteria = self.bacDict[id]
            l.append(bacteria)
        return l
    def stopAll(self):
        for id in self.bacDict:
            bacteria = self.bacDict[id]
            bacteria.pub_vel.publish(Float32(0.0))
    def updateObstacles(self, bac): # Retorna lista com distancias aos obstaculos em cada direcao
        # Atualmente so lida com 1 OBSTACULO!
        l = [100.0, 100.0, 100.0]
        for bacIterator in self.bacList():
            if bacIterator != bac: # So faz sentido se o iterador nao for bac
                v_ro = Point(   bacIterator.position.x - bac.position.x,
                                bacIterator.position.y - bac.position.y,
                                0.0)
                theta = toDegree(atan2(v_ro.y, v_ro.x)) - bac.yaw_atual
                d = hipotenusa(v_ro.x, v_ro.y, 0.0)
                # Frente
                if (-30 <= theta <= 30):
                    if (d < l[1]):
                        direction = "FRE"
                        l[1] = d
                # Direita
                elif (30 < theta <= 90):
                    if (d < l[2]):
                        direction = "DIR"
                        l[2] = d
                # Esquerda
                elif (-90 <= theta < -30):
                    if (d < l[0]):
                        direction = "ESQ"
                        l[0] = d
                else:
                    direction = None
                log = "OBSTACULO: {} a {} do {}".format(bacIterator.id, direction, bac.id)
                # rospy.loginfo("theta >> {}, d >> {}, log >> {}".format(theta, d, log))
        bac.obstacles = l