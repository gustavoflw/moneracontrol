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
from bacterias import Bacteria, Bacterias
from navigation import Navigation

# TODO !!!
# renomear ocorrencias de "bacteria" para "bac"
# renomear a classe "Bacterias" para "Swarm"
# colocar tudo em ingles
# arrumar logs
# arrumr topicos do pwm na classe

arena_w = 0.5

bac2 = Bacteria(2, "/bacteria_2/vel")
bac3 = Bacteria(3, "/bacteria_3/vel")
bacterias = Bacterias([bac2, bac3])

tol_obst = 0.1
nav = Navigation(tol_obst)

class Node:
    def __init__(self):
        # Inicia node
        rospy.init_node("pid_controller", anonymous=True)

        # Publishers e subscribers
        self.topAr = "/ar_pose_marker"
        self.pub_marker = rospy.Publisher("/marker", Marker, queue_size=1)

        # Looprate
        self.looprate = rospy.Rate(1000)

        # Tolerancias de distancia e orientacao
        self.tolerance_dist = 0.01
        self.tolerance_ori = 5.00
       

        # Tempo
        self.time = rospy.Time.now()
        self.last_time = self.time
        self.dt = 0.0

    def update(self):
        # Atualiza tempo
        self.time = rospy.Time.now()
        self.dt = self.time.to_sec() - self.last_time.to_sec()

        # Le topico dos markers e atualiza odometria das bacterias
        self.arCallbackOnce()

        # Atualiza os outros dados das bacterias partir da nova odometria 
        bacterias.update(self.dt)

        self.last_time = self.time

    def arCallbackOnce(self):
        arMsg = rospy.wait_for_message(self.topAr, AlvarMarkers, timeout=None)

        if (len(arMsg.markers) == 0):
            rospy.logwarn("No tags detected!")
        else:
            count = 0
            for i in range(0, len(arMsg.markers)):
                id = arMsg.markers[i].id
                if id == 2 or id == 3 or id == 8:
                    count+=1
                    bacteria = bacterias.bacteria(id)
                    bacteria.position = arMsg.markers[i].pose.pose.position
                    bacteria.orientation = arMsg.markers[i].pose.pose.orientation
                    bacteria.yaw_atual = toDegree(getYaw(bacteria.orientation))
                if (count == 0):
                    rospy.logwarn("0 tags read!")

    def stop(self):
        rospy.logwarn("Stop robot!!!")
        bacterias.stopAll()

    def control(self):
        rospy.logwarn("Controlling robot...")

        # Trabalhando com a 2
        bac = bacterias.bacteria(2)
        bac.pub_pwm1 = rospy.Publisher("/bacteria_2/pwm1", Float32, queue_size=0)
        bac.pub_pwm2 = rospy.Publisher("/bacteria_2/pwm2", Float32, queue_size=0)

        # Define destino
        bac.setDestination(0.0, 0.18, 0.6)

        self.update()

        # Itera ate chegar no destino
        while ((bac.erro_linear > self.tolerance_dist) and (rospy.is_shutdown() == 0)):
            self.update()
            log = ""
            # log += "yaw_atual >> {}, angulo necessario >> {}, d_yaw >> {}\n     ".format(bac.yaw_atual, bac.angulo_necessario, bac.d_yaw)

            bac.pub_vel.publish(40)

            if bac.obstacles[0] > tol_obst and bac.obstacles[1] > tol_obst and bac.obstacles[2] > tol_obst:
                log += nav.toGoal(bac)
            elif bac.obstacles[0] <= tol_obst:
                log += nav.obstLeft(bac)
            elif bac.obstacles[1] <= tol_obst:
                log += nav.obstFront(bac)

            elif bac.obstacles[2] <= tol_obst: 
                log += nav.obstRight(bac)
            else:
                log += nav.stop(bac)

            rospy.loginfo(log)
            self.pubMarkers(bac)
            self.looprate.sleep()
        
        bac.pub_vel.publish(0.0)

        rospy.logwarn("Control END")

    def setupMarker(self, frame_id, nmspace, id, type, scale):
        marker = Marker()

        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = nmspace
        marker.id = id
        marker.type = type
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        return marker

    def pubMarkers(self, bacteria):
        scale = 0.03
        # rospy.loginfo("Publishing markers...")

        # == Marker esfera nos destinos de cada bacteria
        destinoMarker = self.setupMarker("usb_cam", "basic_shapes", 25, Marker.POINTS, scale)
        for bac in bacterias.bacList():
            destinoMarker.points.append(bac.destino)
        self.pub_marker.publish(destinoMarker)
            
        yawRadian = toRadian(bacteria.yaw_atual)
        (vx, vy) = (cos(yawRadian), sin(yawRadian))

        # == Marker linha entre a bacteria e o destino
        lineStripMarker = self.setupMarker("usb_cam", "basic_shapes", 24, Marker.LINE_STRIP, scale/5)
        lineStripMarker.points.append(bacteria.destino)
        lineStripMarker.points.append(bacteria.position)
        lineStripMarker.points.append(Point(bacteria.position.x+vx, bacteria.position.y+vy, 0.6))
        self.pub_marker.publish(lineStripMarker)

    def gas(self):
        rospy.logwarn("Looking for GAS...")

    def manual(self):
        rospy.logwarn("Manual control activated")

    def invalid(self):
        rospy.logwarn("Invalid command!!!Try again!!!")

if __name__ == '__main__':
    node = Node()
    try:
        rospy.loginfo("ACTIONS: p-Stop Robot   c-Set Coordinates(PID)  g-Gas Search	m-Manual Control")
        letter = raw_input(" > ")
        # letter = "c"
        if letter == "p":
            node.stop()
        elif letter == "c":
            node.control()
        elif letter == "g":
            node.gas()
        elif letter == "m":
            node.manual()
        else:
            node.invalid()
    except rospy.ROSInterruptException:
        pass