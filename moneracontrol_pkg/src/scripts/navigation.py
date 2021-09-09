#!/usr/bin/env python
import rospy
from movimentos import Movimentos
import simpful as sf
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

straight = -4
curveCW = 1
curveCCW = 5
pwmCurveCW = [1,0]
pwmCurveCCW = [0,1]

class FuzzyControl():
    def __init__(self, tol_obst):
        # Input
        self.dist = ctrl.Antecedent(np.arange(0.05, tol_obst, tol_obst/100), "dist")
        self.dist["distL"] = fuzz.trimf(self.dist.universe, [0.05, 0.05, 1.2*tol_obst/2])
        self.dist["distH"] = fuzz.trimf(self.dist.universe, [0.8*tol_obst/2, tol_obst, tol_obst])

        # Output
        self.pwm = ctrl.Consequent(np.arange(0.0, 100.0, 1.0), "pwm")
        self.pwm["pwmL"] = fuzz.trimf(self.pwm.universe, [0.0, 0.0, 50.0])
        self.pwm["pwmH"] = fuzz.trimf(self.pwm.universe, [50.0, 100.0, 100.0])

        # Rules
        r1 = ctrl.Rule(self.dist["distL"], self.pwm["pwmL"])
        r2 = ctrl.Rule(self.dist["distH"], self.pwm["pwmH"])
        
        self.pwm_ctrl = ctrl.ControlSystem([r1, r2])
        self.pwm_sim = ctrl.ControlSystemSimulation(self.pwm_ctrl)

        # Visualizacao
        self.dist.view(autoclose=True)
        self.pwm.view()

    def inference(self, dist):
        self.pwm_sim.input["dist"] = dist
        self.pwm_sim.compute()
        return self.pwm_sim.output['pwm']

class Navigation():
    def __init__(self, tol_obst):
        self.fuzzycontrol = FuzzyControl(tol_obst)

    # Se sem obstaculos, navega ate goal
    def toGoal(self, bac):
        log = "toGoal, "
        if abs(bac.d_yaw) <= bac.erro_angular_pequeno:
            log += "small angle! going straight, "
            # bac.pub_vel.publish(straight)
            bac.pub_pwm1.publish(0)
            bac.pub_pwm2.publish(50)
            rospy.sleep(1)
            bac.pub_pwm1.publish(50)
            bac.pub_pwm2.publish(0)
        elif abs(bac.d_yaw <= bac.erro_angular_medio):
            log += "med angle! "
            if (bac.d_yaw) < 0:
                log += "turning right, "
                # bac.pub_vel.publish(curveCW)
                bac.pub_pwm1.publish(pwmCurveCW[0]*50)
                bac.pub_pwm2.publish(pwmCurveCW[1]*50)
            else:
                log += "turning left, "
                # bac.pub_vel.publish(curveCCW)
                bac.pub_pwm1.publish(pwmCurveCCW[0]*50)
                bac.pub_pwm2.publish(pwmCurveCCW[1]*50)
        else:
            log += "big angle! "
            if (bac.d_yaw) < 0:
                log += "turning right, "
                # bac.pub_vel.publish(curveCW)
                bac.pub_pwm1.publish(50)
                bac.pub_pwm2.publish(-50)
            else:
                log += "turning left, "
                # bac.pub_vel.publish(curveCCW)
                bac.pub_pwm1.publish(-50)
                bac.pub_pwm2.publish(50)
        return log

    # Obstaculo a esquerda -> Curva a direita
    def obstLeft(self, bac):
        log = "obstLeft, "
        dist = bac.obstacles[0]
        log += "dist >> {}, ".format(dist)
        inf = self.fuzzycontrol.inference(dist)
        log += "pwm >> {}".format(inf)
        bac.pub_pwm1.publish(pwmCurveCW[0]*inf)
        bac.pub_pwm2.publish(pwmCurveCW[1]*inf)
        return log

    # Obstaculo a frente -> Curva a direita
    def obstFront(self, bac):
        log = "obstFront, "
        dist = bac.obstacles[1]
        log += "dist >> {}, ".format(dist)
        inf = self.fuzzycontrol.inference(dist)
        log += "pwm >> {}".format(inf)
        bac.pub_pwm1.publish(pwmCurveCW[0]*inf)
        bac.pub_pwm2.publish(pwmCurveCW[1]*inf)
        return log

    # Obstaculo a direita -> Curva a esquerda
    def obstRight(self, bac):
        log = "obstRight, "
        dist = bac.obstacles[2]
        log += "dist >> {}, ".format(dist)
        inf = self.fuzzycontrol.inference(dist)
        log += "pwm >> {}".format(inf)
        bac.pub_pwm1.publish(pwmCurveCCW[0]*inf)
        bac.pub_pwm2.publish(pwmCurveCCW[1]*inf)
        return log

    # Para
    def stop(self, bac):
        log = "stop, "
        bac.pub_pwm1.publish(0)
        bac.pub_pwm2.publish(0)
        return log