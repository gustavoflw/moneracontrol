#!/usr/bin/env python
from math import sqrt, pow, atan2, atan, sin, cos
from tf.transformations import euler_from_quaternion

PI = 3.14159

def getYaw(quaternion):
    (roll, pitch, yaw) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return yaw

def hipotenusa(x, y, z):
    return (sqrt(pow(x,2) + pow(y,2) + pow(z,2)))

def toDegree(n):
    return n * (180/PI)

def toRadian(n):
    return n * (PI/180)

def AnguloNecessario(pos_origem, pos_destino): # Em graus
    vy = pos_destino.y-pos_origem.y
    vx = pos_destino.x-pos_origem.x
    return toDegree(atan2(vy, vx))