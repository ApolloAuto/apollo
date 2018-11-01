from math import cos, sin
from vector2d import Vector2

def rotate(v, theta):
    cos_theta = cos(theta)
    sin_theta = sin(theta)
    
    return rotate_fast(v, cos_theta, sin_theta)

def rotate_fast(v, cos_theta, sin_theta):
    x = cos_theta * v.x - sin_theta * v.y
    y = sin_theta * v.x + cos_theta * v.y
    
    return Vector2(x, y)
