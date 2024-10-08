from sympy import *
import math
x, y, t, u, v, theta = symbols('x,y,t,u,v,theta')
# eq = [2 * sin(t) - sin(theta) + v * cos(theta) - x,
#       1-2 * cos(t) + cos(theta) + v * sin(theta) - y]
# eq = asin((x + sin(theta) - v * cos(theta)) / 2) - acos((1 + cos(theta) + v * sin(theta) - y) / 2)
# eq = (x - 2 * sin(t) + sin(theta)) / cos(theta) - (y + 2 * cos(t) - 1 - cos(theta))/sin(theta)
eq = (y - 1 + 2 * cos(t) - cos(theta)) / (x - 2 * sin(t) + sin(theta)) - tan(theta)
result = nonlinsolve(eq, t)
print(result)

# eq1 = [2 * sin(t) - sin(theta) + (y + 2*cos(t) - cos(theta) - 1)/sin(theta) * cos(theta) - x]
# res = nonlinsolve(eq1, [t])
# print(res)
# x = 5
# y = 0
# theta = 180 * pi / 180
# fun1 = simplify((x - 2 * sin(t) + sin(theta)) / cos(theta) - (y + 2 * cos(t) - 1 - cos(theta))/sin(theta))
# t_solve = solve(fun1, t)
# u_solve = theta - t_solve
# v_solve = simplify((sin(theta - 2 * t) - x) / cos(theta))
# print(t_solve)
# x = sqrt(2) - 1
# y = 1 - sqrt(2)
# theta = 90 * pi / 180
# print(x*sin(theta)/2 - y*cos(theta)/2 + cos(theta)/2 + 1/2)
# t = min(theta + math.acos(x*sin(theta)/2 - y*cos(theta)/2 + cos(theta)/2 + 1/2),
#         theta - acos(x*sin(theta)/2 - y*cos(theta)/2 + cos(theta)/2 + 1/2) + 2*pi)
# u = t - theta
# v = (x - 2 * sin(t) + sin(theta)) / cos(theta)
# print(u_solve)
# print(v_solve)
# t = acos(x*sin(theta) - y*cos(theta) + cos(theta))/2
# u = theta - t
# v = -(x + sin(2*t - theta))/cos(theta)
# print(t)
# print(u)
# print(v)