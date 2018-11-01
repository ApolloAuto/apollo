from math import sqrt

class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def add(self, v):
        return Vector2(self.x + v.x, self.y + v.y)
            
    def subtract(self, v):
        return Vector2(self.x - v.x, self.y - v.y)
    
    def dot(self, v):
        return self.x * v.x + self.y * v.y
    
    def norm(self):
        return sqrt(self.x * self.x + self.y * self.y)

    def norm_square(self):
        return self.x * self.x + self.y * self.y
    
    def print_point(self):
        print(str(self.x) + "\t" + str(self.y) + "\n")
