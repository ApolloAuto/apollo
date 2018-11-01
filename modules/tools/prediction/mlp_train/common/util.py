def segment_overlap(a, b, x, y):
    if b < x or a > y:
        return False

    return True

def vector_projection_overlap(p0, p1, p2, p3):
    v = p1.subtract(p0)
    n_square = v.norm_square()
   
    v0 = p2.subtract(p0)
    v1 = p3.subtract(p0)

    t0 = v0.dot(v)
    t1 = v1.dot(v)

    if t0 > t1:
        t = t0
        t0 = t1
        t1 = t
        
    return segment_overlap(t0, t1, 0.0, n_square)
