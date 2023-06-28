class hmqr5_agl_map:
    J4 = -133.21
    AGL_MAP = [69.6834,81.34557,94.5478,J4,-91.4718,-108.142]
    limit = [
        [-170,170],
        [-90,90],
        [-60,190],
        [-150,150],
        [-170,170],
        [-170,170]
    ]
    for i in range(6):#map
        limit[i][0] -= AGL_MAP[i]
        limit[i][1] -= AGL_MAP[i]
        limit[i][0] *= 3.14159/180.
        limit[i][1] *= 3.14159/180.

if __name__=='__main__':
    am = hmqr5_agl_map()
    print(am.limit)