import casadi as ca

def min_angle(alpha) :
    return ca.atan2(ca.sin(alpha), ca.cos(alpha))

def main():
    #print(ca.atan2(1.0,.0))
    #print(ca.atan2(-1.0,.0))
    #print(ca.atan2(1.0,-0.1))
    #print(ca.atan2(1.0,0.1))
    


if __name__ == '__main__':
    main()