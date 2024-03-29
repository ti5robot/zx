#!/usr/bin/env python

from Ti5_py import Ti5_py


def main():
    ayay=Ti5_py()
    
    j=[0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    ayay.move_by_joint(j)
    ayay.get_joint()
    ayay.get_pos()

    port = '/dev/ttyUSB0'
    #ayay.init_serial(port,115200)

    #ayay.read_ser()

    sock = ayay.init_udp(8088)
    ayay.udp_read(sock)

'''
    for i in range (1,10):
    
        j=[0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        ayay.move_by_joint(j)
        ayay.get_joint()
        ayay.get_pos()

        jj=[-0.2, -0.3, -0.4, -0.5, -0.6, -0.7]
        ayay.move_joint(jj)
        ayay.get_joint()
        ayay.get_pos()


        print("get_error")
        ayay.get_error()


        xyzrpy=[-2.361745e-06, -8.3143390e-05, 0.436347686, -5.87181396e-05, -5.824219744e-05, 6.26911e-05]
        ayay.move_by_pos(xyzrpy)
        ayay.get_joint()
        ayay.get_pos()

        print("get_electric")
        ayay.get_electric()
	
	ff = []
	for i in range(6):
	    zx=float(input())
	    ff.append(zx)
	ayay.test_joint(ff)
'''

if __name__=="__main__":
    main()

