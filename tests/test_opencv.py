#!usr/bin/python
import cv2
import numpy as np

H = np.array([[  7.52283648e-02,  -8.32161527e-01,   1.31423490e+03],
             [  -3.50517812e+00,   1.17031710e+01,   1.04372232e+03],
             [   2.89533623e-04,   2.39619736e-02,   1]],float)

def main():
    I = np.ones((320,320)) * 255
    for i in np.arange(10,320,20):
        I[i,:] = 0
        I[:,i] = 0

    try:
        cv2.imwrite("test.png",I)
        print("saving image - pass")
        I = cv2.imread("test.png")
        cv2.imshow("test",I)
        cv2.waitKey(500)
        print("reading image - pass")
        Iw = cv2.warpPerspective(I, H, (1000,1000))
        cv2.imwrite("test_w.png",Iw)
        print("image warpping - pass")
        cv2.imshow("test_w",Iw)
        cv2.waitKey(500)
    except Exception as e:
        print("opencv test fail - fail")

if __name__ == '__main__':
    main()
