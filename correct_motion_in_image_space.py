#!usr/bin/python
import numpy as np
import cv2
import time

H = np.array([[  1.12375325e-01,  -4.86001720e-01,   8.77712777e+02],
             [  -2.36834933e+00,   4.49234588e+00,   6.64034708e+02],
             [   4.01039286e-04,   1.59276405e-02,   1.00000000e+00]],float)


def main():
    print("Let's read two images from step 8 and 9")
    time.sleep(0.5)
    I1 = cv2.imread("./test_log/%05d.png" % 8)
    I1 = cv2.cvtColor(I1,cv2.COLOR_RGB2GRAY)
    I2 = cv2.imread("./test_log/%05d.png" % 9)
    I2 = cv2.cvtColor(I2,cv2.COLOR_RGB2GRAY)
    print("... and warp them using our H matrix")
    time.sleep(0.5)
    I1w = cv2.warpPerspective(I1, H, (600,600))
    I2w = cv2.warpPerspective(I2, H, (600,600))
    II  = np.zeros((600,1200),"uint8")
    II[:,:600] = I1w
    II[:,600:] = I2w
    time.sleep(0.5)
    print("the two views with motion between them")
    print("press any key on the image to continue.")
    cv2.imshow("The two views",II)
    cv2.waitKey(0)
    print("Let's assume that we know the robot motion between them")
    tx = -25.28   # mm
    ty = 2.2      # mm
    yaw = 2.37    # degrees
    #--------------
    c = np.cos(np.deg2rad(yaw))
    s = np.sin(np.deg2rad(yaw))
    print("using the motion of the robot, let's create a rigid transform matrix")
    print("""
               |cos(theta) -sin(theta) tx|
           M = |sin(theta)  cos(theta) ty|

       """)
    M = np.array([[c, -s,tx],[s, c,ty]],float)
    time.sleep(0.5)
    print("let's align the two images using M")
    (cols,rows) = I1w.shape
    dst = cv2.warpAffine(I1w,M,(cols,rows))
    dst = cv2.addWeighted(dst,0.5,I2w,0.5,0)
    cv2.imshow("The two views",dst)
    print("this did not work great because the center of rotation is not")
    print("0,0 (the upper left corner of the image)")
    print("the center of rotation is the middle points between the wheels")
    print("We need to correct for that:")
    print("press any key on the image to continue.")
    cv2.waitKey(0)
    cr = np.array([[0.0],[266]])
    R  = M[:2,:2]
    tc = R.dot(cr) - cr
    M[0,2] -= tc[0]
    M[1,2] -= tc[1]

    # Due to slight motion in the camera, there was a scale effect between the two images.
    # https://math.stackexchange.com/questions/13150/extracting-rotation-scale-values-from-2d-transformation-matrix
    M[:2,:2] *= 0.98

    dst = cv2.warpAffine(I1w,M,(cols,rows))
    dst = cv2.addWeighted(dst,0.5,I2w,0.5,0)
    cv2.imshow("The two views",dst)
    print """
            We use a rotation matrix (R) from M (the 2x2 block on the left)
            Rotate the center of the robot using R (the center in this case in cr =[0,266])
            Then translate the points back to (0,0)
            tc = R . cr - cr
            then we take this translation component out of M
            M[:,2] -= tc
                """
    cv2.waitKey(0)

if __name__ == '__main__':
    main()
