#!usr/bin/python
import sys
sys.path.append("../")
import PiBot as PB
import cv2

def main():
    a = int(input("complete the ip add 10.0.0."))
    pb = PB.PiBot('10.0.0.%d' % a)
    ticks = pb.getMotorTicks()
    #pb.setMotorSpeeds(35,35)
    image = pb.getImageFromCamera()
    cv2.imshow("current_view" , image)
    cv2.waitKey(500)
    print "left and right tickes " , ticks
    print image.shape
    #pb.setMotorSpeeds(0,0)

if __name__ == '__main__':
    main()
