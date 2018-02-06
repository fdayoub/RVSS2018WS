"""
Created on Thu Nov  2 08:26:32 2017

@author: feras dayoub
"""
import numpy as np
import cv2

class FindH(object):
    def __init__(self):
        self.im = []
        self.im_test = []
        self.src_pts = list()
        self.dst_pts = list()
        self.mouse_cb()

    def estimate_H(self,fname):
        self.im = cv2.imread(fname)
        cv2.imshow('image',self.im)
        print "click on the points you want then press escape"
        cv2.waitKey(0)
        print "all points:"
        self.src_pts = np.array(self.src_pts,dtype='float')
        print self.src_pts.shape
        # uncomment and replace points in the world frame below
        #offset = np.array([0,300.0])
        #self.dst_pts = np.array([[217, 0],
        #                         [397,118.5],
        #                         [397, 0],
        #                         [397, -118.5],
        #                         [577, 118.5],
        #                         [577, 0],
        #                         [577, -118.5]
        #                         ],dtype='float')

        self.dst_pts = self.dst_pts + offset
        h, status = cv2.findHomography(self.src_pts, self.dst_pts)
        print h
        self.H = h
        np.save("./H.npy",h)
        im_out1 = cv2.warpPerspective(self.im, self.H, (600,600))
        cv2.imwrite("./result.png",im_out1)
        cv2.imshow('warped image',im_out1)
        cv2.waitKey(0)

    def mouse_cb(self):
        cv2.namedWindow('image')
        cv2.setMouseCallback('image',self.save_clicks)
    def save_clicks(self,event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print "col %d" % x
            print "row %d" % y
            self.src_pts.append([x,y])


def main():

    findH = FindH()
    findH.estimate_H("./00001.png")



if __name__ == "__main__":
    main()
