#!/usr/bin/env python
# queue:http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy.linalg import inv

class image_converter:

    def __init__(self):
       self.image_pub = rospy.Publisher("/sensors/camera/infra1/camera_info",Image,queue_size=1)
       self.bridge = CvBridge()
       self.image_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw",Image,self.callback,queue_size=1)
  
    def callback(self,data):
        try:
           cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
           print(e)
        # to black white
        bi_max = 255
        bi_min = 240
        ret , thresh1 = cv2.threshold(cv_image, bi_min, bi_max, cv2.THRESH_BINARY)

        # rectangle
        cv2.rectangle(thresh1, (0, 0), (600, 480), 0,150)
        cv2.rectangle(thresh1, (0, 330), (600, 480), 0,150)
        cv2.rectangle(thresh1, (250, 250), (450, 480), 0,50)
        cv2.rectangle(thresh1, (200, 100), (300, 12), 0,5)

        #white points(x,y)

        # cv2.rectangle(thresh1, (210, 100), (300, 130), 250,2) 5
        # cv2.rectangle(thresh1, (210, 150), (300, 180), 250,2) 3
        # cv2.rectangle(thresh1, (180, 200), (300, 250), 250,2)1
        # cv2.rectangle(thresh1, (400, 100), (430, 120), 250,2)6
        # cv2.rectangle(thresh1, (410, 140), (500, 160), 250,2)4
        # cv2.rectangle(thresh1, (480, 200), (600, 250), 250,2)2

        
        data=[[180, 200, 300, 250],[480, 200, 600, 250],[210, 150, 300, 180],[410, 140, 500, 160],[210, 100,300, 130],[400, 100, 430, 120]]
        position=[]
        for region in data:
            x1=region[0]
            y1=region[1]
            x2=region[2]
            y2=region[3]
            t1=thresh1[y1:y2,x1:x2]
            acc_i=0 
            acc_j=0 
            num=0
            for i in xrange(t1.shape[0]):
                for j in xrange(t1.shape[1]):
                    pixel = t1.item(i, j)
                    if pixel != 0:
                        acc_i+=i+y1
                        acc_j+=j+x1
                        num+=1
            position+=[[acc_i/num,acc_j/num]]
        print("points:",position)
        
        #extrinsic parameters, rotaion-translation vector

        objectPoints = np.zeros(( 6,3,1 ))
        # z axi is 0
        objectPoints[:,:,0] = np.array( [[0.5, 0.2, 0],[0.5, -0.2, 0],[0.8, 0.2, 0],[0.8, -0.2,0],[1.1, 0.2, 0],[1.1, -0.2, 0]])
        
        imagePoints = np.zeros((6,2,1))
        imagePoints[:,:,0] = np.array(position)
        
        cameraMatrix = np.array([[383.7944641113281, 0.0, 322.3056945800781], [0.0, 383.7944641113281, 241.67051696777344], [0.0, 0.0, 1.0]])
        distCoeffs=np.zeros((5,1))
       
    # https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#bool%20solvePnP(InputArray%20objectPoints,%20InputArray%20imagePoints,%20InputArray%20cameraMatrix,%20InputArray%20distCoeffs,%20OutputArray%20rvec,%20OutputArray%20tvec,%20bool%20useExtrinsicGuess,%20int%20flags)
        retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs) 
        # nom_rvec=np.linalg.norm()            
        print("rotation vector",rvec)
        print(" translation vector", tvec)

        #transformation matrix

        rmat,  jacobian = cv2.Rodrigues(rvec)
        print("rmat",rmat)
        hom_tvec = np.append(tvec, [[1]], axis=0)
        hom_rmat = np.append(rmat, [[0,0,0]], axis=0)
        hom_trans= np.append(hom_rmat,hom_tvec,axis=1)
        print("hom_trans",hom_trans)
        

        invers_trans = inv(hom_trans)
        print("invers_trans",invers_trans)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
        except CvBridgeError as e:
             print(e)
   
def main(args):
        rospy.init_node('image_converter', anonymous=True)
        ic = image_converter()
       
        try:
            rospy.spin()
        except KeyboardInterrupt:
             print("Shutting down")
        cv2.destroyAllWindows()
    
if __name__ == '__main__':
         main(sys.argv)

