import numpy as np
import cv2
import glob

checkboard_rows=6
checkboard_cols=9
square_size=25

criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001)

objp=np.zeros((checkboard_rows*checkboard_cols,3),np.float64)

objp[:,:2]=np.mgrid[0:checkboard_cols,0:checkboard_rows].T.reshape(-1,2)*square_size

objpoints=[]
imagep=[]

images=glob.glob('calibration_images/*.jpg')


for image1 in images:
    img=cv2.imread(image1)
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret,cornors=cv2.findChessboardCorners(gray,(checkboard_cols,checkboard_rows),None)
    if ret==True:
        objpoints.append(objp)
        #increasingthe accuracy
        corners2 = cv2.cornerSubPix(gray, cornors, (11, 11), (-1, -1), criteria)
        cv2.drawChessboardCorners(img,square_size,corners2,ret)
        cv2.imshow(img)
        cv2.waitKey(2000)
        imagep.append(corners2)
        print(imagep)

cv2.destroyAllWindows()
      
if len(objpoints)>5:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imagep, gray.shape[::-1], None, None
    )

    if ret:
        print("Success")
        print(mtx)
        print(dist)
    else:
        print('Failed')
else:
    print('Not captured enough cornors')
