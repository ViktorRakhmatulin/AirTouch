import predict
import os
import cv2


path_to_image = os.path.join(os.path.dirname(__file__),r'YorkTag\test\blur\1\10.png')
print(path_to_image)
image = cv2.imread(path_to_image)
cv2.imshow("Image",image)
cv2.waitKey()
predict.main(path_to_image)