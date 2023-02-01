import predict
import os
import cv2


path_to_image = os.path.join(os.path.dirname(__file__),r'YorkTag\test\blur\1\10.png')
image = cv2.imread(path_to_image)
path_to_save = os.path.join(os.path.dirname(__file__),r'submit\saved.png')
print(path_to_save)

img = predict.rt_predict(image,side_by_side=True)
cv2.imwrite(path_to_save,img)