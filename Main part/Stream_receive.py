import io
import socket
import struct
from PIL import Image
import cv2
import numpy
print('Begin')
# Start a socket listening for connections on 0.0.0.0:8000 (0.0.0.0 means
# all interfaces)
server_socket = socket.socket()
server_socket.bind(('0.0.0.0', 8000))
server_socket.listen(0)
print('Here')
# Accept a single connection and make a file-like object out of it
connection = server_socket.accept()[0].makefile('rb')
print('Before trying')
try:
    while True:
        # Read the length of the image as a 32-bit unsigned int. If the
        # length is zero, quit the loop
        print('to unpack')
        image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
        print('I am here')
        if not image_len:
            break
        # Construct a stream to hold the image data and read the image
        # data from the connection
        image_stream = io.BytesIO()
        image_stream.write(connection.read(image_len))
        # Rewind the stream, open it as an image with PIL and do some
        # processing on it
        image_stream.seek(0)
        image = Image.open(image_stream)
        opencvImage = cv2.cvtColor(numpy.array(image), cv2.COLOR_RGB2BGR)
        cv2.imshow('Image', opencvImage)
        print('Image is %dx%d' % image.size)
        image.verify()
        print('Image is verified')
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
finally:
    connection.close()
    print('Closing')
    server_socket.close()