import cv2 as cv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from my_interfaces.msg import Float
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.bridge = CvBridge()
        self.time_period = 0.02  # seconds

        self.distance2 = 0.0
        self.angle = 0.0
        self.col = "Orange"
        self.colcode = 1

        self.publisher_ = self.create_publisher(Image, 'topic_image', 10)
        self.timer = self.create_timer(self.time_period, self.timer_callback)

        self.publisher2_ = self.create_publisher(Float, 'distance', 10)
        self.timer2 = self.create_timer(self.time_period, self.timer_callback2)

        self.cap = cv.VideoCapture(0)


        # self.cap.set(3, 500)
        # self.cap.set(4, 500)
        self.i = 0
        self.flap = 0
        self.low = 10
        self.high = 25

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            
            if self.flap ==1:
                self.low = 160
                self.high = 179
                self.col = "Red"
                self.colcode = 3
            elif self.flap ==2: 
                self.low = 10
                self.high = 25
                self.colcode = 2
                self.col = "yellow"
            elif self.flap ==3:
                self.low = 80
                self.high = 130
                self.colcode = 4
                self.col = "blue"
            else:
                self.low = 0
                self.high = 10
                self.colcode = 1
                self.col = "Orange"
                self.flap =0
                
            ret, thresh = self.imageRectification(frame, self.low, self.high)

            contours, hierarchy = cv.findContours(thresh, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

            dummy = self.dispCont(frame, contours, self.col)

            

            frame = cv.resize(frame,(1000,1000),interpolation=cv.INTER_AREA)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info(f'\nDistance : {self.distance2} cm \n Angle : {self.angle} degrees \n Color : {self.col}')

            self.i += 1
            if dummy ==0:
                self.flap +=1

    def timer_callback2(self):
            msg2 = Float()
            msg2.distance = self.distance2
            msg2.angle = self.angle
            msg2.col = self.colcode
            self.publisher2_.publish(msg2)
            self.get_logger().info(f'Publishing distance {msg2.distance}')

    def imageRectification(self, Frame, a, b):
        hsv = cv.cvtColor(Frame, cv.COLOR_BGR2HSV)
        lower = (a, 50, 0)
        upper = (b, 255, 255)
        mask = cv.inRange(hsv, lower, upper)
        hsv1 = cv.bitwise_and(Frame, Frame, mask=mask)

        gray = cv.cvtColor(hsv1, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (5, 5), 1)

        return cv.threshold(blur, 50, 255, cv.THRESH_BINARY)
    
    def dispCont(self, Frame, contours, col,obj_length=5):
        a = 0
        for cnt in contours:
                area = cv.contourArea(cnt)
                
                if area > 7000:
                    cv.drawContours(Frame, cnt, 0, (0, 255, 0), 3)
                    x, y, w, h = cv.boundingRect(cnt)
                    cv.rectangle(Frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

                    focal = Frame.shape[1]  # Using frame width as focal length approximation
                    self.distance2 = obj_length * focal / (w) # 1346 for 1080p camera
                    self.angle = (x + w / 2 - Frame.shape[1] / 2) * 0.1 # 0.1 degree per pixel

                    # print("Distance:", self.distance2,"\ncolor:", col, "\nAngle:", self.angle)
                    cv.putText(Frame, f'Distance: {round(self.distance2, 2)} cm and {round(self.angle, 2)} and {col}', (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 1.0,
                                                              (0, 255, 255), 2)
                    a += 1
        return a



def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
