import rclpy
from rclpy.node import Node
from imgProc.msg import imgProc, imgRAW
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import numpy as np
import matplotlib.image as image
import scipy.ndimage as ndimage
from sensor_msgs.msg import Image

class ImgProcesser(Node):

    def __init__(self):
        super().__init__('imgProc_publisher')
        self.publisher = self.create_publisher(ProcImg, 'ProcessedImage', 10)
        self.subscriber = self.create_subscriber(Image, "/camera/color/image_raw", self.imageMask_callback, 10)
        self.subscriber
        self.Stream_config(rs)             #configure the pipeline


    def publish_Center(self, img2, x, y):
        msg2 = imgProc()
        msg2.data = img2
        msg2.cen_x = x
        msg2.cen_y = y
        self.publisher.publish(msg2)
        self.get_logger().info('Publishing: "centroid_x = %d, centroid_y = $d" ', x, y)



    def imageMask_callback(self, msg):
        
        blue = np.zeros((480,640,3))
        img2 = np.copy(msg)

        color_sum = np.sum(img2, axis = 2)

        blue[:,:,:][img2[:,:,2]/color_sum[:,:] > .3665] = 1
        blue = ndimage.gaussian_filter(blue, sigma=2)
        blue[blue < 1] = 0
        centroid = ndimage.center_of_mass(blue)
        y_center = np.uint32(centroid[0])
        x_center = np.uint32(centroid[1])
        publish_Center(blue, x_center, y_center)


def main(args=None):
    rclpy.init(args=args)
    Img_pubsub = ImgProcesser()
    rclpy.spin(Img_pubsub)
    Img_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
