import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Go2CameraFeedNode(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('go2_camera_feed')
      
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.camera_publisher = self.create_publisher(
        Image, 
        'go2_video_frames', 
        10)
      
      
    # Create the timer
    self.timer = self.create_timer(0.1, self.timer_callback)
         
    # Create a VideoCapture object
    # Currently hard coding the address
    gstreamer_str = "udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1"
    # cap = cv2.VideoCapture(gstreamer_str, cv2.CAP_GSTREAMER)

    self.cap = cv2.VideoCapture(gstreamer_str, cv2.CAP_GSTREAMER)
         
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()
          
    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      self.camera_publisher.publish(self.br.cv2_to_imgmsg(frame, encoding='bgr8'))
 
    # Display the message on the console
    # self.get_logger().info('Publishing video frame')
  
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_publisher = Go2CameraFeedNode()
    
    # Spin the node so the callback function is called.
    rclpy.spin(image_publisher)

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()



  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
#   image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
#   rclpy.shutdown()
  
if __name__ == '__main__':
    main()