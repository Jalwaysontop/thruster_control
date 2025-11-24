from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class Pwm_sub(Node):
	def __init__(self):
		super().__init__("thruster_node")
		self.sub=self.create_subscription(String,'pwm_val',self.callback,10)
	def callback(self,msg):
		self.get_logger().info(f'I got PWM :{msg.data}')

def main():
	rclpy.init()
	node=Pwm_sub()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		print('Command to stop...')
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__=='__main__':
	main()
