import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from my_interfaces.msg import Float

class Pwm_pub(Node):
	def __init__(self):
		super().__init__('pwm_publisher')
		self.pub=self.create_publisher(String,"pwm_val",10)
		self.sub=self.create_subscription(Float,'distance',self.sub_callback,10)
		self.last_time=time.time()
		self.count=0
		self.last_dist=0
		self.dist_low=20	#cm
		self.dist_high=100
		self.vel_low=-20	#cm/s
		self.vel_high=-50
		self.pwm_max=1800
		
	def sub_callback(self,dist_msg):
		dist=dist_msg.distance
		if self.count==0:
			self.last_dist=dist
			self.count+=1
			return
		curr_time=time.time()
		vel=(dist-self.last_dist)/(curr_time-self.last_time)
		pwm=self.set_pwm(dist,vel)
		msg=String()
		msg.data=f'{pwm}/{pwm}/{pwm}/{pwm}/{pwm}/{pwm}'
		self.pub.publish(msg)
		self.last_dist=dist
		self.last_time=curr_time
		self.get_logger().info(f'Object distance = {dist:.2f} Object velocity = {vel:.2f}')
		
	def set_pwm(self,dist,vel):
	
		#object moving away
		
		if vel>=0:
			if dist>self.dist_high:
				pwm=self.pwm_max
				
			elif self.dist_low<dist<=self.dist_high:
				pwm = int(1500+ (dist-self.dist_low)/(self.dist_high - self.dist_low)*(self.pwm_max-1500))
                          
			else:
				pwm=1500
				self.reverse()
		
		#object coming closer
		
		else:
			if dist>self.dist_high:
				pwm=self.pwm_max
				
			elif self.dist_low<dist<=self.dist_high:
				pwm = int(1500+ (dist-self.dist_low)/(self.dist_high - self.dist_low)*(self.pwm_max-1500))
                          
			else:
				pwm=1500
				self.reverse()
		return pwm
		
	def reverse(self):
		reverse_pwm=1400
		msg=String()
		msg.data=f'{reverse_pwm}/{reverse_pwm}/{reverse_pwm}/{reverse_pwm}/{reverse_pwm}/{reverse_pwm}'
		self.pub.publish(msg)
		self.get_logger().info(f'Reversing Slightly...')
		time.sleep(0.5)

def main():
	rclpy.init()
	node=Pwm_pub()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		print('Command to stop...')
	finally:
		node.destroy_node()
		rclpy.shutdown()
	
if __name__=='__main__':
	main()
	
			
		
