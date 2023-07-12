import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random

class MovementCommandManager(Node):
    def __init__(self):
        super().__init__("movement_command_subscriber")

        self.targets={} #map, die einen topic_namen auf seinen [publisher,Bewegungsbefehl]  abbuldet        
        
        self.registration_subscription = self.create_subscription(String, "/robotRegistration", self.registration_callback, 10)

        self.timer = self.create_timer(0.8, self.timer_callback)

        #robots can be manually registererd my name for testing
        #self.register_new_robot("klaus")        
        #self.register_new_robot("dieter") 

    def registration_callback(self,msg):
        """
        registers a new robot with the received name
        """
        msg_string= msg.data #message data is: "robot_type/robot_name"
        robot_type=msg_string.split("/")[0] 
        robot_name=msg_string.split("/")[1]
        print("registering new robot {robot_name}")
        self.register_new_robot(robot_name)


    def timer_callback(self):
        """
        sends all stored commands to their respective subscriptions
        """
        for topic_name in self.targets:
            publisher= self.targets[topic_name][0]
            message= self.targets[topic_name][1]
            if message is not None: 
                publisher.publish(message)
                #print("send message:",message," in topic", topic_name)
            
    def receive_command_callback(self,msg,topic_name):
        """
        Takes a new Movement Command and stores it (so it can be repeated)
        """
        print("message:",msg," was registered for in topic", topic_name)
        print(self.targets)
        self.targets[topic_name][1]=msg
        self.timer_callback()


    def register_new_robot(self,robot_name):
        """
        registers a singme robot of a given Name and initialises a (random) movement command
        """
        robot_name="/"+robot_name
        publish_topic_name=robot_name+"/cmd_vel"
        local_publisher = self.create_publisher(Twist, publish_topic_name, 10)

        ###Test
        speed= random.randint(0,2) + random.randint(0,9)/10
        newTwist = Twist()
        newTwist.linear.x = speed
        newTwist.linear.y = 0.0
        newTwist.linear.z = 0.0
        newTwist.angular.x = 0.0
        newTwist.angular.y = 0.0
        newTwist.angular.z = speed
        ###end Test

        subscription_topic_name=robot_name+"/repeatedMovementCommands"
        self.subscription = self.create_subscription(Twist, 
                                                     subscription_topic_name, 
                                                     lambda msg: self.receive_command_callback(msg, subscription_topic_name),
                                                     10)
        self.targets[subscription_topic_name]=[local_publisher,newTwist]


def main(args=None):
    rclpy.init(args=args)
    subscriber = MovementCommandManager()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()