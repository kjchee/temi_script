
import rclpy

from rclpy.node import Node

from std_msgs.msg import String
from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.srv import SubmitTask
from rmf_task_msgs.msg import TaskType, Loop
from rmf_lift_msgs.msg import LiftState

import paho.mqtt.client as mqtt 
import json 
import time


base_url = "ec2-52-77-238-127.ap-southeast-1.compute.amazonaws.com"
client_id = "temi_heartbeat_demo"
WAYPOINT_TOPIC = "temi/00121195615/command/waypoint/goto"
SPEECH_TOPIC = "temi/00121195615/command/tts"



dist_tolerance = 0.3
reset_tolerance = 0.5
speech1_done = False
speech2_done = False
speech3_done = False
speech4_done = False
neuronbot_out_of_lift = False

location_1 = [2.341,-6.34]#[1.8966,-4.6678]#[2.344, -6.358] # temi Liftwait
location_2 = [22.7658, -23.6183] # neuronbot out of the lift
location_3 = [0.958, -8.54] #[-0.8925, -9.0441] # temi in lift
location_reset = [-9.929, -7.82] # reset param point

#point 4: 3.039,-8.46

goto_waypoint_1 = '{\"location\":\"Liftinside\"}'
goto_waypoint_2 = '{\"location\":\"Photoshoot\"}'

speech0 =  '{\"utterance\":\"okay\"}'
speech1 =  '{\"utterance\":\"Please wait while my buddy exits. It will keep out of our way once it’s out from the lift\"}'
speech2 =  '{\"utterance\":\"Please allow me to enter the lift first.\"}'
speech3 =  '{\"utterance\":\"Please join me in taking the lift.\"}'
speech4 =  '{\"utterance\":\"Please exit the lift.\"}'
#speech4 =  '{\"utterance\":\"Please exit the lift. I will catch up when everyone is out.\"}'



class RMFStateSubscriber(Node):

    def __init__(self):

        super().__init__('temi_automation')
        self.node = rclpy.create_node('temi_automation_node')
        self.get_logger().info('hello temi node v1.0 is started..')
        
        self.current_lift_level = "B1"
        self.current_lift_door = 0 #close
        
        self.create_subscription(
            FleetState,
            'fleet_states',
            self.fleet_state_callback,
            10)
        self.create_subscription(
            LiftState,
            'lift_states',
            self.lift_state_callback,
            10)
        self.client = mqtt.Client(client_id) #create new instance
        self.client.username_pw_set("hopermf", "hopermf608614")
        self.client.connect(base_url,1883) #connect to broker
        print('connected to MQTT broker')
        self.client.loop_start()
        
        self.client.publish(SPEECH_TOPIC,speech0)#publish
        print('okay')

    def lift_state_callback(self, msg):
        global neuronbot_out_of_lift,WAYPOINT_TOPIC,goto_waypoint_1
        #self.get_logger().info('I heard: "%s"' % msg.lift_name)
        if (msg.lift_name == "PL5"):
            self.current_lift_level = msg.current_floor  
            self.current_lift_door = msg.door_state
            lift_info = msg.lift_name + ", " + self.current_lift_level  + ", " + str(self.current_lift_door)
            self.get_logger().info('I heard: "%s"' % lift_info)
            '''
            if (neuronbot_out_of_lift and self.current_lift_level == "L2" and self.current_lift_door == 2):
                self.client.publish(WAYPOINT_TOPIC,goto_waypoint_1)#publish
                neuronbot_out_of_lift = False
                self.get_logger().info('Send temi into lift !!!!!!!!!!!!!!!')
            '''

        
    def fleet_state_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.name)
        global reset_tolerance,location_reset,neuronbot_out_of_lift,speech1_done,speech2_done,speech3_done,speech4_done,dist_tolerance,location_1,location_2,location_3,speech1,speech2,speech3,speech4, WAYPOINT_TOPIC,goto_waypoint_1,goto_waypoint_2
        if (msg.name == "Temi"):
            for robot in msg.robots:
                if (robot.name == "temi_rmf1"):
                    cur_x = robot.location.x
                    cur_y = robot.location.y
                    cur_lvl = robot.location.level_name
                    if (speech1_done == False and abs(cur_x - location_1[0])< dist_tolerance and abs(cur_y - location_1[1])< dist_tolerance and cur_lvl == "L2"):
                        self.get_logger().info('in position !!!!!!!!!!!!!!!')
                        if (self.current_lift_level == "L2" and self.current_lift_door == 2):
                            self.client.publish(SPEECH_TOPIC,speech1)#publish
                            speech1_done = True
                            self.get_logger().info('Triggered speech 1 !!!!!!!!!!!!!!!')
                        '''
                        else:
                            self.client.publish(SPEECH_TOPIC,speech1)#publish
                            speech1_done = True
                            self.get_logger().info('Triggered speech 1 testing!!!!!!!!!!!!!!!')
                        '''
                        
                    elif (speech3_done == False and abs(cur_x - location_3[0])< dist_tolerance and abs(cur_y - location_3[1])< dist_tolerance and cur_lvl == "L2"):
                        if (self.current_lift_level == "L2" and self.current_lift_door == 2):
                            self.client.publish(SPEECH_TOPIC,speech3)#publish
                            speech3_done = True
                            self.get_logger().info('Triggered speech 3 !!!!!!!!!!!!!!!')
                    
                    elif (abs(cur_x - location_reset[0])< reset_tolerance and abs(cur_y - location_reset[1])< reset_tolerance):
                        speech1_done = False
                        speech2_done = False
                        speech3_done = False
                        speech4_done = False
                        self.get_logger().info('Reset !!!!!!!!!!!!!!!')
                    '''
                    elif (speech4_done == False and abs(cur_x - location_3[0])< dist_tolerance and abs(cur_y - location_3[1])< dist_tolerance and cur_lvl == "L2"):
                        if (self.current_lift_level == "L3" and self.current_lift_door == 2):
                            self.client.publish(WAYPOINT_TOPIC,goto_waypoint_2)#publish
                            time.sleep(0.5)
                            self.client.publish(SPEECH_TOPIC,speech4)#publish
                            speech4_done = True
                            self.get_logger().info('Triggered speech 4 !!!!!!!!!!!!!!!')
                    '''
                    
                    
        elif (msg.name == "missy"):#neuronbot, missy
            for robot in msg.robots:
                if (robot.name == "missybot"):#neuronbot2, missybot
                    cur_x = robot.location.x
                    cur_y = robot.location.y
                    cur_lvl = robot.location.level_name
                    
                    if (speech1_done == True and speech2_done == False and abs(cur_x - location_2[0])< dist_tolerance and abs(cur_y - location_2[1])< dist_tolerance and cur_lvl == "L2"):
                        self.get_logger().info('neuron is out of lift')
                        if (self.current_lift_level == "L2" and self.current_lift_door == 2):
                            self.client.publish(SPEECH_TOPIC,speech2)#publish
                            time.sleep(0.5)
                            self.client.publish(WAYPOINT_TOPIC,goto_waypoint_1)#publish
                            speech2_done = True
                            self.get_logger().info('Triggered speech 2 !!!!!!!!!!!!!!!')
                            neuronbot_out_of_lift = True
                    
                    '''
                    if (speech1_done == True):
                        time.sleep(2.0)
                        self.get_logger().info('neuron is out of lift')
                        if (self.current_lift_level == "L2" and self.current_lift_door == 2):
                            self.client.publish(SPEECH_TOPIC,speech2)#publish
                            time.sleep(0.5)
                            self.client.publish(WAYPOINT_TOPIC,goto_waypoint_1)#publish
                            speech2_done = True
                            self.get_logger().info('Triggered speech 2 !!!!!!!!!!!!!!!')
                            neuronbot_out_of_lift = True
                    '''
                        


def main(args=None):
    rclpy.init(args=args)

    rmfstate_subscriber = RMFStateSubscriber()
    
    
    rclpy.spin(rmfstate_subscriber)

    rmfstate_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
