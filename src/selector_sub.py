#!/usr/bin/env python
'''Controller Manager ROS Node'''
import rospy
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Imu
from robot import Robot
import roslaunch_api_wrapper as raw 
from graphviz import Digraph
# The system can be in idle mode or in control mode
# In each moment the user can switch between the two modes using G1 and G2
# Idle Mode: the user can select one robot (only one) performing gesture G3
# Control Mode: the controller of the selected robot is enabled so the user can directly teleoperate the robot

MODES = ['Init', 'Idle', 'Control']
# Define the list of robots in the system
robot_list = []
# Define the dict of ros publishers
cmd_pubs = {}
# Set the path of the parameter file (robot list)
filename = "/home/deeplearning/roboware_ws/src/manager_controller/src/robot_id"

# Define the gesture mapping
UP = 1
DOWN = 2
RIGTH = 3
LEFT = 4
CLOCK = 5
ACLOCK = 6

# Initialize main variables of the system 
# The system is initialized in idle mode with no robot selected
 
mode = 0
selected = -1
count = 0
gesture = 0
iteration = 0

# This callback is used to redirect imu data to the selected controller only 
def imu_callback(imu_data):
    '''Imu Callback Function'''
    imu_idle = Imu()
    imu_idle.linear_acceleration.x = 0
    imu_idle.linear_acceleration.y = 0
    imu_idle.linear_acceleration.z = 0
    imu_idle.angular_velocity.x = 0
    imu_idle.angular_velocity.y = 0
    imu_idle.angular_velocity.z = 0
    
    #rospy.loginfo('imu: %f %f %f', imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z)
    if MODES[mode] == 'Control':
            #cmd_pubs[robot_list[selected].name].publish(imu_data)
            if selected != -1:
                cmd_pubs[robot_list[selected].name].publish(imu_data)
    else: #forse questo non serve, vedere controller 
            cmd_pubs[robot_list[selected].name].publish(imu_idle)

# This callback is used to switch between modes and between robots
# according to the recognized gesture
def callback(data):
    '''selector Callback Function'''
    rospy.loginfo('[CALLBACK] '+rospy.get_caller_id() + " I heard gesture: %d", data.data)
    gesture = data.data
    global mode, selected

    if gesture == UP:
        if MODES[mode]== 'Idle' or MODES[mode]== 'Init':
            mode = 2
        #mode = (mode+1) % len(MODES)
        #show_fsm() #uncomment to display fsm
        #selected = 1
        #selected = (selected+1) % len(robot_list)
        #show_list()
    elif gesture == DOWN:
        if MODES[mode]== 'Control':
            mode = 1
        #here we can reset iterator for robot selection
        #selected = -1
    elif gesture == RIGTH:
        if MODES[mode]== 'Idle':
            selected = (selected+1) % len(robot_list)
            #show_list()
    else: 
        if MODES[mode]== 'Init':
            mode = 1

    show_fsm() #uncomment to display fsm      
    show_list() 
    rospy.loginfo('[CALLBACK] mode: '+ MODES[mode]+ ', selected: '+ robot_list[selected].name+ ', %d: ',selected)
    rospy.loginfo(' \n')

# This metod can be used to toggle system mode
def toggle_mode():
    '''selector ToggleMode Function'''
    global mode
    if gesture == CLOCK and MODES[mode] == 'Idle':
        mode = (mode+1) % len(MODES)
    rospy.loginfo("[toggleMode] mode: "+ MODES[mode])
    
# This metod can be used to read a list of robot from file
# cols: _name, _id,_type, _pkg_name, _node_name
# eg. husqvarna 2 g gesture_based_controller gb_controller.py
def read_robot_parameter_from_file():
    '''Read Parameters Function'''
    f = open(filename)
    for line in f:
        _name, _id,_type, _pkg_name, _node_name = map(str, line.split(" "))
        r = Robot(_name, _id, _type, _pkg_name,_node_name)
        robot_list.append(r)
        
def publisher_creator():
    '''Publisher Creator Function'''
    for i in robot_list:
        tmp = i.name
        rospy.loginfo(" A wild "+ tmp + " appears!" )
        cmd_topic = '/'+ tmp + '/teleop/imu'
        cmd_pubs[tmp] = rospy.Publisher(cmd_topic, Imu, queue_size=10)
        rospy.loginfo('created topic: ' + cmd_topic )
        #rospy.loginfo('pubs: %s',cmd_pubs[str(robot_list[selected].name)])

def controllers_launcher():
    '''Controller Launcher Function'''
    #tmp_pkg1 = 'gesture_based_controller'
    #tmp_node1 = 'gb_controller.py'
    #rospy.loginfo("Init node: "+ tmp_node)
    #raw.NodeStarter(tmp_pkg, tmp_node)
    for i in robot_list:
          if i.pkg_name != 'empty':
             tmp_pkg = i.pkg_name
             #tmp_pkg = tmp_pkg.strip('\n')
             tmp_node = i.node_name
             tmp_node = tmp_node.strip('\n')
             rospy.loginfo("Init node: "+ tmp_node)
             raw.NodeStarter(tmp_pkg, tmp_node)
            #raw.LauncherStarter(tmp_pkg, tmp_node) 
            

def show_fsm():
    old1 = MODES[0]
    d = Digraph('modes')
    for m in MODES:
        d.node(m)
        d.edge(old1,m)
        if old1 != MODES[0]:
            d.edge(m,old1)
        old1 = m
        d.node(MODES[0], shape='doublecircle')
        d.node(MODES[mode], color='green')
    d.view()
    
   

def show_list():
    old = ''
    #old = robot_list[selected].name
    d = Digraph('robots')
    d.attr('Node', shape='circle', style='filled')
    for r in robot_list:
        s=r.name
        d.node(s)
        if r.id > 1:
            d.edge(old,s)
        old = s
        if selected == -1:
            d.node(robot_list[selected].name, color='grey')
        else:
            d.node(robot_list[selected].name, color='green')
    d.view()
    
       

            
def listener():
    '''Subscribers Selector'''
    # 1. Init ROS node
    rospy.init_node('selector', anonymous=True)
    
    # 2. Load robots in the system
    read_robot_parameter_from_file()
    
    # 3. Create publisher for each robot
    publisher_creator()
    # 4. Launch Robot Controllers
    controllers_launcher()
    
    # 5. Set Subscribers:  
    # gesture iD
    rospy.Subscriber("chatter", UInt8, callback)
    # imu 
    rospy.Subscriber("/imu_left_hand", Imu, imu_callback)
    
    # debug precious info
    mode = 0
    rospy.loginfo(' mode: '+ MODES[mode]+ ', selected: '+ robot_list[selected].name)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        rate.sleep()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
