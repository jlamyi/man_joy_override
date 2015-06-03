#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import threading
import sys


N_STICK = 2
ROBOT_LIMIT = 4

'''
if len(sys.argv) > ROBOT_LIMIT + 1:
  print ('The amount of robots exceeds the limitation ' + str(ROBOT_LIMIT))
  print ('Only the first ' + str(ROBOT_LIMIT) + ' robot names will be taken')
  N_ROBOT = ROBOT_LIMIT
else:
  N_ROBOT = len(sys.argv)-1

robot_names = []
for r in range(N_ROBOT):
  robot_names.append(sys.argv[r+1])
  print('robot ' + str(r) +  ' is mapped to ' + sys.argv[r+1])'''


def vo_to_twist(vo):
  twist = Twist()
  twist.linear.x = vo[0]
  twist.angular.z = vo[1]
  return twist

class ManJoyState():
  def __init__(self):
    self.cmd_in = [(0,0)]*ROBOT_LIMIT
    self.joy_dest = range(N_STICK)
    self.joy_in = [(0,0)]*N_STICK
    self.lock = threading.Condition()

  def joy_callback(self, joy_msg):
    self.lock.acquire()
    # update joy_in based on analog axes
    self.joy_in[0] = (joy_msg.axes[1], joy_msg.axes[0])
    self.joy_in[1] = (joy_msg.axes[4], joy_msg.axes[3])
    
    # update desired destinations based on buttons
    buttons = []
    buttons.append([joy_msg.axes[6] == 1, joy_msg.axes[7]==1, 
                    joy_msg.axes[6] == -1, joy_msg.axes[7]== -1])
    buttons.append([joy_msg.buttons[2]==1, joy_msg.buttons[3]==1,
                    joy_msg.buttons[1]==1, joy_msg.buttons[0]==1])

    for d in range(N_STICK):
      if True in buttons[d]:
        dest = buttons[d].index(True) 
        self.joy_dest[d] = dest
        for d_other in range(N_STICK):
          if d_other is not d and self.joy_dest[d_other] is dest:
            self.joy_dest[d_other] = ROBOT_LIMIT
          
        rospy.loginfo('Set dest of stick %d to robot %d' % (d, self.joy_dest[d]))
    
    self.lock.release()

  def cmd_callback(self, which, tw_msg):
    self.lock.acquire()
    # update cmd_in
    self.cmd_in[which] = (tw_msg.linear.x, tw_msg.angular.z)
    rospy.loginfo('callback cmd_in[%d] = (%f,%f)' % 
      (which, self.cmd_in[which][0], self.cmd_in[which][1]))
    self.lock.release()




class robot_manager():
  def __init__(self):
    self.lock2 = threading.Condition()
    self.robot_names = ["dummy"]*ROBOT_LIMIT
    self.state = ManJoyState()
    self.pubs = [rospy.Publisher('dummy/cmd_vel', Twist, queue_size = 1)]*4 
    self.subs_init()

  def pubs_update(self):
      for i in range(ROBOT_LIMIT):
        self.pubs[i]=rospy.Publisher(self.robot_names[i] + '/cmd_vel', Twist, queue_size = 1)

  def subs_init(self):
      for i in range(ROBOT_LIMIT):
        rospy.Subscriber('robot' + str(i) + '/cmd_vel_in', Twist, self.curried_callback(i))
      rospy.Subscriber("/online_detector/online_robots", String, self.online_robot_callback)

  def curried_callback(self,j):
    return lambda m: self.state.cmd_callback(j,m)

  def show_mapping(self):
      for i in range(ROBOT_LIMIT):
          print('robot' + str(i) +  ' is mapped to ' + self.robot_names[i])

  def online_robot_callback(self,data):
    online_robot_list = data.data.split(',')

    self.lock2.acquire()
    new = list(set(online_robot_list)-set(self.robot_names))
    lost = list(set(self.robot_names)-set(online_robot_list))

    if len(new) + len(lost) > 0:
        if len(online_robot_list) > ROBOT_LIMIT:
            print ('The amount of robots exceeds the limitation ' + str(ROBOT_LIMIT))
            print ('Only the first ' + str(ROBOT_LIMIT) + ' robot names will be taken')
            self.robot_names = online_robot_list[:ROBOT_LIMIT]
        else:
            for i in range(len(online_robot_list)):
                self.robot_names[i] = online_robot_list[i]
        
        self.pubs_update()
        print "Joy configuration changes: "
        self.show_mapping()
    self.lock2.release()


    


def run():

  rm = robot_manager()
  state = rm.state

  rospy.init_node('man_joy_override', anonymous=True)

  rospy.Subscriber('joy', Joy, state.joy_callback)
  r = rospy.Rate(30)

  while not rospy.is_shutdown():
    # for each publisher, use joystick if joy_dest designates one of the robots
    # otherwise use corresponding cmd_in
    for i in range(ROBOT_LIMIT):

      state.lock.acquire()
      which = i

      if i in state.joy_dest:
        vo_cmd = state.joy_in[state.joy_dest.index(i)]
      else:
        vo_cmd = state.cmd_in[i]
      state.lock.release()

      rm.lock2.acquire()
      rm.pubs[i].publish(vo_to_twist(vo_cmd))
      rm.lock2.release()

    r.sleep()

if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException: pass
