#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

banner = """
1. Reading from the keyboard  
2. Publishing to {topic} (geometry_msgs/Twist)!
---------------------------
        w
   a    s    d
anything else : stop
CTRL-C to quit
"""

keyBindings = {
  'w':(1,0),
  'd':(1,-1),
  'a':(1,1),
  's':(-1,0),
  'z':(0,0),
}

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

speed = 1
turn = 0.6

def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('keyop')
  output_topic = rospy.get_param("~output_topic", "/cmd_vel")
  pub = rospy.Publisher(output_topic, Twist, queue_size=1)
  print(banner.format(topic=output_topic))

  x = 0
  th = 0
  status = 0

  try:
    while(1):
       key = getKey()
       if key in keyBindings.keys():
          x = keyBindings[key][0]
          th = keyBindings[key][1]
       else:
          x = 0
          th = 0
          if (key == '\x03'):
             break
       msg = Twist()
       msg.linear.x = x * speed
       msg.angular.z = th * turn

       pub.publish(msg)

  except:
    print('error')

  finally:
    msg = Twist()
    pub.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
