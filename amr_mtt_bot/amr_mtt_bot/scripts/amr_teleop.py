#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

NORMAL_SPEED = 0.8
BOOST_SPEED = 1.0
NORMAL_TURN = 1.0
BOOST_TURN = 2.0

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Normal Speed (i,j,k,l) : %.2f m/s
Boost Speed (Shift+Key): %.2f m/s

Commands:
   u    i    o
   j    k    l
   m    ,    .

Use Shift + Key for Boost Speed!
(e.g. 'I' run faster, 'i' run normal)

CTRL-C to quit
""" % (NORMAL_SPEED, BOOST_SPEED)

# Mapping: key -> (x, y, z, th, speed, turn_speed)
moveBindings = {
    'i':(1,0,0,0, NORMAL_SPEED, NORMAL_TURN),
    'I':(1,0,0,0, BOOST_SPEED, BOOST_TURN),
    'o':(1,0,0,-1, NORMAL_SPEED, NORMAL_TURN),
    'O':(1,0,0,-1, BOOST_SPEED, BOOST_TURN),
    'j':(0,0,0,1, NORMAL_SPEED, NORMAL_TURN),
    'J':(0,0,0,1, BOOST_SPEED, BOOST_TURN),
    'l':(0,0,0,-1, NORMAL_SPEED, NORMAL_TURN),
    'L':(0,0,0,-1, BOOST_SPEED, BOOST_TURN),
    'u':(1,0,0,1, NORMAL_SPEED, NORMAL_TURN),
    'U':(1,0,0,1, BOOST_SPEED, BOOST_TURN),
    ',':(-1,0,0,0, NORMAL_SPEED, NORMAL_TURN),
    '<':(-1,0,0,0, BOOST_SPEED, BOOST_TURN),
    '.':(-1,0,0,1, NORMAL_SPEED, NORMAL_TURN),
    '>':(-1,0,0,1, BOOST_SPEED, BOOST_TURN),
    'm':(-1,0,0,-1, NORMAL_SPEED, NORMAL_TURN),
    'M':(-1,0,0,-1, BOOST_SPEED, BOOST_TURN),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()

    rclpy.init()
    node = rclpy.create_node('amr_teleop_key')
    pub = node.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

    try:
        print(msg)
        while True:
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                speed = moveBindings[key][4]
                turn = moveBindings[key][5]
                
                twist = Twist()
                twist.linear.x = x * speed
                twist.linear.y = y * speed
                twist.linear.z = z * speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = th * turn
                pub.publish(twist)
                
            elif (key == '\x03'): # Ctrl-C
                break
            else:
                # ถ้าไม่มีการกด หรือกดปุ่มอื่น ให้หยุด (เพื่อความปลอดภัยแบบ Dead man switch)
                # หรือถ้าต้องการให้วิ่งต่อก็ไม่ต้องทำส่วนนี้
                # แต่ในโจทย์ sprint มักจะต้องการกดค้าง
                if (key == '' or key == 'k'):
                    twist = Twist()
                    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
                    pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()
