#!/usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, radians

instantX = 0.0 # Anlık olarak gelen Odometry verileri
instantY = 0.0
instantTh = 0.0

targetX = rospy.get_param('/goToThePoint/targetX', -6.5) # Launch dosyasından hedefler alınıyor.
targetY = rospy.get_param('/goToThePoint/targetY', 3.5)
targetTh = rospy.get_param('/goToThePoint/targetTh', 90)

speed = Twist()
control = 0 # Aracın hedef koordinata ulaşıp ulaşmadığını ve hedef yöne dönüp dönmediğini tutacak.

def GoToTargetPoint():
    global instantX, instantY, instantTh, speed, control, targetX, targetY, targetTh
    newX = targetX - instantX # Hedef koordinat ve anlık bulunulan koordinat arasındaki X ve Y eksenlerindeki mesafe bulunuyor.
    newY = targetY - instantY
    if ( newX < 0.01 ) and ( newY < 0.01 ): # Bu mesafe 0.01'den düşükse hedef koordinata varıldığı anlaşılıyor, 
        control = 1                         # hedef yöne döndürülümek için kontrol değişkeni 1 yapılıyor,
        speed.linear.x = 0.0                # araç durduruluyor.
        speed.angular.z = 0.0
        print( "Hedefe Koordinata Varildi! Hedef Yone Donuluyor...")
    else: # Hedef koordinata varılmadıysa...
        angleToGoal = atan2( newY, newX ) # X ve Y eksenlerindeki mesafenin arctan'ı alınıp hedef noktaya giden açı bulunuyor.
        if abs( angleToGoal - instantTh ) > 0.1: # Bu açı 0.1'den büyükse sola dönülüyor.
            speed.linear.x = 0.0
            speed.angular.z = 0.2
        else: # Bu açı -0.1 ile 0.1 arasındaysa düz gidiliyor.
            speed.linear.x = 0.2
            speed.angular.z = 0.0
    rospy.Publisher( "/cmd_vel", Twist, queue_size = 1 ).publish( speed ) # Topic yayınlanıyor.

def TurnToTargetDirection():
    global instantTh, speed, control, targetTh
    thetaRadians = radians( targetTh ) # Hedef yön radyana çevriliyor.
    if abs( thetaRadians - instantTh ) > 0.1: # Hedef yön ile anlık yön arasındaki fark 0.1'den büyükse sola dönülüyor.
        speed.linear.x = 0.0
        speed.angular.z = 0.2
    else: # Bu açı -0.1 ile 0.1 arasındaysa duruluyor ve hedef yöne de ulaşıldığı anlaşılıyor.
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        print( "Hedef Yone Ulasildi! \nKodu Sonlandirmak Icin 'CTRL + C' Yapiniz!")
        control = -1
    rospy.Publisher( "/cmd_vel", Twist, queue_size = 1 ).publish( speed ) # Topic yayınlanıyor.


def OdomCallback( msg ):
    global instantX, instantY, instantTh, control
    instantX = msg.pose.pose.position.x # Anlık olarak Odometry'den gelen aracın koordinatları ve aracın yönü.
    instantY = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    instantTh = euler_from_quaternion( [ rot_q.x, rot_q.y, rot_q.z, rot_q.w ] )[ 2 ]
    if control == 0: # Hedef koordinata gidiliyor.
        GoToTargetPoint()
    if control == 1: # Hedef yöne dönülüyor.
        TurnToTargetDirection()

def main():
    rospy.init_node( "GoToThePoint" )
    rospy.Subscriber( "/odom", Odometry, OdomCallback ) # Odometry'e abone olunuyor.
    rospy.spin()

if __name__ == '__main__':
    main()