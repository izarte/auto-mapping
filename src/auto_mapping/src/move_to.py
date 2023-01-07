#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import sys

#Uso de la acción move_base en ROS para moverse a un punto determinado
#En ROS una acción es como una petición de un "cliente" a un "servidor"
#En este caso este código es el cliente y el servidor es ROS
#(en concreto el nodo de ROS 'move_base')

class WayPoint:

    def __init__(self,tag,x=0,y=0):
        self.x=float(x)
        self.y=float(y)
        self.tag=tag

    def changeCoordinates(self,new_x,new_y):
        self.x=float(new_x)
        self.y=float(new_y)

    def changeName(self,new_tag):
        self.tag=new_tag

    def printPoint(self):
        print("------------------")
        print("Waypoint")
        print("Name: ", self.tag)
        print("Coordinates:")
        print("  x: ", self.x)
        print("  y: ", self.y)





class ClienteMoveBase:
    def __init__(self):
        #creamos un cliente ROS para la acción, necesitamos el nombre del nodo 
        #y la clase Python que implementan la acción
        #Para mover al robot, estos valores son "move_base" y MoveBaseAction
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #esperamos hasta que el nodo 'move_base' esté activo`
        self.client.wait_for_server()

    def moveTo(self, x, y):
        #un MoveBaseGoal es un punto objetivo al que nos queremos mover
        goal = MoveBaseGoal()
        #sistema de referencia que estamos usando
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x   
        goal.target_pose.pose.position.y = y
        #La orientación es un quaternion. Tenemos que fijar alguno de sus componentes
        goal.target_pose.pose.orientation.w = 1.0

        #enviamos el goal 
        self.client.send_goal(goal)
        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal
        #get_state obtiene el resultado de la acción 
        state = self.client.get_state()
        #ACTIVE es que está en ejecución, PENDING que todavía no ha empezado
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rospy.Rate(10)   #esto nos da la oportunidad de escuchar mensajes de ROS
            state = self.client.get_state()
            print(state)
        return self.client.get_result()

if __name__ == "__main__":
    print("Probando clase waypoint.")
    rospy.init_node('prueba_clientemovebase')

    wayList=list()

    mesa1=WayPoint("Mesa1",1.17,-0.6)
    mesa2=WayPoint("Mesa2",1.1,0.50)
    mesa3=WayPoint("Mesa3",-0.5,2.50)
    Home=WayPoint("Home",0,0)

    wayList.append(mesa1)
    wayList.append(mesa2)
    wayList.append(mesa3)
    wayList.append(Home)

    for punto in wayList:

        punto.printPoint()
        
        cliente = ClienteMoveBase()
        result = cliente.moveTo(punto.x,punto.y)
        print(result)
        if result:
            rospy.loginfo("Goal conseguido!")


 
   