#!/usr/bin/env python3
 
from __future__ import print_function

import rospy
from motion_controller.srv import path, pathResponse
from motion_controller.msg import point

def handle_add_two_ints(req):
    rospy.logdebug(f"Received starting points: {req.start_point}, ending point: {req.end_point}")
    # print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    
    rsp=pathResponse([point([1,2]),point([1,2])])
    #process path
    # rsp.path=[[1,2],[1,2]]
    rospy.logdebug(f"Found a path: {rsp}")
    return rsp
 
def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('path_planner', path, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()