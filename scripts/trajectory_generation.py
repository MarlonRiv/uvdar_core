#!/usr/bin/env python
import rospy
from mrs_msgs.srv import PathSrv
from std_msgs.msg import Header
from mrs_msgs.msg import Reference, Path
from geometry_msgs.msg import Point

def call_trajectory_service(points):
    rospy.init_node('trajectory_generation_client')
    rospy.wait_for_service('/uav1/trajectory_generation/path')
    
    try:
        trajectory_service = rospy.ServiceProxy('/uav1/trajectory_generation/path', PathSrv)
        
        request = Path()
        request.header = Header(seq=0, stamp=rospy.Time.now(), frame_id='')
        request.input_id = 0
        request.use_heading = False
        request.fly_now = False
        request.stop_at_waypoints = False
        request.loop = False
        request.max_execution_time = 0.0
        request.max_deviation_from_path = 0.0
        request.dont_prepend_current_state = False
        request.override_constraints = False
        request.override_max_velocity_horizontal = 0.0
        request.override_max_acceleration_horizontal = 0.0
        request.override_max_jerk_horizontal = 0.0
        request.override_max_velocity_vertical = 0.0
        request.override_max_acceleration_vertical = 0.0
        request.override_max_jerk_vertical = 0.0
        request.relax_heading = False
        request.points = points  
        
        response = trajectory_service(request)
        
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":

    points = [
        Reference(position=Point(x=5.0, y=5.0, z=0.0), heading=0.0),
        Reference(position=Point(x=4.0, y=5.0, z=0.0), heading=0.0),
        Reference(position=Point(x=3.0, y=5.0, z=0.0), heading=0.0),
        Reference(position=Point(x=2.0, y=5.0, z=0.0), heading=0.0),
        Reference(position=Point(x=1.0, y=5.0, z=0.0), heading=0.0),
        Reference(position=Point(x=0.0, y=5.0, z=0.0), heading=0.0)
        #Reference(position=Point(x=-5.0, y=5.0, z=0.0), heading=0.0),
        #Reference(position=Point(x=0.0, y=5.0, z=0.0), heading=0.0)
    ]
    
    response = call_trajectory_service(points)
    rospy.loginfo(response)
