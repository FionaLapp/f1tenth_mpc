
from abc import ABC, abstractmethod, abstractproperty
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rospy import Publisher, Duration, Time


class VisMarker(ABC):
    @abstractmethod
    def __init__(self, x:float, y:float, duration:float):
        self.point = Marker()
        self.point.header.frame_id = 'map'
        self.point.header.stamp=Time.now()
        self.point.type = self.point.SPHERE
        self.point.action = self.point.ADD

        # Scale for points is uniform within each direction for points, or handles line width for line-type markers
        self.point.scale.x = 0.5
        self.point.scale.y=0.5
        
         # Position should be passed by the caller
        self.point.pose.orientation.w = 1.0

        # When should the marker disappear
        self.point.lifetime = Duration(duration)


        p= Point()
        p.x, p.y, p.z = x,y, 0
        self.point.points.append(p)
        self.point.pose.position.x=x
        self.point.pose.position.y=y



    def draw_point(self):
        if not 'channel' in self.__dict__.keys():
            raise Exception("Need to define channel")
        self.channel.publish(self.point)
        

class TargetMarker(VisMarker):
    def __init__(self, x:float, y:float, duration:float):
        super().__init__(x, y, duration)
        #this seems to be a bit pointless because the colour is actually decided in the simulator.rviz file but if I leave it out it doesn't show up
        self.point.color.r = 255
        self.point.color.g = 0
        self.point.color.b = 255
        self.point.color.a = 1.0
        topic_name='/target_vis'
        self.channel=Publisher(topic_name, Marker, queue_size=100)
        

class TrajectoryMarker(VisMarker):
    def __init__(self, x:float, y:float, duration:float):
        super().__init__(x, y, duration)
        #this seems to be a bit pointless because the colour is actually decided in the simulator.rviz file but if I leave it out it doesn't show up
        self.point.color.r = 0
        self.point.color.g = 0
        self.point.color.b = 255
        self.point.color.a = 1.0
        topic_name='/trajectory_vis'
        self.channel=Publisher(topic_name, Marker, queue_size=100)
        
    
        

