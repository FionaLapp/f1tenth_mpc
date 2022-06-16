
from abc import ABC, abstractmethod, abstractproperty

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rospy import Publisher, Duration, Time
from rospy import loginfo


class VisMarker(ABC):
    @abstractmethod
    def __init__(self, x, y, duration:float):
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.header.stamp=Time.now()
        
        self.marker.action = self.marker.ADD

        # Scale for points is uniform within each direction for points, or handles line width for line-type markers
        self.marker.scale.x = 0.5
        self.marker.scale.y=0.5
        self.marker.scale.z = 0.5
     
        # When should the marker disappear
        self.marker.lifetime = Duration(duration)
        self.set_position(x, y)




    def draw_point(self):
        if not 'channel' in self.__dict__.keys():
            raise Exception("Need to define channel")
        self.channel.publish(self.marker)

    
    def set_position(self, x, y):
        # Position should be passed by the caller
        self.marker.pose.orientation.w = 1.0
        try:
           
            #self.marker.pose.position.x=x[0][0]
            #self.marker.pose.position.y=y[0][0]
            for i in range(len(x)):
                p= Point()
                
                p.x, p.y, p.z = x[i],y[i], 0
                self.marker.points.append(p)
        except:
            
            p= Point()
            p.x, p.y, p.z = x,y, 0
            self.marker.pose.position.x=x
            self.marker.pose.position.y=y
            # for i in range(len(x)):
            #   
            

class TargetMarker(VisMarker):
    def __init__(self, x:float, y:float, duration:float):
        super().__init__(x, y, duration)
        self.marker.type = self.marker.SPHERE

        
        #this seems to be a bit pointless because the colour is actually decided in the simulator.rviz file but if I leave it out it doesn't show up
        self.marker.color.r = 255
        self.marker.color.g = 0
        self.marker.color.b = 255
        self.marker.color.a = 1.0
        topic_name='/target_vis'
        self.channel=Publisher(topic_name, Marker, queue_size=100)
    # def set_position(self,x, y):
    #     # Position should be passed by the caller
    #     self.marker.pose.orientation.w = 1.0

    #     p= Point()
    #     p.x, p.y, p.z = x,y, 0
    #     self.marker.points.append(p)
    #     self.marker.pose.position.x=x
    #     self.marker.pose.position.y=y
    #     # for i in range(len(x)):
    #     #     p= Point()
            
    #     #     p.x, p.y, p.z = x[i],y[i], 0
    #     #     self.marker.points.append(p)

class GapMarker(VisMarker):
    def __init__(self, x, y, duration=1):
        super().__init__( x, y, duration)
        self.marker.type = self.marker.LINE_STRIP
        self.marker.scale.y=0.1
        #self.marker.header.frame_id = 'base_link'
        # x=[0,1,2]
        # self.set_position(x,x)
        #this seems to be a bit pointless because the colour is actually decided in the simulator.rviz file but if I leave it out it doesn't show up
        self.marker.color.r = 0
        self.marker.color.g = 225
        self.marker.color.b = 100
        self.marker.color.a = 1.0
        topic_name='/gap_vis'
        self.channel=Publisher(topic_name, Marker, queue_size=100)

    # def set_position(self,x, y):
    #     # Position should be passed by the caller
    #     self.marker.pose.orientation.w = 1.0

        
    #     #self.marker.pose.position.x=x[0][0]
    #     #self.marker.pose.position.y=y[0][0]
    #     for i in range(len(x)):
    #         p= Point()
            
    #         p.x, p.y, p.z = x[i][0],y[i][0], 0
    #         self.marker.points.append(p)
            
        

class TrajectoryMarker(VisMarker):
    def __init__(self, x, y, duration:float):
        super().__init__(x, y, duration)
        self.marker.type = self.marker.LINE_STRIP
        self.marker.scale.y=0.1
        #self.marker.header.frame_id = 'base_link'
        # x=[0,1,2]
        # self.set_position(x,x)
        #this seems to be a bit pointless because the colour is actually decided in the simulator.rviz file but if I leave it out it doesn't show up
        self.marker.color.r = 0
        self.marker.color.g = 0
        self.marker.color.b = 255
        self.marker.color.a = 1.0
        topic_name='/trajectory_vis'
        self.channel=Publisher(topic_name, Marker, queue_size=100)

    # def set_position(self,x, y):
    #     # Position should be passed by the caller
    #     self.marker.pose.orientation.w = 1.0

        
    #     #self.marker.pose.position.x=x[0][0]
    #     #self.marker.pose.position.y=y[0][0]
    #     for i in range(len(x)):
    #         p= Point()
            
    #         p.x, p.y, p.z = x[i][0],y[i][0], 0
    #         self.marker.points.append(p)
            
        
        
        

