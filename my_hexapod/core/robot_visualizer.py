from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class HexapodVisualizer:
    def __init__(self):
        # Alap szín a lábvégeknek (Zöld)
        self.foot_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

    def generate_foot_tips_marker(self, global_positions, ros_time):
        """ 
        Legenerálja a zöld golyókat a lábvégekhez a Foxglove 3D nézethez.
        Később ide jöhet a Bezier-ív rajzoló kód is!
        """
        marker_msg = Marker()
        marker_msg.header.stamp = ros_time
        marker_msg.header.frame_id = "base_link"  # A robot testéhez viszonyítjuk
        marker_msg.ns = "foot_tips"
        marker_msg.id = 0
        marker_msg.type = Marker.SPHERE_LIST
        marker_msg.action = Marker.ADD
        marker_msg.pose.orientation.w = 1.0

        # 1.5 cm átmérőjű gömbök
        marker_msg.scale.x = 0.015
        marker_msg.scale.y = 0.015
        marker_msg.scale.z = 0.015

        # Végigmegyünk a kapott pozíciókon (miközben mm-ből métert csinálunk)
        for pos in global_positions:
            p = Point(x=pos[0]/1000.0, y=pos[1]/1000.0, z=pos[2]/1000.0)
            marker_msg.points.append(p)
            marker_msg.colors.append(self.foot_color)

        return marker_msg