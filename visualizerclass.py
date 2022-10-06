import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA, Int32
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from tf.transformations import quaternion_from_euler

class VisualizerClass(object):
    def __init__(self,name='vis',HZ=20):
        self.name = name
        self.HZ = HZ

        self.init_pub_markers()
        self.init_pub_texts()
        self.init_pub_meshes()

    def init_pub_markers(self):
        self.marker_array = MarkerArray()
        self.n_marker = 0
        self.MARKERS_MAX = 1000
        self.pub_markers = rospy.Publisher("/visualization_engine/markers",
                        MarkerArray, queue_size=5)

    def init_pub_texts(self):
        self.text_array = MarkerArray()
        self.n_text = 0
        self.TEXTS_MAX = 1000
        self.pub_texts = rospy.Publisher("/visualization_engine/texts",
                        MarkerArray, queue_size=5)

    def init_pub_meshes(self):
        self.mesh_array = MarkerArray()
        self.n_mesh = 0
        self.MESHES_MAX = 1000
        self.pub_meshes = rospy.Publisher("/visualization_engine/meshes",
                        MarkerArray, queue_size=5)
                    

    def append_marker(self,x=0.0,y=0.0,z=0.0,r=0.1,frame_id='map',
            color=ColorRGBA(0.0,1.0,0.0,0.5),marker_type=Marker.SPHERE):
        marker = Marker(
                type=marker_type,
                pose=Pose(Point(x, y, z), Quaternion(0, 0, 0, 1)),
                scale=Vector3(r, r, r),
                header=Header(frame_id=frame_id,stamp=rospy.get_rostime()),
                action=Marker.ADD,
                color=color,
                lifetime=rospy.Duration(secs=1/self.HZ)
                )
        self.n_marker += 1
        if(self.n_marker > self.MARKERS_MAX):
             self.marker_array.markers.pop(0)
        self.marker_array.markers.append(marker) # append

    def append_text(self,x=0.0,y=0.0,z=0.0,r=0.1,text='text',scale=Vector3(0,0,1),
            frame_id='map',color=ColorRGBA(1.0,1.0,1.0,0.5)):
        marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                text=text,
                pose=Pose(Point(x, y, z), Quaternion(0, 0, 0, 1)),
                scale=scale,
                header=Header(frame_id=frame_id),
                action=Marker.ADD,
                color=color,
                lifetime=rospy.Duration(secs=1/self.HZ)
                )
        self.n_text += 1
        if(self.n_text > self.TEXTS_MAX):
             self.text_array.markers.pop(0)
        self.text_array.markers.append(marker) # append

    def append_mesh(self,x=0.0,y=0.0,z=0.0,scale=1.0,dae_path='duck.dae',
            frame_id='map',color=ColorRGBA(0.0,1.0,1.0,0.5),
            roll=0.0,pitch=0.0,yaw=0.0):
        marker = Marker(
                type=Marker.MESH_RESOURCE,
                mesh_use_embedded_materials=True,
                mesh_resource=dae_path,
                # pose=Pose(Point(x, y, z), Quaternion(roll, pitch, yaw, 1)),
                pose=Pose(Point(x, y, z), Quaternion(*quaternion_from_euler(roll,pitch,yaw))),
                scale=Vector3(scale, scale, scale),
                header=Header(frame_id=frame_id),
                action=Marker.ADD,
                color=color,
                lifetime=rospy.Duration(secs=1/self.HZ)
                )
        self.n_mesh += 1
        if(self.n_mesh > self.MESHES_MAX):
             self.mesh_array.markers.pop(0)
        self.mesh_array.markers.append(marker) # append

    def reset_markers(self):
        self.marker_array.markers = []
        self.n_marker = 0

    def reset_texts(self):
        self.text_array.markers = []
        self.n_text = 0

    def reset_meshes(self):
        self.mesh_array.markers = []
        self.n_mesh = 0

    def delete_markers(self,frame_id='map'):
        self.marker_array.markers = []
        self.n_marker = 0
        marker = Marker(
                id=0,
                header=Header(frame_id=frame_id,stamp=rospy.get_rostime()),
                action=Marker.DELETEALL,
        )
        self.marker_array.markers.append(marker) # append
        self.pub_markers.publish(self.marker_array)

    def delete_texts(self,frame_id='map'):
        self.text_array.markers = []
        self.n_text = 0
        marker = Marker(
                id=0,
                header=Header(frame_id=frame_id,stamp=rospy.get_rostime()),
                action=Marker.DELETEALL,
        )
        self.text_array.markers.append(marker) # append
        self.pub_texts.publish(self.text_array)

    def delete_meshes(self,frame_id='map'):
        self.mesh_array.markers = []
        self.n_mesh = 0
        marker = Marker(
                id=0,
                header=Header(frame_id=frame_id,stamp=rospy.get_rostime()),
                action=Marker.DELETEALL,
        )
        self.mesh_array.markers.append(marker) # append
        self.pub_meshes.publish(self.mesh_array)

    def publish_markers(self):
        # update maker index
        for m_idx,m in enumerate(self.marker_array.markers):
            m.id = m_idx
        # publish
        self.pub_markers.publish(self.marker_array)

    def publish_texts(self):
        # update maker index
        for m_idx,m in enumerate(self.text_array.markers):
            m.id = m_idx
        # publish
        self.pub_texts.publish(self.text_array)    

    def publish_meshes(self):
        # update maker index
        for m_idx,m in enumerate(self.mesh_array.markers):
            m.id = m_idx
        # publish
        self.pub_meshes.publish(self.mesh_array)