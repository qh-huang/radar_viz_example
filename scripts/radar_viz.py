#!/usr/bin/env python

import rospy
import cantools
import math
from rospkg import RosPack
from std_msgs.msg import String
from can_msgs.msg import Frame
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from pprint import PrettyPrinter


class CanFrameToRadarTargetConverter(object):
    def __init__(self):
        rp = RosPack()
        pkg_path = rp.get_path('radar_viz_example')
        self.can_msg_parser = cantools.database.load_file(pkg_path + '/config/' +
                                                          'ARS408_can_database_ch0_1.dbc')
        self.pub = rospy.Publisher('radar_points', PointCloud,
                                   queue_size=65536)
        # TODO: prompt to ask topic name
        self.subscribed_topic_ = "can0_message"
        print('Subscribed topic: ' + self.subscribed_topic_)
        self.frame_sub = rospy.Subscriber(self.subscribed_topic_, Frame,
                                          self._cb, queue_size=10)

        self.pointcloud_msg_ = PointCloud()
        ch = ChannelFloat32()
        ch.name = 'intensity'
        self.pointcloud_msg_.channels.append(ch)

        self.msg_id_to_msg_name_ = {}
        for msg in self.can_msg_parser.messages:
            self.msg_id_to_msg_name_[msg.frame_id] = msg.name

        # # debug only
        # understood_msgs = []
        # for msg in self.can_msg_parser.messages:
        #     this_msg = {}
        #     this_msg['frame_id'] = msg.frame_id
        #     this_msg['name'] = msg.name
        #     this_msg['signals'] = msg.signals
        #     understood_msgs.append(this_msg)
        # rospy.loginfo("We can interpret the messages:")
        # pp = PrettyPrinter()
        # rospy.loginfo(pp.pformat(understood_msgs))

    def _cb(self, data):
        # message looks like:
        # header:
        #   seq: 10450
        #   stamp:
        #     secs: 1520743044
        #     nsecs: 188934087
        #   frame_id: ''
        # id: 304
        # is_rtr: False
        # is_extended: False
        # is_error: False
        # dlc: 8
        # data: [0, 0, 4, 92, 110, 16, 4, 36]
        try:
            msg = self.can_msg_parser.decode_message(data.id, data.data)
            msg['frame_id'] = data.id
            msg['message_name'] = self.msg_id_to_msg_name_[data.id]
            if msg['message_name'] == 'Cluster_0_Status':
                self.pub.publish(self.pointcloud_msg_)

                # clear message buffer for next scan
                self.pointcloud_msg_.points = []
                # refresh header info for next scan
                self.pointcloud_msg_.header = data.header
                self.pointcloud_msg_.header.frame_id = "world"

            elif msg['message_name'] == 'Cluster_1_General':
                msg['raw_msg'] = str(data.data)
                #human_friendly = str(msg)
                cluster_dist_long = msg['Cluster_DistLong']
                cluster_dist_lat = msg['Cluster_DistLat']
                cluster_rcs = msg['Cluster_RCS']

                # # debug only
                # print('Cluster_DistLong: ' + str(cluster_dist_long))
                # print('Cluster_DistLat: ' + str(cluster_dist_lat))
                # print('Cluster_RCS: ' + str(cluster_rcs))

                pt = Point32()
                pt.x = cluster_dist_long
                pt.y = cluster_dist_lat
                pt.z = 0

                self.pointcloud_msg_.points.append(pt)
                cluster_rcs_norm = (cluster_rcs - (-64.0)) / \
                    (64.0 - (-64.0)) * 255
                self.pointcloud_msg_.channels[0].values.append(
                    cluster_rcs_norm)
        except KeyError:
            msg = {}
            msg['frame_id'] = data.id
            msg['message_name'] = "UNKNOWN_MESSAGE"
            msg['raw_msg'] = str(data.data)
            print('KeyError: ' + str(msg))
        except ValueError:
            msg = {}
            msg['frame_id'] = data.id
            msg['message_name'] = "UNKNOWN_MESSAGE"
            msg['raw_msg'] = str(data.data)
            print('KeyError: ' + str(msg))


if __name__ == '__main__':
    rospy.init_node('CanFrameToRadarTargetConverter')
    fd = CanFrameToRadarTargetConverter()
    rospy.spin()
