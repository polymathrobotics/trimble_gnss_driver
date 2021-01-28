#!/usr/bin/env python
"""
This script parses and converts Trimble GSOF messages incoming from a receiver and publishes the relevant ROS messages.
It  has been adapted from https://kb.unavco.org/kb/article/trimble-netr9-receiver-gsof-messages-806.html

@author: Michael Hutchinson (mhutchinson@sagarobotics.com)
"""


from struct import unpack
from trimble_gnss_driver.conversions import *
from trimble_gnss_driver.parser import parse_maps
import socket
import sys
import math
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped # For orientation
from sensor_msgs.msg import NavSatFix, NavSatStatus # For lat lon h
from tf.transformations import quaternion_from_euler


"""
GSOF messages from https://www.trimble.com/OEM_ReceiverHelp/#GSOFmessages_Overview.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257COverview%257C_____0
"""
# Records we most care about
ATTITUDE = 27          # Attitude information with errors
LAT_LON_H = 2          # Position lat lon h
POSITION_SIGMA = 12    # Errors in position
INS_FULL_NAV = 49      # INS fused full nav info pose, attittude etc
INS_RMS = 50           # RMS errors from reported fused position

# Others
VELOCITY = 8
SERIAL_NUM = 15
GPS_TIME = 1
UTC_TIME = 16
ECEF_POS = 3
BASE_POSITION = 41


class GsofDriver(object):
    """ A class to parse GSOF messages from a TCP stream. """

    def __init__(self):

        rospy.init_node('trimble_gnss_driver')

        port = rospy.get_param('~rtk_port', 21098)
        ip = rospy.get_param('~rtk_ip','192.168.0.50')

        self.gps_main_frame_id = rospy.get_param('~gps_main_frame_id', 'gps_link')
        self.base_frame = rospy.get_param("base_frame", "base_link")

        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
        # For attitude, use Pose type to avoid confusion that data might be from
        # IMU only but at the same time keep compatible with robot_localization
        self.attitude_pub = rospy.Publisher('attitude', PoseWithCovarianceStamped, queue_size=1)

        self.client = self.setup_connection(ip , port)

        self.msg_dict = {}
        self.msg_bytes = None
        self.checksum = None
        self.rec_dict = {}

        self.ins_rms_ts = rospy.Time()
        self.ins_rms_timeout = 0.5

        self.gps_qualities = {
            # Unknown
            -1: [
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # PPS
            3: [
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # Estimated (dead reckoning) mode
            6: [
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # Manual input mode
            7: [
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

        while not rospy.is_shutdown():
            # READ GSOF STREAM
            self.records = []
            self.get_message_header()
            self.get_records()

            # Need to change this cos max output rate of INS_RMS is lower
            if all(ins_records in self.records for ins_records in [INS_FULL_NAV, INS_RMS]):
                # print "Full INS info, filling ROS messages"
                self.send_ins_fix()
                self.send_ins_attitude()
            elif INS_FULL_NAV in self.records and rospy.get_time() - self.ins_rms_ts.to_sec() < self.ins_rms_timeout:
                self.send_ins_fix()
                self.send_ins_attitude()


    def send_ins_fix(self):
        current_time = rospy.get_rostime() # Replace with GPS time?
        fix = NavSatFix()

        fix.header.stamp = current_time
        fix.header.frame_id = self.gps_main_frame_id

        gps_qual = self.gps_qualities[self.rec_dict['GPS_QUALITY']]
        fix.status.service = NavSatStatus.SERVICE_GPS # TODO: Fill correctly
        fix.status.status = gps_qual[0]
        fix.position_covariance_type = gps_qual[1]

        fix.latitude = self.rec_dict['LATITUDE_INS']
        fix.longitude = self.rec_dict['LONGITUDE_INS']
        fix.altitude = self.rec_dict['ALTITUDE_INS']

        fix.position_covariance[0] = self.rec_dict['RMS_LATITUDE'] ** 2
        fix.position_covariance[4] = self.rec_dict['RMS_LONGITUDE'] ** 2
        fix.position_covariance[8] = self.rec_dict['RMS_ALTITUDE'] ** 2

        self.fix_pub.publish(fix)


    def send_ins_attitude(self):
        """
        We send the GNSS fused attitude information as a PoseWithCovarianceStamped
        msg to avoid confusion that it might come purely from an IMU
        """
        current_time = rospy.get_rostime() # Replace with GPS time?
        self.ins_rms_ts = rospy.Time.now()

        # print "current time: ", current_time, "INS time: ", self.ins_rms_ts
        attitude = PoseWithCovarianceStamped()

        attitude.header.stamp = current_time
        attitude.header.frame_id = self.base_frame  # Assume transformation handled by receiver

        heading_enu = 360 - self.normalize_angle(self.rec_dict['YAW'] + 270)
        quaternion = quaternion_from_euler(math.radians(self.rec_dict['ROLL']),     #  roll sign stays the same
                                           - math.radians(self.rec_dict['PITCH']),  # -ve for robots coord system (+ve down)
                                           math.radians(heading_enu))
        print 'r p y receiver_heading [degs]: ', self.rec_dict['ROLL'], self.rec_dict['PITCH'], heading_enu, self.rec_dict['YAW']
        attitude.pose.pose.orientation.x = quaternion[0]
        attitude.pose.pose.orientation.y = quaternion[1]
        attitude.pose.pose.orientation.z = quaternion[2]
        attitude.pose.pose.orientation.w = quaternion[3]

        attitude.pose.covariance[23] = math.radians(self.rec_dict['RMS_ROLL']) ** 2  # [36] size array
        attitude.pose.covariance[29] = math.radians(self.rec_dict['RMS_PITCH']) ** 2  # [36] size array
        attitude.pose.covariance[35] = math.radians(self.rec_dict['RMS_YAW']) ** 2 # [36] size array

        self.attitude_pub.publish(attitude)


    @staticmethod
    def normalize_angle(angle_in):
        while angle_in > 360:
            angle_in = angle_in - 360
        while angle_in < 0:
            angle_in = angle_in + 360
        return angle_in


    def setup_connection(self, _ip, _port):
        port = _port
        ip = None
        attempts_limit = 10
        current_attempt = 0
        connected = False

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        while not connected and not rospy.is_shutdown() and current_attempt < attempts_limit:
            current_attempt += 1

            try:
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.settimeout(5)
                ip = socket.gethostbyname(_ip)
                address = (ip, port)

                rospy.loginfo("Attempting connection to %s:%s ", ip, port)
                client.connect(address)

                rospy.loginfo("=====================================")
                rospy.loginfo("Connected to %s:%s ", ip, port)
                rospy.loginfo("=====================================")
                connected = True
            except Exception as e:
                rospy.logwarn("Connection to IP: " + ip + ": " + e.__str__() +
                              ".\nRetrying connection: Attempt: %s/%s",
                              current_attempt, attempts_limit)

        if not connected:
            rospy.logerr("No connection established. Node shutting down")
            sys.exit()

        return client


    def get_message_header(self):
        data = self.client.recv(7)
        msg_field_names = ('STX', 'STATUS', 'TYPE', 'LENGTH',
                           'T_NUM', 'PAGE_INDEX', 'MAX_PAGE_INDEX')
        self.msg_dict = dict(zip(msg_field_names, unpack('>7B', data)))
        # print "msg dict: ", self.msg_dict
        self.msg_bytes = self.client.recv(self.msg_dict['LENGTH'] - 3)
        (checksum, etx) = unpack('>2B', self.client.recv(2))

        def checksum256(st):
            """Calculate checksum"""
            return reduce(lambda x, y: x+y, map(ord, st)) % 256
        if checksum-checksum256(self.msg_bytes+data[1:]) == 0:
            self.checksum = True
        else:
            self.checksum = False


    def get_records(self):
        self.byte_position = 0
        while self.byte_position < len(self.msg_bytes):
            # READ THE FIRST TWO BYTES FROM RECORD HEADER
            record_type, record_length = unpack('>2B', self.msg_bytes[self.byte_position:self.byte_position + 2])
            self.byte_position += 2
            self.records.append(record_type)
            # print "Record type: ", record_type, " Length: ", record_length
            # self.select_record(record_type, record_length)
            if record_type in parse_maps:
                self.rec_dict.update(dict(zip(parse_maps[record_type][0], unpack(parse_maps[record_type][1], self.msg_bytes[self.byte_position:self.byte_position + record_length]))))
            else:
                rospy.logwarn("Record type %s is not in parse maps.", record_type)
            self.byte_position += record_length



if __name__ == '__main__':
    try:
        GsofDriver()
    except rospy.ROSInterruptException:
        pass
