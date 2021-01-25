#!/usr/bin/env python
"""
This script parses and converts Trimble GSOF messages incoming from a receiver and publishes the relevant ROS messages.
It  has been adapted from https://kb.unavco.org/kb/article/trimble-netr9-receiver-gsof-messages-806.html

@author: Michael Hutchinson (mhutchinson@sagarobotics.com)
"""


from struct import unpack
# from utilities import llh2xyz
from trimble_gnss_driver.conversions import llh2xyz
import socket
import math
import argparse
import rospy

class Gsof(object):
    """ A class to parse GSOF messages from a TCP stream. """

    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
        self.msg_dict = {}
        self.msg_bytes = None
        self.checksum = None
        self.rec_dict = {}

    def connect(self, host, port):
        self.sock.connect((host, port))

    def get_message_header(self):
        data = self.sock.recv(7)
        msg_field_names = ('STX', 'STATUS', 'TYPE', 'LENGTH',
                           'T_NUM', 'PAGE_INDEX', 'MAX_PAGE_INDEX')
        self.msg_dict = dict(zip(msg_field_names, unpack('>7B', data)))
        self.msg_bytes = self.sock.recv(self.msg_dict['LENGTH'] - 3)
        (checksum, etx) = unpack('>2B', self.sock.recv(2))

        def checksum256(st):
            """Calculate checksum"""
            return reduce(lambda x, y: x+y, map(ord, st)) % 256
        if checksum-checksum256(self.msg_bytes+data[1:]) == 0:
            self.checksum = True
        else:
            self.checksum = False

    def get_records(self):
        while len(self.msg_bytes) > 0:
            # READ THE FIRST TWO BYTES FROM RECORD HEADER
            record_type, record_length = unpack('>2B', self.msg_bytes[0:2])
            self.msg_bytes = self.msg_bytes[2:]
            self.select_record(record_type, record_length)

    def select_record(self, record_type, record_length):
        if record_type == 1:
            rec_field_names = ('GPS_TIME', 'GPS_WEEK', 'SVN_NUM',
                               'FLAG_1', 'FLAG_2', 'INIT_NUM')
            rec_values = unpack('>LH4B', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 2:
            rec_field_names = ('LATITUDE', 'LONGITUDE', 'HEIGHT')
            rec_values = unpack('>3d', self.msg_bytes[0:record_length])
            rec_values = list(rec_values)
            rec_values = (math.degrees(rec_values[0]), math.degrees(rec_values[1]), rec_values[2])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 3:
            rec_field_names = ('X_POS', 'Y_POS', 'Z_POS')
            rec_values = unpack('>3d', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 4:
            rec_field_names = ('LOCAL_DATUM_ID', 'LOCAL_DATUM_LAT',
                               'LOCAL_DATUM_LON', 'LOCAL_DATUM_HEIGHT', 'OPRT')
            rec_values = unpack('>8s3dB', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 5:
            rec_field_names = ('LOCAL_DATUM_ID', 'LOCAL_ZONE_ID',
                               'LOCAL_ZONE_NORTH', 'LOCAL_ZONE_EAST', 'LOCAL_DATUM_HEIGHT')
            rec_values = unpack('>2s3d', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 6:
            rec_field_names = ('DELTA_X', 'DELTA_Y', 'DELTA_Z')
            rec_values = unpack('>3d', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 7:
            rec_field_names = ('DELTA_EAST', 'DELTA_NORTH', 'DELTA_UP')
            rec_values = unpack('>3d', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 8:
            rec_field_names = ('VEL_FLAG', 'VELOCITY', 'HEADING', 'VERT_VELOCITY')
            rec_values = unpack('>B3f', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 9:
            rec_field_names = ('PDOP', 'HDOP', 'VDOP', 'TDOP')
            rec_values = unpack('>4f', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 10:
            rec_field_names = ('CLOCK_FLAG', 'CLOCK_OFFSET', 'FREQ_OFFSET')
            rec_values = unpack('>B2d', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 11:
            rec_field_names = ('POSITION_RMS_VCV', 'VCV_XX', 'VCV_XY', 'VCV_XZ',
                               'VCV_YY', 'VCV_YZ', 'VCV_ZZ', 'UNIT_VAR_VCV', 'NUM_EPOCHS_VCV')
            rec_values = unpack('>8fh', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 12:
            rec_field_names = ('POSITION_RMS_SIG', 'SIG_EAST', 'SIG_NORT', 'COVAR_EN', 'SIG_UP',
                               'SEMI_MAJOR', 'SEMI_MINOR', 'ORIENTATION', 'UNIT_VAR_SIG',
                               'NUM_EPOCHS_SIG')
            rec_values = unpack('>9fh', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 15:
            rec_field_names = 'SERIAL_NUM'
            rec_values = unpack('>l', self.msg_bytes[0:record_length])
            self.rec_dict.update({rec_field_names: rec_values[0]})
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 16:
            rec_field_names = ('GPS_MS_OF_WEEK', 'CT_GPS_WEEK', 'UTC_OFFSET', 'CT_FLAGS')
            rec_values = unpack('>l2hB', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 26:
            rec_field_names = ('UTC_MS_OF_WEEK', 'UTC_GPS_WEEK', 'UTC_SVS_NUM', 'UTC_FLAG_1', 'UTC_FLAG_2',
                               'UTC_INIT_NUM')
            rec_values = unpack('>lh4B', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 34:
            NUM_OF_SVS = unpack('>B', self.msg_bytes[0])
            self.msg_bytes = self.msg_bytes[1:]
            rec_field_names = ('PRN', 'SV_SYSTEM', 'SV_FLAG1', 'SV_FLAG2', 'ELEVATION', 'AZIMUTH',
                               'SNR_L1', 'SNR_L2', 'SNR_L5')
            for field in xrange(len(rec_field_names)):
                self.rec_dict[rec_field_names[field]] = []
            for sat in xrange(NUM_OF_SVS[0]):
                rec_values = unpack('>5Bh3B', self.msg_bytes[0:10])
                self.msg_bytes = self.msg_bytes[10:]
                for num in xrange(len(rec_field_names)):
                    self.rec_dict[rec_field_names[num]].append(rec_values[num])
        elif record_type == 37:
            rec_field_names = ('BATT_CAPACITY', 'REMAINING_MEM')
            rec_values = unpack('>hd', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        elif record_type == 41:
            rec_field_names = ('GPS_TIME_REF', 'GPS_WEEK_REF', 'LAT_REF', 'LONG_REF',
                               'HEIGHT_REF', "QI")
            rec_values = unpack('>LH3dB', self.msg_bytes[0:record_length])
            self.rec_dict.update(dict(zip(rec_field_names, rec_values)))
            self.msg_bytes = self.msg_bytes[record_length:]
        else:
            """Unknown record type? Skip it for now!"""
            #print record_type
            self.msg_bytes = self.msg_bytes[record_length:]


def main():
    rospy.init_node('trimble_gnss_driver')
    parser = argparse.ArgumentParser(prog='getGsofStream', usage='%(prog)s -i IP_ADDRESS -p PORT -r X Y Z ')
    parser.add_argument('-i', '--IP', action="store", default="69.44.87.212", type=str)
    parser.add_argument('-p', '--PORT', action="store", default=9999, type=int)
    parser.add_argument('-r', '--ref', nargs=3, action='append')
    results = parser.parse_args()
    # OPEN GSOF STREAM
    s = Gsof()
    s.connect(results.IP, results.PORT)
    try:
        # Use user provided reference coordinates if given
        ref_x, ref_y, ref_z = results.ref[0]
        print "# Reference specified, using X =%.3f Y=%.3f Z=%.3f" % (ref_x, ref_y, ref_z)
    except:
        # Get reference coordinates from first epoch if not provided by user
        s.get_message_header()
        s.get_records()
        ref_x, ref_y, ref_z = s.rec_dict['X_POS'], s.rec_dict['Y_POS'], s.rec_dict['Z_POS']
        print "# Reference not specified, using first epoch: X =%.4f Y=%.4f Z=%.4f" % (ref_x, ref_y, ref_z)
    print ""
    print "WN - GPS WEEK"
    print "SOW - GPS time"
    print "FLAG1 FLAG2" 
    print "SATS - # SVS"
    print "X[m] Y[m] Z[m]"
    print "eX[m] eY[m] eZ[m] - Position VCV Position RMS"
    print "E[m] N[m] U[m]"
    print ""
    print "# WN  SOW     FLAG1 FLAG2 SATS  X[m]          Y[m]            Z[m]       eX[m]   eY[m]     eZ[m]   E[m]   N[m]   U[m]"
    while 1:
        # READ GSOF STREAM
        s.get_message_header()
        s.get_records()
        # PRINT GSOF STREAM
        x = s.rec_dict['X_POS']
        y = s.rec_dict['Y_POS']
        z = s.rec_dict['Z_POS']
        neu = xyz2neu(ref_x, ref_y, ref_z, x, y, z)
        output = "%04d %.3f %3d %3d %2d %14.4f %14.4f %14.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f" % (
            s.rec_dict['GPS_WEEK'],
            s.rec_dict['GPS_TIME']/1000.0,
            s.rec_dict['FLAG_1'],
            s.rec_dict['FLAG_2'],
            s.rec_dict['SVN_NUM'],
            s.rec_dict['X_POS'],
            s.rec_dict['Y_POS'],
            s.rec_dict['Z_POS'],
            math.sqrt(s.rec_dict['VCV_XX']),
            math.sqrt(s.rec_dict['VCV_YY']),
            math.sqrt(s.rec_dict['VCV_ZZ']),
            neu[1],
            neu[0],
            neu[2],
        )
        print(output)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\n#: Stopping, you hit ctl+C. "
        #traceback.print_exc()