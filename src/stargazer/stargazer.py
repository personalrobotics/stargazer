"""
Driver class for Hagisonic Stargazer, with no ROS dependencies. 
"""
from serial import Serial
from collections import deque
import re
import yaml
import time
import logging
import rospy
import numpy as np
from threading import Thread, Event
from tf import transformations

# STX: char that represents the start of a properly formed message
STX = '~'
# ETX: char that represents the end of a properly formed message
ETX = '`'
# DELIM: char that splits data
DELIM = '|'
# CMD: char that indicates command
CMD = '#'
# CMD: char that indicates command
RESPONSE = '!'
# RESULT: char that indicates that the message contains result data
RESULT = '^'
# NOTIFY: char that indicates a notification message of some kind
NOTIFY = '*'

class StarGazer(object):
    def __init__(self, device, marker_map, callback_global=None, callback_local=None):
        """
        Connect to a Hagisonic StarGazer device and receive poses.

        device:          The device location for the serial connection. 

        marker_map:      dictionary of marker transforms, formatted:
                         {marker_id: (4,4) matrix}

        callback_global: will be called whenever a new pose is received from the
                         Stargazer, will be called with (n,4,4) matrix of poses
                         of the location of the Stargazer in the global frame.
                         These are computed from marker_map. 

        callback_local: will be called whenever a new poses is received from the 
                        Stargazer, with a dict: {marker_id: [xyz, angle]}
        """
        self.device = device
        self.marker_map = marker_map
        self.connection = None

        # chunk_size: how many characters to read from the serial bus in
        # between checking the buffer for the STX/ETX characters
        self._chunk_size = 80

        self._callback_global =  callback_global
        self._callback_local  =  callback_local

        self._stopped = Event()
        self._thread = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, type, value, traceback):
        if self.is_connected:
            self.disconnect()

    @property
    def is_connected(self):
        """
        Returns whether the driver is currently connected to a serial port.
        """
        return self.connection is not None

    def connect(self):
        """
        Connect to the StarGazer over the specified RS-232 port.
        """
        assert not self.is_connected
        self.connection = Serial(port=self.device, baudrate=115200, timeout=1.0)

    def disconnect(self):
        """
        Disconnects from the StarGazer and closes the RS-232 port.
        """
        if self.is_connected:
            self.connection.close()
            self.connection = None

    @property
    def is_streaming(self):
        """
        Returns whether the driver is currently streaming pose data. 
        """
        return self._thread is not None

    def start_streaming(self):
        """
        Begin streaming pose data from the StarGazer.
        """
        assert self.is_connected and not self.is_streaming
        self._send_command('CalcStart')
        self._thread = Thread(target=self._read, args=()).start()

    def stop_streaming(self):
        """
        Stop streaming pose data from the StarGazer.
        """
        assert self.is_connected
        if self.is_streaming:
            self._stopped.set()
            self._thread.join()
        self._send_command('CalcStop')

    def set_parameter(self, name, value):
        """
        Set a StarGazer configuration parameter.

        This function can only be called while the StarGazer is
        connected, but not streaming.

        Arguments
        ---------
        name:  string name of the parameter to set
        value: string value of the parameter to set

        Example
        -------
            set_parameter('MarkType', 'HLD1L')
        """
        assert self.is_connected and not self.is_streaming
        self._send_command(name, value)

    def _send_command(self, *args):
        """
        Send a command to the StarGazer.

        Arguments
        ---------
        command: string, or list. If string of single command, send just that.
                 if list, reformat to add delimiter character 

        Example
        -------
            _send_command('CalcStop')
            _send_command('MarkType', 'HLD1L')
        """
        delimited   = DELIM.join(str(i) for i in args)
        command_str = STX + CMD + delimited + ETX
        rospy.loginfo('Sending command to StarGazer: %s', command_str)

        # The StarGazer requires a 50 ms delay between each byte.
        for ch in command_str:
            self.connection.write(ch)
            time.sleep(0.05)

        # Wait for a response.
        response_expected = STX + RESPONSE + delimited + ETX
        response_actual = self.connection.read(len(response_expected))
        
        # Scan for more incoming characters until we get a read timeout.
        # (This is useful if there is still some incoming data from previous
        # commands in intermediate serial buffers.)
        while response_actual[-len(response_expected):] != response_expected:
            c = self.connection.read()

            if c:
                # Add new characters to the response string.
                response_actual += c
            else:
                # If we run out of characters and still don't match, report
                # the invalid response as an exception.
                raise Exception(
                    'Command "{:s}" received invalid response "{:s}"; '
                    'expected "{:s}".'
                    .format(command_str, response_actual, response_expected)
                )

    def _read(self):
        """
        Read from the serial connection to the StarGazer, process buffer,
        then execute callbacks. 
        """
        # Compute a regular expression that returns the last valid
        # message in a StarGazer stream.
        msg_pattern = ('.*' + STX + '(?P<type>.)(?P<payload>.+)' + ETX +
                       '(?P<remainder>.*)$')
        msg_matcher = re.compile(msg_pattern)

        # Compute a regular expression that converts a StarGazer message
        # into a list of tuples containing parsed groups.
        delimiter = '\\' + DELIM
        number = '[\d\+\-\.]'
        tag_pattern = (r'(?P<id>\d+)' + delimiter +
                       r'(?P<yaw>' + number + '+)' + delimiter +
                       r'(?P<x>' + number + '+)' + delimiter +
                       r'(?P<y>' + number + '+)' + delimiter +
                       r'(?P<z>' + number + '+)')
        tag_matcher = re.compile(tag_pattern)


        def process_buffer(message_buffer):
            """
            Looks at current message_buffer string for STX and ETX chars.

            Proper behavior is to process string found between STX/ETX for poses
            and remove everything in the buffer up the last observed ETX.

            Valid readings:
                ~^148|-175.91|+98.74|+7.10|182.39`
                ~^248|-176.67|+98.38|+8.39|181.91|370|-178.41|-37.05|+8.97|179.51`

            No valid readings:
                ~*DeadZone`
            """
            # Look for a matching message, return the buffer if none are found.
            message = msg_matcher.match(message_buffer)
            if not message:
                return message_buffer

            if message.group('type') == RESULT:
                markers = tag_matcher.finditer(message.group('payload'))

                local_poses = {}
                for marker in markers:
                    # Parse pose information for this marker.
                    _id = marker.group('id')
                    yaw = -np.radians(float(marker.group('yaw')))
                    x = 0.01 * float(marker.group('x'))
                    y = 0.01 * float(marker.group('y'))
                    # Note: this axis is negated.
                    z = -0.01 * float(marker.group('z'))

                    # Convert the pose to a transform and store it by ID.
                    marker_to_stargazer = fourdof_to_matrix((x, y, z), yaw)
                    local_poses[_id] = np.linalg.inv(marker_to_stargazer)

                if self._callback_local:
                    self._callback_local(local_poses)

                if self._callback_global:
                    global_poses, unknown_ids = local_to_global(self.marker_map,
                                                                local_poses)
                    self._callback_global(global_poses, unknown_ids)

            elif message.group('type') == NOTIFY:
                # TODO: Report deadzone messages in here!
                pass
            else:
                pass

            # Return the rest of the message buffer.
            return message.group('remainder')

        rospy.loginfo('Entering read loop.')

        message_buffer = ''
        while not self._stopped.is_set() and self.connection:
            try:
                message_buffer += self.connection.read(self._chunk_size)
                message_buffer  = process_buffer(message_buffer)
            except Exception as e:
                rospy.logerr('Error processing current buffer: %s (content: "%s")', 
                    str(e), message_buffer
                )
                message_buffer  = ''

                break # For debugging purposes.

        rospy.loginfo('Exited read loop.')

    def close(self):
        self._stopped.set()
        self._send_command('CalcStop')
        self.connection.close()
                
def local_to_global(marker_map, local_poses):
    """
    Transform local marker coordinates to map coordinates.
    """
    global_poses = dict()
    unknown_ids = set()

    for _id, pose in local_poses.iteritems():
        if _id in marker_map:
            marker_to_map   = marker_map[_id]
            local_to_marker = np.linalg.inv(pose)
            local_to_map    = np.dot(marker_to_map, local_to_marker)
            global_poses[_id] = local_to_map
        else:
            unknown_ids.add(_id)

    return global_poses, unknown_ids

def fourdof_to_matrix(translation, yaw):
    """
    Convert from a Cartesian translation and yaw to a homogeneous transform.
    """
    T        = transformations.rotation_matrix(yaw, [0,0,1])
    T[0:3,3] = translation
    return T

def _callback_dummy(data): 
    return

def _callback_print(data):
    print(data)
