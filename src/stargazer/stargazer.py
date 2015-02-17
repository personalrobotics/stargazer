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


class StarGazer(object):
    def __init__(self, device, marker_map, callback_global=None, callback_local=None):
        """
        Connect to a Hagisonic Stargazer device and receive poses. 

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
        if self.connection:
            self.disconnect()

    def connect(self):
        """ Connect to the Stargazer over the specified RS-232 port.
        """
        assert not self.connection
        self.connection = Serial(port=self.device, baudrate=115200, timeout=1.0)

    def disconnect(self):
        """ Disconnects from the Stargazer and closes the RS-232 port.
        """
        assert self.connection
        self.connection.close()
        self.connection = None

    def start_streaming(self):
        assert self._thread is None

        self.send_command('CalcStart')

        self._thread = Thread(target=self._read, args=()).start()

    def stop_streaming(self):
        if self._thread is not None:
            self._stopped.set()
            self._thread.join()

        self.send_command('CalcStop')

    def set_parameter(self, name, value):
        self.send_command(name, value)

        # TODO: Wait for the response.

    def send_command(self, *args):
        """
        Send a command to the stargazer. 

        Arguments
        ---------
        command: string, or list. If string of single command, send just that.
                 if list, reformat to add delimiter character 

                 example: send_command('CalcStop')
                          send_command('MarkType', 'HLD1L')
        """
        delimited   = ''.join([str(i) + DELIM for i in args])[:-1]
        command_str = STX + CMD + delimited + ETX
        rospy.loginfo('Sending command to StarGazer: %s', command_str)

        # The StarGazer requires a 50 ms delay between each byte.
        for ch in command_str:
            self.connection.write(ch)
            time.sleep(0.05)

        # Wait for a response.
        response_expected = STX + RESPONSE + delimited + ETX
        response_actual = self.connection.read(len(response_expected))

        if response_actual != response_expected:
            raise Exception(
                'Command "{:s}" received invalid response "{:s}"; expected "{:s}".'\
                .format(command_str, response_actual, response_expected)
            )

    def _read(self):
        """
        Read from the serial connection to the stargazer, process buffer,
        then execute callbacks. 
        """
        def process_raw_pose(message):
            """
            Turn a raw message into floating point arrays, and then
            execute callbacks with the new data
            """
            # the first character of the message is the number of markers observed
            marker_count = int(message[1])
            #the rest of the message 
            raw_split = message[2:].split(DELIM)
            if len(raw_split) != (marker_count*5):
                rospy.logerr('Message contained incorrect data length!: %s', message)
                return

            pose_local = dict()
            for split in np.reshape(raw_split, (-1,5)):
                marker_id        = int(split[0])
                # marker angle comes from stargazer in degrees
                # immediately converted to radians
                local_angle     = np.radians(float(split[1]))
                # marker Cartesian pose comes from stargazer in cm
                # immediately converted to meters
                local_cartesian       = (np.array(split[2:]).astype(float)*.01).tolist()
                #rospy.logerr('%d %f %f %f %f', marker_id, local_cartesian[0], local_cartesian[1], local_cartesian[2], local_angle)
                local_cartesian[2] = -local_cartesian[2]
                marker_to_stargazer = fourdof_to_matrix(local_cartesian, -local_angle)

                pose_local[marker_id] = np.linalg.inv(marker_to_stargazer)

            if self._callback_global:
                global_pose, unknown_ids = local_to_global(self.marker_map, pose_local)
                self._callback_global(global_pose, unknown_ids)

            if self._callback_local:
                self._callback_local(pose_local)

        def process_buffer(message_buffer):
            """
            Looks at current message_buffer string for _STX and _ETX chars
            Proper behavior is to process string found between STX/ETX for poses
            and remove everything in the buffer up the last observed ETX
            """
            for candidate in matcher.findall(message_buffer):
                #candidate still has _ETX char on the end from the regex
                process_raw_pose(candidate[:-1])
            # nuke everything in the buffer before last _ETX as it is either 
            # processed or garbage
            if ETX in message_buffer:
                return message_buffer[message_buffer.rindex(ETX)+1:]
            else:
                return message_buffer

        pattern        = '(?<=' + STX + ').+?' + ETX
        matcher        = re.compile(pattern)
        message_buffer = '' 

        rospy.loginfo('Entering read loop.')
   
        while not self._stopped.is_set():
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
        self.send_command('CalcStop')
        self.connection.close()
                
def local_to_global(marker_map, local_poses):
    """
    Transform local marker coordinates to map coordinates.
    """
    global_poses = dict()
    unknown_ids = set()

    for marker_id, pose in local_poses.iteritems():
        # Marker IDs might be accidentally passed as integer.
        marker_id = str(marker_id)

        if marker_id in marker_map:
            marker_to_map   = marker_map[marker_id]
            local_to_marker = np.linalg.inv(pose)
            local_to_map    = np.dot(marker_to_map, local_to_marker)
            global_poses[marker_id] = local_to_map    
        else:
            unknown_ids.add(marker_id)

    return global_poses, unknown_ids

def fourdof_to_matrix(cartesian, angle):
    T        = transformations.rotation_matrix(angle, [0,0,1])
    T[0:3,3] = cartesian
    return T

def _callback_dummy(data): 
    return

def _callback_print(data):
    print(data)
