'''
Driver class for Hagisonic Stargazer, with no ROS dependancies. 
'''
from serial import Serial
from collections import deque
import re
import yaml
import time
import logging
import numpy as np
from threading import Thread, Event

import transformations as tr

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())

MAP_FILE = 'marker_map.yaml'

class StarGazer:
    def __init__(self, 
                 device          = '/dev/ttyUSB0', 
                 marker_map      = None,
                 callback_global = None,
                 callback_local  = None):
        '''
        Connect to a Hagisonic Stargazer device and get poses. 

        device:          The device location for the serial connection. 

        marker_map:      dictionary of marker transforms, formatted:
                         {marker_id: (4,4) matrix}
                         If not specified, defaults loaded from file. 

        callback_global: will be called whenever a new pose is recieved from the
                         Stargazer, will be called with (n,4,4) matrix of poses
                         of the location of the Stargazer in the global frame.
                         These are computed from marker_map. 

        callback_local: will be called whenever a new poses is recieved from the 
                        Stargazer, with a dict: {marker_id: [xyz, angle]}
        '''

        # connect to device with pyserial, baudrate is from hagisonic manual
        self.connection = Serial(port     = device, 
                                 baudrate = 115200,
                                 timeout  = 1.0)

        # chunk_size: how many charecters to read from the serial bus in
        # between checking the buffer for the STX/ETX charecters
        self._chunk_size = 80
        # STX: char that represents the start of a properly formed message
        self._STX = '~'
        # ETX: char that represents the end of a properly formed message
        self._ETX = '`'
        # DELIM: char that splits data
        self._DELIM = '|'
        # CMD: char that indicates command
        self._CMD = '#'

        # RESULT: char that indicates that the message contains result data
        self._RESULT = '^'

        # the fixed locations of the markers, which will be
        # used to back out the position of the stargazer
        if marker_map != None: self.marker_map = marker_map
        else:                  self.marker_map = yaml.load(open(MAP_FILE, 'rb'))
        
        self._callback_global =  callback_global
        self._callback_local  =  callback_local

        self._send_stargazer_startup()
        self._stopped = Event()
        self._thread  = Thread(target=self._read, args=()).start()

    def _send_stargazer_startup(self):
        startup = ('CalcStop',
                   'MarkDim|HLD1L',
                   'CalcStart')
        for command in startup:
            self.send_command(command)

    def send_command(self, *args):
        '''
        Send a command to the stargazer. 

        Arguments
        ---------
        command: string, or list. If string of single command, send just that.
                 if list, reformat to add delimeter charecter 

                 example: send_command('CalcStop')
                          send_command('MarkType', 'HLD1L')
        '''
        time.sleep(1)
        delimeted   = ''.join([str(i) + self._DELIM for i in args])[:-1]
        command_str = self._STX + self._CMD + delimeted + self._ETX
        log.info('Sending command to StarGazer: %s', command_str)
        self.connection.write(command_str)

    def _read(self):
        '''
        Read from the serial connection to the stargazer, process buffer,
        then execute callbacks. 
        '''
        def process_raw_pose(message):
            '''
            Turn a raw message into floating point arrays, and then
            execute callbacks with the new data
            '''
            # the first charecter of the message is the number of markers observed
            marker_count = int(message[1])
            #the rest of the message 
            raw_split = message[2:].split(self._DELIM)
            if len(raw_split) <> (marker_count*5):
                log.error('Message contained incorrect data length!: %s', message)
                return
            pose_local = dict()
            for split in np.reshape(raw_split, (-1,5)):
                marker_id        = int(split[0])
                # marker angle comes from stargazer in degrees
                # immediately converted to radians
                local_angle     = np.radians(float(split[1]))
                # marker cartesian pose comes from stargazer in cm
                # immediately converted to meters
                local_cartesian       = (np.array(split[2:]).astype(float)*.01).tolist()
                pose_local[marker_id] = fourdof_to_matrix(local_cartesian, local_angle)

            if self._callback_global:
                global_pose = local_to_global(self.marker_map, pose_local)
                self._callback_global(global_pose)
            if self._callback_local:
                self._callback_local(pose_local)
            
        def process_buffer(message_buffer):
            '''
            Looks at current message_buffer string for _STX and _ETX chars
            Proper behavior is to process string found between STX/ETX for poses
            and remove everything in the buffer up the last observed ETX
            '''
            for candidate in matcher.findall(message_buffer):
                #candidate still has _ETX char on the end from the regex
                process_raw_pose(candidate[:-1])
            # nuke everything in the buffer before last _ETX as it is either 
            # processed or garbage
            if self._ETX in message_buffer:
                return message_buffer[message_buffer.rindex(self._ETX)+1:]
            return message_buffer
        pattern        = '(?<=' + self._STX + ').+?' + self._ETX
        matcher        = re.compile(pattern)
        message_buffer = '' 
   
        while not self._stopped.is_set():
            try:
                message_buffer += self.connection.read(self._chunk_size)
                message_buffer  = process_buffer(message_buffer)
            except:
                log.error('Error processing current buffer: %s', 
                          message_buffer, 
                          exc_info=True)
                message_buffer  = ''

    def close(self):
        self._stopped.set()
        self.send_command('CalcStop')
        self.connection.close()
                
    def __del__(self):
        self.close()

def local_to_global(marker_map, local_poses):
    '''
    Transform local marker coordinates to map coordinates.
    '''
    global_poses = dict()
    for marker_id, pose in local_poses.iteritems():
        if not marker_id in marker_map:
            log.warn('Marker ID %i isn\'t in map!')
            continue
        marker_to_map   = marker_map[marker_id]
        local_to_marker = np.linalg.inv(pose)
        local_to_map    = np.dot(marker_to_map, local_to_marker)
        global_poses[marker_id] = local_to_map    
    return global_poses

def fourdof_to_matrix(cartesian, angle):
    T        = tr.rotation_matrix(angle, [0,0,1])
    T[0:3,3] = cartesian
    return T

def _callback_dummy(data): 
    return

def _callback_print(data):
    print(data)

if __name__ == '__main__':
    formatter = logging.Formatter("[%(asctime)s] %(levelname)-7s \
(%(filename)s:%(lineno)3s) %(message)s", "%Y-%m-%d %H:%M:%S")
    handler_stream = logging.StreamHandler()
    handler_stream.setFormatter(formatter)
    level = logging.DEBUG
    handler_stream.setLevel(level)
    log.addHandler(handler_stream)
    log.setLevel(level)

    s = StarGazer(callback_global=_callback_print)