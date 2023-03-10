'''Simple and lightweight module for working with RPLidar rangefinder scanners.

Usage example:

>>> from rplidar import RPLidar
>>> lidar = RPLidar('/dev/ttyUSB0')
>>> 
>>> info = lidar.get_info()
>>> print(info)
>>> 
>>> health = lidar.get_health()
>>> print(health)
>>> 
>>> for i, scan in enumerate(lidar.iter_scans()):
...  print('%d: Got %d measurments' % (i, len(scan)))
...  if i > 10:
...   break
...
>>> lidar.stop()
>>> lidar.stop_motor()
>>> lidar.disconnect()

For additional information please refer to the RPLidar class documentation.
'''
import logging
import sys
import time
import codecs
import serial
import struct


# Message bytes from:
# https://bucket-download.slamtec.com/6494fd238cf5e0d881f56d914c6d1f355c0f582a/LR001_SLAMTEC_rplidar_protocol_v2.4_en.pdf
SYNC_BYTE = b'\xA5'
SYNC_BYTE2 = b'\x5A'

GET_INFO_BYTE = b'\x50'
GET_HEALTH_BYTE = b'\x52'

STOP_BYTE = b'\x25'
RESET_BYTE = b'\x40'

SCAN_BYTE = b'\x20'
FORCE_SCAN_BYTE = b'\x21'

DESCRIPTOR_LEN = 7
INFO_LEN = 20
HEALTH_LEN = 3

INFO_TYPE = 4
HEALTH_TYPE = 6
SCAN_TYPE = 129

#Constants & Command to start A2 motor
MAX_MOTOR_PWM = 1023
DEFAULT_MOTOR_PWM = 660
SET_PWM_BYTE = b'\xF0'

_HEALTH_STATUSES = {
    0: 'Good',
    1: 'Warning',
    2: 'Error',
}


class RPLidarException(Exception):
    '''Basic exception class for RPLidar'''


def _process_scan(raw):
    """Processes input raw data and returns measurement data"""
    new_scan = bool(raw[0] & 0b1)
    inversed_new_scan = bool((raw[0] >> 1) & 0b1)
    quality = raw[0] >> 2
    if new_scan == inversed_new_scan:
        raise RPLidarException('New scan flags mismatch')
    check_bit = raw[1] & 0b1
    if check_bit != 1:
        raise RPLidarException('Check bit not equal to 1')
    angle = ((raw[1] >> 1) + (raw[2] << 7)) / 64.
    distance = (raw[3] + (raw[4] << 8)) / 4.
    return new_scan, quality, angle, distance


class RPLidar(object):
    """Class for communicating with RPLidar rangefinder scanners"""

    _serial = None  #: serial port connection
    port = ''  #: Serial port name, e.g. /dev/ttyUSB0
    timeout = 1  #: Serial port timeout
    motor = False  #: Is motor running?
    baudrate = 115200  #: Baudrate for serial port

    def __init__(self, port, baudrate=115200, timeout=1, logger=None):
        '''Initilize RPLidar object for communicating with the sensor.

        Parameters
        ----------
        port : str
            Serial port name to which sensor is connected
        baudrate : int, optional
            Baudrate for serial connection (the default is 115200)
        timeout : float, optional
            Serial port connection timeout in seconds (the default is 1)
        logger : logging.Logger instance, optional
            Logger instance, if none is provided new instance is created
        '''
        self._serial = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.motor_running = None
        if logger is None:
            logger = logging.getLogger('rplidar')
        self.logger = logger
        self.connect()
        self.start_motor()

    def connect(self):
        '''Connects to the serial port with the name `self.port`. If it was
        connected to another serial port disconnects from it first.'''
        if self._serial is not None:
            self.disconnect()
        try:
            self._serial = serial.Serial(
                self.port, self.baudrate,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout, dsrdtr=True)
        except serial.SerialException as err:
            raise RPLidarException('Failed to connect to the sensor '
                                   'due to: %s' % err)

    def disconnect(self):
        '''Disconnects from the serial port'''
        if self._serial is None:
            return
        self._serial.close()

    def set_pwm(self, pwm):
        assert(0 <= pwm <= MAX_MOTOR_PWM)
        payload = struct.pack("<H", pwm)
        self._send_payload_cmd(SET_PWM_BYTE, payload)

    def start_motor(self, pwm=DEFAULT_MOTOR_PWM):
        '''Starts sensor motor'''
        self.logger.info('Starting motor')
        # For A1
        self._serial.dtr = False

        # For A2
        self.set_pwm(pwm)
        self.motor_running = True

    def stop_motor(self):
        '''Stops sensor motor'''
        self.logger.info('Stoping motor')
        # For A2
        self.set_pwm(0)
        time.sleep(.001)
        # For A1
        self._serial.dtr = True
        self.motor_running = False

    def _send_payload_cmd(self, cmd, payload):
        '''Sends `cmd` command with `payload` to the sensor'''
        size = struct.pack('B', len(payload))
        req = SYNC_BYTE + cmd + size + payload
        checksum = 0
        for v in struct.unpack('B'*len(req), req):
            checksum ^= v
        req += struct.pack('B', checksum)
        self._serial.write(req)
        self.logger.debug('Command sent: %s' % req)

    def _send_cmd(self, cmd):
        '''Sends `cmd` command to the sensor'''
        req = SYNC_BYTE + cmd
        self._serial.write(req)
        self.logger.debug('Command sent: %s' % req)

    def _read_descriptor(self):
        '''Reads descriptor packet'''
        descriptor = self._serial.read(DESCRIPTOR_LEN)
        self.logger.debug('Received descriptor: %s', descriptor)
        if len(descriptor) != DESCRIPTOR_LEN:
            raise RPLidarException('Descriptor length mismatch')
        elif not descriptor.startswith(SYNC_BYTE + SYNC_BYTE2):
            raise RPLidarException('Incorrect descriptor starting bytes')
        is_single = descriptor[-2] == 0
        return descriptor[2], is_single, descriptor[-1]

    def _read_response(self, dsize):
        '''Reads response packet with length of `dsize` bytes'''
        self.logger.debug('Trying to read response: %d bytes', dsize)
        data = self._serial.read(dsize)
        self.logger.debug('Received data: %s', data)
        if len(data) != dsize:
            self.logger.debug('Wrong body size')
        return data

    def get_info(self):
        '''Get device information

        Returns
        -------
        dict
            Dictionary with the sensor information
        '''
        self._send_cmd(GET_INFO_BYTE)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != INFO_LEN:
            raise RPLidarException('Wrong get_info reply length')
        if not is_single:
            raise RPLidarException('Not a single response mode')
        if dtype != INFO_TYPE:
            raise RPLidarException('Wrong response data type')
        raw = self._read_response(dsize)
        serialnumber = codecs.encode(raw[4:], 'hex').upper()
        serialnumber = codecs.decode(serialnumber, 'ascii')
        data = {
            'model': raw[0],
            'firmware': (raw[2], raw[1]),
            'hardware': raw[3],
            'serialnumber': serialnumber,
        }
        return data

    def get_health(self):
        """Get device health state. When the core system detects some
        potential risk that may cause hardware failure in the future,
        the returned status value will be 'Warning'. But sensor can still work
        as normal. When sensor is in the Protection Stop state, the returned
        status value will be 'Error'. In case of warning or error statuses
        non-zero error code will be returned.

        Returns
        -------
        status : str
            'Good', 'Warning' or 'Error' statuses
        error_code : int
            The related error code that caused a warning/error.
        """
        self._send_cmd(GET_HEALTH_BYTE)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != HEALTH_LEN:
            raise RPLidarException('Wrong get_info reply length')
        if not is_single:
            raise RPLidarException('Not a single response mode')
        if dtype != HEALTH_TYPE:
            raise RPLidarException('Wrong response data type')
        raw = self._read_response(dsize)
        status = _HEALTH_STATUSES[raw[0]]
        error_code = (raw[1] << 8) + raw[2]
        return status, error_code

    def clear_input(self):
        """Clear input buffer by reading all available data"""
        self._serial.read_all()

    def stop(self):
        """Stop scanning process.

        This command disables laser diode and measurement system, and moves
        sensor to the idle state.
        """
        self.logger.info('Stoping scanning')
        self._send_cmd(STOP_BYTE)
        # "Since RPLIDAR won’t send response packet for this request, host
        # systems should wait for at least 1 milliseconds (ms) before sending
        # another request."
        time.sleep(.001)
        self.clear_input()

    def reset(self):
        """Reset sensor core.

        This reverts it to a similar state as when it has just been powered up.
        """
        self.logger.info('Resetting the sensor')
        self._send_cmd(RESET_BYTE)
        # "Since RPLIDAR won’t send response packet for this request, host
        # systems should wait for at least 2 milliseconds (ms) before sending
        # another request."
        time.sleep(.002)

    def iter_measurements(self, max_buf_meas=500):
        """Iterate over measurements.

        Note that consumer must be fast enough, otherwise data will be
        accumulated inside USB buffer and consumer will get data with
        increasing lag.

        Parameters
        ----------
        max_buf_meas : int
            Maximum number of measurements to be stored inside the buffer. Once
            number exceeds this limit buffer will be emptied out.

        Yields
        ------
        new_scan : bool
            True if measurement belongs to a new scan
        quality : int
            Reflected laser pulse strength
        angle : float
            The measurement heading angle in degree unit [0, 360)
        distance : float
            Measured object distance related to the sensor's rotation center.
            In millimeter unit. Set to 0 when measurement is invalid.
        """
        self.start_motor()
        status, error_code = self.get_health()
        self.logger.debug('Health status: %s [%d]', status, error_code)
        if status == _HEALTH_STATUSES[2]:
            self.logger.warning('Trying to reset sensor due to the error. '
                                'Error code: %d', error_code)
            self.reset()
            status, error_code = self.get_health()
            if status == _HEALTH_STATUSES[2]:
                raise RPLidarException('RPLidar hardware failure. '
                                       'Error code: %d' % error_code)
        elif status == _HEALTH_STATUSES[1]:
            self.logger.warning('Warning sensor status detected! '
                                'Error code: %d', error_code)
        cmd = SCAN_BYTE
        self._send_cmd(cmd)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != 5:
            raise RPLidarException('Wrong get_info reply length')
        if is_single:
            raise RPLidarException('Not a multiple response mode')
        if dtype != SCAN_TYPE:
            raise RPLidarException('Wrong response data type')
        while True:
            raw = self._read_response(dsize)
            self.logger.debug('Received scan response: %s' % raw)
            if len(raw) == 0:
                continue
            if max_buf_meas:
                data_in_buf = self._serial.in_waiting
                if data_in_buf > max_buf_meas*dsize:
                    self.logger.warning(
                        'Too many measurments in the input buffer: %d/%d. '
                        'Clearing buffer...',
                        data_in_buf//dsize, max_buf_meas)
                    self._serial.read(data_in_buf // dsize * dsize)
            yield _process_scan(raw)

    def iter_scans(self, max_buf_meas=500, min_len=5):
        '''Iterate over scans. Note that consumer must be fast enough,
        otherwise data will be accumulated inside buffer and consumer will get
        data with increasing lag.

        Parameters
        ----------
        max_buf_meas : int
            Maximum number of measurements to be stored inside the buffer. Once
            number exceeds this limit buffer will be emptied out.
        min_len : int
            Minimum number of measurements in the scan for it to be yielded.

        Yields
        ------
        scan : list
            List of the measurements. Each measurement is tuple with following
            format: (quality, angle, distance). For values description please
            refer to `iter_measurements` method's documentation.
        '''
        scan = []
        iterator = self.iter_measurements(max_buf_meas)
        for new_scan, quality, angle, distance in iterator:
            if new_scan:
                if len(scan) > min_len:
                    yield scan
                scan = []
            if quality > 0 and distance > 0:
                scan.append((quality, angle, distance))

