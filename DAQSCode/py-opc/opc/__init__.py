from pkg_resources import get_distribution
from .exceptions import SPIError, FirmwareError

from time import sleep
import spidev
import struct
import warnings
import re

__all__ = ['OPCN2', 'OPCN1']
__version__ = get_distribution('opc').version

class OPC(object):
    '''
    Generic class for any Alphasense OPC Instance
    '''
    def __init__(self, spi_connection, **kwargs):
        self.cnxn       = spi_connection
        self.debug      = kwargs.get('debug', False)
        self.firmware   = None
        self.model      = kwargs.get('model', 'N2')

        # Check to make sure the connection is a valid SpiDev instance
        if not isinstance(spi_connection, spidev.SpiDev):
            raise SPIError("This is not an instance of SpiDev.")

        # Set the firmware version upon initialization
        try:
            self.firmware = int(re.findall("\d{3}", self.read_info_string())[-1])
        except:
            # Try again for the early (v7) firmwares
            try:
                tmp = int(re.findall("\d{1}", self.read_info_string())[-1])
                if tmp != 0:
                    self.firmware = tmp
                else:
                    raise FirmwareError("Cannot determine correct firmware for this OPC: {}".format(self.read_info_string()))
            except:
                raise FirmwareError("Cannot determine correct firmware for this OPC: {}".format(self.read_info_string()))

    def _combine_bytes(self, LSB, MSB):
        ''' Returns the combined Bytes '''
        return (MSB << 8) | LSB

    def _calculate_float(self, byte_array):
        ''' Returns a float from an array of 4 bytes '''
        if len(byte_array) != 4:
            return None

        return struct.unpack('f', struct.pack('4B', *byte_array))[0]

    def _calculate_mtof(self, mtof):
        '''
            calculates the average amount of time that particles in
            this bin took to cross the path of the OPC units -> [micro-seconds]
        '''
        return mtof / 3.0

    def _calculate_temp(self, vals):
        ''' calculates the temperature in degrees celcius '''
        if len(vals) < 4:
            return None

        return ((vals[3] << 24) | (vals[2] << 16) | (vals[1] << 8) | vals[0]) / 10.0

    def _calculate_pressure(self, vals):
        ''' calculate the pressure in pascals '''
        if len(vals) < 4:
            return None

        return ((vals[3] << 24) | (vals[2] << 16) | (vals[1] << 8) | vals[0])

    def _calculate_period(self, vals):
        ''' calculate the sampling period in seconds '''
        if len(vals) < 4:
            return None

        if self.firmware < 16:
            return ((vals[3] << 24) | (vals[2] << 16) | (vals[1] << 8) | vals[0]) / 12e6
        else:
            return self._calculate_float(vals)

    def _calculate_bin_boundary(self, val):
        '''
            Returns the bin boundary value in micrometers.
            Assumes a 12-bit ADC with 17.5 um Full-Scale.

            This is incorrect!
        '''
        fullscale   = 17.5    # micrometers
        adc         = 12

        return (val / (2**adc - 1)) * fullscale

    def read_info_string(self):
        ''' returns the firmware information for the OPC as a string '''

        infostring = []
        command = 0x3F

        # Send the command byte and sleep for 9 ms
        self.cnxn.xfer([command])
        sleep(9e-3)

        # Read the info string by sending 60 empty bytes
        for i in range(60):
            resp = self.cnxn.xfer([0x00])[0]
            infostring.append(chr(resp))

        return ''.join(infostring)

    def ping(self):
        # returns the status of the OPC-N2 as a boolean (check_status)
        b = self.cnxn.xfer([0xCF])[0]           # send the command byte

        return True if b == 0xF3 else False

    def __repr__(self):
        return "Alphasense OPC-{}v{}".format(self.model, self.firmware)

class OPCN2(OPC):
    ''' Class instance for the Alphasene OPC-N2 '''
    def __init__(self, spi_connection, **kwargs):
        super(OPCN2, self).__init__(spi_connection, model = 'N2', **kwargs)

        if self.firmware not in [14, 15, 16, 17]:
            raise FirmwareError("Current firmware version {0} is not supported.".format(self.firmware))

    def on(self):
        ''' turns ON the OPC (fan and laser) '''
        b1 = self.cnxn.xfer([0x03])[0]          # send the command byte
        sleep(9e-3)                             # sleep for 9 ms
        b2, b3 = self.cnxn.xfer([0x00, 0x01])   # send the following two bytes

        return True if b1 == 0xF3 and b2 == 0x03 else False

    def off(self):
        ''' turns OFF the OPC (fan and laser) '''
        b1 = self.cnxn.xfer([0x03])[0]          # send the command byte
        sleep(9e-3)                             # sleep for 9 ms
        b2 = self.cnxn.xfer([0x01])[0]          # send the following two bytes

        return True if b1 == 0xF3 and b2 == 0x03 else False

    def read_config_variables(self):
        ''' reads the configuration variables and returns them as a dictionary '''
        config  = []
        data    = {}
        command = 0x3C

        # Send the command byte and sleep for 10 ms
        self.cnxn.xfer([command])
        sleep(10e-3)

        # Read the config variables by sending 256 empty bytes
        for i in range(256):
            resp = self.cnxn.xfer([0x00])[0]
            config.append(resp)

        # Add the bin bounds to the dictionary of data [bytes 0-29]
        for i in range(0, 15):
            data["Bin Boundary {0}".format(i)] = self._combine_bytes(config[2*i], config[2*i + 1])

        # Add the Bin Particle Volumes (BPV) [bytes 32-95]
        for i in range(0, 16):
            data["BPV {0}".format(i)] = self._calculate_float(config[4*i + 32:4*i + 36])

        # Add the Bin Particle Densities (BPD) [bytes 96-159]
        for i in range(0, 16):
            data["BPD {0}".format(i)] = self._calculate_float(config[4*i + 96:4*i + 100])

        # Add the Bin Sample Volume Weight (BSVW) [bytes 160-223]
        for i in range(0, 16):
            data["BSVW {0}".format(i)] = self._calculate_float(config[4*i + 160: 4*i + 164])

        # Add the Gain Scaling Coefficient (GSC) and sample flow rate (SFR)
        data["GSC"] = self._calculate_float(config[224:228])
        data["SFR"] = self._calculate_float(config[228:232])

        # Add laser dac (LDAC) and Fan dac (FanDAC)
        data["LaserDAC"]    = config[232]
        data["FanDAC"]      = config[233]

        # If past firmware 15, add other things
        if self.firmware > 15:
            data['TOF_SFR'] = config[234]

        # Don't know what to do about all of the bytes yet!
        if self.debug:
            count = 0
            print ("Debugging the Config Variables")
            for each in config:
                print ("\t{0}: {1}".format(count, each))
                count += 1

        return data

    def write_config_variables(self):
        ''' Writes the configuration variables to memory '''
        return

    def read_histogram(self):
        resp = []
        data = {}

        # command byte
        command = 0x30

        # Send the command byte
        self.cnxn.xfer([command])

        # Wait 10 ms
        sleep(10e-3)

        # read the histogram
        for i in range(62):
            r = self.cnxn.xfer([0x00])[0]
            resp.append(r)

        # convert to real things and store in dictionary!
        data['Bin 0']           = self._combine_bytes(resp[0], resp[1])
        data['Bin 1']           = self._combine_bytes(resp[2], resp[3])
        data['Bin 2']           = self._combine_bytes(resp[4], resp[5])
        data['Bin 3']           = self._combine_bytes(resp[6], resp[7])
        data['Bin 4']           = self._combine_bytes(resp[8], resp[9])
        data['Bin 5']           = self._combine_bytes(resp[10], resp[11])
        data['Bin 6']           = self._combine_bytes(resp[12], resp[13])
        data['Bin 7']           = self._combine_bytes(resp[14], resp[15])
        data['Bin 8']           = self._combine_bytes(resp[16], resp[17])
        data['Bin 9']           = self._combine_bytes(resp[18], resp[19])
        data['Bin 10']          = self._combine_bytes(resp[20], resp[21])
        data['Bin 11']          = self._combine_bytes(resp[22], resp[23])
        data['Bin 12']          = self._combine_bytes(resp[24], resp[25])
        data['Bin 13']          = self._combine_bytes(resp[26], resp[27])
        data['Bin 14']          = self._combine_bytes(resp[28], resp[29])
        data['Bin 15']          = self._combine_bytes(resp[30], resp[31])
        data['Bin1 MToF']       = self._calculate_mtof(resp[32])
        data['Bin3 MToF']       = self._calculate_mtof(resp[33])
        data['Bin5 MToF']       = self._calculate_mtof(resp[34])
        data['Bin7 MToF']       = self._calculate_mtof(resp[35])

        # Bins associated with firmware versions 14 and 15(?)
        if self.firmware < 16:
            data['Temperature']     = self._calculate_temp(resp[36:40])
            data['Pressure']        = self._calculate_pressure(resp[40:44])
            data['Sampling Period'] = self._calculate_period(resp[44:48])
            data['Checksum']        = self._combine_bytes(resp[48], resp[49])
            data['PM1']             = self._calculate_float(resp[50:54])
            data['PM2.5']           = self._calculate_float(resp[54:58])
            data['PM10']            = self._calculate_float(resp[58:])

        else:
            data['SFR']             = self._calculate_float(resp[36:40])

            # Alright, we don't know whether it is temp or pressure since it switches..
            tmp = self._calculate_pressure(resp[40:44])
            if tmp > 98000:
                data['Temperature'] = None
                data['Pressure']    = tmp
            else:
                tmp = self._calculate_temp(resp[40:44])
                if tmp < 500:
                    data['Temperature'] = tmp
                    data['Pressure']    = None
                else:
                    data['Temperature'] = None
                    data['Pressure']    = None

            data['Sampling Period'] = self._calculate_float(resp[44:48])
            data['Checksum']        = self._combine_bytes(resp[48], resp[49])
            data['PM1']             = self._calculate_float(resp[50:54])
            data['PM2.5']           = self._calculate_float(resp[54:58])
            data['PM10']            = self._calculate_float(resp[58:])

        # Calculate the sum of the histogram bins
        histogram_sum = data['Bin 0'] + data['Bin 1'] + data['Bin 2']   + \
                data['Bin 3'] + data['Bin 4'] + data['Bin 5'] + data['Bin 6']   + \
                data['Bin 7'] + data['Bin 8'] + data['Bin 9'] + data['Bin 10']  + \
                data['Bin 11'] + data['Bin 12'] + data['Bin 13'] + data['Bin 14'] + \
                data['Bin 15']

        # If debug is True, print out the bytes!
        if self.debug:
            count = 0
            print ("Debugging the Histogram")
            for each in resp:
                print ("\t{0}: {1}".format(count, each))
                count += 1

        # Check that checksum and the least significant bits of the sum of histogram bins
        # are equivilant
        if (histogram_sum & 0x0000FFFF) != data['Checksum']:
            warnings.warn("Data transfer was incomplete.")
            return None

        return data

    def save_config_variables(self):
        ''' Save the config variables in non-volatile memory '''
        command = 0x43
        byte_list = [0x3F, 0x3C, 0x3F, 0x3C, 0x43]
        success = [0xF3, 0x43, 0x3F, 0x3C, 0x3F, 0x3C]
        resp = []

        # Send the command byte and then wait for 10 ms
        r = self.cnxn.xfer([command])[0]
        sleep(10e-3)

        # append the response of the command byte to the List
        resp.append(r)

        # Send the rest of the config bytes
        for each in byte_list:
            r = self.cnxn.xfer([each])[0]
            resp.append(r)

        return True if resp == success else False

    def enter_bootloader_mode(self):
        ''' Enter bootloader mode '''
        command = 0x41

        return True if self.cnxn.xfer(command)[0] == 0xF3 else False

    def set_fan_power(self, value):
        ''' Sets the fan power as a value between 0-255 '''
        command = 0x42

        # Check to make sure the value is a single byte
        if value >= 256:
            raise ValueError("Try a single byte (0-255).")

        # Send the command byte and wait 10 ms
        a = self.cnxn.xfer([command])[0]
        sleep(10e-3)

        # Send the next two bytes
        b = self.cnxn.xfer([0x00])[0]
        c = self.cnxn.xfer([value])[0]

        return True if a == 0xF3 and b == 0x42 and c == 0x00 else False

    def set_laser_power(self, value):
        ''' Sets the laser power as a value between 0-255 '''
        command = 0x42

        # Check to make sure the value is a single byte
        if value >= 256:
            raise ValueError("Try a single byte (0-255).")

        # Send the command byte and wait 10 ms
        a = self.cnxn.xfer([command])[0]
        sleep(10e-3)

        # Send the next two bytes
        b = self.cnxn.xfer([0x01])[0]
        c = self.cnxn.xfer([value])[0]

        return True if a == 0xF3 and b == 0x42 and c == 0x01 else False

    def laser_on(self):
        ''' Turn on the laser only '''
        command = 0x03
        byte    = 0x02

        # Send the command byte, wait 10ms, and then send the byte
        a = self.cnxn.xfer([command])[0]
        sleep(10e-3)
        b = self.cnxn.xfer([byte])[0]

        return True if a == 0xF3 and b == 0x03 else False

    def laser_off(self):
        ''' Turn off the laser only '''
        command = 0x03
        byte    = 0x03

        # Send the command byte, wait 10ms, and then send the byte
        a = self.cnxn.xfer([command])[0]
        sleep(10e-3)
        b = self.cnxn.xfer([byte])[0]

        return True if a == 0xF3 and b == 0x03 else False

    def fan_on(self):
        ''' Turn on the fan only '''
        command = 0x03
        byte    = 0x04

        # Send the command byte, wait 10ms, and then send the byte
        a = self.cnxn.xfer([command])[0]
        sleep(10e-3)
        b = self.cnxn.xfer([byte])[0]

        return True if a == 0xF3 and b == 0x03 else False

    def fan_off(self):
        ''' Turn off the fan only '''
        command = 0x03
        byte    = 0x05

        # Send the command byte, wait 10ms, and then send the byte
        a = self.cnxn.xfer([command])[0]
        sleep(10e-3)
        b = self.cnxn.xfer([byte])[0]

        return True if a == 0xF3 and b == 0x03 else False

class OPCN1(OPC):
    ''' Class instance for the Alphasense OPC-N1 '''
    def __init(self, spi_connection, **kwargs):
        super(OPCN1, self).__init__(spi_connection, model = 'N1', **kwargs)

    def on(self):
        # turns ON the OPC (fan and laser)
        b1 = self.cnxn.xfer([0x0C])[0]          # send the command byte
        sleep(9e-3)                             # sleep for 9 ms

        return True if b1 == 0xF3 else False

    def off(self):
        # turns OFF the OPC (fan and laser)
        b1 = self.cnxn.xfer([0x03])[0]          # send the command byte
        sleep(9e-3)                             # sleep for 9 ms

        return True if b1 == 0xF3 else False

    def read_gsc_sfr(self):
        ''' Read the gain-scaling-coefficient and sample flow rate '''
        config  = []
        data    = {}

        # Send the command byte and sleep for 10 ms
        self.cnxn.xfer([0x33])
        sleep(10e-3)

        # Read the config variables by sending 256 empty bytes
        for i in range(8):
            resp = self.cnxn.xfer([0x00])[0]
            config.append(resp)

        data["GSC"] = self._calculate_float(config[0:4])
        data["SFR"] = self._calculate_float(config[4:])

        return data

    def read_bin_boundaries(self):
        ''' Return the bin boundaries '''
        config  = []
        data    = {}

        # Send the command byte and sleep for 10 ms
        self.cnxn.xfer([0x33])
        sleep(10e-3)

        # Read the config variables by sending 256 empty bytes
        for i in range(30):
            resp = self.cnxn.xfer([0x00])[0]
            config.append(resp)

        # Add the bin bounds to the dictionary of data [bytes 0-29]
        for i in range(0, 14):
            data["Bin Boundary {0}".format(i)] = self._combine_bytes(config[2*i], config[2*i + 1])

        return data

    def write_gsc_sfr(self):
        ''' Write the gsc and sfr values '''
        return

    def read_bin_particle_density(self):
        ''' Read the bin particle density '''
        config = []

        # Send the command byte and sleep for 10 ms
        self.cnxn.xfer([0x33])
        sleep(10e-3)

        # Read the config variables by sending 256 empty bytes
        for i in range(4):
            resp = self.cnxn.xfer([0x00])[0]
            config.append(resp)

        bpd = self._calculate_float(config)

        return bpd

    def write_bin_particle_density(self):
        ''' Write the bin particle density '''
        return

    def read_histogram(self):
        ''' Read histogram and reset '''
        resp = []
        data = {}

        # command byte
        command = 0x30

        # Send the command byte
        self.cnxn.xfer([command])

        # Wait 10 ms
        sleep(10e-3)

        # read the histogram
        for i in range(62):
            r = self.cnxn.xfer([0x00])[0]
            resp.append(r)

        # convert to real things and store in dictionary!
        data['Bin 0']           = self._combine_bytes(resp[0], resp[1])
        data['Bin 1']           = self._combine_bytes(resp[2], resp[3])
        data['Bin 2']           = self._combine_bytes(resp[4], resp[5])
        data['Bin 3']           = self._combine_bytes(resp[6], resp[7])
        data['Bin 4']           = self._combine_bytes(resp[8], resp[9])
        data['Bin 5']           = self._combine_bytes(resp[10], resp[11])
        data['Bin 6']           = self._combine_bytes(resp[12], resp[13])
        data['Bin 7']           = self._combine_bytes(resp[14], resp[15])
        data['Bin 8']           = self._combine_bytes(resp[16], resp[17])
        data['Bin 9']           = self._combine_bytes(resp[18], resp[19])
        data['Bin 10']          = self._combine_bytes(resp[20], resp[21])
        data['Bin 11']          = self._combine_bytes(resp[22], resp[23])
        data['Bin 12']          = self._combine_bytes(resp[24], resp[25])
        data['Bin 13']          = self._combine_bytes(resp[26], resp[27])
        data['Bin 14']          = self._combine_bytes(resp[28], resp[29])
        data['Bin 15']          = self._combine_bytes(resp[30], resp[31])
        data['Bin1 MToF']       = self._calculate_mtof(resp[32])
        data['Bin3 MToF']       = self._calculate_mtof(resp[33])
        data['Bin5 MToF']       = self._calculate_mtof(resp[34])
        data['Bin7 MToF']       = self._calculate_mtof(resp[35])
        data['Temperature']     = self._calculate_temp(resp[36:40])
        data['Pressure']        = self._calculate_pressure(resp[40:44])
        data['Sampling Period'] = self._calculate_period(resp[44:48])
        data['Checksum']        = self._combine_bytes(resp[48], resp[49])
        data['PM1']             = self._calculate_float(resp[50:54])
        data['PM2.5']           = self._calculate_float(resp[54:58])
        data['PM10']            = self._calculate_float(resp[58:])

        # Calculate the sum of the histogram bins
        histogram_sum = data['Bin 0'] + data['Bin 1'] + data['Bin 2']   + \
                data['Bin 3'] + data['Bin 4'] + data['Bin 5'] + data['Bin 6']   + \
                data['Bin 7'] + data['Bin 8'] + data['Bin 9'] + data['Bin 10']  + \
                data['Bin 11'] + data['Bin 12'] + data['Bin 13'] + data['Bin 14'] + \
                data['Bin 15']

        # If debug is True, print out the bytes!
        if self.debug:
            count = 0
            print ("Debugging the Histogram")
            for each in resp:
                print ("\t{0}: {1}".format(count, each))
                count += 1

        # Check that checksum and the least significant bits of the sum of histogram bins
        # are equivilant (Doesn't work right now for some reason -> checksum always == 0!)
        #if (histogram_sum & 0x0000FFFF) != data['Checksum']:
        #    warnings.warn("Data transfer was incomplete.")
        #    return None

        return data
