from smbus import SMBus
import time
import SHT31DConstant

class SHT31D:
    def __init__(self, i2cAddr):
        self.address = i2cAddr
        self.bus = SMBus(1)
        time.sleep(0.05)
    def readTemperatureAndHumidity(self):
        self.writeCommand(SHT31DConstant.SHT31_MEAS_HIGHREP)
        time.sleep(0.015)
        TempMSB, TempLSB, TempCRC, HumMSB, HumLSB, HumCRC = self.readData(SHT31DConstant.SHT31_MEAS_HIGHREP);
        STemp = TempMSB*16*16+TempLSB
        SHum = HumMSB*16*16+HumLSB
        # print TempCRC
        if TempCRC != self.CRC8([TempMSB, TempLSB]):
            TC = "nan"
        else:
            TC = -45+175*STemp/float(2**16-1)
        # print HumCRC
        if HumCRC != self.CRC8([HumMSB, HumLSB]):
            RH = "nan"
        else:
            RH = 100*SHum/float(2**16-1)
        return (TC,RH)
        # return
    def readTemperature(self):
        (TC,RH) = self.readTemperatureAndHumidity()
        return TC
    def readHumidity(self):
        (TC,RH) = self.readTemperatureAndHumidity()
        return RH
    def clearStatus(self):
        self.writeCommand(SHT31DConstant.SHT31_CLEARSTATUS)
    def readStatus(self):
        self.writeCommand(SHT31DConstant.SHT31_READSTATUS)
        StatusMSB, StatusLSB, StatusCRC = self.readData(SHT31DConstant.SHT31_READSTATUS, 3)
        if StatusCRC != self.CRC8([StatusMSB, StatusLSB]):
            return 'nan'
        else:
            return StatusMSB << 8 | StatusLSB
    def heaterEnable(self):
        self.writeCommand(SHT31DConstant.SHT31_HEATEREN)
    def heaterDisable(self):
        self.writeCommand(SHT31DConstant.SHT31_HEATERDIS)
    def softReset(self):
        self.writeCommand(SHT31DConstant.SHT31_SOFTRESET)
    def writeCommand(self,cmd):
        self.bus.write_word_data(self.address,cmd >> 8, cmd & 0xFF)
    def readData(self,cmd,numBytes = 6):
        return self.bus.read_i2c_block_data(self.address, cmd, numBytes)
    def CRC8(self, buffer):
        """ Polynomial 0x31 (x8 + x5 +x4 +1) """

        polynomial = 0x31;
        crc = 0xFF;

        index = 0
        # print buffer
        for index in range(0, len(buffer)):
            crc ^= int(buffer[index])
            for i in range(8, 0, -1):
                if crc & 0x80:
                    crc = (crc << 1) ^ polynomial
                else:
                    crc = (crc << 1)
        # print crc & 0xFF
        return crc & 0xFF
    # def CRC8(self, input_bitstring):
    # 	'''
    # 	Calculates the CRC remainder of a string of bits using a chosen polynomial.
    # 	initial_filler should be '1' or '0'.
    # 	'''
    #     polynomial_bitstring = '100110001'
    # 	len_input = len(input_bitstring)
    # 	initial_padding = '0'  * (len(polynomial_bitstring) - 1)
    # 	input_padded_array = list(input_bitstring + initial_padding)
    #     print input_padded_array
    # 	polynomial_bitstring = polynomial_bitstring.lstrip('0')
    # 	while '1' in input_padded_array[:len_input]:
    # 		cur_shift = input_padded_array.index('1')
    # 		for i in range(len(polynomial_bitstring)):
    # 			if polynomial_bitstring[i] == input_padded_array[cur_shift + i]:
    # 				input_padded_array[cur_shift + i] = '0'
    # 			else:
    # 				input_padded_array[cur_shift + i] = '1'
    #     # print int(''.join(input_padded_array)[len_input:],2) ^ 0x00
    # 	return int(''.join(input_padded_array)[len_input:],2)
