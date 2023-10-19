'''
    upython driver for APDS9250 Light Sensor

    Author: Eike Friedrich
    Date:   Aug-2023

'''
# from machine import I2C, Pin
import time
import utime
#import ustruct
#import sys

###############################################################################
# Constants

# I2C address
APDS9250_I2C_ADDR = 0x52

# Registers (t) = triggers new measurement
APDS9250_REG_MAIN_CTRL        = 0x00 # (t)(rw) LS operation mode control, SW reset
APDS9250_REG_LS_MEAS_RATE     = 0x04 # (t)(rw) LS measurement rate and resolution in active mode
APDS9250_REG_LS_GAIN          = 0x05 # (t)(rw) LS analog gain range
APDS9250_REG_PART_ID          = 0x06 #    (ro) Part number ID and revision ID
APDS9250_REG_MAIN_STATUS      = 0x07 #    (ro) Power-on status, interrupt status, data status
APDS9250_REG_LS_DATA_IR_0     = 0x0A #    (ro) IR    ADC measurement data - LSB
APDS9250_REG_LS_DATA_IR_1     = 0x0B #    (ro) IR    ADC measurement data - ISB
APDS9250_REG_LS_DATA_IR_2     = 0x0C #    (ro) IR    ADC measurement data - MSB
APDS9250_REG_LS_DATA_GREEN_0  = 0x0D #    (ro) Green ADC measurement data - LSB
APDS9250_REG_LS_DATA_GREEN_1  = 0x0E #    (ro) Green ADC measurement data - ISB
APDS9250_REG_LS_DATA_GREEN_2  = 0x0F #    (ro) Green ADC measurement data - MSB
APDS9250_REG_LS_DATA_BLUE_0   = 0x10 #    (ro) Blue  ADC measurement data - LSB
APDS9250_REG_LS_DATA_BLUE_1   = 0x11 #    (ro) Blue  ADC measurement data - ISB
APDS9250_REG_LS_DATA_BLUE_2   = 0x12 #    (ro) Blue  ADC measurement data - MSB
APDS9250_REG_LS_DATA_RED_0    = 0x13 #    (ro) Red   ADC measurement data - LSB
APDS9250_REG_LS_DATA_RED_1    = 0x14 #    (ro) Red   ADC measurement data - ISB
APDS9250_REG_LS_DATA_RED_2    = 0x15 #    (ro) Red   ADC measurement data - MSB
APDS9250_REG_INT_CFG          = 0x19 #    (rw) Interrupt configuration
APDS9250_REG_INT_PERSISTENCE  = 0x1A #    (rw) Interrupt persist setting
APDS9250_REG_LS_THRES_UP_0    = 0x21 #    (rw) LS interrupt upper threshold - LSB
APDS9250_REG_LS_THRES_UP_1    = 0x22 #    (rw) LS interrupt upper threshold - ISB
APDS9250_REG_LS_THRES_UP_2    = 0x23 #    (rw) LS interrupt upper threshold - MSB
APDS9250_REG_LS_THRES_LOW_0   = 0x24 #    (rw) LS interrupt lower threshold - LSB
APDS9250_REG_LS_THRES_LOW_1   = 0x25 #    (rw) LS interrupt lower threshold - ISB
APDS9250_REG_LS_THRES_LOW_2   = 0x26 #    (rw) LS interrupt lower threshold - MSB
APDS9250_REG_LS_THRES_VAR     = 0x27 #    (rw) LS interrupt variance threshold

APDS9250_CTRL_SW_RESET    = 1 << 4 # Trigger software reset
APDS9250_CTRL_CS_MODE_ALS = 0 << 2 # Channel Select 0 - ALS & IR mode (default)
APDS9250_CTRL_CS_MODE_RGB = 1 << 2 # Channel Select 1 - RGB & IR mode
APDS9250_CTRL_CS_MASK     = 1 << 2 # Channel Select mask
APDS9250_CTRL_LS_EN       = 1 << 1 # Light Sensor enabled
APDS9250_CTRL_LS_DIS      = 0 << 1 # Light Sensor disabled

# Sensor resolution, with minimum integration time
APDS9250_RESOLUTION_20BIT = 0 << 4 # 20 bit resolution, 400ms integration time
APDS9250_RESOLUTION_19BIT = 1 << 4 # 19 bit resolution, 200ms integration time
APDS9250_RESOLUTION_18BIT = 2 << 4 # 18 bit resolution, 100ms integration time (default)
APDS9250_RESOLUTION_17BIT = 3 << 4 # 17 bit resolution, 50ms integration time
APDS9250_RESOLUTION_16BIT = 4 << 4 # 16 bit resolution, 25ms integration time
APDS9250_RESOLUTION_13BIT = 5 << 4 # 13 bit resolution, 3.125ms integration time
APDS9250_RESOLUTION_MASK  = 7 << 4 # Mask for resolution bits

# Sensor measurement rate -- if integration time is greater then integration time overrides
APDS9250_MEAS_RATE_25MS   = 0 << 0
APDS9250_MEAS_RATE_50MS   = 1 << 0
APDS9250_MEAS_RATE_100MS  = 2 << 0
APDS9250_MEAS_RATE_200MS  = 3 << 0
APDS9250_MEAS_RATE_500MS  = 4 << 0
APDS9250_MEAS_RATE_1000MS = 5 << 0
APDS9250_MEAS_RATE_2000MS = 6 << 0
APDS9250_MEAS_RATE_DUP    = 7 << 0 # 2000ms time (duplicate)
APDS9250_MEAS_RATE_MASK   = 7 << 0 # Mask for resolution bits

APDS9250_LS_GAIN_1X       = 0 << 0 # Gain 1x
APDS9250_LS_GAIN_3X       = 1 << 0 # Gain 3x
APDS9250_LS_GAIN_6X       = 2 << 0 # Gain 6x
APDS9250_LS_GAIN_9X       = 3 << 0 # Gain 9x
APDS9250_LS_GAIN_18X      = 4 << 0 # Gain 18x
APDS9250_LS_GAIN_MASK     = 7 << 0 # Gain mask

class MyError(Exception):
    pass

class APDS9250:
    ''' APDS9250 object
        
    '''
    def __init__(self, i2c):
        self.i2c = i2c
        #self.set_gain()
        #self.set_res_rate()
        # self._text = text
        # self._font = fontself.setReg(APDS9930_PPULSE, 8)
        #self.setReg(APDS9930_CONTROL, 0x2C)
        #self.ALS_Enable()
        #self.Power()self.adc_bits = res_int
        self.meas_rate = 0  # measuring rate as number
        self.adc_bits = 0   # bit resolution as number
        self.gain = 0       # gain as an integer
        self.devid = 0      # part no, revision (11,2)
        self.reading = None
        self.interrupt = None
        # get power up status
        # get part id to confirm comms
        # fail if either of above do not work
        tests = (self.get_pu_status(), self.get_part_id())
        if all(tests):
            print("config OK")
            # sleep 5ms to allow voltages to settle
            time.sleep_ms(5)
        else:
            print('init failed')
            # raise Exception("an error occurred")
        #self.get_part_id()

    def reg_write(self, reg, data):
        ''' Write bytes to the specified register.
        '''
        # Construct message
        msg = bytearray()
        msg.append(data)
        # Write out message to register
        self.i2c.writeto_mem(APDS9250_I2C_ADDR, reg, msg)

    def reg_read(self, reg, nbytes=1):
        ''' Read byte(s) from specified register. If nbytes > 1, 
            read from consecutive registers.
        '''
        # Check to make sure caller is asking for 1 or more bytes
        if nbytes < 1:
            return bytearray()
        # Request data from specified register(s) over I2C
        data = self.i2c.readfrom_mem(APDS9250_I2C_ADDR, reg, nbytes)
        return data

    def set_mode(self, mode=APDS9250_CTRL_CS_MODE_RGB, enable=APDS9250_CTRL_LS_EN):
        ''' ALS|CS|standby main_ctl
        '''
        data = mode | enable
        self.reg_write(APDS9250_REG_MAIN_CTRL, data)
        self.mode = None
        # TD self.mode & enable properties

    def set_gain(self, gain=APDS9250_LS_GAIN_1X):
        ''' set gain, check return & report if not match
        '''
        if gain == APDS9250_LS_GAIN_1X:
            gain_int = 1
        elif gain == APDS9250_LS_GAIN_3X:
            gain_int = 3
        elif gain == APDS9250_LS_GAIN_1X:
            gain_int = 6
        elif gain == APDS9250_LS_GAIN_3X:
            gain_int = 9
        elif gain == APDS9250_LS_GAIN_1X:
            gain_int = 18
        else:
            gain_int = 1
            gain = APDS9250_LS_GAIN_1X
        self.reg_write(APDS9250_REG_LS_GAIN, gain)
        self.gain = gain_int
        print(f"gain set to {gain_int}X")
        #print(f"gain register:{self.reg_read(APDS9250_REG_LS_GAIN)}")

    def set_res_rate(self, adc_bits=APDS9250_RESOLUTION_18BIT, meas_rate=APDS9250_MEAS_RATE_100MS):
        ''' set adc resolution & measure rate (ms)
            check return & report if not match
            measure rate must be checked against adc min integration time
            MUST? maybe not, maybe just log a warning
        '''
        # min_rate
        if adc_bits == APDS9250_RESOLUTION_20BIT:
            min_rate = 400 # APDS9250_MEAS_RATE_500MS # check
            res_int = 20
        elif adc_bits == APDS9250_RESOLUTION_19BIT:
            min_rate = 200 # APDS9250_MEAS_RATE_200MS
            res_int = 19
        elif adc_bits == APDS9250_RESOLUTION_18BIT:
            min_rate = 100 # APDS9250_MEAS_RATE_100MS
            res_int = 18
        elif adc_bits == APDS9250_RESOLUTION_17BIT:
            min_rate = 50 # APDS9250_MEAS_RATE_50MS 
            res_int = 17
        elif adc_bits == APDS9250_RESOLUTION_16BIT:
            min_rate = 25 # APDS9250_MEAS_RATE_25MS
            res_int = 16
        elif adc_bits == APDS9250_RESOLUTION_13BIT:
            min_rate = 3.125 # APDS9250_MEAS_RATE_25MS # check
            res_int = 13
        else:
            min_rate = 100 # APDS9250_MEAS_RATE_100MS
            adc_bits = APDS9250_RESOLUTION_18BIT
            res_int = 18
            # log error
        # measure rate to number
        if meas_rate == APDS9250_MEAS_RATE_2000MS:
            meas_rate_int = 2000
        elif meas_rate == APDS9250_MEAS_RATE_1000MS:
            meas_rate_int = 1000
        elif meas_rate == APDS9250_MEAS_RATE_500MS:
            meas_rate_int = 500
        elif meas_rate == APDS9250_MEAS_RATE_200MS:
            meas_rate_int = 200
        elif meas_rate == APDS9250_MEAS_RATE_100MS:
            meas_rate_int = 100
        elif meas_rate == APDS9250_MEAS_RATE_50MS:
            meas_rate_int = 50
        elif meas_rate == APDS9250_MEAS_RATE_25MS:
            meas_rate_int = 25
        else:
            meas_rate = APDS9250_MEAS_RATE_100MS
            meas_rate_int = 100
            # log error
        #rate = max(min_rate, meas_rate)
        self.reg_write(APDS9250_REG_LS_MEAS_RATE, adc_bits)
        self.reg_write(APDS9250_REG_LS_MEAS_RATE, meas_rate)
        # warn if measure rate is too small
        if meas_rate_int < min_rate:
            print(f'Warning: ADC integration time is less than measure rate')
        # save values as integers
        '''for name, value in globals().items(): 
            if value == adc_bits and 'APDS9250_RESOLUTION' in name:
                print(name)
                self.adc_bits = name
        for name, value in globals().items(): 
            if value == rate and 'APDS9250_MEAS' in name:
                print(name)
                self.meas_rate = name'''
        # set the values
        # store values as properties
        self.adc_bits = res_int
        self.meas_rate = meas_rate_int

    def set_interrupt(self):
        ''' 
        '''
        self.interrupt = (None, None, None)

    def get_pu_status(self):
        ''' read power up status
            abandon this approach - a reset nedds to be done outside of class
            to reset the i2c?
        '''
        ready = False
        retries = 0
        # TD wrap this in a try
        '''while not ready and retries < 5:
            print('try a read')
            data = self.reg_read(APDS9250_REG_MAIN_STATUS)
            print(f'data:{data} int:{int.from_bytes(data, 'big')}')
            if int.from_bytes(data, 'big') & (1 << 5):
                # we have had a power up event
                ready = True
                # if 0 then maybe do a reset? & try again until timeout?
            elif (int.from_bytes(data, 'big') & (1 << 5)) == 0:
                retries += 1
                print(f'try a reset, attempt {retries}')
                self.reg_write(APDS9250_REG_MAIN_CTRL, APDS9250_CTRL_SW_RESET)
                #self.i2c.writeto_mem(APDS9250_I2C_ADDR, reg, msg)
                print(self.i2c.scan())
                #self.i2c.writeto(82, b'0')
                print(self.reg_read(APDS9250_REG_MAIN_CTRL))
                #i2c.stop()
                #time.sleep(2)
                print('sleep1')
                time.sleep_ms(50)
                #print('write 1')
                #self.reg_write(APDS9250_REG_MAIN_CTRL, 0 << 4)
                print('sleep2')
                time.sleep_ms(100)
                #print('end of sleep2')
                retries += 1
                print('next')
                print(f'retries:{retries}')
                print('next2')
            else:
                print('odd')
                retries += 1
        print('done')'''
        # TD put this in a try (i2c not responding)
        data = self.reg_read(APDS9250_REG_MAIN_STATUS)
        #print(f'data:{data} int:{int.from_bytes(data, 'big')}')
        if int.from_bytes(data, 'big') & (1 << 5):
            print('we have had a power up event')
            #pass
        elif int.from_bytes(data, 'big') & (1 << 5) == 0:
            # this is not a power on event
            pass
        ready = True
        return ready

    def get_part_id(self):
        ''' Read device ID to make sure that we can communicate with the ADPS
        '''
        result = False
        data = self.reg_read(APDS9250_REG_PART_ID)
        #print(data)
        high, low = (data[0]>> 4) & 0xf , (data[0]) & 0xf  # high nibble, low nibble
        #print(f"part:{high}, rev:{low}")
        self.devid = (high,low)
        if high and low:
            result = True
        return result

    def get_reading(self):
        ''' check if a new reading is available
            if not wait 1/10  of measure interval
            loop until timeout/reading available
            return list of channs & set object property
        '''
        #TD test with diff modes?
        all_channs = None
        #TD check ready
        if self.check_ready():
            # get reading
            all_channs = self.reg_read(APDS9250_REG_LS_DATA_IR_0, nbytes=12)
            lsb=int.from_bytes(all_channs[0:1], 'big')
            isb=int.from_bytes(all_channs[1:2], 'big')
            msb=int.from_bytes(all_channs[2:3], 'big') & 0xf0 # mask upper nibble
            ir_value = lsb | isb << 8 | msb << 16
            lsb=int.from_bytes(all_channs[3:4], 'big')
            isb=int.from_bytes(all_channs[4:5], 'big')
            msb=int.from_bytes(all_channs[5:6], 'big') & 0xf0 # mask upper nibble
            gn_value = lsb | isb << 8 | msb << 16 # Lux
            lsb=int.from_bytes(all_channs[6:7], 'big')
            isb=int.from_bytes(all_channs[7:8], 'big')
            msb=int.from_bytes(all_channs[8:9], 'big') & 0xf0 # mask upper nibble
            bl_value = lsb | isb << 8 | msb << 16
            lsb=int.from_bytes(all_channs[9:10], 'big')
            isb=int.from_bytes(all_channs[10:11], 'big')
            msb=int.from_bytes(all_channs[11:12], 'big') & 0xf0 # mask upper nibble
            rd_value = lsb | isb << 8 | msb << 16
            all_chans = (ir_value, gn_value, bl_value, rd_value)
        self.reading = all_chans
        return all_chans

    def check_ready(self):
        ''' is a reading available?
            if not are we mneasuring - enable
                wait until timeout or reading available
                wait 1/10  of measure interval, loop
            return boolean success
            timeout as a multiple of measure rate? 1.2x? requires measure rate as a number
        '''
        data_ready = False
        timeout_ms = self.meas_rate_int + 50
        delay = timeout_ms * 0.1
        # start a timer
        start = time.ticks_ms()
        # duration = presentTime - startTime
        try:
            while not data_ready:
                data = self.reg_read(APDS9250_REG_MAIN_STATUS)
                if data & (1 << 3):
                    # we have new (unread) data waiting
                    data_ready = True
                if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                    raise TimeoutError
                time.sleep_ms(delay)
        except TimeoutError:
            data_ready = False
            # TD log a timeout error message
        return data_ready


# https://realpython.com/python-getter-setter/
# https://forums.raspberrypi.com/viewtopic.php?t=241491
# pin_led = Pin(16, mode=Pin.OUT, value=0) # low
# pin_14 = Pin(14, mode=Pin.IN) #High
'''
while(clockCount<16){
	bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_05, LOW);
	bcm2835_delayMicroseconds(1);
	bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_05, HIGH);
	bcm2835_delayMicroseconds(1);
	clockCount=clockCount+1;
'''