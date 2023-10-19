'''
    APDS9250 ALS demo

    Author: Eike Friedrich
    Date:   Aug-2023


'''
from machine import Pin, I2C, SoftI2C
import time
from math import log
from APDS9250 import * # use constants without prefix

def normalise_result(list_in, tolerance = 0.55, variation = 0.01):
    ''' given an array of 1 dimension
        return an array limited to results within 1 std dev of mean
        tolerance is percentage of samples within 1 std deviation of mean
        variation is std deviation values as a percentage of the samples mean
    '''
    # Calculate the mean and standard deviation
    mean_value = sum(list_in) / len(list_in)
    std_deviation = (sum((x - mean_value) ** 2 for x in list_in) / len(list_in)) ** 0.5

    # Print the original array and the filtered array
    list_variation = std_deviation / mean_value
    print("Mean:", mean_value)
    print(f"std dev: {std_deviation}, variation {list_variation:.4f}")
    print("Original Array:\n", list_in)

    # TD this needs to allow more samples to be included if the variation is lower
    # not sure about method below
    # if the std_dev is small then filter the list less
    #std_dev_compensated = std_deviation * (2 - list_variation)
    #print("std_dev_compensated = ",std_dev_compensated)
    
    # Filter values within 1 standard deviation of the mean
    #filtered_list = [x for x in list_in if mean_value - std_dev_compensated <= x <= mean_value + std_dev_compensated]
    
    # attempt2
    # get Z score for each value
    threshold = 1 # remove all but n std deviations from mean
    #outlier = [] 
    #for i in df ['MonthlyIncome']: 
    #    z = (i-MonthlyIncome_mean)/MonthlyIncome_std 
    #    if z >= threshold: 
    #        outlier.append(i)
    #        df = df[~df.MonthlyIncome.isin(outlier)]
    filtered_list = [x for x in list_in if abs((x - mean_value)/ std_deviation) < threshold ]
    print(f"Filtered Array Z-score within {threshold}:\n {filtered_list}")
    
    # return filtered_list
    samples = len(list_in)
    if (len(filtered_list) / samples) < tolerance or list_variation > variation:
        print(f"unstable result, {(len(filtered_list) / samples):.2f} results returned, {list_variation:.2%} variation")
        result = False
    else:
        print(f"stable result, {(len(filtered_list) / samples):.2%} results returned, {list_variation:.2%} {variation:.2%}")
        result = sum(filtered_list) / len(filtered_list)
    return result



led = Pin(15, Pin.OUT)
#i2c=SoftI2C(scl=Pin(17),
#        sda=Pin(16),
#        freq=400000,
#        timeout=50000)
i2c=I2C(0,
         scl=Pin(17),
         sda=Pin(16),
         freq=100000)
apds = APDS9250(i2c)
apds.set_gain(gain=APDS9250_LS_GAIN_18X)
apds.set_res_rate(adc_bits=APDS9250_RESOLUTION_18BIT, meas_rate=APDS9250_MEAS_RATE_100MS)
apds.set_mode(mode=APDS9250_CTRL_CS_MODE_RGB, enable=APDS9250_CTRL_LS_EN)
print(f'devid:{apds.devid}')
#time.sleep(0.11)


def read_blue_channel(nbr_samples=5):
    count = 0
    readings = []
    led.high()
    time.sleep(0.2) # TD should be 2x integration time
    while count < nbr_samples:
        #print(apds.get_reading())
        apds.get_reading()
        readings.append(apds.reading[2]) # blue channel
        #print(apds.reading)
        #print(apds.reading[2])
        #print(apds.reading[6:9])
        #print(apds.reading[8], apds.reading[7], apds.reading[6])
        count+= 1
    led.low()
    #print(readings)
    return(readings)


def get_stable_reading():
    ''' should have variable for which channel(s) to return
        currently hardcoded for blue channel
    '''
    max_attempts = 5
    count = 0
    stable_reading = None
    while not stable_reading and count in range(max_attempts + 1):
        # get 15 samples from blue channel
        readings = read_blue_channel(15)

        # get rid of outliers
        readings = normalise_result(readings, tolerance = 0.6, variation = 0.015)
        if readings:
            print("result is", readings)
            stable_reading = readings
        else:
            print("need to resample, count:", count)
            stable_reading = None
        count += 1
    return stable_reading

beer_value = None
cal_value = get_stable_reading()
if cal_value:
    # now we have a cal value
    # allow time to swap sample
    print("got sample")
    time.sleep(6)
    beer_value = get_stable_reading()
else:
    print("failed to get sample")

if beer_value:
    print("got transmitted sample value")
    # calc EBC
    dilution = 1 # TD add as parameter
    optical_density = log(cal_value / beer_value, 10)
    ebc = 25 * optical_density * dilution
    print(f"OD={optical_density:.2f}, EBC={ebc:.2f}")
#     
#time.sleep_ms(5)
#apds.get_part_id()
# set gain
# set meaure rate
# (set interrupt)
# set mode, enable measurements
# get readings (all channels)
# if reading =99% of max then reduce gain
# if reading < 33% increase gain

''' having gain & meaure rate as integers would be helpful
'''
