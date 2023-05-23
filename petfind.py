import smbus
import time
import serial
import mysql.connector
import time
from datetime import datetime

# BUS VOLTAGE REGISTER (R)
_REG_BUSVOLTAGE             = 0x02

# CALIBRATION REGISTER (R/W)
_REG_CALIBRATION            = 0x05

db = mysql.connector.connect(
    host="db-mysql-nyc1-51434-do-user-7374472-0.b.db.ondigitalocean.com",
    user="doadmin",
    port="25060",
    password="AVNS_f5GEbHv1IfvVWUVD8yO",
    database="petfind"
)

def parse_gprmc(gprmc_sentence):
    fields = gprmc_sentence.split(',')

    if fields[2] != 'A':
        return (None, None, None)

    lat = float(fields[3][:2]) + float(fields[3][2:]) / 60
    lon = -1 * (float(fields[5][:3]) + float(fields[5][3:]) / 60)
    speed = float(fields[7]) * 1.852

    if fields[4] == 'S':
        lat = -1 * lat
    if fields[6] == 'E':
        lon = -1 * lon

    return (lat, lon, speed)

class INA219:
    def __init__(self, i2c_bus=1, addr=0x40):
        self.bus = smbus.SMBus(i2c_bus);
        self.addr = addr

        # Set chip to known config values to start
        self._cal_value = 0
        self._current_lsb = 0
        self._power_lsb = 0
        self.set_calibration_16V_5A()

    def read(self,address):
        data = self.bus.read_i2c_block_data(self.addr, address, 2)
        return ((data[0] * 256 ) + data[1])

    def write(self,address,data):
        temp = [0,0]
        temp[1] = data & 0xFF
        temp[0] =(data & 0xFF00) >> 8
        self.bus.write_i2c_block_data(self.addr,address,temp)

    def set_calibration_16V_5A(self):
        self._cal_value = 26868

    def getBusVoltage_V(self):
        self.write(_REG_CALIBRATION,self._cal_value)
        self.read(_REG_BUSVOLTAGE)
        return (self.read(_REG_BUSVOLTAGE) >> 3) * 0.004


ina219 = INA219(addr=0x43)

cursor = db.cursor()

ser = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
counter = 0
while True:
    line = ser.readline().decode('ISO-8859-1').rstrip()
    bus_voltage = ina219.getBusVoltage_V()
    p = (bus_voltage - 3)/1.2*100
    if(p > 100):p = 100
    if(p < 0):p = 0
    print(f"Percent:       {int(p)}%")
    print("")
    sql = "UPDATE petfind.battery_status t SET t.percent_charged = (%s) WHERE t.battery_id = 3"
    cursor.execute(sql, (int(p),))
    db.commit()

    if line.startswith('$GPRMC'):
        lat, lon, speed = parse_gprmc(line)
        if lat is None:
            continue
        print(f'Latitud: {lat}, Longitud: {lon}, Velocitat: {speed} knots')
        if(counter == 30):
            print("Inserting into database last coordinates")
            sql = "INSERT INTO coordinate_log (lat, lon, speed, time_stamp) VALUES (%s, %s, %s, %s)"
            cursor.execute(sql, (lat, lon, speed, datetime.now()))
            db.commit()
            counter = 0
        else:
            counter++
