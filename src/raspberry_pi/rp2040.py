class RP2040:
    def __init__(self):
        self.battery_on = False
        self.encoder_on = False
        self.distance_on = False
        self.bus = smbus.SMBus(1)  
        self.addr = 0x08 
    
    def set_battery(self, on):
        # send command to rp2040
        if self.battery_on == on:
            return
        self.bus.write_byte(self.addr, 'B' if on else 'b')
        self.battery_on = on
    
    def set_encoder(self, on):
        # send command to rp2040
        if self.encoder_on == on:
            return
        self.bus.write_byte(self.addr, 'E' if on else 'e')
        self.encoder_on = on

    def set_distance(self, on):
        # send command to rp2040
        if self.distance_on == on:
            return
        self.bus.write_byte(self.addr, 'D' if on else 'd')
        self.distance_on = on

    def request_data(self):
        # send command to rp2040
        try:
            # Request two bytes of data (assuming two sensor values)
            #sensor_data1 = bus.read_byte(address)  # Read first data point
            
            pass
        except Exception as e:
            print(f"Error reading data: {e}")
            return None, None

