from base_ble import *
import atexit

GLOBAL_BLE_DEVICE = None


def get_ble_controller():
    global GLOBAL_BLE_DEVICE
    if GLOBAL_BLE_DEVICE:
        GLOBAL_BLE_DEVICE.reload_config()
        return GLOBAL_BLE_DEVICE
    else:
        GLOBAL_BLE_DEVICE = ArtemisBLEController()
        return GLOBAL_BLE_DEVICE


def exit_handler():
    LOG.info('Exiting...')
    try:
        GLOBAL_BLE_DEVICE.disconnect()
        LOG.info('Gracefully Exiting')
    except Exception as e:
        LOG.warn("Could not disconnect before exiting")
        LOG.warn(str(e))


class BaseBLEController(object):
    def __init__(self, address, service_uuid, max_write_length):
        self.device = BLEAsyncDevice(address, service_uuid)
        self.max_write_length = max_write_length

        LOG.debug("Python Info: {}".format(sys.version_info))
        LOG.debug("System Info: {}".format(platform.platform()))

        if sys.version_info >= (3, 7):
            self._wait = wait_b
        else:
            self._wait = wait_a

        atexit.register(exit_handler)

    def sleep(self, seconds=1):
        self._wait(asyncio.sleep(seconds))

    def scan(self, timeout=10):
        LOG.info('Scanning for Bluetooth devices...')
        scanner = BleakScanner()
        devices = self._wait(scanner.discover(timeout))
        return [{'name': device.name, 'address': device.address} for device in devices]

    def is_connected(self):
        return self.device.client.is_connected

    def connect(self, max_retries=3):
        retry_ctr = 0

        while retry_ctr < max_retries:
            success = self._wait(self.device._connect())
            if success:
                break

            retry_ctr = retry_ctr + 1
            time.sleep(1)
            LOG.warn("Attempting to connect again...")
        else:
            raise Exception(
                "Failed to connect after {} attempts".format(max_retries))

        time.sleep(1)

    def disconnect(self):
        self._wait(self.device._disconnect())

    def write(self, uuid, byte_array):
        self._wait(self.device._write(uuid, byte_array))

    def read(self, uuid):
        return self._wait(self.device._read(uuid))

    def start_notify(self, uuid, notification_handler):
        self._wait(self.device._start_notify(uuid, notification_handler))

    def stop_notify(self, uuid):
        self._wait(self.device._stop_notify(uuid))

    def explore_services(self):
        self._wait(self.device._explore_services())
    
    def __del__(self):
        try:
            self.disconnect()
            LOG.info('BaseBLEController Deleted')
        except Exception as e:
            LOG.warn("Could not delete BaseBLEController instance before exiting")
            LOG.warn(str(e))


class ArtemisBLEController(BaseBLEController):
    _instantiated = False
    def __init__(self, config='connection.yaml', max_write_length=150):
        if ArtemisBLEController._instantiated == True:
            raise Exception("Cannot create more than one instance of ArtemisBLEController. \n Use the function get_ble_controller() to always return a single instance of the class.")
        else:
            ArtemisBLEController._instantiated = True
        
        self.conn_config = config
        address, service_uuid, self.uuid = self._load_config()

        super(ArtemisBLEController, self).__init__(address, service_uuid, max_write_length)

    def _load_config(self):
        try:
            with open(self.conn_config) as file:
                config_list = yaml.load(file,
                                        Loader=yaml.FullLoader)
                address = config_list["artemis_address"]
                def add_leading_0s(address):
                    pieces = address.split(':')
                    for i,piece in enumerate(pieces):
                        if(len(piece) == 1):
                            pieces[i] = "0" + piece
                    return (':').join(pieces)
                address = add_leading_0s(address)
                service_uuid = config_list["ble_service"]
                uuid = config_list["characteristics"]
                return address, service_uuid, uuid
                
        except Exception as e:
            LOG.error("Error loading config file: " + self.conn_config)
            LOG.error(e)
    
    def reload_config(self):
        address, service_uuid, self.uuid = self._load_config()
        self.device.set_address(address, service_uuid)

    def bytearray_to_float(self, byte_array):
        return struct.unpack('<f', byte_array)[0]

    def bytearray_to_int(self, byte_array):
        return struct.unpack('<i', byte_array)[0]

    def bytearray_to_string(self, byte_array):
        return byte_array.decode()

    def receive_float(self, uuid):
        return self.bytearray_to_float(self.read(uuid))

    def receive_int(self, uuid):
        return self.bytearray_to_int(self.read(uuid))

    def receive_string(self, uuid):
        return self.bytearray_to_string(self.read(uuid))
    
    def send_command(self, cmd_type, data):
        cmd_string = str(cmd_type.value) + ":" + str(data)

        if len(cmd_string) < self.max_write_length:
            self.write(self.uuid['TX_CMD_STRING'], bytearray(map(ord, cmd_string)))
        else:
            raise Exception("Cannot write string larger than {} bytes".format(
                self.max_write_length))
   