import asyncio
import time
import bleach
from bleak import BleakClient, BleakScanner, BleakError
import nest_asyncio
import struct
import yaml
import sys
import platform

# If macOS >=12.0
OS_PLATFORM = platform.system()
IS_ATLEAST_MAC_OS_12 = False
if OS_PLATFORM == 'Darwin':
    import objc
    IS_ATLEAST_MAC_OS_12 = objc.macos_available(12,0)

from utils import setup_logging
LOG = setup_logging("ble.log")

# Ref: https://stackoverflow.com/questions/31875/is-there-a-simple-elegant-way-to-define-singletons    
def wait_a(coroutine):
    loop = asyncio.get_event_loop()
    return loop.run_until_complete(coroutine)

def wait_b(coroutine):
    return asyncio.run(coroutine)

# Ref: https://github.com/alexandrebarachant/muse-lsl/pull/148/files
class BLEAsyncDevice():
    def __init__(self, address, service_uuid):
        nest_asyncio.apply()

        self.set_address(address, service_uuid)
        
        self.client = None
        self.error_msg = None

    def set_address(self, address, service_uuid):
        self.address = address
        self.service_uuid = service_uuid

    def disconnect_handler(self, data):
        LOG.info("Disconnected from {}".format(data.address))
    
    def _is_atleast_mac_os_12(self):
        return IS_ATLEAST_MAC_OS_12
    
    def _get_platform(self):
        return OS_PLATFORM
    
    async def _get_ble_device(self, timeout=10.0):
        if OS_PLATFORM == "Windows" or OS_PLATFORM == "Linux":
            device = None
            async with BleakScanner() as scanner:
                start_time = time.time()
                while (time.time() - start_time) <= timeout:
                    await asyncio.sleep(2)
                    if scanner.discovered_devices_and_advertisement_data:
                        #loop through available devices and their advertisement info
                        for (mac_address,(discovered_device, advertisement_data)) in scanner.discovered_devices_and_advertisement_data.items():
                            #Check that mac address and uuid match json file
                            if((mac_address == self.address.upper()) and (len(advertisement_data.service_uuids) > 0) and (self.service_uuid == advertisement_data.service_uuids[0])):
                                device = discovered_device
                                return device

            if device == None:
                raise Exception('Could not find device with address: {} and service uuid: {}'.format(self.address, self.service_uuid))
        elif IS_ATLEAST_MAC_OS_12:
            device = None
            async with BleakScanner(service_uuids = [self.service_uuid]) as scanner:
                start_time = time.time()
                while (time.time() - start_time) <= timeout:
                    await asyncio.sleep(2)

                    if scanner.discovered_devices:
                        device = scanner.discovered_devices[0]
                        break
            if device == None:
                raise Exception('Could not find device with address: {} and service uuid: {}'.format(self.address, self.service_uuid))
            else:
                return device
        else:
            return self.address


    async def _connect(self):
        if self.client and self.client.is_connected:
            LOG.info("Already connected to a BLE device")
            return True
        else:
            LOG.info('Looking for Artemis Nano Peripheral Device: {}'.format(self.address))
            success = False
            device = await self._get_ble_device()
            self.client = BleakClient(device)
            try:
                await self.client.connect()
                success = True
            except Exception as e:
                self.error_msg = str(e)
                LOG.error(e)

            if self.client.is_connected:
                self.client.set_disconnected_callback(self.disconnect_handler)
                LOG.info("Connected to {}".format(self.address))
            
            return success
    
    async def _disconnect(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()
        else:
            raise Exception("Not connected to a BLE device")

    async def _write(self, uuid, byte_array):
        if self.client and self.client.is_connected:
            await self.client.write_gatt_char(uuid, byte_array,response=True)
        else:
            raise Exception("Not connected to a BLE device")

    async def _read(self, uuid):
        if self.client and self.client.is_connected:
            return await self.client.read_gatt_char(uuid)
        else:
            raise Exception("Not connected to a BLE device")

    async def _start_notify(self, uuid, notification_handler):
        if self.client and self.client.is_connected:
            await self.client.start_notify(uuid, notification_handler)
        else:
            raise Exception("Not connected to a BLE device")

    async def _stop_notify(self, uuid):
        if self.client and self.client.is_connected:
            await self.client.stop_notify(uuid)
        else:
            raise Exception("Not connected to a BLE device")

# Ref: https://github.com/hbldh/bleak/blob/develop/examples/service_explorer.py
    async def _explore_services(self):
        LOG.info(f"Connected to: {self.client.is_connected}")

        for service in self.client.services:
            LOG.info(f"[Service] {service}")
            for char in service.characteristics:
                if "read" in char.properties:
                    try:
                        value = bytes(await self.client.read_gatt_char(char.uuid))
                        LOG.info(
                            f"\t[Characteristic] {char} ({','.join(char.properties)}), Value: {value}"
                        )
                    except Exception as e:
                        LOG.error(
                            f"\t[Characteristic] {char} ({','.join(char.properties)}), Value: {e}"
                        )

                else:
                    value = None
                    LOG.info(
                        f"\t[Characteristic] {char} ({','.join(char.properties)}), Value: {value}"
                    )

                for descriptor in char.descriptors:
                    try:
                        value = bytes(
                            await self.client.read_gatt_descriptor(descriptor.handle)
                        )
                        LOG.info(f"\t\t[Descriptor] {descriptor}) | Value: {value}")
                    except Exception as e:
                        LOG.error(f"\t\t[Descriptor] {descriptor}) | Value: {e}")
