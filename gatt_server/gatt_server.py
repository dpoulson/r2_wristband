from __future__ import print_function
import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
import requests
import array
import functools
import urllib

try:
  from gi.repository import GObject
except ImportError:
  import gobject as GObject

from random import randint

import exceptions
import adapters

BLUEZ_SERVICE_NAME = 'org.bluez'
LE_ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE = 'org.freedesktop.DBus.Properties'

LE_ADVERTISEMENT_IFACE = 'org.bluez.LEAdvertisement1'

GATT_MANAGER_IFACE = 'org.bluez.GattManager1'

GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE =    'org.bluez.GattCharacteristic1'
GATT_DESC_IFACE =    'org.bluez.GattDescriptor1'


serviceUUID         = "0000fff0-0000-1000-8000-00805f9b34fb"
# The characteristic of the remote service we are interested in.
charMainBatteryUUID = "0000fff2-0000-1000-8000-00805f9b34fb"
charCtrlBatteryUUID = "0000fff3-0000-1000-8000-00805f9b34fb"
charVolumeUUID      = "0000fff4-0000-1000-8000-00805f9b34fb"
charDroidNameUUID   = "0000fff1-0000-1000-8000-00805f9b34fb"
charTempUUID        = "0000fff5-0000-1000-8000-00805f9b34fb"
charWifiUUID        = "0000fff6-0000-1000-8000-00805f9b34fb"
charInternetUUID    = "0000fff7-0000-1000-8000-00805f9b34fb"



class Application(dbus.service.Object):
    """
    org.bluez.GattApplication1 interface implementation
    """
    def __init__(self, bus):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)
        self.add_service(AstromechService(bus, 0))

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        print('GetManagedObjects')

        for service in self.services:
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
                descs = chrc.get_descriptors()
                for desc in descs:
                    response[desc.get_path()] = desc.get_properties()

        return response


class Service(dbus.service.Object):
    """
    org.bluez.GattService1 interface implementation
    """
    PATH_BASE = '/org/bluez/example/service'

    def __init__(self, bus, index, uuid, primary):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
                GATT_SERVICE_IFACE: {
                        'UUID': self.uuid,
                        'Primary': self.primary,
                        'Characteristics': dbus.Array(
                                self.get_characteristic_paths(),
                                signature='o')
                }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

    def get_characteristic_paths(self):
        result = []
        for chrc in self.characteristics:
            result.append(chrc.get_path())
        return result

    def get_characteristics(self):
        return self.characteristics

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_SERVICE_IFACE:
            raise exceptions.InvalidArgsException()

        return self.get_properties()[GATT_SERVICE_IFACE]


class Characteristic(dbus.service.Object):
    """
    org.bluez.GattCharacteristic1 interface implementation
    """
    def __init__(self, bus, index, uuid, flags, service):
        self.path = service.path + '/char' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.descriptors = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
                GATT_CHRC_IFACE: {
                        'Service': self.service.get_path(),
                        'UUID': self.uuid,
                        'Flags': self.flags,
                        'Descriptors': dbus.Array(
                                self.get_descriptor_paths(),
                                signature='o')
                }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_descriptor(self, descriptor):
        self.descriptors.append(descriptor)

    def get_descriptor_paths(self):
        result = []
        for desc in self.descriptors:
            result.append(desc.get_path())
        return result

    def get_descriptors(self):
        return self.descriptors

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise exceptions.InvalidArgsException()

        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE,
                        in_signature='a{sv}',
                        out_signature='ay')
    def ReadValue(self, options):
        print('Default ReadValue called, returning error')
        raise exceptions.NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        print('Default WriteValue called, returning error')
        raise exceptions.NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        print('Default StartNotify called, returning error')
        raise exceptions.NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        print('Default StopNotify called, returning error')
        raise exceptions.NotSupportedException()

    @dbus.service.signal(DBUS_PROP_IFACE,
                         signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated):
        pass


class Descriptor(dbus.service.Object):
    """
    org.bluez.GattDescriptor1 interface implementation
    """
    def __init__(self, bus, index, uuid, flags, characteristic):
        self.path = characteristic.path + '/desc' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.chrc = characteristic
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
                GATT_DESC_IFACE: {
                        'Characteristic': self.chrc.get_path(),
                        'UUID': self.uuid,
                        'Flags': self.flags,
                }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_DESC_IFACE:
            raise exceptions.InvalidArgsException()

        return self.get_properties()[GATT_DESC_IFACE]

    @dbus.service.method(GATT_DESC_IFACE,
                        in_signature='a{sv}',
                        out_signature='ay')
    def ReadValue(self, options):
        print('Default ReadValue called, returning error')
        raise exceptions.NotSupportedException()

    @dbus.service.method(GATT_DESC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        print('Default WriteValue called, returning error')
        raise exceptions.NotSupportedException()


class AstromechService(Service):
    """

    """
    def __init__(self, bus, index):
        Service.__init__(self, bus, index, serviceUUID, True)
        self.add_characteristic(mainBatteryChrc(bus, 0, self))
        self.add_characteristic(ctrlBatteryChrc(bus, 1, self))
        self.add_characteristic(volumeChrc(bus, 2, self))
        self.add_characteristic(droidNameChrc(bus, 3, self))
        self.add_characteristic(temperatureChrc(bus, 4, self))
        self.add_characteristic(wifiChrc(bus, 5, self))
        self.add_characteristic(internetChrc(bus, 6, self))

        self.energy_expended = 0


class mainBatteryChrc(Characteristic):

    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                charMainBatteryUUID,
                ['read'],
                service)

    def ReadValue(self, options):
        return [ 0x40 ]

class ctrlBatteryChrc(Characteristic):

    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                charCtrlBatteryUUID,
                ['read'],
                service)

    def ReadValue(self, options):
        return [ 0x31 ]

class volumeChrc(Characteristic):

    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                charVolumeUUID,
                ['read'],
                service)
        self.volume = 30
        GObject.timeout_add(5000, self.check_volume) 

    def ReadValue(self, options):
        print('Reading Volume: %s' % dbus.Byte(self.volume))
        return [ dbus.Byte(self.volume) ]

    def check_volume(self):
        print('Updating volume level')
        url = "http://localhost:5000/audio/volume"
        f = urllib.urlopen(url)
        cur_vol = float(f.readline())
        print('Volume level = %s' % cur_vol)
        self.volume = int(cur_vol*100)
        return True

class droidNameChrc(Characteristic):

    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                charDroidNameUUID,
                ['read'],
                service)

    def ReadValue(self, options):
        return [ 0x52, 0x32, 0x44, 0x32 ]

class temperatureChrc(Characteristic):

    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                charTempUUID,
                ['read'],
                service)

    def ReadValue(self, options):
        return [ 0x2E ]

class wifiChrc(Characteristic):

    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                charWifiUUID,
                ['read'],
                service)

    def ReadValue(self, options):
        return [ 0x01 ]

class internetChrc(Characteristic):

    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                charInternetUUID,
                ['read'],
                service)
        self.internet = 0
        GObject.timeout_add(5000, self.check_internet)

    def ReadValue(self, options):
        print('Reading Internet: %s' % dbus.Byte(self.internet))
        return [ dbus.Byte(self.internet) ]

    def check_internet(self):
        print('Checking Internet status')
        url = "http://localhost:5000/internet"
        f = urllib.urlopen(url)
        cur_internet = f.readline()
        if (cur_internet == "True"):
           self.internet = 1
        else: 
           self.internet = 0
        return True



class CharacteristicUserDescriptionDescriptor(Descriptor):
    """
    Writable CUD descriptor.

    """
    CUD_UUID = '2901'

    def __init__(self, bus, index, characteristic):
        self.writable = 'writable-auxiliaries' in characteristic.flags
        self.value = array.array('B', b'This is a characteristic for testing')
        self.value = self.value.tolist()
        Descriptor.__init__(
                self, bus, index,
                self.CUD_UUID,
                ['read', 'write'],
                characteristic)

    def ReadValue(self, options):
        return self.value

    def WriteValue(self, value, options):
        if not self.writable:
            raise exceptions.NotPermittedException()
        self.value = value

def register_app_cb():
    print('GATT application registered')


def register_app_error_cb(mainloop, error):
    print('Failed to register application: ' + str(error))
    mainloop.quit()


def gatt_server_main(mainloop, bus, adapter_name):
    adapter = adapters.find_adapter(bus, GATT_MANAGER_IFACE, adapter_name)
    if not adapter:
        raise Exception('GattManager1 interface not found')

    service_manager = dbus.Interface(
            bus.get_object(BLUEZ_SERVICE_NAME, adapter),
            GATT_MANAGER_IFACE)

    app = Application(bus)

    print('Registering GATT application...')

    service_manager.RegisterApplication(app.get_path(), {},
                                    reply_handler=register_app_cb,
                                    error_handler=functools.partial(register_app_error_cb, mainloop))

