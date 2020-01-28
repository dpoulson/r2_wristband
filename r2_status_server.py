# Standard modules
import os
import dbus
try:
    from gi.repository import GObject
except ImportError:
    import gobject as GObject

# Bluezero modules
from bluezero import constants
from bluezero import adapter
from bluezero import advertisement
from bluezero import localGATT
from bluezero import GATT

# constants
CPU_TMP_SRVC = '5320c666-9755-48fa-8ccc-f345d3f7db3f'
CPU_TMP_CHRC = 'beb54841-36e1-4688-b7f5-ea07361b26a8'
CPU_FMT_DSCP = '2904'

serviceUUID         = "0000fff0-0000-1000-8000-00805f9b34fb"
# The characteristic of the remote service we are interested in.
charMainBatteryUUID = "0000fff2-0000-1000-8000-00805f9b34fb"
charCtrlBatteryUUID = "0000fff3-0000-1000-8000-00805f9b34fb"
charVolumeUUID      = "0000fff4-0000-1000-8000-00805f9b34fb"
charDroidNameUUID   = "0000fff1-0000-1000-8000-00805f9b34fb"

class TemperatureChrc(localGATT.Characteristic):
    def __init__(self, service):
        localGATT.Characteristic.__init__(self,
                                          1,
                                          CPU_TMP_CHRC,
                                          service,
                                          ['21.4'],
                                          False,
                                          ['read', 'notify'])

    def temperature_cb(self):
        reading = ['22.22']
        print('Getting new temperature',
              reading,
              self.props[constants.GATT_CHRC_IFACE]['Notifying'])
        self.props[constants.GATT_CHRC_IFACE]['Value'] = reading

        self.PropertiesChanged(constants.GATT_CHRC_IFACE,
                               {'Value': dbus.Array(cpu_temp_sint16(reading))},
                               [])
        print('Array value: ', cpu_temp_sint16(reading))
        return self.props[constants.GATT_CHRC_IFACE]['Notifying']

    def _update_temp_value(self):
        if not self.props[constants.GATT_CHRC_IFACE]['Notifying']:
            return

        print('Starting timer event')
        GObject.timeout_add(500, self.temperature_cb)

    def ReadValue(self, options):
        reading = [get_cpu_temperature()]
        self.props[constants.GATT_CHRC_IFACE]['Value'] = reading
        return dbus.Array(
            cpu_temp_sint16(self.props[constants.GATT_CHRC_IFACE]['Value'])
        )

class ble:
    def __init__(self):
        self.bus = dbus.SystemBus()
        self.app = localGATT.Application()
        self.srv = localGATT.Service(1, CPU_TMP_SRVC, True)

        self.charc = TemperatureChrc(self.srv)

        self.charc.service = self.srv.path

        cpu_format_value = dbus.Array([dbus.Byte(0x0E),
                                       dbus.Byte(0xFE),
                                       dbus.Byte(0x2F),
                                       dbus.Byte(0x27),
                                       dbus.Byte(0x01),
                                       dbus.Byte(0x00),
                                       dbus.Byte(0x00)])
        self.cpu_format = localGATT.Descriptor(4,
                                               CPU_FMT_DSCP,
                                               self.charc,
                                               cpu_format_value,
                                               ['read'])

        self.app.add_managed_object(self.srv)
        self.app.add_managed_object(self.charc)
        self.app.add_managed_object(self.cpu_format)

        self.srv_mng = GATT.GattManager(adapter.list_adapters()[0])
        self.srv_mng.register_application(self.app, {})

        self.dongle = adapter.Adapter(adapter.list_adapters()[0])
        advert = advertisement.Advertisement(1, 'peripheral')

        advert.service_UUIDs = [CPU_TMP_SRVC]
        # eddystone_data = tools.url_to_advert(WEB_BLINKT, 0x10, TX_POWER)
        # advert.service_data = {EDDYSTONE: eddystone_data}
        if not self.dongle.powered:
            self.dongle.powered = True
        ad_manager = advertisement.AdvertisingManager(self.dongle.address)
        ad_manager.register_advertisement(advert, {})

    def add_call_back(self, callback):
        self.charc.PropertiesChanged = callback

    def start_bt(self):
        self.app.start()


if __name__ == '__main__':
    print("R2 Status BLE")
    r2_status = ble()
    r2_status.start_bt()

