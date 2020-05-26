"""
Tango device for setting MRF injection table for different trigger rates
for RF and kicker / chopper

Created 2020-05-19

@author: Filip Lindau
"""

from PyTango.server import Device, DeviceMeta
from PyTango.server import attribute, command
from PyTango.server import device_property
import PyTango as pt
import numpy as np
import math
import threading
import copy
import json

import logging
logger = logging.getLogger()
while len(logger.handlers):
    logger.removeHandler(logger.handlers[0])

f = logging.Formatter("%(asctime)s - %(module)s.   %(funcName)s - %(levelname)s - %(message)s")
fh = logging.StreamHandler()
fh.setFormatter(f)
logger.addHandler(fh)
logger.setLevel(logging.DEBUG)


class ReprateDS(pt.Device):
    __metaclass__ = DeviceMeta

    # --- Operator attributes
    #
    injection_table = attribute(label='injection table',
                                dtype=((int, ), ),
                                access=pt.AttrWriteType.READ,
                                unit="",
                                format="%d",
                                max_dim_x=3,
                                max_dim_y=1024,
                                fget="get_injection_table",
                                fisallowed="is_injection_table_allowed",
                                doc="Reconstructed pulse time intensity FWHM", )

    event_time_offset = attribute(label='event time offset',
                                  dtype=int,
                                  access=pt.AttrWriteType.READ_WRITE,
                                  unit="clocks",
                                  format="%d",
                                  min_value=-100000000,
                                  max_value=100000000,
                                  fget="get_event_offset",
                                  fset="set_event_offset",
                                  fisallowed="is_event_offset_allowed",
                                  doc="Time offset in clock cycles between the two events with different frequencies", )

    event0_frequency = attribute(label='event0 frequency',
                                 dtype=float,
                                 access=pt.AttrWriteType.READ_WRITE,
                                 unit="Hz",
                                 format="%2.1f",
                                 min_value=-1.0,
                                 max_value=100.0,
                                 fisallowed="is_event0_frequency_allowed",
                                 fget="get_event0_frequency",
                                 fset="set_event0_frequency",
                                 doc="Frequency for event 0", )

    # event1_frequency = attribute(label='event1 frequency',
    #                              dtype=float,
    #                              access=pt.AttrWriteType.READ_WRITE,
    #                              unit="Hz",
    #                              format="%2.1f",
    #                              min_value=-1.0,
    #                              max_value=100.0,
    #                              fget="get_event1_frequency",
    #                              fset="set_event1_frequency",
    #                              doc="Frequency for event 1", )

    event1_subfactor = attribute(label='event1 subfactor',
                                 dtype=int,
                                 access=pt.AttrWriteType.READ_WRITE,
                                 unit="",
                                 format="%d",
                                 min_value=1,
                                 max_value=1000,
                                 fget="get_event1_factor",
                                 fset="set_event1_factor",
                                 fisallowed="is_event1_factor_allowed",
                                 doc="Frequency for event 1 = f_0 / sub_factor", )

    # --- Device properties
    #
    evg_name = device_property(dtype=str,
                               doc="Device name of the EVG to be controlled",)

    n_max = device_property(dtype=int,
                            doc="Number of entries available in the injection table",
                            default_value=10)

    t0 = device_property(dtype=float,
                         doc="RF clock period in seconds",
                         default_value=10e-9)

    event0_code = device_property(dtype=int,
                                  doc="Event0 code transmitted by the MRF EVG",
                                  default_value=0xa0)

    event1_code = device_property(dtype=int,
                                  doc="Event1 code transmitted by the MRF EVG",
                                  default_value=0xa4)

    def __init__(self, klass, name):
        self.evg_device = None
        self.normal_sequence = None
        self.event_time_offset_data = None
        self.injection_table_data = None
        self.event0_frequency_data = None
        self.event1_frequency_data = None
        self.event1_factor_data = None

        self.data_lock = threading.Lock()

        Device.__init__(self, klass, name)

    def init_device(self):
        self.debug_stream("In init_device:")
        Device.init_device(self)

        try:
            self.evg_device = pt.DeviceProxy(self.evg_name)
        except pt.DevFailed as e:
            self.error_stream("Could not connect to device {0}. {1}".format(self.evg_name, e))
            self.evg_device = None
            self.set_state(pt.DevState.UNKNOWN)
        self.info_stream("Connected to EVG device {0}".format(self.evg_name))
        self.set_state(pt.DevState.ON)
        self.set_status("Connected to device {0}".format(self.evg_name))
        self.read_sequence_from_device2()

    def read_sequence_from_device(self):
        if self.evg_device is not None:
            seq_prop = self.evg_device.get_property("sequence")
            if not seq_prop["sequence"]:
                self.debug_stream("Property not set. Assume standard sequence")
                seq = "[0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9]"
            else:
                seq = seq_prop["sequence"][0]
            timestamp_list = list()
            enable_list = list()
            event_list = list()
            s_list = seq.strip("[]").split(",")
            attr_dict = dict()
            inj_table = list()
            n_s = max(10, len(s_list))
            try:
                for k in range(n_s):
                    t_name = "Event{0:2d}Timestamp".format(k)
                    e_name = "Event{0:2d}Enable".format(k)
                    ts = self.evg_device.read_attribute(t_name).value
                    en = self.evg_device.read_attribute(e_name).value
                    ev = int(s_list[k], 0)
                    timestamp_list.append(ts)
                    enable_list.append(en)
                    event_list.append(ev)
                    attr_dict[t_name] = ts
                    attr_dict[e_name] = en
                    inj_table.append([ts, ev, en])
                with self.data_lock:
                    self.normal_sequence = inj_table
                    self.injection_table_data = inj_table
            except pt.DevFailed as e:
                self.set_state(pt.DevState.UNKNOWN)
                self.set_status("Error reading from event generator device.")
                raise e
            return inj_table

    def read_sequence_from_device2(self):
        if self.evg_device is not None:
            try:
                inj_table_str = self.evg_device.read_attribute("injection_table_dump")
                inj_table = json.loads(inj_table_str)
                with self.data_lock:
                    self.normal_sequence = inj_table
                    self.injection_table_data = inj_table
            except pt.DevFailed as e:
                self.set_state(pt.DevState.UNKNOWN)
                self.set_status("Error reading from event generator device.")
                raise e
            return inj_table

    def generate_sequence(self, freq_list=[10, 2], ev_list=[0xa0, 0xa4]):
        f = np.array(freq_list)
        max_f = f.max()
        ih = f.argmax()
        il = f.argmin()
        fh = float(f[ih])
        fl = float(f[il])
        fc, mc, kc = self.find_common(f[0], f[1], self.n_max)
        tp = 1 / fc / self.t0
        ep = 0x7f
        th = np.arange(mc) / fh / self.t0
        tl = np.arange(kc) / fl / self.t0
        eh = ev_list[ih] * np.ones(mc)
        el = ev_list[il] * np.ones(kc)
        es = np.hstack((eh, el, ep)).astype(int)
        ts = np.hstack((th, tl, tp)).astype(int)
        i_sort = ts.argsort()
        ess = es[i_sort]
        tss = ts[i_sort]
        ev_seq = ",".join(([hex(e) for e in ess]))
        inj_table = list()
        for k, t in enumerate(tss):
            inj_table.append([t, ess[k], 1])
        return inj_table

    def generate_sequence_factor(self, f0, factor, event0, event1):
        f1 = np.double(f0) / factor
        t_mrf = self.t0
        tp = 1 / f1 / t_mrf           # Full cycle period in clocks
        ep = 0x7f                       # End sequence event code
        t0 = np.arange(factor) / f0 / t_mrf
        t1 = np.arange(1) / f1 / t_mrf
        e0 = event0 * np.ones(factor)
        e1 = event1 * np.ones(1)
        es = np.hstack((e0, e1, ep)).astype(int)
        ts = np.hstack((t0, t1, tp)).astype(int)
        i_sort = ts.argsort()
        ess = es[i_sort]
        tss = ts[i_sort]
        ev_seq = ",".join(([hex(e) for e in ess]))
        inj_table = list()
        for k, t in enumerate(tss):
            inj_table.append([t, ess[k], 1])
        return inj_table

    def write_sequence_to_device(self, inj_table):
        self.info_stream("Writing injection table to device")
        self.debug_stream("{0}".format(inj_table))
        # Extract data from table
        timestamp = [it[0] for it in inj_table]
        event = [hex(it[1]) for it in inj_table]
        enable = [it[2] for it in inj_table]
        # Stop injection
        try:
            self.evg_device.command_inout("inject_stop")
            # Set sequence property
            seq = "[{0}]".format(",".join((event)))
            self.evg_device.put_property({"sequence": seq})
            # Set attributes
            for k in range(len(timestamp)):
                t_name = "Event{0:2d}Timestamp".format(k)
                e_name = "Event{0:2d}Enable".format(k)
                self.evg_device.write_attribute(t_name, timestamp[k])
                self.evg_device.write_attribute(e_name, enable[k])
            # Init device to load sequence
            self.evg_device.command_inout("init")
            # Start injection
        except pt.DevFailed as e:
            self.set_state(pt.DevState.UNKNOWN)
            self.set_status("Error reading from event generator device.")
            raise e
        return True

    def find_common(self, f0, f1, n_max=100):
        fh = float(max(f0, f1))
        fl = float(min(f0, f1))
        k = np.arange(1, n_max + 1, dtype=int)
        m = k * fh / fl
        try:
            ind = np.where(np.modf(m)[0] == 0)[0][0]
            fc = fh / m[ind]
            mc = int(m[ind])
            kc = k[ind]
        except IndexError:
            fc = None
            mc = None
            kc = None
        return fc, mc, kc

    def find_primes(self, n):
        """

        :param n:
        :type n:
        :return:
        :rtype:
        """
        prime_list = list()
        m = n
        if m % 2 == 0:
            prime_list.append(2)
            while m % 2 == 0:
                m = m // 2

        d_max = np.ceil(np.sqrt(m)) + 1
        next_p = 3
        d = np.arange(3, d_max, 2, dtype=int)
        while True:

            logger.debug("m = {0}".format(m))
            logger.debug("d: {0}".format(d))
            logger.debug("d mod: {0}".format(d[np.modf(m / d)[0] == 0]))
            try:
                next_p = d[np.modf(m / d)[0] == 0][0]
            except IndexError:
                break
            prime_list.append(next_p)
            while m % next_p == 0:
                m = m // next_p
        prime_list.append(m)
        return prime_list

    def get_event_offset(self):
        return self.event_time_offset_data

    def set_event_offset(self, value):
        self.event_time_offset_data = value
        return True

    def is_event_offset_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_injection_table(self):
        with self.data_lock:
            inj_table = copy.copy(self.injection_table_data)
        return inj_table

    def is_injection_table_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_standard_table(self):
        with self.data_lock:
            inj_table = copy.copy(self.normal_sequence)
        return inj_table

    def is_standard_table_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_event0_frequency(self):
        with self.data_lock:
            f = self.event0_frequency_data
        return f

    def set_event0_frequency(self, value):
        with self.data_lock:
            self.event0_frequency_data = value
            inj_table = self.generate_sequence_factor(value, self.event1_factor_data,
                                                      self.event0_code, self.event1_code)
            self.injection_table_data = inj_table
        return True

    def is_event0_frequency_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_event1_frequency(self):
        return self.event1_frequency_data

    def set_event1_frequency(self, value):
        self.event1_frequency_data = value
        return True

    def get_event1_factor(self):
        with self.data_lock:
            f = self.event1_factor_data
        return f

    def set_event1_factor(self, value):
        with self.data_lock:
            self.event1_factor_data = value
            inj_table = self.generate_sequence_factor(self.event0_frequency_data, value,
                                                      self.event0_code, self.event1_code)
            self.injection_table_data = inj_table
        return True

    def is_event1_factor_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    @command(dtype_in=None)
    def set_dual_reprate_table(self):
        inj_table = self.get_injection_table()
        self.write_sequence_to_device(inj_table)
        self.set_state(pt.DevState.RUNNING)
        self.set_status("Using dual reprate {0} + {1} Hz".format(self.event0_frequency_data,
                                                                 self.event0_frequency_data / self.event1_factor_data))

    @command(dtype_in=None)
    def set_standard_table(self):
        inj_table = self.get_standard_table()
        self.write_sequence_to_device(inj_table)
        self.set_state(pt.DevState.RUNNING)
        self.set_status("Using standard injection table {0}".format(inj_table))


if __name__ == "__main__":
    pt.server.server_run((ReprateDS, ))
