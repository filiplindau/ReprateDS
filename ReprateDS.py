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


class GPIODummy(object):
    def __init__(self):
        self.IN = "IN"
        self.OUT = "OUT"
        self.BOARD = "BOARD"
        self.BCM = "BCM"
        self.HIGH = "HIGH"
        self.LOW = "LOW"

    def output(self, pin_list, value_list):
        logger.info("Raspberry output pins: {0} to {1}".format(pin_list, value_list))

    def input(self, pin_list, value_list):
        logger.info("Raspberry input pins: {0} to {1}".format(pin_list, value_list))

    def setup(self, channel_list, dir_list):
        logger.info("Raspberry setup: {0} direction {1}".format(channel_list, dir_list))

    def setmode(self, mode):
        logger.info("Raspberry mode: {0}".format(mode))


try:
    import RPi.GPIO as gpio
except ImportError:
    gpio = GPIODummy()


class ReprateDS(Device):
    __metaclass__ = DeviceMeta

    # --- Operator attributes
    #

    inj_table_r1 = attribute(label='injection table for R1',
                                   # dtype=((int, ), ),
                                   dtype=str,
                                   access=pt.AttrWriteType.READ,
                                   unit="",
                                   format="%s",
                                   #max_dim_x=3,
                                   #max_dim_y=1024,
                                   fget="get_injection_table_r1",
                                   doc="Injection table used for R1",
                                   )

    std_table_r1 = attribute(label='standard table for R1',
                                   # dtype=((int, ), ),
                                   dtype=str,
                                   access=pt.AttrWriteType.READ,
                                   unit="",
                                   format="%s",
                                   #max_dim_x=3,
                                   #max_dim_y=1024,
                                   fget="get_standard_table_r1",
                                   doc="Standard injection table used for R1",
                                   )

    inj_table_r3 = attribute(label='injection table for R3',
                                # dtype=((int, ), ),
                                dtype=str,
                                access=pt.AttrWriteType.READ,
                                unit="",
                                format="%s",
                                # max_dim_x=3,
                                # max_dim_y=1024,
                                fget="get_injection_table_r3",
                                doc="Injection table used for R3",
                                )

    std_table_r3 = attribute(label='standard table for R3',
                                   # dtype=((int, ), ),
                                   dtype=str,
                                   access=pt.AttrWriteType.READ,
                                   unit="",
                                   format="%s",
                                   #max_dim_x=3,
                                   #max_dim_y=1024,
                                   fget="get_standard_table_r3",
                                   doc="Standard injection table used for R3",
                                   )

    ev_offset_r1 = attribute(label='event time offset R1',
                                     dtype=int,
                                     access=pt.AttrWriteType.READ_WRITE,
                                     unit="clocks",
                                     format="%d",
                                     min_value=-100000000,
                                     max_value=100000000,
                                     fisallowed="",
                                     fget="get_ev_offset_r1",
                                     fset="set_ev_offset_r1",

                                     doc="Time offset in clock cycles between the two events with different frequencies",
                                     memorized=True,
                                     hw_memorized=True,
                                     )

    ev_offset_r3 = attribute(label='event time offset R3',
                                     dtype=int,
                                     access=pt.AttrWriteType.READ_WRITE,
                                     unit="clocks",
                                     format="%d",
                                     min_value=-100000000,
                                     max_value=100000000,
                                     fisallowed="",
                                     fget="get_ev_offset_r3",
                                     fset="set_ev_offset_r3",

                                     doc="Time offset in clock cycles between the two events with different frequencies",
                                     memorized=True,
                                     hw_memorized=True,
                                     )

    # event_time_offset_r3 = attribute(label='event time offset R3',
    #                                  dtype=int,
    #                                  access=pt.AttrWriteType.READ_WRITE,
    #                                  unit="clocks",
    #                                  format="%d",
    #                                  min_value=-100000000,
    #                                  max_value=100000000,
    #                                  fget="get_event_offset_r3",
    #                                  fset="set_event_offset_r3",
    #                                  fisallowed="is_event_offset_r3_allowed",
    #                                  doc="Time offset in clock cycles between the two events with different frequencies",
    #                                  memorized=True,
    #                                  hw_memorized=True,)

    event0_frequency_r1 = attribute(label='R1 event0 frequency',
                                    dtype=float,
                                    access=pt.AttrWriteType.READ_WRITE,
                                    unit="Hz",
                                    format="%2.1f",
                                    min_value=-1.0,
                                    max_value=100.0,
                                    fisallowed="",
                                    fget="get_event0_frequency_r1",
                                    fset="set_event0_frequency_r1",
                                    doc="Frequency for event 0",
                                    memorized=True,
                                    hw_memorized=True,)

    event0_frequency_r3 = attribute(label='R3 event0 frequency',
                                    dtype=float,
                                    access=pt.AttrWriteType.READ_WRITE,
                                    unit="Hz",
                                    format="%2.1f",
                                    min_value=-1.0,
                                    max_value=100.0,
                                    fisallowed="",
                                    fget="get_event0_frequency_r3",
                                    fset="set_event0_frequency_r3",
                                    doc="Frequency for event 0",
                                    memorized=True,
                                    hw_memorized=True,
                                    )

    event1_subfactor_r1 = attribute(label='event1 subfactor for R1',
                                    dtype=int,
                                    access=pt.AttrWriteType.READ_WRITE,
                                    unit="",
                                    format="%d",
                                    min_value=1,
                                    max_value=1000,
                                    fget="get_event1_factor_r1",
                                    fset="set_event1_factor_r1",
                                    fisallowed="",
                                    doc="Frequency for event 1 = f_0 / sub_factor",
                                    memorized=True,
                                    hw_memorized=True,
                                    )

    event1_subfactor_r3 = attribute(label='event1 subfactor for R3',
                                    dtype=int,
                                    access=pt.AttrWriteType.READ_WRITE,
                                    unit="",
                                    format="%d",
                                    min_value=1,
                                    max_value=1000,
                                    fget="get_event1_factor_r3",
                                    fset="set_event1_factor_r3",
                                    fisallowed="",
                                    doc="Frequency for event 1 = f_0 / sub_factor",
                                    memorized=True,
                                    hw_memorized=True,
                                    )

    current_ring = attribute(label='current ring target',
                             dtype=str,
                             access=pt.AttrWriteType.READ,
                             unit="",
                             format="%s",
                             fget="get_current_ring",
                             doc="Frequency for event 1 = f_0 / sub_factor", )

    # --- Device properties
    #
    evg_name_r1 = device_property(dtype=str,
                                  doc="Device name of the EVG to be controlled for R1",)

    evg_name_r3 = device_property(dtype=str,
                                  doc="Device name of the EVG to be controlled for R3",)

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

    data_pins = device_property(dtype=pt.DevVarShortArray,
                                doc="RPi pin numbers of the 2 data bits for switches",
                                default_value=[16, 29])

    ref_std_table_r1 = device_property(dtype=str,
                                       doc="Reference table for standard injection into R1",
                                       update_db=True,
                                       default_value="[[30, 160, 1]\n "
                                                     "[18500, 161, 1]\n "
                                                     "[3, 162, 0]\n "
                                                     "[4, 163, 0]\n "
                                                     "[5, 164, 0]\n "
                                                     "[6, 165, 0]\n "
                                                     "[7, 166, 0]\n "
                                                     "[8, 167, 0]\n "
                                                     "[9, 168, 0]\n "
                                                     "[10, 169, 0]]")

    ref_std_table_r3 = device_property(dtype=str,
                                       doc="Reference table for standard injection into R3",
                                       update_db=True,
                                       default_value="[[136, 160, 1]\n "
                                                     "[10000, 161, 1]\n "
                                                     "[3, 164, 0]\n "
                                                     "[4, 160, 0]\n "
                                                     "[5, 161, 0]\n "
                                                     "[6, 160, 0]\n "
                                                     "[7, 161, 0]\n "
                                                     "[8, 164, 0]\n "
                                                     "[9, 160, 0]\n "
                                                     "[10, 161, 0]]")

    def __init__(self, klass, name):
        self.evg_device_r1 = None
        self.evg_device_r3 = None
        self.normal_sequence_r1_data = None
        self.normal_sequence_r3_data = None
        self.event_time_offset_r1_data = None
        self.event_time_offset_r3_data = None
        self.injection_table_r1_data = None
        self.injection_table_r3_data = None
        self.event0_frequency_r1_data = None
        self.event0_frequency_r3_data = None
        self.event1_frequency_r1_data = None
        self.event1_frequency_r3_data = None
        self.event1_factor_r1_data = None
        self.event1_factor_r3_data = None
        self.current_ring_data = "R1"

        self.status_r1_table = "standard"
        self.status_r3_table = "standard"

        self.data_lock = threading.Lock()

        Device.__init__(self, klass, name)

    def init_device(self):
        """
        Init device:
        Connect to EVG device servers and read their current injection tables.
        Setup GPIO pins for switch control.

        :return:
        :rtype:
        """
        self.debug_stream("In init_device:")
        Device.init_device(self)

        try:
            self.evg_device_r1 = pt.DeviceProxy(self.evg_name_r1)
        except pt.DevFailed as e:
            self.error_stream("Could not connect to device {0}. {1}".format(self.evg_name_r1, e))
            self.evg_device_r1 = None
            self.set_state(pt.DevState.UNKNOWN)
            return
        self.info_stream("Connected to EVG device {0}".format(self.evg_name_r1))

        try:
            self.evg_device_r3 = pt.DeviceProxy(self.evg_name_r3)
        except pt.DevFailed as e:
            self.error_stream("Could not connect to device {0}. {1}".format(self.evg_name_r3, e))
            self.evg_device_r3 = None
            self.set_state(pt.DevState.UNKNOWN)
            return
        self.info_stream("Connected to EVG device {0}".format(self.evg_name_r3))
        self.set_state(pt.DevState.ON)
        self.set_status("Connected to devices {0}, {1}".format(self.evg_name_r1, self.evg_name_r3))
        self.read_inj_table_from_device("r1")
        self.read_inj_table_from_device("r3")
        gpio.setmode(gpio.BOARD)
        gpio.setup(self.data_pins[0], gpio.OUT)
        gpio.setup(self.data_pins[1], gpio.OUT)

    def _gen_status(self):
        """
        Generate and set status string from current injection table and ring selections
        :return:
        :rtype:
        """
        s = "R1 table: {0}\n" \
            "R3 table: {1}\n" \
            "Target: {2}\n\n" \
            "{3}".format(self.status_r1_table.upper(), self.status_r3_table.upper(),
                         self.current_ring_data.upper(), self.state())
        self.set_status(s)
        return True

    def read_sequence_from_device(self):
        """
        Read injection table from EVG using the sequence property and timestamp, enable attributes.
        Also stores this table in normal_sequence_xx_data for later write-back to device.

        Not efficient. Use read_inj_table_from_device instead.

        :return: Injection table
        :rtype:
        """
        if self.evg_device_r1 is not None:
            seq_prop = self.evg_device_r1.get_property("sequence")
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
                    ts = self.evg_device_r1.read_attribute(t_name).value
                    en = self.evg_device_r1.read_attribute(e_name).value
                    ev = int(s_list[k], 0)
                    timestamp_list.append(ts)
                    enable_list.append(en)
                    event_list.append(ev)
                    attr_dict[t_name] = ts
                    attr_dict[e_name] = en
                    inj_table.append([ts, ev, en])
                with self.data_lock:
                    self.normal_sequence_r1_data = inj_table
                    self.injection_table_r1_data = inj_table
            except pt.DevFailed as e:
                self.set_state(pt.DevState.UNKNOWN)
                self.set_status("Error reading from event generator device.")
                raise e
            return inj_table

    def read_inj_table_from_device(self, ring):
        """
        Read injection table from EVG using the injection_table_dump attribute.
        Also stores this table in normal_sequence_xx_data for later write-back to device.

        :param ring: Which ring to read from, r1 or r3
        :type ring: str
        :return: Injection table, list of lists:
            [[timestamp0, event code0, enable0],
             [timestamp1, event code1, enable1],
             [timestamp2, event code2, enable2],
             [timestamp3, event code3, enable3...]]
        :rtype:
        """
        if ring == "r1":
            if self.evg_device_r1 is not None:
                try:
                    inj_table_str = self.evg_device_r1.read_attribute("injection_table_dump").value
                    inj_table = json.loads(inj_table_str.replace("\n", ",")[1:])
                    with self.data_lock:
                        self.normal_sequence_r1_data = inj_table
                        self.injection_table_r1_data = inj_table
                    self.info_stream("R1 Injection table: {0}".format(self.injection_table_r1_data))
                except pt.DevFailed as e:
                    self.set_state(pt.DevState.UNKNOWN)
                    self.set_status("Error reading from event generator device {0}.".format(self.evg_name_r1))
                    raise e
            else:
                return None
        else:
            if self.evg_device_r3 is not None:
                try:
                    inj_table_str = self.evg_device_r3.read_attribute("injection_table_dump").value
                    inj_table = json.loads(inj_table_str.replace("\n", ",")[1:])
                    with self.data_lock:
                        self.normal_sequence_r3_data = inj_table
                        self.injection_table_r3_data = inj_table
                    self.info_stream("R3 Injection table: {0}".format(self.injection_table_r3_data))
                except pt.DevFailed as e:
                    self.set_state(pt.DevState.UNKNOWN)
                    self.set_status("Error reading from event generator device {0}.".format(self.evg_name_r3))
                    raise e
            else:
                return None
        return inj_table

    def generate_sequence_freq(self, freq_list=[10, 2], ev_list=[0xa0, 0xa4]):
        """
        Generate injection table from two frequencies and corresponding event codes.
        :param freq_list: 2-element list of frequencies to use
        :type freq_list: List
        :param ev_list: 2-element list of event codes to use
        :type ev_list: List
        :return: Injection table, list of lists:
            [[timestamp0, event code0, enable0],
             [timestamp1, event code1, enable1],
             [timestamp2, event code2, enable2],
             [timestamp3, event code3, enable3...]]
        :rtype:
        """
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
        """
        Generate injection table from event0 (higher frequency) and a divider factor to get f1=f0/factor
        :param f0: Event0 freuqency
        :type f0: float
        :param factor: Divider
        :type factor: int
        :param event0: Event code for f0 e.g. 0xa0
        :type event0: int
        :param event1: Event code for f1 e.g. 0xa4
        :type event1: int
        :return: Injection table, list of lists:
            [[timestamp0, event code0, enable0],
             [timestamp1, event code1, enable1],
             [timestamp2, event code2, enable2],
             [timestamp3, event code3, enable3...]]
        :rtype: List of lists of int
        """
        if f0 is None:
            f0 = 2.0
        if factor is None:
            factor = 1
        f1 = np.double(f0) / factor
        t_mrf = self.t0                 # MRF clock period
        tp = 1 / f1 / t_mrf             # Full cycle period in clocks
        ep = 0x7f                       # End sequence event code
        t0 = np.arange(factor) / f0 / t_mrf + 1
        t1 = np.arange(1) / f1 / t_mrf + 1
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
        self.debug_stream("Generate sequence:\n {0}\n len {1}".format(inj_table, len(inj_table)))
        return inj_table

    def write_sequence_to_device(self, inj_table, ring):
        """
        Write injection table sequence to a ring EVG. Timestamp data in clock cycles.

        :param inj_table: Injection table to write, list of lists:
            [[timestamp0, event code0, enable0],
             [timestamp1, event code1, enable1],
             [timestamp2, event code2, enable2],
             [timestamp3, event code3, enable3...]]

             Max 10 entries

        :type inj_table: List of lists of ints
        :param ring: Ring EVG selection, R1 or R3
        :type ring: str
        :return:
        :rtype:
        """
        self.info_stream("Writing injection table to device")
        self.debug_stream("{0}, len {1}".format(inj_table, len(inj_table)))
        # Extract data from table
        timestamp = [it[0] for it in inj_table]
        event = [hex(it[1]) for it in inj_table]
        enable = [it[2] for it in inj_table]

        attr_len = len(timestamp)
        if attr_len > 10:
            attr_len = 10
        timestamp = timestamp[:attr_len]
        event = event[:attr_len]
        enable = enable[:attr_len]

        self.debug_stream("\nTimestamps: {0}\nEvents: {1}\nEnable: {2}".format(timestamp, event, enable))
        if ring.lower() == "r1":
            dev = self.evg_device_r1
        else:
            dev = self.evg_device_r3

        # Stop injection
        try:
            dev.command_inout("inject_stop")
            self.info_stream("{0} Injection stopped".format(ring))
        except pt.DevFailed as e:
            self.set_state(pt.DevState.UNKNOWN)
            self.set_status("Error stopping {1} injection. {0}".format(e, ring))
            self.error_stream("Error stopping {1} injection. {0}".format(e, ring))
            raise e

        # Set sequence property
        seq = "[{0}]".format(",".join((event)))
        self.debug_stream("Sequence: {0}".format(seq))
        try:
            dev.put_property({"sequence": seq})
            self.info_stream("{0} Sequence property".format(ring))
        except pt.DevFailed as e:
            self.set_state(pt.DevState.UNKNOWN)
            self.set_status("Error {1} sequence property. {0}".format(e, ring))
            self.error_stream("Error {1} sequence property. {0}".format(e, ring))
            raise e

        # Set attributes
        try:
            for k in range(len(timestamp)):
                t_name = "Event{0:02d}Timestamp".format(k)
                e_name = "Event{0:02d}Enable".format(k)
                self.debug_stream("Writing {0} to timestamp {1}, type {2}".format(timestamp[k], t_name, type(timestamp[k])))
                dev.write_attribute(t_name, int(timestamp[k]))
                dev.write_attribute(e_name, int(enable[k]))
        except pt.DevFailed as e:
            self.set_state(pt.DevState.UNKNOWN)
            self.set_status("Error R1 timestamp, enable attributes. {0}".format(e))
            self.error_stream("Error R1 timestamp, enable attributes. {0}".format(e))
            raise e
        self.info_stream("{0} Timestamps and enable written".format(ring))

        # Init device to load sequence
        try:
            dev.command_inout("init")
        except pt.DevFailed as e:
            self.set_state(pt.DevState.UNKNOWN)
            self.set_status("Error R1 init. {0}".format(e))
            self.error_stream("Error R1 init. {0}".format(e))
            raise e
        self.info_stream("{0} Generator init".format(ring))
        # Start injection

        return True

    def find_common(self, f0, f1, n_max=100):
        """
        Find common divisors of two frequencies

        :param f0:
        :type f0:
        :param f1:
        :type f1:
        :param n_max:
        :type n_max:
        :return:
        :rtype:
        """
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
        Find prime numbers in integer n.
        For factorizing.

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

    def get_ev_offset_r1(self):
        print("offset")
        return self.event_time_offset_r1_data

    def set_ev_offset_r1(self, value):
        self.info_stream("Set event offset R1: {0}".format(value))
        self.event_time_offset_r1_data = value
        return True

    def is_ev_offset_r1_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_ev_offset_r3(self):
        print("offset")
        return self.event_time_offset_r3_data

    def set_ev_offset_r3(self, value):
        self.info_stream("Set event offset R3: {0}".format(value))
        self.event_time_offset_r3_data = value
        return True

    def is_ev_offset_r1_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_event_offset_r1(self):
        print("offset")
        return 10

    def set_event_offset_r1(self, value):
        self.info_stream("Set event offset R1: {0}".format(value))
        self.event_time_offset_r1_data = value
        return True

    def is_event_offset_r1_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_event_offset_r3(self):
        return self.event_time_offset_r3_data

    def set_event_offset_r3(self, value):
        self.info_stream("Set event offset R3: {0}".format(value))
        self.event_time_offset_r3_data = value
        return True

    def is_event_offset_r3_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_injection_table_r1(self):
        with self.data_lock:
            inj_table = copy.copy(self.injection_table_r1_data)
        return str(inj_table)

    def is_injection_table_r1_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_injection_table_r3(self):
        print("test")
        self.info_stream("Injection table R3: {0}".format(self.injection_table_r3_data))
        with self.data_lock:
            inj_table = copy.copy(self.injection_table_r3_data)
        self.info_stream("String: \n {0}".format(str(inj_table)))
        return str(inj_table)
        #return "test"

    def is_injection_table_r3_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_injection_table(self, ring):
        if ring.lower() == "r1":
            s = self.injection_table_r1_data
        else:
            s = self.injection_table_r3_data
        self.info_stream("inj tab:\n {0}".format(s))
        return str(s)

    def get_standard_table_r1(self):
        with self.data_lock:
            inj_table = copy.copy(self.normal_sequence_r1_data)
        return str(inj_table)

    def is_standard_table_r1_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_standard_table_r3(self):
        with self.data_lock:
            inj_table = copy.copy(self.normal_sequence_r3_data)
        return str(inj_table)

    def is_standard_table_r3_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_event0_frequency_r1(self):
        with self.data_lock:
            f = self.event0_frequency_r1_data
        return f

    def set_event0_frequency_r1(self, value):
        """
        Set the event0 frequency for ring r1
        :param value: frequency
        :type value: float
        :return:
        :rtype:
        """
        self.info_stream("Set event frequency R1: {0}".format(value))
        with self.data_lock:
            self.event0_frequency_r1_data = value
            inj_table = self.generate_sequence_factor(value, self.event1_factor_r1_data,
                                                      self.event0_code, self.event1_code)
            self.injection_table_r1_data = inj_table
        self._gen_status()
        return True

    def is_event0_frequency_r1_allowed(self):
        print("is_event0_frequency_r1_allowed")
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_event0_frequency_r3(self):
        with self.data_lock:
            f = self.event0_frequency_r3_data
        return f

    def set_event0_frequency_r3(self, value):
        """
        Set the event0 frequency for ring r3
        :param value: frequency
        :type value: float
        :return:
        :rtype:
        """
        self.info_stream("Set event frequency R3: {0}".format(value))
        with self.data_lock:
            self.event0_frequency_r3_data = value
            inj_table = self.generate_sequence_factor(value, self.event1_factor_r3_data,
                                                      self.event0_code, self.event1_code)
            self.injection_table_r3_data = inj_table
        self._gen_status()
        return True

    def is_event0_frequency_r3_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_event1_factor_r1(self):
        with self.data_lock:
            f = self.event1_factor_r1_data
        return f

    def set_event1_factor_r1(self, value):
        """
        Set the integer factor to divide R1 event0 frequency with to generate secondary frequency f1=f0/value
        :param value: Divider
        :type value: int
        :return:
        :rtype:
        """
        self.info_stream("Set event factor R1: {0}".format(value))
        with self.data_lock:
            self.event1_factor_r1_data = value
            inj_table = self.generate_sequence_factor(self.event0_frequency_r1_data, value,
                                                      self.event0_code, self.event1_code)
            self.injection_table_r1_data = inj_table
        self._gen_status()
        return True

    def is_event1_factor_r1_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_event1_factor_r3(self):
        with self.data_lock:
            f = self.event1_factor_r3_data
        return f

    def set_event1_factor_r3(self, value):
        """
        Set the integer factor to divide R3 event0 frequency with to generate secondary frequency f1=f0/value
        :param value: Divider
        :type value: int
        :return:
        :rtype:
        """
        self.info_stream("Set event factor R3: {0}".format(value))
        with self.data_lock:
            self.event1_factor_r3_data = value
            inj_table = self.generate_sequence_factor(self.event0_frequency_r3_data, value,
                                                      self.event0_code, self.event1_code)
            self.injection_table_r3_data = inj_table
        self._gen_status()
        return True

    def is_event1_factor_r3_allowed(self):
        if self.get_state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            return True
        else:
            return False

    def get_current_ring(self):
        with self.data_lock:
            f = self.current_ring_data
        return f

    @command(dtype_in=None, doc_in="Set configured dual reprate table to EVG of ring R1")
    def set_dual_reprate_table_to_r1(self):
        if self.state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            inj_table = self.injection_table_r1_data
            self.write_sequence_to_device(inj_table, "r1")
            self.set_state(pt.DevState.RUNNING)
            self.status_r1_table = "DUAL REPRATE {0} + {1} Hz".format(self.event0_frequency_r1_data,
                                                                      self.event0_frequency_r1_data / self.event1_factor_r1_data)
            self._gen_status()

    @command(dtype_in=None, doc_in="Set configured dual reprate table to EVG of ring R3")
    def set_dual_reprate_table_to_r3(self):
        if self.state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            inj_table = self.injection_table_r3_data
            self.write_sequence_to_device(inj_table, "r3")
            self.set_state(pt.DevState.RUNNING)
            self.status_r3_table = "DUAL REPRATE {0} + {1} Hz".format(self.event0_frequency_r3_data,
                                                                      self.event0_frequency_r3_data / self.event1_factor_r3_data)
            self._gen_status()

    @command(dtype_in=None, doc_in="Set standard injection table (it was sampled at init of this device) to EVG of ring R1")
    def set_current_std_table_to_r1(self):
        if self.state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            inj_table = self.normal_sequence_r1_data
            self.write_sequence_to_device(inj_table, "r1")
            self.set_state(pt.DevState.RUNNING)
            self.status_r3_table = "STANDARD"
            self._gen_status()

    @command(dtype_in=None, doc_in="Set standard injection table (it was sampled at init of this device) to EVG of ring R3")
    def set_current_std_table_to_r3(self):
        if self.state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            inj_table = self.normal_sequence_r3_data
            self.write_sequence_to_device(inj_table, "r3")
            self.set_state(pt.DevState.RUNNING)
            self.status_r3_table = "STANDARD"
            self._gen_status()

    @command(dtype_in=str, doc_in="Set which ring's EVG to use to trigger accelerator. R1 or R3.")
    def set_ring_injection_target(self, ring="r1"):
        if self.state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            with self.data_lock:
                self.current_ring_data = ring
                if ring.lower() == "r1":
                    gpio.output(self.data_pins[0], 1)
                    gpio.output(self.data_pins[1], 0)
                else:
                    gpio.output(self.data_pins[0], 0)
                    gpio.output(self.data_pins[1], 1)
            self.set_state(pt.DevState.RUNNING)
            self._gen_status()

    @command(dtype_in=None, doc_in="Store R1 standard injection table (it was sampled at init of this device) in property ref_std_table_r1")
    def store_current_std_table_to_ref_r1(self):
        s = str(self.normal_sequence_r1_data)
        self.ref_std_table_r1 = s

    @command(dtype_in=None, doc_in="Store R3 standard injection table (it was sampled at init of this device) in property ref_std_table_r3")
    def store_current_std_table_to_ref_r3(self):
        s = str(self.normal_sequence_r3_data)
        self.ref_std_table_r3 = s

    @command(dtype_in=None, doc_in="Set injection table stored in property ref_std_table_r1 to EVG of ring R1")
    def set_ref_table_to_r1(self):
        if self.state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            self.write_sequence_to_device(json.loads(self.ref_std_table_r1), "r1")
            self.status_r1_table = "REFERENCE"
            self._gen_status()

    @command(dtype_in=None, doc_in="Set injection table stored in property ref_std_table_r3 to EVG of ring R3")
    def set_ref_table_to_r3(self):
        if self.state() in [pt.DevState.ON, pt.DevState.RUNNING]:
            self.write_sequence_to_device(json.loads(self.ref_std_table_r3), "r3")
            self.status_r3_table = "REFERENCE"
            self._gen_status()


if __name__ == "__main__":
    pt.server.server_run((ReprateDS, ))
