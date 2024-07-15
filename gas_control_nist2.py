"""Valves, temperature, and Mass flow control module

__author__ = "Jorge Moncada Vivas"
__version__ = "1.0"
__email__ = "moncadaja@gmail.com"

Notes:
By Jorge Moncada Vivas and contributions of Ryuichi Shimogawa
"""

import os
import time
import serial
from serial.tools import list_ports
import time
from datetime import datetime

import propar
import minimalmodbus


# The HID for the valve 3. This is should ideally be a specified in a different file.
# HID_VALVE = "USB VID:PID=067B:2303 SER= LOCATION=1-11" #RS232
HID_VALVE = "COM10" #RS485
HID_MFC = "COM9"
BAUD_MFC = 38400
HID_TMP = "COM8"
SUB_ADD_TMP = 2

# This is a dictionary that maps the valve position and ID to an integer.
VALVE_POSITION = {"A": 0, "B": 1, "Unknown": 1, "pulse": 0, "cont": 1, "mix": 1}
VALVE_ID = {"A": 1, "B": 2, "C": 3, "D": 4, "E": 5, "F": 6, "G": 7, "H": 8, "I": 9}


class GasControl:
    def __init__(
        self,
        valves_hid: str = HID_VALVE,
        valves_comport: str = None,
        num_valves=9,
        mfc_hid: str = HID_MFC,
        mfc_comport: str = None,
        mfc_baud: int = BAUD_MFC,
        tmp_hid: str = HID_TMP,
        tmp_comport: str = None,
        sub_address_tmp: int = SUB_ADD_TMP,
    ) -> None:
        """Initialize the valve control device
        You can specify the HID or the comport of the valve control device
        It will print the available comports if no comport is specified

        Args:
            valves_hid (str): HID of the valve control device, you can also specify the name or hid of the comport [default: HID_VALVE]
            valves_comport (str): Comport of the valve control device [default: None]
            num_valves (int): Number of valves connected to the valve control device [default: 9]
            mfc_hid (str): HID of the mfc device, you can also specify the name or hid of the comport [default: HID_MFC]
            mfc_comport (str): Comport of the mfc device [default: None]
            tmp_hid (str): HID of the temperature controller device, you can also specify the name or hid of the comport [default: HID_TMP]
            tmp_comport (str): Comport of the temperature controller device [default: None]
        """

        self.status: list[str] = [None] * num_valves

        self.valves_hid: str = valves_hid
        self.valves_comport: str = valves_comport
        self.init_valves_comport()
        print("Valve comport: {}".format(self.valves_comport))
        self.serial_connection_valves()

        self.mfc_hid: str = mfc_hid
        self.mfc_comport: str = mfc_comport
        self.init_mfc_comport()
        print("MFC comport: {}".format(self.mfc_comport))
        self.mfc_master = propar.master(self.mfc_comport, 38400)
        self.define_flowsms()

        self.tmp_hid: str = tmp_hid
        self.tmp_comport: str = tmp_comport
        self.sub_address_tmp: int = sub_address_tmp
        self.init_tmp_comport()
        print("TMP comport: {}".format(self.tmp_comport))
        self.tmp_master = minimalmodbus.Instrument(self.tmp_comport, self.sub_address_tmp)

    def init_valves_comport(self):
        """Initialize the comport of the valve control device
        It will print the available comports if no comport is specified
        """

        if self.valves_hid:
            valves_port = list_ports.grep(self.valves_hid)
            valves_port = list(valves_port)

            if (len(valves_port) == 0) and (self.valves_comport is None):
                self.print_available_comports()

                raise ValueError(
                    "No comport found for valves_hid: {}".format(self.valves_hid)
                )
            elif len(valves_port) == 1:
                self.valves_comport = valves_port[0].device
            else:
                self.print_available_comports()
                raise ValueError(
                    "Multiple comports found for valves_hid: {}".format(
                        self.valves_hid
                    )
                )

        if self.valves_comport is None:
            self.print_available_comports()
            raise ValueError("No comport specified")

    def init_mfc_comport(self):
        """Initialize the comport of the mfc device
        It will print the available comports if no comport is specified
        """

        if self.mfc_hid:
            mfc_port = list_ports.grep(self.mfc_hid)
            mfc_port = list(mfc_port)

            if (len(mfc_port) == 0) and (self.mfc_comport is None):
                self.print_available_comports()

                raise ValueError(
                    "No comport found for mfc_hid: {}".format(self.mfc_hid)
                )
            elif len(mfc_port) == 1:
                self.mfc_comport = mfc_port[0].device
            else:
                self.print_available_comports()
                raise ValueError(
                    "Multiple comports found for mfc_hid: {}".format(self.mfc_hid)
                )

        if self.mfc_comport is None:
            self.print_available_comports()
            raise ValueError("No comport specified")
        
    def init_tmp_comport(self):
        """Initialize the comport of the tmp device
        It will print the available comports if no comport is specified
        """

        if self.tmp_hid:
            tmp_port = list_ports.grep(self.tmp_hid)
            tmp_port = list(tmp_port)

            if (len(tmp_port) == 0) and (self.tmp_comport is None):
                self.print_available_comports()

                raise ValueError(
                    "No comport found for tmp_hid: {}".format(self.tmp_hid)
                )
            elif len(tmp_port) == 1:
                self.tmp_comport = tmp_port[0].device
            else:
                self.print_available_comports()
                raise ValueError(
                    "Multiple comports found for tmp_hid: {}".format(self.tmp_hid)
                )

        if self.tmp_comport is None:
            self.print_available_comports()
            raise ValueError("No comport specified")

    def print_available_comports(self):
        """Prints the available comports along with their description and hardware id"""
        comports_available = list_ports.comports()
        print("Available comports:")
        for comport in comports_available:
            print(
                "{}: {} [{}]".format(comport.device, comport.description, comport.hwid)
            )

    def serial_connection_valves(self):
        """Function that establishes the serial connection with the valve controller
        It will connect to the comport specified in self.control_comport
        """
        self.ser = serial.Serial()
        self.ser.baudrate = 9600
        self.ser.port = self.valves_comport
        parity = serial.PARITY_NONE
        stopbits = serial.STOPBITS_ONE
        bytesize = serial.EIGHTBITS

        if self.ser.isOpen() == False:
            self.ser.timeout = 0.1
            self.ser.open()
        else:
            print("The Port is closed: " + self.ser.portstr)

    def get_valve_position(self, valve):
        self.ser.write('/{}CP\r'.format(valve).encode())
        current_position = self.ser.readline().decode('utf-8').strip()
        valve_no = current_position[1]
        position = current_position[-2]
        if position == 'A':
            return valve_no, 'OFF'
        elif position == 'B':
            return valve_no, 'ON'
        else:
            return valve_no, 'Unknown'
        
    def display_valve_positions(self, valve=None):
        if valve is not None:
            valve_no, position = self.get_valve_position(valve)
            print('Valve "{}" position is {}'.format(valve_no, position))
        else:
            valves = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I']
            positions = [self.get_valve_position(valve) for valve in valves]
            for valve_no, position in positions:
                print('Valve "{}" position is {}'.format(valve_no, position))

    def move_valve_to_position(self, valve, position):
        if position == 'ON':
            position_real = 'B'
            command = 'CC'
        elif position == 'OFF':
            position_real = 'A'
            command = 'CW'
        else:
            print('Invalid position specified.')
            return        
        self.ser.write('/{}{}\r'.format(valve, command).encode())
        time.sleep(0.3)
        self.ser.write('/{}CP\r'.format(valve).encode())
        new_position = self.ser.readline().decode('utf-8').strip()[-2]
        if new_position != position_real:
            self.ser.write('/{}{}\r'.format(valve, command).encode())
        else:
            # print('Valve "{}" successfully moved to position {}'.format(valve, position))
            pass
    
    def carrier_He_A(self):
        """Fuction that selects He as carrier gas for the Gas Line A"""
        self.move_valve_to_position('G', 'OFF')
        # self.ser.write(b'/GCW\r')
        print("Feeding He to Gas Line A")

    def carrier_Ar_A(self):
        """Fuction that selects Ar as carrier gas for Gas Line A"""
        self.move_valve_to_position('G', 'ON')
        # self.ser.write(b"/GCC\r")
        print("Feeding Ar to Gas Line A")

    def carrier_He_B(self):
        """Fuction that selects He as carrier gas for Gas Line B"""
        self.move_valve_to_position('F', 'ON')
        # self.ser.write(b"/FCC\r")
        print("Feeding He to Gas Line B")

    def carrier_Ar_B(self):
        """Function that selects Ar as carrier gas for Gas Line B"""
        self.move_valve_to_position('F', 'OFF')
        # self.ser.write(b"/FCW\r")
        print("Feeding Ar to Gas Line B")

    def feed_CO2_AB(self):
        """Fuction that selects carbon monoxide as gas source for Gas Line A and B"""
        self.move_valve_to_position('D', 'ON')
        # self.ser.write(b"/DCC\r")
        print("Feeding CO2 to Gas Line A and B")

    def feed_CO_AB(self):
        """Fuction that selects carbon monoxide as gas source for Gas Line A and B"""
        self.move_valve_to_position('D', 'OFF')
        # self.ser.write(b"/DCW\r")
        print("Feeding CO to Gas Line A and B")

    def feed_H2_A(self):
        """Function that selects hydrogen as gas source for Gas Line A"""
        self.move_valve_to_position('I', 'OFF')
        # self.ser.write(b"/ICW\r")
        print("Feeding H2 to Gas Line A")

    def feed_D2_A(self):
        """Function that selects deuterium as gas source for Gas Line A"""
        self.move_valve_to_position('I', 'ON')
        # self.ser.write(b"/ICC\r")
        print("Feeding D2 to Gas Line A")

    def feed_H2_B(self):
        """Function that selects hydrogen as gas source for Gas Line B"""
        self.move_valve_to_position('H', 'ON')
        # self.ser.write(b"/HCC\r")
        print("Feeding H2 to Gas Line B")

    def feed_D2_B(self):
        """Function that selects deuterium as gas source for Gas Line B"""
        self.move_valve_to_position('H', 'OFF')
        # self.ser.write(b"/HCW\r")
        print("Feeding D2 to Gas Line B")

    def feed_CH4_AB(self):
        """Fuction that selects methane as gas source for Gas Line A and B"""
        self.move_valve_to_position('E', 'ON')
        # self.ser.write(b"/ECC\r")
        print("Feeding CH4 to Gas Line A and B")

    def feed_C2H6_AB(self):
        """Function that selects ethane as gas source for Gas Line A and B"""
        self.move_valve_to_position('E', 'OFF')
        # self.ser.write(b"/ECW\r")
        print("Feeding C2H6 to Gas Line A and B")

    def feed_O2_AB(self):
        """Fuction that selects CO as carbon monoxide gas source for the mixing line
        This function is not implemented in the valve control module"""
        pass

    def valve_C(self, position: str):
        """Function that selects the position of Valve C (Reaction mode selection module)

        Args:
            position (str): Position of the valve, can be "off" or "on"
                            "off" means that the valve is in the position Gas Line A/B -> reactor
                            "on" means that the valve is in the position Gas Line A/B -> gas loop
        """
        if position == "OFF":
            self.move_valve_to_position('C', position)
            # self.ser.write(b"/CCW\r")
            print("Gas Line A/B valve position: off (Gas Line A/B -> reactor)")
        elif position == "ON":
            self.move_valve_to_position('C', position)
            # self.ser.write(b"/CCC\r")
            print("Gas Line A/B valve position: on (Gas Line A/B -> loop)")

    def valve_B(self, position: str):
        """Function that selects the position of Valve B (Reaction mode selection module)

        Args:
            position (str): Position of the valve, can be "off" or "on"
                            "off" means that the valve is in the position Gas Line A -> reactor
                            "on" means that the valve is in the position Gas Line B -> reactor
        """

        if position == "OFF":
            self.move_valve_to_position('B', position)
            # self.ser.write(b"/BCW\r")
            print("Valve B position: off \n(Gas Line A -> reactor)\n(Gas Line B -> pulses)")
        elif position == "ON":
            self.move_valve_to_position('B', position)
            # self.ser.write(b"/BCC\r")
            print("Valve B position: off \n(Gas Line B -> reactor)\n(Gas Line A -> pulses)")

    def valve_A(self, position: str):
        """Function that selects the position of Valve A (Reaction mode selection module)

        Args:
            position (str): Position of the valve, can be "off" or "on"
                            "off" means that the valve is in the loop 1 -> reactor, loop 2 -> vent
                            "on" means that the valve is in the loop 2 -> reactor, loop 1 -> vent
        """
        if position == "OFF":
            self.move_valve_to_position('A', position)
            # self.ser.write(b"/ACW\r")
            print(
                "Pulses line valve position: off (Gas Line A -> loop 1 -> vent / Gas Line B -> loop 2 -> reactor)"
            )
        elif position == "ON":
            self.move_valve_to_position('A', position)
            # self.ser.write(b"/ACC\r")
            print(
                "Pulses line valve position: on (Gas Line B -> loop 2 -> vent / Gas Line A -> loop 1 -> reactor)"
            )

    def cont_mode_A(self, verbose: bool = True):
        """Function that selects the position of the valves in the reaction mode selection
        module to the continuous mode gas line A mode

        Gas Line A -> reactor ... Gas Line B -> loops -> vent

        Args:
            verbose (bool): If True, prints the valve status [default: True]
        """
        self.move_valve_to_position('A', 'OFF')
        self.move_valve_to_position('B', 'OFF')
        self.move_valve_to_position('C', 'OFF')
        # self.ser.write(b"/ACW\r")
        # self.ser.write(b"/BCW\r")
        # self.ser.write(b"/CCW\r")
        if verbose:
            print("Valves operation mode: continuous mode Gas Line A")
            print("Gas Line A -> reactor ... Gas Line B -> loops -> vent")

    def cont_mode_B(self, verbose: bool = True):
        """Function that selects the position of the valves in the reaction mode selection
        module to the continuous mode gas line B mode

        Gas Line B -> reactor ... Gas Line A -> loops -> waste

        Args:
            verbose (bool): If True, prints the valve status [default: True]
        """
        self.move_valve_to_position('A', 'OFF')
        self.move_valve_to_position('B', 'ON')
        self.move_valve_to_position('C', 'OFF')
        # self.ser.write(b"/ACW\r")
        # self.ser.write(b"/BCC\r")
        # self.ser.write(b"/CCW\r")
        if verbose:
            print("Valves operation mode: continuous mode Gas Line B")
            print("Gas Line B -> reactor ... Gas Line A -> loops -> waste")

    # def modulation(
    #     self,
    #     pulses=10,
    #     time1=10,
    #     time2=10,
    #     start_gas="pulse",
    #     end_gas="pulse",
    #     monitoring_interval=0.01,
    #     save_log="./log.txt",
    # ):
    #     """Function that modulates the valves in the reaction mode selection module
    #     between the cont_mode_A and cont_mode_B

    #     Args:
    #         pulses (int): Number of pulses to be performed [default: 10]
    #         time1 (int): Time in seconds for the valve to be in Gas Line B mode [default: 10]
    #         time2 (int): Time in seconds for the valve to be in Gas Line A mode [default: 10]
    #         start_gas (str): Gas to be used as carrier gas in the pulses line at the beginning of the modulation [default: "pulse"]
    #         end_gas (str): Gas to be used as carrier gas in the pulses line at the end of the modulation [default: "pulse"]
    #         monitoring_interval (float): Time in seconds between each valve status check [default: 0.01]
    #         save_log (str): Path to the file where the valve status will be saved [default: "log.txt"]
    #     """
    #     if save_log is not None:
    #         os.makedirs(os.path.dirname(save_log), exist_ok=True)

    #         if not os.path.isfile(save_log):
    #             with open(save_log, "w") as f:
    #                 f.write("Time, Valve1\n")

    #     start_time = time.time()
    #     end_time = start_time + pulses * (time1 + time2)

    #     if start_gas == "pulse":
    #         valve_fun1 = self.pulses_mode
    #         valve_fun2 = self.cont_mode_dry
    #     else:
    #         valve_fun1 = self.cont_mode_dry
    #         valve_fun2 = self.pulses_mode

    #     self.get_status()
    #     if start_gas in VALVE_POSITION.keys():
    #         start_gas_id = VALVE_POSITION[start_gas]
    #     else:
    #         raise ValueError(f"start_gas must be in {VALVE_POSITION.keys()}")

    #     if end_gas in VALVE_POSITION.keys():
    #         end_gas_id = VALVE_POSITION[end_gas]
    #     else:
    #         raise ValueError(f"end_gas must be in {VALVE_POSITION.keys()}")

    #     if VALVE_POSITION[start_gas] == 1:
    #         valve_fun1 = self.pulses_mode
    #         valve_fun2 = self.cont_mode_dry
    #     else:
    #         valve_fun1 = self.cont_mode_dry
    #         valve_fun2 = self.pulses_mode

    #     if VALVE_POSITION[end_gas] == 0:
    #         valve_end_fun = self.pulses_mode
    #     else:
    #         valve_end_fun = self.cont_mode_dry

    #     while True:
    #         current_time = time.time()
    #         accumulated_time = current_time - start_time

    #         current_pulse = int(accumulated_time / (time1 + time2))
    #         current_time_in_pulse = accumulated_time - current_pulse * (time1 + time2)

    #         if current_time_in_pulse < time1:
    #             if VALVE_POSITION[self.status[0]] == start_gas_id:
    #                 time.sleep(monitoring_interval)
    #                 continue
    #             else:
    #                 self.get_status()
    #                 if VALVE_POSITION[self.status[0]] == start_gas_id:
    #                     time.sleep(monitoring_interval)
    #                     continue
    #                 else:
    #                     valve_fun1(verbose=False)
    #                     if save_log is not None:
    #                         self.get_status()
    #                         with open(save_log, "a") as f:
    #                             f.write(
    #                                 f"{current_time}, {VALVE_POSITION[self.status[0]]}\n"
    #                             )
    #         else:
    #             if VALVE_POSITION[self.status[0]] != start_gas_id:
    #                 time.sleep(monitoring_interval)
    #                 continue
    #             else:
    #                 self.get_status()
    #                 if VALVE_POSITION[self.status[0]] != start_gas_id:
    #                     time.sleep(monitoring_interval)
    #                     continue
    #                 else:
    #                     valve_fun2(verbose=False)
    #                     if save_log is not None:
    #                         self.get_status()
    #                         with open(save_log, "a") as f:
    #                             f.write(
    #                                 f"{current_time}, {VALVE_POSITION[self.status[0]]}\n"
    #                             )

    #         time.sleep(monitoring_interval)

    #         if current_time > end_time:
    #             break

    #     valve_end_fun()

    def pulses_loop_mode_A(self, verbose=True):
        """Function that selects the position of the valves in the reaction mode selection
        module to the pulses loop mode

        Gas Line B -> loop 2 -> reactor ... Gas Line A -> loop 1 -> vent

        Args:
            verbose (bool): If True, prints the valve status [default: True]
        """
        self.move_valve_to_position('A', 'ON')
        self.move_valve_to_position('B', 'OFF')
        self.move_valve_to_position('C', 'ON')        
        # self.ser.write(b"/ACC\r")
        # self.ser.write(b"/BCW\r")
        # self.ser.write(b"/CCC\r")
        if verbose:
            print("Valves operation mode: pulses with gas loops")
            print("Gas Line B -> loop 2 -> reactor ... Gas Line A -> loop 1 -> vent")

    def pulses_loop_mode_B(self, verbose=True):
        """Function that selects the position of the valves in the reaction mode selection
        module to the pulses loop mode

        Gas Line B -> loop 2 -> reactor ... Gas Line A -> loop 1 -> vent

        Args:
            verbose (bool): If True, prints the valve status [default: True]
        """
        self.move_valve_to_position('A', 'ON')
        self.move_valve_to_position('B', 'ON')
        self.move_valve_to_position('C', 'ON')        
        # self.ser.write(b"/ACC\r")
        # self.ser.write(b"/BCW\r")
        # self.ser.write(b"/CCC\r")
        if verbose:
            print("Valves operation mode: pulses with gas loops")
            print("Gas Line B -> loop 2 -> reactor ... Gas Line A -> loop 1 -> vent")

    def send_pulses_loop_A(self,pulses,time_bp):
        #total_time_loop = float(pulses) * float(time_bp)
        #total_time.append(total_time_loop)
        # tmp.pulse_ON()
        self.pulses_loop_mode_A()
        int_pulses = int(pulses)
        float_time = float(time_bp)
        print('Valves operation mode: pulses (dual loop alternation)')
        print('Number of pulses (loop): {}\nTime in between pulses (s): {}'.format(pulses,time_bp))
        print('Valve Position Off: Gas Line B -> loop 2 -> reactor /// Gas Line A -> loop 1 -> vent')
        print('Valve Position On: Gas line B -> loop 1 -> reactor /// Gas Line A -> loop 2 -> vent')
        for pulse in range(0, int_pulses):
            # tmp.pulse_ON()
            self.ser.write(b'/ATO\r') # Comand that executes the pulses valve actuation
            print('Sending pulse number {} of {}'.format(pulse+1,int_pulses), end = "\r") # Pulse status message for terminal window
            time.sleep(float_time) # Conversion of seconds to miliseconds
            # tmp.pulse_OFF()
        print('Pulses have finished') # End of the pulses message

    def send_pulses_loop_B(self,pulses,time_bp):
        #total_time_loop = float(pulses) * float(time_bp)
        #total_time.append(total_time_loop)
        # tmp.pulse_ON()
        self.pulses_loop_mode_B()
        int_pulses = int(pulses)
        float_time = float(time_bp)
        print('Valves operation mode: pulses (dual loop alternation)')
        print('Number of pulses (loop): {}\nTime in between pulses (s): {}'.format(pulses,time_bp))
        print('Valve Position Off: Gas Line A -> loop 2 -> reactor /// Gas Line B -> loop 1 -> vent')
        print('Valve Position On: Gas Line A -> loop 1 -> reactor /// Gas Line B -> loop 2 -> vent')
        for pulse in range(0, int_pulses):
            # tmp.pulse_ON()
            self.ser.write(b'/ATO\r') # Comand that executes the pulses valve actuation
            print('Sending pulse number {} of {}'.format(pulse+1,int_pulses), end = "\r") # Pulse status message for terminal window
            time.sleep(float_time) # Conversion of seconds to miliseconds
            # tmp.pulse_OFF()
        print('Pulses have finished') # End of the pulses message

    def send_pulses_valve_A(self,pulses,time_vo,time_bp):
        #total_time_loop = (float(pulses) * float(time_bp)) + (float(pulses) * float(time_vo))
        #total_time.append(total_time_loop)
        valve_actuation_time = 0.145
        self.cont_mode_A()
        int_pulses = int(pulses) # Preparing the integer input for the loop range
        float_time_vo = float(time_vo) # Preparing the float input for the sleep function vo
        float_time_bp = float(time_bp) # Preparing the float input for the sleep function bp
        print('Valves operation mode: pulses (valve)')
        print('Number of pulses (valve): {}\nTime valve open (s): {}\nTime in between pulses (s): {}'.format(pulses,time_vo,time_bp))
        print('Valve Position Off: mixing line -> reactor /// pulses line carrier -> loop 2 -> loop 1 -> waste')
        print('Valve Position On: pulses line carrier -> reactor /// mixing line -> loop 2 -> loop 1 -> waste')
        for pulse in range(0, int_pulses):
            self.cont_mode_B() # Comand that executes the pulses valve actuation
            time.sleep(float_time_vo + valve_actuation_time) # Conversion of seconds to miliseconds
            self.cont_mode_A() # Comand that executes the pulses valve actuation
            print('Sending pulse number {} of {}'.format(pulse+1,int_pulses), end = "\r") # Pulse status message for terminal window
            time.sleep(float_time_bp) # Conversion of seconds to miliseconds
        print('Pulses have finished') # End of the pulses message

    def define_flowsms(self):
        """Function to define the parameters of the Flow-SMS mass flow controllers
        The parameters are defined in the following dictionaries:

        gas_list: List of the available gases
        gas_dict: Dictionary that assigns a number to each gas
        gas_ID: Dictionary that assigns a node ID to each gas
        gas_cal: Dictionary that assigns a calibration ID to each gas. If there it is applicable to one gas, the value is None.
        gas_flow_range: Dictionary that assigns a flow range to each gas
        calibration_factor: Dictionary that assigns a calibration factor to each gas
        feed_gas_functions: Dictionary that assigns a function to each gas
        gas_float_to_int_factor: Dictionary that assigns a conversion factor from float to int to each gas
        """
        self.gas_list = [
            "H2_A",
            "H2_B",
            "D2_A",
            "D2_B",
            "O2_A",
            "O2_B",
            "CO_AH",
            "CO_AL",
            "CO_BH",
            "CO_BL",
            "CO2_AH",
            "CO2_AL",
            "CO2_BH",
            "CO2_BL",
            "CH4_A",
            "CH4_B",
            "C2H6_A",
            "C2H6_B",
            "C3H8_A",
            "C3H8_B",
            "He_A",
            "He_B",
            "Ar_A",
            "Ar_B",
            "N2_A",
            "N2_B",
        ]

        self.gas_dict = {self.gas_list[i]: i for i in range(len(self.gas_list))}

        self.gas_ID = {
            "H2_A": 4,
            "D2_A":4,
            "O2_A": 5,
            "CO_AH": 6,
            "CO_AL": 6,
            "CO2_AH": 6,
            "CO2_AL": 6,
            "CH4_A": 7,
            "C2H6_A": 7,
            "C3H8_A": 7,
            "He_A": 8,
            "Ar_A": 8,
            "N2_A": 8,
            "He_B": 9,
            "Ar_B": 9,
            "N2_B": 9,
            "CH4_B": 10,
            "C2H6_B": 10,
            "C3H8_B": 10,
            "CO_BH": 11,
            "CO_BL": 11,
            "CO2_BH": 11,
            "CO2_BL": 11,
            "O2_B": 12,
            "H2_B": 13,
            "D2_B":13,
        }

        self.gas_cal = {
            "H2_A": 0,
            "H2_B": 0,
            "D2_A": 1,
            "D2_B": 1,
            "O2_A": 0,
            "O2_B": 0,
            "CO_AH": 0,
            "CO_BH": 0,
            "CO2_AH": 1,
            "CO2_BH": 1,
            "CO2_AL": 2,
            "CO2_BL": 2,
            "CO_AL": 3,
            "CO_BL": 3,
            "CH4_A": 0,
            "CH4_B": 0,
            "C2H6_A": 1,
            "C2H6_B": 1,
            "C3H8_A": 2,
            "C3H8_B": 2,
            "He_A": 0,
            "He_B": 0,
            "Ar_A": 1,
            "Ar_B": 1,
            "N2_A": 2,
            "N2_B": 2,
        }

        self.gas_flow_range = {
            "CO2_AH": [0.6, 30.0],
            "CO2_AL": [0.26, 13.0],
            "CO2_BH": [0.6, 30.0],
            "CO2_BL": [0.26, 13.0],
            "CO_AH": [0.6, 30.0],
            "CO_AL": [0.36, 18.0],
            "CO_BH": [0.6, 30.0],
            "CO_BL": [0.36, 18.0],
            "CH4_A": [0.6, 30.0],
            "CH4_B": [0.6, 30.0],
            "C2H6_A": [0.6, 30.0],
            "C2H6_B": [0.6, 30.0],
            "C3H8_A": [0.6, 30.0],
            "C3H8_B": [0.6, 30.0],
            "H2_A": [0.6, 30.0],
            "H2_B": [0.6, 30.0],
            "D2_A": [0.6, 30.0],
            "D2_B": [0.6, 30.0],
            "O2_A": [0.6, 30.0],
            "O2_B": [0.6, 30.0],
            "He_A": [1.2, 60.0],
            "He_B": [1.2, 60.0],
            "Ar_A": [1.2, 60.0],
            "Ar_B": [1.2, 60.0],
            "N2_A": [1.2, 60.0],
            "N2_B": [1.2, 60.0],
        }

        self.calibration_factor = {
            "H2_A": 1.0,
            "H2_B": 1.0,
            "D2_A": 1.0,
            "D2_B": 1.0,
            "O2_A": 1.0,
            "O2_B": 1.0,
            "CO_AH": 1.0,
            "CO_AL": 1.0,
            "CO_BH": 1.0,
            "CO_BL": 1.0,
            "CH4_A": 1.0,
            "CH4_B": 1.0,
            "C2H6_A": 1.0,
            "C2H6_B": 1.0,
            "C3H8_A": 1.0,
            "C3H8_B": 1.0,
            "CO2_AH": 1.0,
            "CO2_AL": 1.0,
            "CO2_BH": 1.0,
            "CO2_AL": 1.0,
            "He_A": 1.0,
            "He_B": 1.0,
            "Ar_A": 1.0,
            "Ar_B": 1.0,
            "N2_A": 1.0,
            "N2_B": 1.0,
        }

        self.feed_gas_functions = {
            "H2_A": self.feed_H2_A,
            "H2_B": self.feed_H2_B,
            "D2_A": self.feed_D2_A,
            "D2_B": self.feed_D2_B,
            "O2_A": self.feed_O2_AB,
            "O2_B": self.feed_O2_AB,
            "CO_AH": self.feed_CO_AB,
            "CO_AL": self.feed_CO_AB,
            "CO_BH": self.feed_CO_AB,
            "CO_BL": self.feed_CO_AB,
            "CH4_A": self.feed_CH4_AB,
            "CH4_B": self.feed_CH4_AB,
            "C2H6_A": self.feed_C2H6_AB,
            "C2H6_B": self.feed_C2H6_AB,
            "C3H8_A": self.feed_C2H6_AB,
            "C3H8_B": self.feed_C2H6_AB,
            "CO2_AH": self.feed_CO2_AB,
            "CO2_AL": self.feed_CO2_AB,
            "CO2_BH": self.feed_CO2_AB,
            "CO2_AL": self.feed_CO2_AB,
            "He_A": self.carrier_He_A,
            "He_B": self.carrier_He_B,
            "Ar_A": self.carrier_Ar_A,
            "Ar_B": self.carrier_Ar_B,
            "N2_A": self.carrier_Ar_A,
            "N2_B": self.carrier_Ar_B,
        }

        self.gas_float_to_int_factor = {
            "H2_A": 30,
            "H2_B": 30,
            "D2_A": 30,
            "D2_B": 30,
            "O2_A": 30,
            "O2_B": 30,
            "CO_AH": 30,
            "CO_AL": 18,
            "CO_BH": 30,
            "CO_BL": 18,
            "CH4_A": 30,
            "CH4_B": 30,
            "C2H6_A": 30,
            "C2H6_B": 30,
            "C3H8_A": 30,
            "C3H8_B": 30,
            "CO2_AH": 30,
            "CO2_AL": 13,
            "CO2_BH": 30,
            "CO2_BL": 13,
            "He_A": 60,
            "He_B": 60,
            "Ar_A": 60,
            "Ar_B": 60,
            "N2_A": 60,
            "N2_B": 60,
        }

    def set_flowrate(
        self,
        gas: str,
        flow: float,
    ):
        """Function that sets the flow rate of a gas in the Flow-SMS mass flow controllers

        Args:
            gas (str): Gas for which the flow rate will be set
            flow (float): Flow rate in sccm
        """
        if gas not in self.gas_list:
            raise ValueError("Gas not in list of available gases")

        while True:
            if (flow is None) or (flow == 0.0):
                flow_conv = 0.0
                break

            flow_conv = flow / self.calibration_factor[gas]

            if flow_conv < self.gas_flow_range[gas][0]:
                print(
                    f"{gas} flow lower than minimum {self.gas_flow_range[gas][0]} sccm"
                )
                interval = input(
                    'Write "Yes" for setting a new flow or "No" for quiting the program: '
                )
                if interval == "Yes":
                    flow = float(input("Enter new flow: "))
                elif interval == "No":
                    raise SystemExit
                else:
                    break

            elif flow_conv > self.gas_flow_range[gas][1]:
                print(
                    f"{gas} flow higher than maximum {self.gas_flow_range[gas][1]} sccm"
                )
                interval = input(
                    'Write "Yes" for setting a new flow or "No" for quiting the program: '
                )
                if interval == "Yes":
                    flow = float(input("Enter new flow: "))
                elif interval == "No":
                    raise SystemExit
                else:
                    break
            else:
                break

        if flow_conv > 0.0:
            self.feed_gas_functions[gas]()

        flow_data = int(flow_conv * 32000 / self.gas_float_to_int_factor[gas])

        param = []

        if self.gas_cal[gas] is not None:
            param.append(
                {
                    "node": self.gas_ID[gas],
                    "proc_nr": 1,
                    "parm_nr": 16,
                    "parm_type": propar.PP_TYPE_INT8,
                    "data": self.gas_cal[gas],
                }
            )

        param.append(
            {
                "node": self.gas_ID[gas],
                "proc_nr": 1,
                "parm_nr": 1,
                "parm_type": propar.PP_TYPE_INT16,
                "data": flow_data,
            }
        )

        status = self.mfc_master.write_parameters(param)

    def flowsms_setpoints(
        self,
        H2_A: float = None,
        D2_A: float = None,
        O2_A: float = None,
        CO_AH: float = None,
        CO2_AH: float = None,
        CO_AL: float = None,
        CO2_AL: float = None,
        CH4_A: float = None,
        C2H6_A: float = None,
        C3H8_A: float = None,
        He_A: float = None,
        Ar_A: float = None,
        N2_A: float = None,
        He_B: float = None,
        Ar_B: float = None,
        N2_B: float = None,
        CH4_B: float = None,
        C2H6_B: float = None,
        C3H8_B: float = None,
        CO_BH: float = None,
        CO2_BH: float = None,
        CO_BL: float = None,
        CO2_BL: float = None,
        O2_B: float = None,
        H2_B: float = None,
        D2_B: float = None,
    ):
        """Function that sets the flow rates of the gases in the Flow-SMS mass flow controllers

        Args:
            H2_A (float): Flow rate of H2 in sccm for gas line A [default: None]
            H2_B (float): Flow rate of H2 in sccm for gas line B [default: None]
            D2_A (float): Flow rate of D2 in sccm for gas line A [default: None]
            D2_B (float): Flow rate of D2 in sccm for gas line B [default: None]
            O2_A (float): Flow rate of O2 in sccm for gas line A [default: None]
            O2_B (float): Flow rate of O2 in sccm for gas line B [default: None]
            CO_AH (float): Flow rate of CO in sccm for gas line A with high flow calibration curve [default: None]
            CO_AL (float): Flow rate of CO in sccm for gas line A with low flow calibration curve [default: None]
            CO_BH (float): Flow rate of CO in sccm for gas line B with high flow calibration curve [default: None]
            CO_BL (float): Flow rate of CO in sccm for gas line B with low flow calibration curve [default: None]
            CO2_AH (float): Flow rate of CO2 in sccm for gas line A with high flow calibration curve [default: None]
            CO2_AL (float): Flow rate of CO2 in sccm for gas line A with low flow calibration curve [default: None]
            CO2_BH (float): Flow rate of CO2 in sccm for gas line B with high flow calibration curve [default: None]
            CO2_BL (float): Flow rate of CO2 in sccm for gas line B with low flow calibration curve [default: None]
            CH4_A (float): Flow rate of CH4 in sccm for gas line A [default: None]
            CH4_B (float): Flow rate of CH4 in sccm for gas line B [default: None]
            C2H6_A (float): Flow rate of C2H6 in sccm for gas line A [default: None]
            C2H46_B (float): Flow rate of C2H6 in sccm for gas line B [default: None]            
            He_A (float): Flow rate of He in sccm for gas line A [default: None]
            He_B (float): Flow rate of He in sccm for gas line B [default: None]
            Ar_A (float): Flow rate of Ar in sccm for gas line A [default: None]
            Ar_B (float): Flow rate of Ar in sccm for gas line B [default: None]
            N2_A (float): Flow rate of N2 in sccm for gas line A [default: None]
            N2_B (float): Flow rate of N2 in sccm for gas line B [default: None]
        """
        if CO_AH is not None and CO_AH > 0.0:
            self.set_flowrate("CO_AH", CO_AH)
        elif CO_AL is not None and CO_AL > 0.0:
            self.set_flowrate("CO_AL", CO_AL)
        elif CO2_AH is not None and CO2_AH > 0.0:
            self.set_flowrate("CO2_AH", CO2_AH)
        else:
            self.set_flowrate("CO2_AL", CO2_AL)

        if CO_BH is not None and CO_BH > 0.0:
            self.set_flowrate("CO_BH", CO_BH)
        elif CO_BL is not None and CO_BL > 0.0:
            self.set_flowrate("CO_BL", CO_BL)
        elif CO2_BH is not None and CO2_BH > 0.0:
            self.set_flowrate("CO2_BH", CO2_BH)
        else:
            self.set_flowrate("CO2_BL", CO2_BL)
        
        if CH4_A is not None and CH4_A > 0.0:
            self.set_flowrate("CH4_A", CH4_A)
        elif C2H6_A is not None and C2H6_A > 0.0:
            self.set_flowrate("C2H6_A", C2H6_A)
        else:
            self.set_flowrate("C3H8_A", C3H8_A)

        if CH4_B is not None and CH4_B > 0.0:
            self.set_flowrate("CH4_B", CH4_B)
        elif C2H6_B is not None and C2H6_B > 0.0:
            self.set_flowrate("C2H6_B", C2H6_B)
        else:
            self.set_flowrate("C3H8_B", C3H8_B)

        if H2_A is not None and H2_A > 0.0:
            self.set_flowrate("H2_A", H2_A)
        else:
            self.set_flowrate("D2_A", D2_A)
        
        if H2_B is not None and H2_B > 0.0:
            self.set_flowrate("H2_B", H2_B)
        else:
            self.set_flowrate("D2_B", D2_B)
        
        if He_A is not None and He_A > 0.0:
            self.set_flowrate("He_A", He_A)
        elif Ar_A is not None and Ar_A > 0.0:
            self.set_flowrate("Ar_A", Ar_A)
        else:
            self.set_flowrate("N2_A", N2_A)

        if He_B is not None and He_B > 0.0:
            self.set_flowrate("He_B", He_B)
        elif Ar_B is not None and Ar_B > 0.0:
            self.set_flowrate("Ar_B", Ar_B)
        else:
            self.set_flowrate("N2_B", N2_B)

        self.set_flowrate("O2_A", O2_A)

        self.set_flowrate("O2_B", O2_B)

    def flowsms_status(self, delay=0.0):
        """Function that reads the flow rates of the gases in the Flow-SMS mass flow controllers

        Args:
            delay (float): Delay time in seconds before reading the flow rates [default: 0.0]
        """

        # Node ID values assigned in the MFCs configuration

        ID_P_A = 3
        ID_H2_D2_A = 4
        ID_O2_A = 5
        ID_CO_CO2_A = 6
        ID_HC_A = 7
        ID_CARRIER_A = 8
        ID_CARRIER_B = 9
        ID_HC_B = 10
        ID_CO_CO2_B = 11
        ID_O2_B = 12
        ID_H2_D2_B = 13
        ID_P_B = 14

        # ID assigned in the MFCs configuration for calibration curve allocation
        H2_A = 0
        D2_A = 1
        H2_B = 0
        D2_B = 1
        O2_A = 0
        O2_B = 0
        CO_AH = 0
        CO2_AH = 1
        CO2_AL = 2
        CO_AL = 3
        CO_BH = 0
        CO2_BH = 1
        CO2_BL = 2
        CO_BL = 3
        CH4_A = 0
        C2H6_A = 1
        C3H8_A = 2
        CH4_B = 0
        C2H6_B = 1
        C3H8_B = 2
        CARRIER_He_A = 0
        CARRIER_Ar_A = 1
        CARRIER_N2_A = 2
        CARRIER_He_B = 0
        CARRIER_Ar_B = 1
        CARRIER_N2_B = 2

        # Setting a delay time before reading the actual flows
        # If non present zero delay will be applied before reading

        time.sleep(delay)

        # Parameters to be read from the Flow-SMS mass flow controllers
        
        params_h2_d2_a = [
            {
                "node": ID_H2_D2_A,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_H2_D2_A,
                "proc_nr": 33,
                "parm_nr": 3,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_H2_D2_A,
                "proc_nr": 1,
                "parm_nr": 16,
                "parm_type": propar.PP_TYPE_INT8,
            }
        ]
        params_h2_d2_b = [
            {
                "node": ID_H2_D2_B,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_H2_D2_B,
                "proc_nr": 33,
                "parm_nr": 3,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_H2_D2_B,
                "proc_nr": 1,
                "parm_nr": 16,
                "parm_type": propar.PP_TYPE_INT8,
            }
        ]
        params_o2_a = [
            {
                "node": ID_O2_A,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_O2_A,
                "proc_nr": 33,
                "parm_nr": 3,
                "parm_type": propar.PP_TYPE_FLOAT,
            }
        ]
        params_o2_b = [
            {
                "node": ID_O2_B,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_O2_B,
                "proc_nr": 33,
                "parm_nr": 3,
                "parm_type": propar.PP_TYPE_FLOAT,
            }
        ]
        params_hc_a = [
            {
                "node": ID_HC_A,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_HC_A,
                "proc_nr": 33,
                "parm_nr": 3,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_HC_A,
                "proc_nr": 1,
                "parm_nr": 16,
                "parm_type": propar.PP_TYPE_INT8,
            }
        ]
        params_hc_b = [
            {
                "node": ID_HC_B,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_HC_B,
                "proc_nr": 33,
                "parm_nr": 3,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_HC_B,
                "proc_nr": 1,
                "parm_nr": 16,
                "parm_type": propar.PP_TYPE_INT8,
            }
        ]
        params_co_co2_a = [
            {
                "node": ID_CO_CO2_A,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_CO_CO2_A,
                "proc_nr": 33,
                "parm_nr": 3,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_CO_CO2_A,
                "proc_nr": 1,
                "parm_nr": 16,
                "parm_type": propar.PP_TYPE_INT8,
            }
        ]
        params_co_co2_b = [
            {
                "node": ID_CO_CO2_B,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_CO_CO2_B,
                "proc_nr": 33,
                "parm_nr": 3,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_CO_CO2_B,
                "proc_nr": 1,
                "parm_nr": 16,
                "parm_type": propar.PP_TYPE_INT8,
            }
        ]
        params_carrier_a = [
            {
                "node": ID_CARRIER_A,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_CARRIER_A,
                "proc_nr": 33,
                "parm_nr": 3,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_CARRIER_A,
                "proc_nr": 1,
                "parm_nr": 16,
                "parm_type": propar.PP_TYPE_INT8,
            }
        ]
        params_carrier_b = [
            {
                "node": ID_CARRIER_B,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_CARRIER_A,
                "proc_nr": 33,
                "parm_nr": 3,
                "parm_type": propar.PP_TYPE_FLOAT,
            },
            {
                "node": ID_CARRIER_B,
                "proc_nr": 1,
                "parm_nr": 16,
                "parm_type": propar.PP_TYPE_INT8,
            }
        ]
        params_p_a = [
            {
                "node": ID_P_A,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            }
        ]
        params_p_b = [
            {
                "node": ID_P_B,
                "proc_nr": 33,
                "parm_nr": 0,
                "parm_type": propar.PP_TYPE_FLOAT,
            }
        ]

        # Sending the specified parameters to the Flow-SMS
        values_h2_d2_a = self.mfc_master.read_parameters(params_h2_d2_a)
        values_h2_d2_b = self.mfc_master.read_parameters(params_h2_d2_b)
        values_o2_a = self.mfc_master.read_parameters(params_o2_a)
        values_o2_b = self.mfc_master.read_parameters(params_o2_b)
        values_co_co2_a = self.mfc_master.read_parameters(params_co_co2_a)
        values_co_co2_b = self.mfc_master.read_parameters(params_co_co2_b)
        values_hc_a = self.mfc_master.read_parameters(params_hc_a)
        values_hc_b = self.mfc_master.read_parameters(params_hc_b)
        values_carrier_a = self.mfc_master.read_parameters(params_carrier_a)
        values_carrier_b = self.mfc_master.read_parameters(params_carrier_b)
        values_p_a = self.mfc_master.read_parameters(params_p_a)
        values_p_b = self.mfc_master.read_parameters(params_p_b)

        # Creating induviduals lists for the read values from each MFC
        lst_h2_d2_a = []
        for value in values_h2_d2_a:
            if "data" in value:
                flow = value.get("data")
            lst_h2_d2_a.append(format(flow, ".2f"))
        fluid_h2_d2_a = float(lst_h2_d2_a[2])
        if fluid_h2_d2_a == 0:
            fluid_h2_d2_a = "H2_A"
        elif fluid_h2_d2_a == 1:
            fluid_h2_d2_a = "D2_A"

        lst_h2_d2_b = []
        for value in values_h2_d2_b:
            if "data" in value:
                flow = value.get("data")
            lst_h2_d2_b.append(format(flow, ".2f"))
        fluid_h2_d2_b = float(lst_h2_d2_b[2])
        if fluid_h2_d2_b == 0:
            fluid_h2_d2_b = "H2_B"
        elif fluid_h2_d2_b == 1:
            fluid_h2_d2_b = "D2_B"

        lst_o2_a = []
        for value in values_o2_a:
            if "data" in value:
                flow = value.get("data")
            lst_o2_a.append(format(flow, ".2f"))

        lst_o2_b = []
        for value in values_o2_b:
            if "data" in value:
                flow = value.get("data")
            lst_o2_b.append(format(flow, ".2f"))   
        
        lst_co_co2_a = []
        for value in values_co_co2_a:
            if "data" in value:
                flow = value.get("data")
            lst_co_co2_a.append(format(flow, ".2f"))
        fluid_co_co2_a = float(lst_co_co2_a[2])
        if fluid_co_co2_a == 0:
            fluid_co_co2_a = "CO_AH"
        elif fluid_co_co2_a == 1:
            fluid_co_co2_a = "CO2_AH"
        elif fluid_co_co2_a == 2:
            fluid_co_co2_a = "CO2_AL"
        elif fluid_co_co2_a == 3:
            fluid_co_co2_a = "CO_AL"

        lst_co_co2_b = []
        for value in values_co_co2_b:
            if "data" in value:
                flow = value.get("data")
            lst_co_co2_b.append(format(flow, ".2f"))
        fluid_co_co2_b = float(lst_co_co2_b[2])
        if fluid_co_co2_b == 0:
            fluid_co_co2_b = "CO_BH"
        elif fluid_co_co2_b == 1:
            fluid_co_co2_b = "CO2_BH"
        elif fluid_co_co2_b == 2:
            fluid_co_co2_b = "CO2_BL"
        elif fluid_co_co2_b == 3:
            fluid_co_co2_b = "CO_BL"

        lst_hc_a = []
        for value in values_hc_a:
            if "data" in value:
                flow = value.get("data")
            lst_hc_a.append(format(flow, ".2f"))
        fluid_hc_a = float(lst_hc_a[2])
        if fluid_hc_a == 0:
            fluid_hc_a = "CH4_A"
        elif fluid_hc_a == 1:
            fluid_hc_a = "C2H6_A"
        elif fluid_hc_a == 2:
            fluid_hc_a = "C3H8_A"

        lst_hc_b = []
        for value in values_hc_b:
            if "data" in value:
                flow = value.get("data")
            lst_hc_b.append(format(flow, ".2f"))
        fluid_hc_b = float(lst_hc_b[2])
        if fluid_hc_b == 0:
            fluid_hc_b = "CH4_B"
        elif fluid_hc_b == 1:
            fluid_hc_b = "C2H6_B"
        elif fluid_hc_b == 2:
            fluid_hc_b = "C3H8_B"

        lst_carrier_a = []
        for value in values_carrier_a:
            if "data" in value:
                flow = value.get("data")
            lst_carrier_a.append(format(flow, ".2f"))
        fluid_carrier_a = float(lst_carrier_a[2])
        if fluid_carrier_a == 0:
            fluid_carrier_a = "He"
        elif fluid_carrier_a == 1:
            fluid_carrier_a = "Ar"
        elif fluid_carrier_a == 2:
            fluid_carrier_a = "N2"

        lst_carrier_b = []
        for value in values_carrier_b:
            if "data" in value:
                flow = value.get("data")
            lst_carrier_b.append(format(flow, ".2f"))
        fluid_carrier_b = float(lst_carrier_b[2])
        if fluid_carrier_b == 0:
            fluid_carrier_b = "He"
        elif fluid_carrier_b == 1:
            fluid_carrier_b = "Ar"
        elif fluid_carrier_b == 2:
            fluid_carrier_b = "N2"

        lst_p_a = []
        for value in values_p_a:
            if "data" in value:
                pressure = value.get("data")
            lst_p_a.append(format(pressure, ".2f"))
            return lst_p_a[0]

        lst_p_b = []
        for value in values_p_b:
            if "data" in value:
                pressure = value.get("data")
            lst_p_b.append(format(pressure, ".2f"))
            return lst_p_b[0]

        # Calculating percentage values for the actual flows

        total_flow_a = float(
            format(
                float(lst_h2_d2_a[0])
                + float(lst_o2_a[0])
                + float(lst_co_co2_a[0])
                + float(lst_hc_a[0])
                + float(lst_carrier_a[0]),
                ".2f",
            )
        )
        if total_flow_a != 0:
            H2_D2_percent_a = format((float(lst_h2_d2_a[0]) / total_flow_a) * 100, ".1f")
            O2_percent_a = format((float(lst_o2_a[0]) / total_flow_a) * 100, ".1f")
            CO_CO2_percent_a = format((float(lst_co_co2_a[0]) / total_flow_a) * 100, ".1f")
            HC_percent_a = format((float(lst_hc_a[0]) / total_flow_a) * 100, ".1f")
            # carrier_a_percent = format((float(lst_carrier_a[0])/total_flow_a)*100, '.1f')

        total_flow_b = float(
            format(
                float(lst_h2_d2_b[0])
                + float(lst_o2_b[0])
                + float(lst_co_co2_b[0])
                + float(lst_hc_b[0])
                + float(lst_carrier_b[0]),
                ".2f",
            )
        )
        if total_flow_b != 0:
            H2_D2_percent_b = format((float(lst_h2_d2_b[0]) / total_flow_b) * 100, ".1f")
            O2_percent_b = format((float(lst_o2_b[0]) / total_flow_b) * 100, ".1f")
            CO_CO2_percent_b = format((float(lst_co_co2_b[0]) / total_flow_b) * 100, ".1f")
            HC_percent_b = format((float(lst_hc_b[0]) / total_flow_b) * 100, ".1f")
            # carrier_b_percent = format((float(lst_carrier_b[0])/total_flow_b)*100, '.1f')

        # Creating and printing table with the actual and set flows, and line pressures
        print(" ")
        print("------------------------------------------------------------")
        print("-------------------")
        print("--- Flow Report ---")
        print("-------------------")

        if float(lst_h2_d2_a[1]) == 0:
            pass
        else:
            print(
                "{}_A: measured flow is {} sccm. Flow setpoint is {} sccm. Concentration is {}%".format(
                    fluid_h2_d2_a, lst_h2_d2_a[0], lst_h2_d2_a[1], H2_D2_percent_a
                )
            )

        if float(lst_h2_d2_b[1]) == 0:
            pass
        else:
            print(
                "{}_B: measured flow is {} sccm. Flow setpoint is {} sccm. Concentration is {}%".format(
                    fluid_h2_d2_b, lst_h2_d2_b[0], lst_h2_d2_b[1],H2_D2_percent_b
                )
            )

        if float(lst_o2_a[1]) == 0:
            pass
        else:
            print(
                "O2_A: measured flow is {} sccm. Flow setpoint is {} sccm. Concentration is {}%".format(
                    lst_o2_a[0], lst_o2_a[1], O2_percent_a
                )
            )

        if float(lst_o2_b[1]) == 0:
            pass
        else:
            print(
                "O2_B: measured flow is {} sccm. Flow setpoint is {} sccm. Concentration is {}%".format(
                    lst_o2_b[0], lst_o2_b[1], O2_percent_b
                )
            )

        if float(lst_co_co2_a[1]) == 0:
            pass
        else:
            print(
                "{}_A: measured flow is {} sccm. Flow setpoint is {} sccm. Concentration is {}%".format(
                    fluid_co_co2_a, lst_co_co2_a[0], lst_co_co2_a[1], CO_CO2_percent_a
                )
            )

        if float(lst_co_co2_b[1]) == 0:
            pass
        else:
            print(
                "{}_B: measured flow is {} sccm. Flow setpoint is {} sccm. Concentration is {}%".format(
                    fluid_co_co2_b, lst_co_co2_b[0], lst_co_co2_b[1], CO_CO2_percent_b
                )
            )

        if float(lst_hc_a[1]) == 0:
            pass
        else:
            print(
                "{}_A: measured flow is {} sccm. Flow setpoint is {} sccm. Concentration is {}%".format(
                    fluid_hc_a, lst_hc_a[0], lst_hc_a[1], HC_percent_a
                )
            )

        if float(lst_hc_b[1]) == 0:
            pass
        else:
            print(
                "{}_B: measured flow is {} sccm. Flow setpoint is {} sccm. Concentration is {}%".format(
                    fluid_hc_b, lst_hc_b[0], lst_hc_b[1], HC_percent_b
                )
            )

        if float(lst_carrier_a[1]) == 0:
            pass
        else:
            print(
                "{}_A: measured flow is {} sccm. Flow setpoint is {} sccm".format(
                    fluid_carrier_a, lst_carrier_a[0], lst_carrier_a[1]
                )
            )

        if float(lst_carrier_b[1]) == 0:
            pass
        else:
            print(
                "{}_B: measured flow is {} sccm. Flow setpoint is {} sccm".format(
                    fluid_carrier_b, lst_carrier_b[0], lst_carrier_b[1]
                )
            )

        print("Total flow line A: {} sccm".format(total_flow_a))

        print("Total flow line B: {} sccm".format(total_flow_b))

        print("-----------------------")
        print("--- Pressure Report ---")
        print("-----------------------")

        print("Pressure in line A: {} psia".format(lst_p_a[0]))

        print("Pressure in line B: {} psia".format(lst_p_b[0]))

        print("------------------------------------------------------------")
        # return lst_p_a[0], lst_p_b[0]

    def get_pv_loop1(self):
        """Return the process value (PV) for loop1."""
        pv = self.tmp_master.read_register(2, 1)
        print("PV = {} degC".format(pv))
    
    def heating_event(self, rate_sp = None, sp = None):
        """Loops over actual temperature in an heating event until setpoint is reached"""
        print('Starting heating event:')
        try:
            print('Heating rate: {} C/min'.format(rate_sp))
            rate_sp = float(rate_sp)
            self.tmp_master.write_register(35, rate_sp, 1)
        except:
            rate_sp = None      
        try:
            print('Setpoint: {} C'.format(sp))
            sp = float(sp)
            self.tmp_master.write_register(24, sp, 1)
        except:
            sp = None      
        while True:
            try:
                temp_tc = self.tmp_master.read_register(1, 1)
                temp_programmer = self.tmp_master.read_register(5, 1)
                power_out = self.tmp_master.read_register(85, 1)
            except IOError:
                continue
                # print("Failed to read from instrument")
            except ValueError:
                continue
                # print("Instrument response is invalid")
            try:
                result = float(temp_tc) < float(sp)
                if result == True:
                    temp_tc = float(temp_tc)
                    pressure_a = self.flowsms_status()
                    pressure_b = self.flowsms_status()
                    print("-----------------------------------------------------------------------------------------------------")
                    print(f"Pressure in line A: {pressure_a} psia")
                    print(f"Pressure in line B: {pressure_b} psia")
                    print(f"Setpoint Temp: {sp} C | Programmer Temp: {temp_programmer} C | Reactor Temp: {temp_tc} C | Power out: {power_out}%")
                    print("-----------------------------------------------------------------------------------------------------")
                    print("\033[F\033[F\033[F\033[F\033[F", end="")
                    time.sleep(1)
                else:
                    print('{} C setpoint reached!'.format(sp))
                    break
            except TypeError:
                continue
      
        
    def cooling_event(self, rate_sp = None, sp = None):
        """Loops over actual temperature in an cooling event until setpoint is reached"""
    
        print('Starting cooling event:')
        try:
            print('Heating rate: {} C/min'.format(rate_sp))
            rate_sp = float(rate_sp)
            self.tmp_master.write_register(35, rate_sp, 1)
        except:
            rate_sp = None      
        try:
            print('Setpoint: {} C'.format(sp))
            sp = float(sp)
            self.tmp_master.write_register(2, sp, 1)
        except:
            sp = None      
        while True:
            try:
                temp_tc = self.tmp_master.read_register(1, 1)
                temp_programmer = self.tmp_master.read_register(5, 1)
                power_out = self.tmp_master.read_register(85, 1)
            except IOError:
                continue
                # print("Failed to read from instrument")
            except ValueError:
                continue
                # print("Instrument response is invalid")
            try:
                result = float(temp_tc) > float(sp)
                if result == True:
                    temp_tc = float(temp_tc)
                    sp = float(sp)
                    pressure_a = self.flowsms_status()
                    pressure_b = self.flowsms_status()
                    print("-----------------------------------------------------------------------------------------------------")
                    print(f"Pressure in line A: {pressure_a} psia")
                    print(f"Pressure in line B: {pressure_b} psia")
                    print(f"Setpoint Temp: {sp} C | Programmer Temp: {temp_programmer} C | Reactor Temp: {temp_tc} C | Power out: {power_out}% ---")
                    print("-----------------------------------------------------------------------------------------------------")
                    print("\033[F\033[F\033[F\033[F\033[F", end="")                    
                    time.sleep(1)
                else:
                    print('{} C setpoint reached!'.format(sp))
                    break
            except TypeError:
                continue
        
    def temperature_ramping_event(self,rate_sp = None, sp = None ):
        while True:
            try:
                temp_pv = self.tmp_master.read_register(289, 1) 
            except IOError:
                continue
                # print("Failed to read from instrument")
            except ValueError:
                continue
                # print("Instrument response is invalid")
            try:
                result = float(temp_pv) > float(sp)
                if result == True:
                    self.cooling_event(rate_sp, sp)
                    print('start cooling event')
                    break
                else:
                    self.heating_event(rate_sp, sp)
                    print('start heating event')
                    break
            except TypeError:
                continue

    def setpoint_finish_experiment(self):
        """Loops over actual temperature in an cooling event until setpoint is reached"""
        rate_sp=10
        sp=18
    
        print('adjust temperature set point to 18C:')
        try:
            print('cooling rate: {} C/min'.format(rate_sp))
            rate_sp = float(rate_sp)
            self.tmp_master.write_register(35, rate_sp, 1)
        except:
            rate_sp = None
      
        try:
            print('Setpoint: {} C'.format(sp))
            sp = float(sp)
            self.tmp_master.write_register(24, sp, 1)
        except:
            sp = None
      
    def time_event(self, time_in_seconds: int, argument: str):
        """Waits for a specified time while printing the elapsed time on the terminal.

        Args:
            time_in_seconds (int): The time to wait in seconds.
        """
        start_time = time.time()
        while True:
            elapsed_time = time.time() - start_time
            if elapsed_time < time_in_seconds:                
                pressure_a = self.flowsms_status()
                pressure_b = self.flowsms_status()
                print("-----------------------------------------------------------------------------------------------------")
                print(f"Pressure in line A: {pressure_a} psia")
                print(f"Pressure in line B: {pressure_b} psia")
                print(f"Elapsed time for {str(argument)}: {int(elapsed_time)} seconds")
                print("-----------------------------------------------------------------------------------------------------")
                print("\033[F\033[F\033[F\033[F\033[F", end="")
                time.sleep(1)
            else:
                print("-----------------------------------------------------------------------------------------------------\n",
                f"Wait time of {time_in_seconds} seconds completed.",
                "-------------------------------------------------------------------\n",
                "-----------------------------------------------------------------------------------------------------", end="\r")
                break
            

    
    ## Remote Triggering
    def DRIFTS_PID(self):    
        self.tmp_master.write_register(6, 86.92)
        self.tmp_master.write_register(8, 95.52)
        self.tmp_master.write_register(9, 15.92)
        p=self.tmp_master.read_register(6, 2)
        i=self.tmp_master.read_register(8, 2)
        d=self.tmp_master.read_register(9, 2)    
        print("PID for DRIFTS cell is imported" + " ,proportional band={}".format(p) + " , integral time={}".format(i) + ", derivative time={}".format(d) + ", please switch output to LOCAL")
   
    def Clausen_Cell_PID(self):    
        self.tmp_master.write_register(6, 600)
        self.tmp_master.write_register(8, 20)
        self.tmp_master.write_register(9, 4)
        p=self.tmp_master.read_register(6)
        i=self.tmp_master.read_register(8)
        d=self.tmp_master.read_register(9)    
        print("PID for Clausen cell is imported" + " ,proportional band={}".format(p) + ", integral time={}".format(i) + ", derivative time={}".format(d) + ", please switch output to AUX")
  
    def MS_ON(self):
        """Sends a logic value (0 or 1) to perform remote digital triggering to RlyAA"""
        self.tmp_master.write_register(363, 0)
        sleep(10)
        print('MS sequence started')
    
    def MS_OFF(self):
        """Sends a logic value (0 or 1) to perform remote digital triggering to RlyAA"""
        self.tmp_master.write_register(363, 1)
        sleep(10)
        print('MS sequence stopped')
    
    def IR_ON(self):
        """Sends 5V to perform remote triggering to logic A"""    
        self.tmp_master.write_register(376, 5)
        sleep(1)
        self.tmp_master.write_register(376, 0)
        print('IR data acquisition started')
        now = datetime.now()
        dt_start = now.strftime("%m/%d/%Y %H:%M:%S")
        print('\ndate and time =', dt_start)
    
    def pulse_ON(self):
        """Sends 5V to perform remote triggering to logic A"""    
        self.tmp_master.write_register(376, 3)
        #sleep(1)
        #self.write_register(376, 0)
        print('Pulse ON')
        now = datetime.now()
        dt_start = now.strftime("%m/%d/%Y %H:%M:%S")
        print('\ndate and time =', dt_start)
    
    def pulse_OFF(self):
        """Sends 5V to perform remote triggering to logic A"""    
        self.tmp_master.write_register(376, 0)
        #sleep(1)
        #self.write_register(376, 0)
        print('Pulse OFF')
        now = datetime.now()
        dt_start = now.strftime("%m/%d/%Y %H:%M:%S")
        print('\ndate and time =', dt_start)
    
    def IR_STATUS(self):
        """Sends 5V to perform remote triggering to logic A"""    
        while True:
            try:
                result = self.tmp_master.read_register(361)
            except IOError:
                continue
                # print("Failed to read from instrument")
            except ValueError:
                continue
                # print("Instrument response is invalid")
            if result == 1:
                break
            elif result == 0:
                sleep(0.1)
                continue


if __name__ == "__main__":
    gc = GasControl()
    gc.cont_mode_A()
    gc.display_valve_positions()

    gc.flowsms_setpoints(
        Ar_A=15,
        Ar_B=15,
    )

    gc.carrier_Ar_B()

    gc.flowsms_status()
    gc.flowsms_setpoints()
    gc.flowsms_status()

    gc.get_pv_loop1()


