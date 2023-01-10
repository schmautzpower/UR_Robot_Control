#  Copyright (c) 2023. By Benjamin Byrdeck
"""
This Software communicate with the UR Robot and Read the data.
"""
from cmath import pi
import socket
import time
import re
from rtde import rtde
from rtde import rtde_config
import logging

logging.basicConfig(level=logging.INFO)

home = [0.09891, -0.6914, 0.60185, 0.0, 3.14, 0.0]
home_joints = [-4.712388980384691, -1.5707963267948966, 1.570796326794897,
               -1.5707963267948966, -1.5707963267948966, 8.881784197001252e-16]


class URControl:
    STATE_REPLY_PATTERN = re.compile(r"(\w+):\W+(\d+)?")

    def __init__(self, robot_ip, instand_mode=False):
        # instance for Connection
        self.robot = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)  # Network type
        self.robot.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.host = robot_ip  # Robot IP
        if instand_mode:
            self.port = 30001  # Robot TCP Port 30003 = Remote Port / 30020 Interpreter Port
        else:
            self.port = 30020
        self.robot.settimeout(5)  # 5 seconds

        # Set home position
        self.home = [0.09891, -0.6914, 0.60185, 0.0, 3.14159, 0.0]

        self.connect_socket()

    def connect_socket(self):
        """
        Opens a socket connection with the robot for communication.
        :return: None
        """
        try:
            if self.port == 30001:
                self.set_output("Connecting instand_mode")
            else:
                self.set_output('Connecting interpreter_mode')
            self.robot.connect((self.host, self.port))
        except OSError as error:
            self.set_output("OS error: {0}".format(error))
            return

    def reconnect_socket(self):
        """
        Reconnect if connection is lost or disturbed.
        :return:
        """
        self.robot.close()
        self.robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_socket()

    def client_exit(self):
        """
        Exits the program and closes the socket connection.
        :return:
        """
        self.robot.close()
        exit()

    def disconnect_robot(self):
        self.robot.close()

    def create_movej(self, x, y, z, rx, ry, rz, a=1.4, v=1.05, t=0, r=0):
        """
        Create a binary command for UR Robots
        • a = 1.4 → acceleration is 1.4 rad/s/s
        • v = 1.05 → velocity is 1.05 rad/s
        • t = 0 the time (seconds) to make move is not specified. If it were specified the command would ignore the a and v values.
        • r = 0 → the blend radius is zero meters.
        :return: Binary Command for UR Robots
        """
        command = f'movej(p[{x}, {y}, {z}, {rx}, {ry}, {rz}], {a}, {v}, {t}, {r})\n'
        command = command.encode()
        return command

    def create_movel(self, x, y, z, rx, ry, rz, a=0.1, v=0.1, t=0, r=0):
        """
        Create a binary command for UR Robots
        a: tool acceleration [m/s^2]
        v: tool speed [m/s]
        t:time[S]
        r: blend radius [m]
        :return: Binary Command for UR Robots
        """
        command = f'movel(p[{x}, {y}, {z}, {rx}, {ry}, {rz}], {a}, {v}, {t}, {r})\n'
        command = command.encode()
        return command

    def create_targetj(self, posix, a=1.4, v=1.05, t=0, r=0):
        """
        Create Target where robot should move
        • a = 1.4 → acceleration is 1.4 rad/s/s
        • v = 1.05 → velocity is 1.05 rad/s
        • t = 0 the time (seconds) to make move is not specified. If it were specified the command would ignore the a and v values.
        • r = 0 → the blend radius is zero meters.
        :return: Binary Command
        """
        command = f'movej(p{posix}, {a}, {v}, {t}, {r})\n'
        command = command.encode()
        return command

    def create_targetl(self, posix, a=1.2, v=0.25, t=0, r=0):
        """
        Create Target where robot should move
        a: tool acceleration [m/s^2]
        v: tool speed [m/s]
        t: time[S]
        r: blend radius [m]
        :return:
        """
        command = f'movel(p{posix}, {a}, {v}, {t}, {r})\n'
        command = command.encode()
        return command

    def create_joints(self, joints, a=1.2, v=0.25, t=0, r=0):
        """
        • a = 1.4 → acceleration is 1.4 rad/s/s
        • v = 1.05 → velocity is 1.05 rad/s
        • t = 0 the time (seconds) to make move is not specified. If it were specified the command would ignore the a and v values.
        • r = 0 → the blend radius is zero meters.
        :return:
        """
        command = f'movej({joints}, {a}, {v}, {t}, {r})\n'
        command = command.encode()
        return command

    def send_home(self):
        """
        Must be used first. Sets the robot in a hardcoded base position.
        :return:
        """
        self.send_joints(home_joints, a=1, v=1)

    def send_command(self, command):
        try:
            self.robot.send(command)
        except OSError as error:
            self.set_output("OS error: {0}".format(error))
            return False
        return True

    def send_movel(self, posix, a=1.2, v=0.25, t=0, r=0):
        """
        Create move command where robot should move
        a: tool acceleration [m/s^2]
        v: tool speed [m/s]
        t: time[S]
        r: blend radius [m]
        :return:
        """
        self.send_command(self.create_targetl(posix, a, v, t, r))

    def send_movej(self, posix, a=1.4, v=1.05, t=0, r=0):
        """
        Create move command where robot should move
        • a = 1.4 → acceleration is 1.4 rad/s/s
        • v = 1.05 → velocity is 1.05 rad/s
        • t = 0 the time (seconds) to make move is not specified. If it were specified the command would ignore the a and v values.
        • r = 0 → the blend radius is zero meters.
        :return: Binary Command
        """
        self.send_command(self.create_targetj(posix, a, v, t, r))

    def send_joints(self, joints, a=1.2, v=0.25, t=0, r=0):
        """
        • a = 1.4 → acceleration is 1.4 rad/s/s
        • v = 1.05 → velocity is 1.05 rad/s
        • t = 0 the time (seconds) to make move is not specified. If it were specified the command would ignore the a and v values.
        • r = 0 → the blend radius is zero meters.
        :return:
        """
        command = f'movej({joints}, {a}, {v}, {t}, {r})\n'
        command = command.encode()
        self.send_command(command)

    def set_digital_out(self, out: int, on: bool = True):
        """
        Send a digital out command.
        DO0 = 0
        DO1 = 1
        DO2 = 2
        DO3 = 3
        DO4 = 4
        DO5 = 5
        DO6 = 6
        DO7 = 7
        TDO0 = 8
        TDO1 = 9
        """
        command = f'set_digital_out({str(out)}, {str(on)})\n'
        self.send_command(command.encode())

    def set_analog_out(self, out: int, level: float):
        self.send_command(
            f'set_standard_analog_outputdomain({out},1)\n'.encode())
        command = f'set_standard_analog_out({out},{level})\n'
        self.send_command(command.encode())

    def set_output(self, output_string):
        """
        Outputs the Errors or Succeed Messages.
        :return:
        """
        print(output_string)

    def get_reply(self):
        """
        read one line from the socket
        :return: text until new line
        """
        collected = b''
        while True:
            try:
                part = self.robot.recv(1)
            except:
                part = b"\n"
            if part != b"\n":
                collected += part
            elif part == b"\n":
                break
        return collected.decode("utf-8")

    def execute_command(self, command):
        """
        Send single line command to interpreter mode, and wait for reply
        :param command:
        :return: ack, or status id
        """
        if not command.endswith("\n"):
            command += "\n"

        self.robot.send(command.encode("utf-8"))
        raw_reply = self.get_reply()
        # parse reply, raise exception if command is discarded
        reply = self.STATE_REPLY_PATTERN.match(raw_reply)
        try:
            if reply.group(1) == "discard":
                return 0
                # raise Exception("Interpreter discarded message", raw_reply)
            return int(reply.group(2))
        except:
            return 0

    def clear(self):
        return self.execute_command("clear_interpreter()")

    def skip(self):
        return self.execute_command("skipbuffer")

    def abort_move(self):
        return self.execute_command("abort")

    def get_last_interpreted_id(self):
        return self.execute_command("statelastinterpreted")

    def get_last_executed_id(self):
        return self.execute_command("statelastexecuted")

    def get_last_cleared_id(self):
        return self.execute_command("statelastcleared")

    def get_unexecuted_count(self):
        return self.execute_command("stateunexecuted")

    def end_interpreter(self):
        return self.execute_command("end_interpreter()")


class RTDE:
    def __init__(self, robot_ip):
        # set Robot host and port for RTDE connection
        self.robot_host = robot_ip
        self.robot_port = 30004

        # set home position
        self.home = [0.09891, -0.6914, 0.60185, 0.0, 3.141, 0.0]

        # Set logger
        logging.getLogger().setLevel(logging.INFO)

        # Construct configuration
        self.configfile = "control_loop_configuration.xml"  # Config File Path
        self.config = rtde_config.ConfigFile(self.configfile)
        self.state_names, self.state_types = self.config.get_recipe('state')

        # Construct Robot
        self.rob = rtde.RTDE(self.robot_host, self.robot_port)

        # Connect to Robot
        self.rob.connect()

        # submit the configuration
        self.rob.send_output_setup(
            self.state_names, self.state_types, frequency=125)

        self.rob.send_start()

    def disconnect_robot(self):
        """
        disconnect the Robot.
        """
        self.rob.disconnect()

    def is_connected(self):
        return self.rob.is_connected()

    def controller_version(self):
        return self.rob.get_controller_version()

    def reconnect_robot(self):
        """
        Reconnect Robot. Return
        :return:
        """
        self.rob.send_pause()
        self.rob.disconnect()
        self.rob = rtde.RTDE(self.robot_host, self.robot_port)
        self.rob.send_output_setup(self.state_names, self.state_types)
        self.rob.send_start()

    def get_TCP_force(self):
        state = self.rob.receive()
        return state.actual_TCP_force

    def get_TCP_pose(self):
        state = self.rob.receive()
        return state.actual_TCP_pose

    def get_joints(self):
        state = self.rob.receive()
        return state.actual_q

    def get_TCP_speed(self):
        state = self.rob.receive()
        return state.actual_TCP_speed

    def get_runtime_state(self):
        state = self.rob.receive()
        return state.runtime_state

    def get_payload(self):
        state = self.rob.receive()
        return state.payload

    def get_payloag_cog(self):
        state = self.rob.receive()
        return state.payload_cog

    def get_tcp_force_scalar(self):
        state = self.rob.receive()
        return state.tcp_force_scalar

    def get_script_control_line(self):
        state = self.rob.receive()
        return state.script_control_line

    def get_target_TCP_pose(self):
        state = self.rob.receive()
        return state.target_TCP_pose

    def get_actual_digital_output_bits(self):
        state = self.rob.receive()
        return state.actual_digital_output_bits

    def get_digital_output_on(self, out_id: int):
        """
        DO0 = 0
        DO1 = 1
        DO2 = 2
        DO3 = 3
        DO4 = 4
        DO5 = 5
        DO6 = 6
        DO7 = 7
        CO0 = 8
        CO1 = 9
        CO2 = 10
        CO3 = 11
        CO4 = 12
        CO5 = 13
        CO6 = 14
        CO7 = 15
        TDO0 = 16
        TDO1 = 17
        """
        if (self.get_actual_digital_output_bits() & 1 << out_id):
            return True
        else:
            return False

    def get_analog_out(self, out_id: int):
        state = self.rob.receive()
        if out_id == 0:
            return state.standard_analog_output0
        elif out_id == 1:
            return state.standard_analog_output1
        else:
            return 'Outputs 0 and 1 only supported.'

    def target_reached(self, target, level=5, joints=False):
        if joints:
            actual_pose = self.get_joints()
        else:
            target = target[0:3]
            actual_pose = self.get_TCP_pose()[0:3]
        if normalize(actual_pose, level) == normalize(target, level):
            return True
        return False


class DashboardServer:
    def __init__(self, robot_ip: str):
        self.ds = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ds.settimeout(1)
        self.robot_ip = robot_ip
        self.port = 29999
        self.ds.connect((self.robot_ip, self.port))
        logging.info(self.ds.recv(1028).decode())

    def reconnect(self):
        try:
            self.ds.connect((self.robot_ip, self.port))
        except TimeoutError as e:
            logging.error(f'Connection failed {str(e)}')

    def disconnect(self):
        self.ds.close()

    def __send(self, command: str):
        command += '\n'
        self.ds.sendall(command.encode())
        try:
            re = self.ds.recv(4096).decode()
        except Exception as e:
            re = f'No Response {e}'
        logging.info(re)
        return re

    def load(self, program: str):
        """Returns when both program and associated installation has loaded (or failed). 
        The load command fails if the associated installation requires confirmation of safety. 
        The return value in this case will be 'Error while loading program'.

        Args:
            program (str): program name or path

        Returns:
            _type_: str
            On success:
            • "Loadingprogram: <program.urp>"
            On Failure:
            • "Filenotfound: <program.urp>"
            • "Errorwhileloading program:
            <program.urp>"
        """
        return self.__send(f'load {program}')

    def play(self):
        """Returns failure if the program fails to start. In previous versions this did not happen in all cases.

        Returns:
            _type_: str
            On success:
            • "Startingprogram"
            On failure:
            • "Failedtoexecute: play"
        """
        return self.__send('play')

    def stop(self):
        """Returns failure if the program fails to stop. In previous versions this did not happen in all cases..

        Returns:
            _type_: str
            On success:
            • "Stopped"
            On failure:
            • "Failedtoexecute: stop"
        """
        return self.__send('stop')

    def pause(self):
        """Returns failure if the program fails to pause. In previous versions this did not happen in all cases.

        Returns:
            _type_: str
            On success:
            • "Pausingprogram"
            On failure:
            • "Failedtoexecute: pause"
        """
        return self.__send('pause')

    def quit(self):
        """Closes connection

        Returns:
            _type_: str
            "Disconnected"
        """
        return self.__send('quit')

    def shutdown(self):
        """Shuts down and turns off robot and controller

        Returns:
            _type_: str
            "Shutting down"
        """
        return self.__send('shutdown')

    def running(self):
        """Execution state enquiry

        Returns:
            _type_: str
            "Program running: true" OR "Program running: false"
        """
        return self.__send('running')

    def robotmode(self):
        """Robot mode enquiry

        Returns:
            _type_: str
            text is returned "Robotmode: <mode>", where <mode> is
            • NO_CONTROLLER • DISCONNECTED
            • CONFIRM_SAFETY • BOOTING
            • POWER_OFF • POWER_ON • IDLE
            • BACKDRIVE • RUNNING
        """
        return self.__send('robotmode')

    def get_loaded_program(self):
        """Which program is loaded

        Returns:
            _type_: str 
            "Loaded program: <path to loaded program file>" OR "No program loaded"
        """
        return self.__send('get loaded program')

    def popup(self, msg: str):
        """The popup-text will be translated to the selected language, if the text exists in the language file

        Args:
            msg (str): popup text

        Returns:
            _type_: str
            "showing popup"
        """
        return self.__send(f'popup {msg}')

    def close_popup(self):
        """Closes the popup

        Returns:
            _type_: str
            "closing popup"
        """
        return self.__send('close popup')

    def add_to_log(self, msg: str):
        """Adds log-message to the Log history

        Args:
            msg (str): log-message

        Returns:
            _type_: str
            "Added log message" Or "No log message to add"
        """
        return self.__send(f'addToLog {msg}')

    def is_program_saved(self):
        """Returns the save state of the active program and path to loaded program file.

        Returns:
            _type_: str
            "true <program.name>" OR "false <program.name>"
        """
        return self.__send('isProgramSaved')

    def program_state(self):
        """Returns the state of the active program and path to loaded program file, or STOPPED if no program is loaded

        Returns:
            _type_: str
            STOPPED" if no program is running
            "PLAYING" if program is running
            "PAUSED" if program is paused
        """
        return self.__send('programState')

    def polyscope_version(self):
        """Returns version information for the UR software installed on the robot

        Returns:
            _type_: str
            version number, like "URSoftware 5.12.0.1101319 (Mar 22 2022)"
        """
        return self.__send('PolyscopeVersion')

    def version(self):
        """Returns the version number of the UR software installed on the robot

        Returns:
            _type_: str
            Software version number, e.g. "5.13.0.11013"
        """
        return self.__send('version')

    def set_operational_mode(self, mode: str):
        """Controls the operational mode. See User manual for details.
            Warning: This functionality is intended for using e.g. Ethernet based Key Card Readers 
            to switch operational modes. 
            The device for switching operational mode should be placed in vicinity to the robot.
            If this function is called the operational mode cannot be changed from PolyScope, 
            and the user password is disabled.

        Args:
            mode (str): manual, automatic

        Returns:
            _type_: str
            "Setting operational mode: <mode>" OR "Failed setting operational mode: <mode>"
            • manual=Loading and editing programs is allowed
            • automatic=Loading and editing programs and installations is not allowed, only playing programs
        """
        return self.__send(f'set operational mode {mode}')

    def get_operational_mode(self):
        """Returns the operational mode as MANUAL or AUTOMATIC if the password has been set for Mode in Settings.
            Returns NONE if the password has not been set.      

        Returns:
            _type_: str
            MANUAL, AUTOMATIC or NONE
        """
        return self.__send('get operational mode')

    def clear_operational_mode(self):
        """If this function is called the operational mode can again be changed from PolyScope, 
            and the user password is enabled. 

        Returns:
            _type_: str
            "operational mode is no longer controlled by Dashboard Server"
        """
        return self.__send('clear operational mode')

    def power_on(self):
        """Powers on the robot arm

        Returns:
            _type_: str
            "Powering on"
        """
        return self.__send('power on')

    def power_off(self):
        """Powers off the robot arm

        Returns:
            _type_: str
            "Powering off"
        """
        return self.__send('power off')

    def brake_release(self):
        """Releases the brakes

        Returns:
            _type_: str
            "Brake releasing"
        """
        return self.__send('brake release')

    def safety_status(self):
        """Safety status inquiry.
            This differs from 'safetymode' by specifying if a given Safeguard Stop was caused by the permanent 
            safeguard I/O stop, a configurable I/O automatic mode safeguard stop or a configurable I/O three 
            position enabling device stop. 

        Returns:
            _type_: str
            "Safetystatus: <status>", where <status> is
            • NORMAL
            • REDUCED
            • PROTECTIVE_STOP
            • RECOVERY
            • SAFEGUARD_STOP
            • SYSTEM_ EMERGENCY_STOP
            • ROBOT_ EMERGENCY_STOP
            • VIOLATION
            • FAULT
            • AUTOMATIC_MODE_ SAFEGUARD_STOP
            • SYSTEM_THREE_ POSITION_ ENABLING_STOP
        """
        return self.__send('safetystatus')

    def unlock_protective_stop(self):
        """Closes the current popup and unlocks protective stop. 
        The unlock protective stop command fails if less than 5 seconds has passed since the protective stop occurred.

        Returns:
            _type_: str
            On success:
            • "Protectivestop releasing"
            On failure:
            • "Cannotunlock protective stop until 5s after occurrence. Always inspect cause of protective stop before unlocking"
        """
        return self.__send('unlock protective stop')

    def close_safety_popup(self):
        """Closes a safety popup

        Returns:
            _type_: str
            "closing safety popup"
        """
        return self.__send('close safety popup')

    def load_installation(self, installation: str):
        """Loads the specified installation file but does not return until the load has completed (or failed).
            The load command fails if the associated installation requires confirmation of safety. 
            The return value will be 'Failed to load installation'.

        Args:
            installation (str): default.installation

        Returns:
            _type_: str
            On success:
            • "Loadinginstallation: <default.installatio n>"
            On failure:
            • "Filenotfound: <default.installatio n>"
            • "Failedtoload installation: <default.installatio n>"
        """
        return self.__send(f'load installation {installation}')

    def restart_safety(self):
        """Used when robot gets a safety fault or violation to restart the safety. 
        After safety has been rebooted the robot will be in Power Off. 
        IMPORTANT: You should always ensure it is okay to restart the system. 
        It is highly recommended to check the error log before using this command (either via PolyScope 
        or e.g. ssh connection).

        Returns:
            _type_: str
            Restarting safety
        """
        return self.__send('restart safety')

    def remote_control(self):
        """Returns the remote control status of the robot.
        If the robot is in remote control it returns true and if remote control is disabled or robot is 
        in local control it returns false.

        Returns:
            _type_: str
            "true" or "false"
        """
        return self.__send('is in remote control')

    def get_serial_number(self):
        """Returns serial number of the robot.

        Returns:
            _type_: str
            Serial number like "20175599999"
        """
        return self.__send('get serial number')

    def get_robot_model(self):
        """Returns the robot model

        Returns:
            _type_: str
            UR3, UR5, UR10, UR16
        """
        return self.__send('get robot model')

    def generate_flight_report(self, type: str):
        """Triggers a Flight Report of the following type:
            • Controller- report with information specific for diagnosing controller errors. 
            For example, in case of protective stops, faults or violations.
            • Software- report with information specific for polyscope software failures.
            • System- report with information about robot configuration, programs, installations etc.
            It is required to wait at least 30 seconds between triggering software or controller reports.

        Args:
            type (str): possible report types are:
            • controller 
            • software 
            • system

        Returns:
            _type_: str
            On success report id is printed.
            Error Message on a failure. Command can take few minutes to complete.
        """
        return self.__send(f'generate flight report {type}')

    def generate_support_file(self, path: str):
        """Generates a flight report of the type "System" and creates a compressed collection of 
        all the existing flight reports on the robot along with the generated flight report.
        Result file ur_[robot serial number]_ YYYY-MM-DD_HH- MM-SS.zip is saved inside <Directory path>

        Args:
            path (str): <Directory path> where <Directory path> represents path to an already existing 
            directory location inside the programs directory. 
            In particular path can point to special usbdisk subfolders inside programs folder.

        Returns:
            _type_: str
            On success "Completed successfully: <result file name>" is printed otherwise an error message with possible cause of the error is shown.
            Command can take up to 10 minutes to complete.
        """
        return self.__send(f'generate support file {path}')


def normalize(numbers, rounds=5):
    index = 0
    for i in numbers:
        if index <= 2:
            numbers[index] = round(i, rounds)
        else:
            numbers[index] = round(i, rounds - 1)
        index += 1
    return numbers


def degree_to_radians(degree: float):
    return round(degree * pi / 180, 5)


def radians_to_degree(radian: float):
    return round(radian * 180 / pi, 5)


if __name__ == '__main__':
    import time
    robot_ip = '192.168.8.229'
    info = RTDE(robot_ip)
    control = URControl(robot_ip, True)
    icontrol = URControl(robot_ip, False)

    while 1:
        ao = info.get_analog_out(0)
        print(ao)
        if ao <= 0.1:
            control.set_analog_out(0, 0.5)
        else:
            control.set_analog_out(0, 0)
        time.sleep(1)
