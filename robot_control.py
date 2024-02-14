#  Copyright (c) 2024. By Benjamin Schmautz
"""
This Software communicate with the UR Robot and Read the data.
"""
from cmath import pi
import socket
import re
import time
from rtde import rtde
from rtde import rtde_config
import logging

logging.basicConfig(level=logging.INFO)

_log = logging.getLogger("UR_Control")
interpreter_port: int = 30020
control_port: int = 30001  # alternative 30003
rtde_port: int = 30004
dashboard_port = 29999


class Robot:

    def __init__(self, robot_ip: str, refresh_rate: int = 125) -> None:
        self.robot_ip = robot_ip
        self.refresh_rate = refresh_rate

        logging.getLogger().setLevel(logging.INFO)

        self.configfile = "control_loop_configuration.xml"
        self.config = rtde_config.ConfigFile(self.configfile)
        self.state_names, self.state_types = self.config.get_recipe('state')

        self.control = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        self.control.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.control_con = False
        self.interpreter = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        self.interpreter.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.interpreter_con = False
        self.rtde = rtde.RTDE(self.robot_ip, rtde_port)
        self.rtde_con = False
        self.dashboard = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        self.dashboard.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.dashboard_con = False

    def connect(self):
        try:
            self.control.connect((self.robot_ip, control_port))
            self.control_con = True
            _log.info('Control port connected.')
        except Exception as e:
            _log.error("Failed to connect control port" + str(e))
            self.control_con = False
        try:
            self.interpreter.connect((self.robot_ip, interpreter_port))
            self.interpreter_con = True
            _log.info('Interpreter port connected.')
        except Exception as e:
            _log.error("Failed to connect interpreter port" + str(e))
            self.interpreter_con = False
        try:
            self.rtde.connect()
            self.rtde.send_output_setup(
                self.state_names, self.state_types, frequency=self.refresh_rate)
            self.rtde.send_start()
            self.rtde_con = True
            _log.info('RTDE port connected.')
        except Exception as e:
            _log.error("Failed to connect data exchange port" + str(e))
            self.rtde_con = False
        try:
            self.dashboard.connect((self.robot_ip, dashboard_port))
            self.dashboard_con = True
            _log.info('Control port connected.')
        except Exception as e:
            _log.error("Failed to connect dashboard port" + str(e))
            self.dashboard_con = False

    def disconnect(self):
        self.control.close()
        self.control_con = False
        self.interpreter.close()
        self.interpreter_con = False
        self.rtde.disconnect()
        self.rtde_con = False
        self.send_dashboard('quit')
        self.dashboard.close()
        self.dashboard_con = False

    def is_connected(self):
        if self.control_con and self.interpreter_con and self.rtde_con and self.dashboard_con:
            return True
        else:
            return False

    # region rtde

    def get_controller_version(self):
        return self.rtde.get_controller_version()

    def get_state(self):
        state = self.rtde.receive()
        d = {
            'tcp-force': state.actual_TCP_force,
            'tcp-pose': state.actual_TCP_pose,
            'joints': state.actual_q,
            'tcp-speed': state.actual_TCP_speed,
            'runtime-state': state.runtime_state,
            'payload': state.payload,
            'payload-cog': state.payload_cog,
            'tcp-force-scalar': state.tcp_force_scalar,
            'script-control-line': state.script_control_line,
            'tcp-target': state.target_TCP_pose,
            'digital-output-bits': state.actual_digital_output_bits,
            'standard-analog-output0': state.standard_analog_output0,
            'standard-analog-output1': state.standard_analog_output1
        }
        return d

    def get_TCP_force(self):
        state = self.rtde.receive()
        return state.actual_TCP_force

    def get_TCP_pose(self):
        state = self.rtde.receive()
        return state.actual_TCP_pose

    def get_joints(self):
        state = self.rtde.receive()
        return state.actual_q

    def get_TCP_speed(self):
        state = self.rtde.receive()
        return state.actual_TCP_speed

    def get_runtime_state(self):
        state = self.rtde.receive()
        return state.runtime_state

    def get_payload(self):
        state = self.rtde.receive()
        return state.payload

    def get_payloag_cog(self):
        state = self.rtde.receive()
        return state.payload_cog

    def get_tcp_force_scalar(self):
        state = self.rtde.receive()
        return state.tcp_force_scalar

    def get_script_control_line(self):
        state = self.rtde.receive()
        return state.script_control_line

    def get_target_TCP_pose(self):
        state = self.rtde.receive()
        return state.target_TCP_pose

    def get_actual_digital_output_bits(self):
        state = self.rtde.receive()
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
        state = self.rtde.receive()
        if out_id == 0:
            return state.standard_analog_output0
        elif out_id == 1:
            return state.standard_analog_output1
        else:
            return 'Outputs 0 and 1 only supported.'

    def target_reached(self, target, area: float = 0.001, joints: bool = False):
        """Check if the target is reached. 

        Args:
            target (list): list of xyz coordinates
            area (float, optional): Area around the targetb to trigger in Meter. Defaults to 0.001.
            joints (bool, optional): If joints calculation will be done in rads. Defaults to False.

        Returns:
            _type_: _description_
        """
        if joints:
            actual_pose = self.get_joints()
        else:
            target = target[0:3]
            actual_pose = self.get_TCP_pose()[0:3]
        check = []
        for i, x in zip(actual_pose, target):
            if x - area <= i and i <= x + area:
                check.append(True)
            else:
                check.append(False)
        if not check.__contains__(False):
            return True

    # endregion rtde

    # region dashboard

    def send_dashboard(self, command: str):
        command += '\n'
        self.dashboard.sendall(command.encode())
        try:
            re = self.dashboard.recv(4096).decode()
        except Exception as e:
            re = f'No Response {e}'
        logging.debug(re)
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
        return self.send_dashboard(f'load {program}')

    def play(self):
        """Returns failure if the program fails to start. In previous versions this did not happen in all cases.

        Returns:
            _type_: str
            On success:
            • "Startingprogram"
            On failure:
            • "Failedtoexecute: play"
        """
        return self.send_dashboard('play')

    def stop(self):
        """Returns failure if the program fails to stop. In previous versions this did not happen in all cases..

        Returns:
            _type_: str
            On success:
            • "Stopped"
            On failure:
            • "Failedtoexecute: stop"
        """
        return self.send_dashboard('stop')

    def pause(self):
        """Returns failure if the program fails to pause. In previous versions this did not happen in all cases.

        Returns:
            _type_: str
            On success:
            • "Pausingprogram"
            On failure:
            • "Failedtoexecute: pause"
        """
        return self.send_dashboard('pause')

    def shutdown(self):
        """Shuts down and turns off robot and controller

        Returns:
            _type_: str
            "Shutting down"
        """
        return self.send_dashboard('shutdown')

    def running(self):
        """Execution state enquiry

        Returns:
            _type_: str
            "Program running: true" OR "Program running: false"
        """
        return self.send_dashboard('running')

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
        return self.send_dashboard('robotmode')

    def get_loaded_program(self):
        """Which program is loaded

        Returns:
            _type_: str 
            "Loaded program: <path to loaded program file>" OR "No program loaded"
        """
        return self.send_dashboard('get loaded program')

    def popup(self, msg: str):
        """The popup-text will be translated to the selected language, if the text exists in the language file

        Args:
            msg (str): popup text

        Returns:
            _type_: str
            "showing popup"
        """
        return self.send_dashboard(f'popup {msg}')

    def close_popup(self):
        """Closes the popup

        Returns:
            _type_: str
            "closing popup"
        """
        return self.send_dashboard('close popup')

    def add_to_log(self, msg: str):
        """Adds log-message to the Log history

        Args:
            msg (str): log-message

        Returns:
            _type_: str
            "Added log message" Or "No log message to add"
        """
        return self.send_dashboard(f'addToLog {msg}')

    def is_program_saved(self):
        """Returns the save state of the active program and path to loaded program file.

        Returns:
            _type_: str
            "true <program.name>" OR "false <program.name>"
        """
        return self.send_dashboard('isProgramSaved')

    def program_state(self):
        """Returns the state of the active program and path to loaded program file, or STOPPED if no program is loaded

        Returns:
            _type_: str
            STOPPED" if no program is running
            "PLAYING" if program is running
            "PAUSED" if program is paused
        """
        return self.send_dashboard('programState')

    def polyscope_version(self):
        """Returns version information for the UR software installed on the robot

        Returns:
            _type_: str
            version number, like "URSoftware 5.12.0.1101319 (Mar 22 2022)"
        """
        return self.send_dashboard('PolyscopeVersion')

    def version(self):
        """Returns the version number of the UR software installed on the robot

        Returns:
            _type_: str
            Software version number, e.g. "5.13.0.11013"
        """
        return self.send_dashboard('version')

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
        return self.send_dashboard(f'set operational mode {mode}')

    def get_operational_mode(self):
        """Returns the operational mode as MANUAL or AUTOMATIC if the password has been set for Mode in Settings.
            Returns NONE if the password has not been set.      

        Returns:
            _type_: str
            MANUAL, AUTOMATIC or NONE
        """
        return self.send_dashboard('get operational mode')

    def clear_operational_mode(self):
        """If this function is called the operational mode can again be changed from PolyScope, 
            and the user password is enabled. 

        Returns:
            _type_: str
            "operational mode is no longer controlled by Dashboard Server"
        """
        return self.send_dashboard('clear operational mode')

    def power_on(self):
        """Powers on the robot arm

        Returns:
            _type_: str
            "Powering on"
        """
        return self.send_dashboard('power on')

    def power_off(self):
        """Powers off the robot arm

        Returns:
            _type_: str
            "Powering off"
        """
        return self.send_dashboard('power off')

    def brake_release(self):
        """Releases the brakes

        Returns:
            _type_: str
            "Brake releasing"
        """
        return self.send_dashboard('brake release')

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
        return self.send_dashboard('safetystatus')

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
        return self.send_dashboard('unlock protective stop')

    def close_safety_popup(self):
        """Closes a safety popup

        Returns:
            _type_: str
            "closing safety popup"
        """
        return self.send_dashboard('close safety popup')

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
        return self.send_dashboard(f'load installation {installation}')

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
        return self.send_dashboard('restart safety')

    def remote_control(self):
        """Returns the remote control status of the robot.
        If the robot is in remote control it returns true and if remote control is disabled or robot is 
        in local control it returns false.

        Returns:
            _type_: str
            "true" or "false"
        """
        return self.send_dashboard('is in remote control')

    def get_serial_number(self):
        """Returns serial number of the robot.

        Returns:
            _type_: str
            Serial number like "20175599999"
        """
        return self.send_dashboard('get serial number')

    def get_robot_model(self):
        """Returns the robot model

        Returns:
            _type_: str
            UR3, UR5, UR10, UR16
        """
        return self.send_dashboard('get robot model')

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
        return self.send_dashboard(f'generate flight report {type}')

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
        return self.send_dashboard(f'generate support file {path}')

    # endregion dashboard

    # region interpreter

    STATE_REPLY_PATTERN = re.compile(r"(\w+):\W+(\d+)?")

    def send_interpreter(self, command: str):
        """
        Send single line command to interpreter mode, and wait for reply
        :param command:
        :return: ack, or status id
        """
        if not command.endswith("\n"):
            command += "\n"

        self.interpreter.send(command.encode("utf-8"))
        raw_reply = self._get_reply()
        # parse reply, raise exception if command is discarded
        reply = self.STATE_REPLY_PATTERN.match(raw_reply)
        try:
            if reply.group(1) == "discard":
                return 0
            return int(reply.group(2))
        except:
            return 0

    def _get_reply(self):
        """
        read one line from the socket
        :return: text until new line
        """
        collected = b''
        while True:
            try:
                part = self.interpreter.recv(1)
            except:
                part = b"\n"
            if part != b"\n":
                collected += part
            elif part == b"\n":
                break
        return collected.decode("utf-8", "replace")

    def start_interpreter(self):
        return self.send_control("interpreter_mode()")

    def clear_interpreter(self):
        return self.send_interpreter("clear_interpreter()")

    def skip_interpreter_buffer(self):
        return self.send_interpreter("skipbuffer")

    def abort_interpreter(self):
        return self.send_interpreter("abort")

    def get_last_interpreted_id(self):
        return self.send_interpreter("statelastinterpreted")

    def get_last_executed_id(self):
        return self.send_interpreter("statelastexecuted")

    def get_last_cleared_id(self):
        return self.send_interpreter("statelastcleared")

    def get_unexecuted_count(self):
        return self.send_interpreter("stateunexecuted")

    def end_interpreter(self):
        return self.send_interpreter("end_interpreter()")

    def run_program(self, program: list):
        self.send_control('interpreter_mode()')
        self.clear_interpreter()
        time.sleep(1)
        buffer_limit = 500
        command_count = 1
        for line in program:
            command_id = self.send_interpreter(line)
            if command_count % buffer_limit == 0:
                while self.get_last_executed_id() != command_id:
                    _log.info(
                        f"Last executed id {self.get_last_executed_id()}/{command_id}")
                    time.sleep(2)
                self.clear_interpreter()
            command_count += 1
            while self.get_last_executed_id() != command_id:
                time.sleep(1)
        return True

    # endregion interpreter

    # region control

    def send_control(self, command: str):
        """
        Send command without waiting for reply.

        Args:
            command (str): string formatted
        """
        if not command.endswith("\n"):
            command += "\n"

        self.control.send(command.encode("utf-8"))

    def set_tcp(self, tcp):
        """Set TCP
        Will sleep for 1 second, otherwise robot will not take command. 

        Args:
            tcp (list): X, Y, Z, rX, rY, rZ
        """
        self.send_control(f'set_tcp(p{tcp})')
        time.sleep(1)

    def set_payload(self, m: float, cog: list = [0, 0, 0]):
        """Set Payload in kilograms
        Will sleep for half second, otherwise robot will not take command. 

        Args:
            m (float): mass in kilograms
            cog (float): mass in kilograms
        """
        self.send_control(f'set_payload({m}, {cog})')
        time.sleep(0.5)

    def move(self, command, wait: bool = False, area: float = 0.001):
        """Executes the genarated movement of robot. If wait == True, then the scrip is blocked until the target is reached.
        You can specify the the accuracy with level. Higher is more accurate. 

        Args:
            command (str): Command generated with joints, movel or movej function. 
            wait (bool, optional): Blocks script until target is reached. Defaults to False.
            level (int, optional): Level of accuracy while checking the target reached. Defaults to 5.

        Returns:
            bool: returns true if target is reached. 
        """
        self.send_control(command)
        if wait:
            target = [float(i) for i in command.split(
                '[')[1].split(']')[0].split(', ')]
            if command.__contains__('movej(['):
                while not self.target_reached(target, area, True):
                    time.sleep(0.1)
            elif command.__contains__('movej(p'):
                while not self.target_reached(target, area, False):
                    time.sleep(0.1)
            elif command.__contains__('movel(p'):
                while not self.target_reached(target, area, False):
                    time.sleep(0.1)
            return True

    # TODO: Work on this feature.
    # def tool_contact(self, direction: list):
    #     while True:
    #         step_back = tool_contact()
    #         if step_back <= 0:
    #             # Continue moving with 100mm/s
    #             speedl([0, 0, -0.100, 0, 0, 0], 0.5, t=get_steptime())
    #         else:
    #             # Contact detected!
    #             # Get q for when the contact was first seen
    #             q = get_actual_joint_positions_history(step_back)
    #             # Stop the movement
    #             stopl(3)
    #             # Move to the initial contact point
    #             movel(q)
    #             break

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
        command = f'set_digital_out({str(out)}, {str(on)})'
        self.send_control(command)

    def set_analog_out(self, out: int, level: float):
        """Send analog out command in V

        Args:
            out (int): Output selection
            level (float): level in Volts
        """
        self.send_control(
            f'set_standard_analog_outputdomain({out}, 1)')
        command = f'set_standard_analog_out({out}, {level/10})'
        self.send_control(command)

    # endregion control


def degree_to_radians(degree: float):
    return round(degree * pi / 180, 5)


def radians_to_degree(radian: float):
    return round(radian * 180 / pi, 5)


def movej(posix: list, a=1.4, v=1.05, t=0, r=0):
    """
    Create Target where robot should move fastest
    • a = 1.4 → acceleration is 1.4 rad/s/s
    • v = 1.05 → velocity is 1.05 rad/s
    • t = 0 the time (seconds) to make move is not specified. If it were specified the command would ignore the a and v values.
    • r = 0 → the blend radius is zero meters.
    :return: Binary Command
    """
    return f'movej(p{posix}, {a}, {v}, {t}, {r})\n'


def movel(posix: list, a=1.2, v=0.25, t=0, r=0):
    """
    Create Target where robot should move Linear
    a: tool acceleration [m/s^2]
    v: tool speed [m/s]
    t: time[S]
    r: blend radius [m]
    :return:
    """
    return f'movel(p{posix}, {a}, {v}, {t}, {r})\n'


def joints(joints: list, a=1.2, v=0.25, t=0, r=0, deg=False):
    """
    • a = 1.4 → acceleration is 1.4 rad/s/s
    • v = 1.05 → velocity is 1.05 rad/s
    • t = 0 the time (seconds) to make move is not specified. If it were specified the command would ignore the a and v values.
    • r = 0 → the blend radius is zero meters.
    • deg = If True it will calculate the joints from degrees to radians. 
    :return:
    """
    if deg:
        joints = [degree_to_radians(i) for i in joints]
    return f'movej({joints}, {a}, {v}, {t}, {r})\n'


def speedl(direction, a=1.2, t=0, aRot='a'):
    """
    direction: tool speed [m/s] (spatial vector)
    a: tool position acceleration [m/s^2]
    t: time [s] before function returns (optional)
    aRot: tool acceleration [rad/s^2] (optional), if not defined a, position acceleration, is used
    """
    return f'speedl({direction}, {a}, {t}, aRot="{aRot}")\n'


def speedj(joints, a=1.2, t=0):
    """
    joints: tool speed [rad/s]
    a: joint acceleration [rad/s^2] (of leading axis)
    t: time [s] before function returns (optional)
    """
    return f'speedj({joints}, {a}, {t})\n'


if __name__ == "__main__":
    rob = Robot("192.168.8.229")
    rob.connect()
    rob.get_controller_version()
    # rob.power_on()
    # rob.brake_release()
    rob.move(movej([0.086, -0.52813, 0.017, 0, 3.14, 0], t=1), True, 0.0001)
    rob.move(joints([-4.712388980384691, -1.5707963267948966, 1.570796326794897,
                     -1.5707963267948966, -1.5707963267948966, 8.881784197001252e-16], t=1), True)
    rob.move(joints([90, -90, 90, -90, -90, 0], t=3, deg=True), True)
    rob.move(movel([200e-3, -300e-3, 100e-3, 2.221, 2.221, 0], t=3), True)
    rob.move(movel([100e-3, -1000e-3, 10e-3, 2.221, 2.221, 0], t=5), True)
