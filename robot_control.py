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
    robot_ip = '192.168.11.11'
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
