import time
import serial
import pandas as pd
import numpy as np
import threading
from conn_library import motorObj


global data_buffer


def init_motor_status(motor):
    motor.setMotorMode('!\r\n')   # restart
    print(motor.readUntilEmpty())
    time.sleep(0.5)
    motor.clearReadBuff()


def appendBuffer(motor, test_time, buff_time):
    global data_buffer
    assert isinstance(motor, motorObj)
    assert isinstance(data_buffer, bytes)
    start_time = time.time()
    while motor.checkBuffer() or (time.time() - start_time < test_time):
        data_buffer += motor.readBytes(decode=False)
        time.sleep(buff_time)


def set_parameter(motor,
                  s_kp, s_ki, s_kd,
                  p_kp, p_ki, p_kd, ctl_op, pos_ctl):
    params = [s_kp, s_ki, s_kd,
              p_kp, p_ki, p_kd, ctl_op, pos_ctl]
    labels = ["sp", "si", "sd",
              "pp", "pi", "pd", "sm", "pm"]

    for (param, label) in zip(params, labels):
        motor.setMotorMode(' '.join([label, str(param)]) + '\r\n')
        time.sleep(0.5)
        print(motor.readUntilEmpty())


def set_home(motor):
    print("---- Go home start ----")
    motor.setMotorMode('z5\r\n')   # position mode
    time.sleep(0.1)
    motor.setMotorMode('g 0\r\n')   # goto
    time.sleep(3)
    print(motor.readUntilEmpty())
    motor.setMotorMode('z0\r\n')
    print("---- Go home end ----")
    print()

if __name__ == '__main__':
    print('---- Dynamic Friction Test ----')

    motor = motorObj(f_port="COM6")  # initialize object
    rpm = 1
    test_time = 60*1
    buff_time = 3

    # init motor status
    init_motor_status(motor)

    # Get version
    print("Motor firmware version:", motor.getVersion())
    motor.clearReadBuff()
    print()

    # Set parameter
    set_parameter(motor, 100, 20, 100, 12, 1, 1, 15000, 80)

    # Set home
    set_home(motor)

    # Electric cal
    print("---- Electric cal. start ----")
    motor.setMotorMode('z2\r\n')
    time.sleep(3)
    print(motor.readUntilEmpty())
    print("---- Electric cal. end ----")
    print()

    # Dummy run
    print("---- Dummy run start ----")
    motor.setMotorMode('z4 1000\r\n')   # speed mode
    time.sleep(0.5)
    motor.setMotorMode('g\r\n')   # start
    time.sleep(3)
    motor.setMotorMode('z0\r\n')   # no control
    print(motor.readUntilEmpty())
    print("---- Dummy run end ----")
    print()

    ### Start before encoder ###
    motor.setMotorMode('z4 ' + str(int(100*rpm)) + '\r\n')
    time.sleep(0.1)
    motor.clearReadBuff()

    print('---- Dynamics friction test Start ----')
    print('rpm =', rpm)
    print('test time =', test_time)
    print()

    # Dynamics friction test
    global data_buffer
    data_buffer = b''
    # data = np.array([])
    logger = threading.Thread(target=appendBuffer, args=(motor, test_time, buff_time, ))

    motor.continuousEncoding(True)
    logger.start()

    ### Start after encoder ###
    # motor.setMotorMode('z4 ' + str(int(100 * rpm)) + '\r\n')

    time.sleep(test_time)
    motor.continuousEncoding(False)
    logger.join()

    # Decode
    data = data_buffer.decode().split('\n\r')[:-1]
    table = np.zeros((len(data), 9))
    for idx in range(len(data)):
        print(data[idx])
        table[idx][0] = int(data[idx].split(',')[0])  # encoder reading
        table[idx][1] = int(data[idx].split(',')[1])  # target position
        table[idx][2] = float(data[idx].split(',')[2])  # target rpm
        table[idx][3] = float(data[idx].split(',')[3])  # current rpm
        table[idx][4] = int(data[idx].split(',')[4])  # control current
        table[idx][5] = float(data[idx].split(',')[5])  # voltage
        table[idx][6] = int(data[idx].split(',')[6])  # encoder signal strength
        table[idx][7] = int(data[idx].split(',')[7])  # magnetic strength
        table[idx][8] = int(data[idx].split(',')[8])  # raw encoder reading

    table = pd.DataFrame(data=table, columns=['Encoder', 'TargetPosition', 'TargetRPM', 'CurrentRPM', 'ControlCurrent',
                                              'Voltage', 'EncoderStrength', 'MagneticStrength', 'RawEncoder'])

    freq = len(data) / test_time
    table.index = np.linspace(0, test_time - 1/freq, len(data))
    print(table)
    print()

    table.to_csv(r"./data/" + str(rpm) + 'rpm_tmp.csv')

    print("Avg. sample freq:", freq)
    print()

    motor.setMotorMode('z0\r\n')
    print('---- Dynamics friction test End ----')
