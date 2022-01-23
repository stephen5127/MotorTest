import time
import pandas as pd
import numpy as np
import threading
from conn_library import motorObj
import itertools
import datetime

global data_buffer


def init_motor_status(motor):
    motor.setMotorMode('!\r\n')   # restart
    print(motor.readUntilEmpty())
    time.sleep(0.5)
    motor.clearReadBuff()


def electric_cal(motor):
    print("---- Electric cal. start ----")
    motor.setMotorMode('z2\r\n')   # elec. cal.
    time.sleep(3)
    print(motor.readUntilEmpty())
    print("---- Electric cal. end ----")
    print()


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


def appendBuffer(motor, test_time, buff_time):
    global data_buffer
    assert isinstance(motor, motorObj)
    assert isinstance(data_buffer, bytes)
    start_time = time.time()
    while motor.checkBuffer() or (time.time() - start_time < test_time):
        data_buffer += motor.readBytes(decode=False)
        time.sleep(buff_time)


def dynamics_friction_test(motor, rpm, test_time, buff_time=3):
    print('---- Dynamics friction test Start ----')
    print('rpm =', rpm)
    print('test time =', test_time)
    print()

    # Set motion
    motor.setMotorMode('z4 ' + str(int(100 * rpm)) + '\r\n')
    time.sleep(0.1)
    motor.clearReadBuff()

    # Read
    global data_buffer
    data_buffer = b''  # init buffer
    logger = threading.Thread(target=appendBuffer, args=(motor, test_time, buff_time, ))

    motor.continuousEncoding(True)
    logger.start()

    ### Position control add here ###

    time.sleep(test_time)
    motor.continuousEncoding(False)
    logger.join()

    # Decode
    data = data_buffer.decode().split('\n\r')[:-1]
    table = np.zeros((len(data), 9))
    for idx in range(len(data)):
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
    print("Avg. sample freq:", freq)
    print()

    # Stop motion
    motor.setMotorMode('z0\r\n')
    print('---- Dynamics friction test End ----')

    return table


if __name__ == '__main__':
    motor = motorObj(f_port="COM6")  # initialize object
    # Test params
    s_kp_list = [50, 75, 100]
    s_ki_list = [20, 25, 30]
    s_kd_list = [50, 75, 100]
    p_kp_list = [12]
    p_ki_list = [1]
    p_kd_list = [1]
    ctl_op_list = [15000]
    pos_ctl_list = [80]
    rpm = 1
    test_time = 30

    # init motor status
    init_motor_status(motor)

    # Get version
    print("Motor firmware version:", motor.getVersion())
    motor.clearReadBuff()
    print()

    # Set home
    set_home(motor)

    # Electric cal
    electric_cal(motor)

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

    search_space = list(itertools.product(s_kp_list, s_ki_list, s_kd_list,
                                          p_kp_list, p_ki_list, p_kd_list,
                                          ctl_op_list, pos_ctl_list))
    print("Seach space size:", len(search_space))
    cnt = 0

    for (s_kp, s_ki, s_kd,
         p_kp, p_ki, p_kd, ctl_op, pos_ctl) in search_space:

        tmp = (s_kp, s_ki, s_kd,
               p_kp, p_ki, p_kd, ctl_op, pos_ctl)
        tmp = [str(i) for i in tmp]
        cnt += 1

        if s_kp == 0 and s_kd == 0:
            continue

        print()
        print("--- Parameter set", cnt, "---")
        print(tmp)
        print()

        # set param
        set_parameter(motor,
                      s_kp, s_ki, s_kd,
                      p_kp, p_ki, p_kd, ctl_op, pos_ctl)
        set_home(motor)
        electric_cal(motor)

        # dynamics friction
        data = dynamics_friction_test(motor, rpm, test_time)

        # Save data
        data.to_csv(r'./data/1rpm_' + '_'.join(tmp) + "_" + str(datetime.date.today()).replace("-", "") + ".csv")

        init_motor_status(motor)
