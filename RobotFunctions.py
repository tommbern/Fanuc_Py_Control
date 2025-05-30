from fanucpy import Robot
import numpy as np
import requests
import roboticstoolbox as rtb
import spatialmath as sm
import time


esp32_ip = "192.168.4.1"                    # ESP32 IP address
relay_on_url = f"http://{esp32_ip}/L"
relay_off_url = f"http://{esp32_ip}/H"


def control_relay(action):
    response = requests.get(relay_on_url if action == "on" else relay_off_url)
    if response.status_code == 200:
        print(f"Relay turned {action.upper()}")
    else:
        print(f"Failed to turn {action.upper()} relay")


def relay_task(action):
    time.sleep(0.01)
    control_relay("on")
    time.sleep(1)
    # control_relay("off")


def LRMate200iD4S_gen():  # Define the LRMate200iD4S robot using roboticstoolbox
    # Define DH parameters for your specific robot model
    # Replace these with the correct values for your robot
    dh_params = [
        # rtb.RevoluteDH(d=0.330, alpha=np.pi / 2),
        rtb.RevoluteDH(alpha=-np.pi / 2),
        rtb.RevoluteDH(a=0.260, alpha=np.pi, offset=-np.pi / 2),
        rtb.RevoluteDH(a=-0.02, alpha=np.pi / 2, offset=-np.pi),
        rtb.RevoluteDH(d=-0.290, alpha=-np.pi / 2),
        rtb.RevoluteDH(alpha=-np.pi / 2, offset=-np.pi),
        rtb.RevoluteDH(d=-0.07, alpha=-np.pi)
    ]

    # Create the robot model
    robot = rtb.DHRobot(dh_params, name="LRMate200iD4s")
    # robot.tool = sm.SE3(0.25, 0, 0.2) * sm.SE3.Ry(np.pi)
    # robot.base = sm.SE3(0, 0, -0.3)
    # print(robot)
    return robot


def joints_fanuc2corke(q):
    if q.ndim == 1:
        q = [q]
    q_adj = q
    q_adj[:, 2] = q[:, 2] + q[:, 1]
    q_adj = q_adj / 180 * np.pi
    return q_adj


def corke2fanuc(q):
    q_adj = np.array(q)
    if q_adj.ndim == 1:
        q_adj = q_adj[None, :]              # Convert 1D array to 2D
    q_adj[:, 2] = q_adj[:, 2] - q_adj[:, 1]
    q_adj = q_adj / np.pi * 180
    return q_adj


def move_robot_cartesian(lrmate, fanuc_robot):
    try_pose = []
    try:
        current_joint_positions = fanuc_robot.get_curjpos()                                                             # si legge la posizione dei joint
        print("Current Joint Positions:", current_joint_positions)
        current_cartesian_positions = fanuc_robot.get_curpos()                                                          # si legge la posizione cartesiana (non conosciamo la base)
        print("Current Cartesian Positions:", current_cartesian_positions)

        x_translation_input = input("Enter X translation value in millimeters (press Enter for 0): ")
        x_translation = float(x_translation_input) / 1000 if x_translation_input else 0
        y_translation_input = input("Enter Y translation value in millimeters (press Enter for 0): ")
        y_translation = float(y_translation_input) / 1000 if y_translation_input else 0
        z_translation_input = input("Enter Z translation value in millimeters (press Enter for 0): ")
        z_translation = float(z_translation_input) / 1000 if z_translation_input else 0

        # Cinematica Diretta
        joint_angles = np.zeros([1, 6])
        joint_angles[0, :] = current_joint_positions
        current_joint_positions_1 = joints_fanuc2corke(joint_angles)                                                    # trasformazione angoli joint per sfruttare il toolbox di Corke
        current_fkine_position = lrmate.fkine(current_joint_positions_1)
        print("Current fkine Positions:", current_fkine_position)                                                       # con la cinematica diretta si trova la posizione cartesiana secondo la base definita qui

        # Cinematica Inversa
        euler_angles = sm.SE3.rpy(current_fkine_position, 'deg', 'zyx')                                     # si ottengono gli angoli di eulero per yaw-pitch-roll partendo dalla cinematica diretta
        rotation_matrix = sm.SO3.RPY([euler_angles[0], euler_angles[1], euler_angles[2]], unit="deg")           # viene definita la matrice di rotazione con gli angoli di eulero
        try_pose = sm.SE3(rotation_matrix)                                                                              # si crea la matrice di trasformazione omogenea, vengono inseriti i valore di orientamento
        try_pose.A[0, 3] = current_fkine_position.A[0, 3] + x_translation  # Add user input to X position
        try_pose.A[1, 3] = current_fkine_position.A[1, 3] + y_translation  # Add user input to Y position
        try_pose.A[2, 3] = current_fkine_position.A[2, 3] + z_translation  # Add user input to Z position                                               # si inserisce la posizione cartesiana Z
        current_ikine_position = lrmate.ikine_LM(try_pose, q0=current_joint_positions)                                  # si calcola la cinematica inversa

        current_ikine_position_deg = corke2fanuc(current_ikine_position.q)                                              # trasformazione in base Fanuc (da Corke)
        print("Current ikine values deg:", current_ikine_position_deg)

        original_value = np.array(current_ikine_position_deg)
        rounded_value = np.round(original_value, decimals=2)
        print(rounded_value)
        transformed_values = rounded_value.tolist()[0]
        new_joint_positions = transformed_values                                                                        # si crea la nuova lista di Joint-position da fornire al robot.move()
        print(new_joint_positions)

        # control_relay("on")
        fanuc_robot.move("joint", vals=new_joint_positions, velocity=5, acceleration=50, cnt_val=0, linear=False)
        # control_relay("off")

    except Exception as e:
        print(f"Failed to move: {str(e)}")


def move_robot_cartesian_large_displacement(lrmate, fanuc_robot, x_translation, y_translation, z_translation):
    try:
        max_translation = max(abs(x_translation), abs(y_translation), abs(z_translation)) * 1000
        joint_positions_list = []

        for i in range(int(max_translation)):
            if i == 0:
                current_joint_positions = fanuc_robot.get_curjpos()
            else:
                current_joint_positions = joint_positions_list[-1]

            joint_angles = np.zeros([1, 6])
            joint_angles[0, :] = current_joint_positions
            current_joint_positions_1 = joints_fanuc2corke(joint_angles)
            current_fkine_position = lrmate.fkine(current_joint_positions_1)

            try_pose = sm.SE3(current_fkine_position.A)
            increment_x = x_translation / max_translation
            increment_y = y_translation / max_translation
            increment_z = z_translation / max_translation
            try_pose.A[0, 3] += increment_x
            try_pose.A[1, 3] += increment_y
            try_pose.A[2, 3] += increment_z

            current_ikine_position = lrmate.ikine_LM(try_pose, q0=current_joint_positions)
            current_ikine_position_deg = np.round(corke2fanuc(current_ikine_position.q), decimals=5)

            joint_positions_list.append(current_ikine_position_deg[0].tolist())
            print(f"Iteration {i + 1}: Joint positions - {current_ikine_position_deg}")

        return joint_positions_list

    except Exception as e:
        print(f"Failed to move: {str(e)}")
        return []


def move_robot_with_joint_positions(robot, joint_positions_list):
    try:
        # control_relay("on")

        for joint_positions in joint_positions_list:
            robot.move("joint", vals=joint_positions, velocity=20, acceleration=100, cnt_val=50, linear=False)

        # control_relay("off")

    except Exception as e:
        print(f"Failed to move: {str(e)}")


def move_robot(robot, joint_positions, mode, velocity):
    try:
        robot.move(mode, vals=joint_positions, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
        print("Robot moved successfully.")
    except Exception as e:
        print(f"Failed to move the robot: {str(e)}")


def move_between_positions(robot, mode, velocity, position1, position2, position3, position4, position5, position6):
    for _ in range(5):
        robot.move(mode, vals=position1, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
        robot.move(mode, vals=position2, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
    for _ in range(5):
        robot.move(mode, vals=position3, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
        robot.move(mode, vals=position4, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
    for _ in range(5):
        robot.move(mode, vals=position5, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
        robot.move(mode, vals=position6, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
    control_relay("off")


def move_fd(robot, start_position, end_position, mode, velocity):
    try:
        control_relay("on")
        robot.move(mode, vals=end_position, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
        print("Robot moved successfully.")
    except Exception as e:
        print(f"Failed to move the robot: {str(e)}")
    control_relay("off")
    robot.move(mode, vals=start_position, velocity=50, acceleration=100, cnt_val=0, linear=True)


def move_test2(robot, mode, velocity, iteration, position1, position2):
    max_iteration = iteration
    for i in range(max_iteration):
        robot.move(mode, vals=position2, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        print(f"Iteration {i + 1} out of {max_iteration}")

    control_relay("off")
    robot.move(mode, vals=position1, velocity=50, acceleration=100, cnt_val=0, linear=True)


def move_test3(robot, mode, velocity, iteration, position1, position2, position3, position4, position5, position6):
    test3_iterations = iteration
    velocity = velocity
    for i in range(test3_iterations):
                    # control_relay("on")
        robot.move(mode, vals=position1, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position2, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        robot.move(mode, vals=position4, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position3, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        robot.move(mode, vals=position5, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position6, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        print(f"Iteration {i + 1} out of {test3_iterations}")


def move_test33(robot, mode, velocity, iteration, position1, position2, position3, position4, position5, position6):
    test33_iterations = iteration
    velocity = velocity
    for i in range(test33_iterations):
        # control_relay("on")
        robot.move(mode, vals=position1, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position2, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        robot.move(mode, vals=position4, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position3, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        robot.move(mode, vals=position5, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position6, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        print(f"Iteration {i + 1} out of {test33_iterations}")


def move_test34(robot, mode, velocity, iteration, position1, position2, position3, position4, position5, position6):
    test34_iterations = iteration
    velocity = velocity
    for i in range(test34_iterations):
        # control_relay("on")
        robot.move(mode, vals=position1, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position2, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        robot.move(mode, vals=position3, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position4, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position3, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        robot.move(mode, vals=position5, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position6, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position5, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        print(f"Iteration {i + 1} out of {test34_iterations}")


def move_test6(robot, mode, velocity, iteration, position1, position2, position3, position4, position5,
                position6, position7, position8, position9, position10, position11, position12):
    test6_iterations = iteration
    velocity = velocity
    for i in range(test6_iterations):
        # control_relay("on")
        robot.move(mode, vals=position1, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position2, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        robot.move(mode, vals=position4, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position3, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        robot.move(mode, vals=position5, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position6, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        robot.move(mode, vals=position8, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position7, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        robot.move(mode, vals=position9, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position10, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        robot.move(mode, vals=position12, velocity=100, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        robot.move(mode, vals=position11, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        control_relay("off")
        print(f"Iteration {i + 1} out of {test6_iterations}")


def move_test4(robot, mode, velocity, position1, position2, position3, position4):
    test3_iterations = 1
    for i in range(test3_iterations):
        # control_relay("on")
        robot.move(mode, vals=position1, velocity=100, acceleration=50, cnt_val=0, linear=True)
        control_relay("on")
        time.sleep(5)
        control_relay("off")
        robot.move(mode, vals=position2, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
        control_relay("on")
        time.sleep(5)
        control_relay("off")
        robot.move(mode, vals=position3, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
        control_relay("on")
        time.sleep(5)
        control_relay("off")
        robot.move(mode, vals=position4, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
        control_relay("on")
        time.sleep(5)
        control_relay("off")
        print(f"Iteration {i + 1} out of {test3_iterations}")


def move_test5(robot, mode, velocity, position1, position2, position3, position4):
    max_iteration = 5

    robot.move(mode, vals=position1, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
    control_relay("on")
    for i in range(max_iteration):
        robot.move(mode, vals=position2, velocity=10, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=10, acceleration=100, cnt_val=0, linear=True)
        print(f"Iteration {i + 1} out of {max_iteration}")
    control_relay("off")
    robot.move(mode, vals=position1, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
    control_relay("on")
    for i in range(max_iteration):
        robot.move(mode, vals=position2, velocity=50, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=50, acceleration=100, cnt_val=0, linear=True)
        print(f"Iteration {i + 1} out of {max_iteration}")
    control_relay("off")

    robot.move(mode, vals=position2, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
    control_relay("on")
    for i in range(max_iteration):
        robot.move(mode, vals=position3, velocity=50, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position2, velocity=50, acceleration=100, cnt_val=0, linear=True)
        print(f"Iteration {i + 1} out of {max_iteration}")
    control_relay("off")
    robot.move(mode, vals=position2, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
    control_relay("on")
    for i in range(max_iteration):
        robot.move(mode, vals=position3, velocity=10, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position2, velocity=10, acceleration=100, cnt_val=0, linear=True)
        print(f"Iteration {i + 1} out of {max_iteration}")
    control_relay("off")

    # robot.move(mode, vals=position2, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
    # control_relay("on")
    # for i in range(max_iteration):
    #     robot.move(mode, vals=position3, velocity=50, acceleration=100, cnt_val=0, linear=True)
    #     robot.move(mode, vals=position2, velocity=10, acceleration=100, cnt_val=0, linear=True)
    #     print(f"Iteration {i + 1} out of {max_iteration}")
    # control_relay("off")
    #
    # robot.move(mode, vals=position3, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
    # control_relay("on")
    # for i in range(max_iteration):
    #     robot.move(mode, vals=position4, velocity=50, acceleration=100, cnt_val=0, linear=True)
    #     robot.move(mode, vals=position3, velocity=10, acceleration=100, cnt_val=0, linear=True)
    #     print(f"Iteration {i + 1} out of {max_iteration}")
    # control_relay("off")

    robot.move(mode, vals=position1, velocity=velocity, acceleration=50, cnt_val=0, linear=True)


def move_test10(robot, mode, velocity, iteration, position1, position2, position3, position4, position5, position6,
                position7, position8, position9, position10, position11):
    max_iteration = iteration
    for i in range(max_iteration):
        robot.move(mode, vals=position2, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)

        robot.move(mode, vals=position3, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)

        robot.move(mode, vals=position4, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)

        robot.move(mode, vals=position5, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)

        robot.move(mode, vals=position6, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)

        robot.move(mode, vals=position7, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)

        robot.move(mode, vals=position8, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)

        robot.move(mode, vals=position9, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)

        robot.move(mode, vals=position10, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)

        robot.move(mode, vals=position11,  velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        robot.move(mode, vals=position1, velocity=velocity, acceleration=100, cnt_val=0, linear=True)
        print(f"Iteration {i + 1} out of {max_iteration}")

    control_relay("off")
    robot.move(mode, vals=position1, velocity=50, acceleration=100, cnt_val=0, linear=True)


def move_robot_cycle(robot, mode, velocity):
    try:

        for _ in range(2):
            cart_positions = [[350, 150, -100, 180, 0, 0],
                              [350, -150, -100, 180, 0, 0],
                              [450, -150, -100, 180, 0, 0],
                              [450, 150, -100, 180, 0, 0]]

            for position in cart_positions:
                robot.move(mode, vals=position, velocity=velocity, acceleration=50, cnt_val=0, linear=True)

        # cart_positions = [350, 150, -100, 180, 0, 0]
        # robot.move(mode, vals=cart_positions, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
        # cart_positions = [350, -150, -100, 180, 0, 0]
        # robot.move(mode, vals=cart_positions, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
        # cart_positions = [450, -150, -100, 180, 0, 0]
        # robot.move(mode, vals=cart_positions, velocity=velocity, acceleration=50, cnt_val=0, linear=True)
        # cart_positions = [450, 150, -100, 180, 0, 0]
        # robot.move(mode, vals=cart_positions, velocity=velocity, acceleration=50, cnt_val=0, linear=True)

        print("Robot moved successfully.")
    except Exception as e:
        print(f"Failed to move the robot: {str(e)}")


def move_robot_joint(robot, joint_index, degrees, velocity=5):
    try:
        current_joint_positions = robot.get_curjpos()
        joint_positions = current_joint_positions.copy()
        joint_positions[joint_index] += degrees
        robot.move(
            "joint",
            vals=joint_positions,
            velocity=velocity,  # Reduced velocity
            acceleration=50,
            cnt_val=0,
            linear=True
        )
        print(f"Moved joint {joint_index + 1} by {degrees} degrees.")
    except Exception as e:
        print(f"Failed to move joint {joint_index + 1}: {str(e)}")


def move_gaia(robot, cart_positions, mode, velocity):
    try:
        robot.move(mode, vals=cart_positions, velocity=50, acceleration=100, cnt_val=0, linear=True)
        control_relay("on")
        time.sleep(9)
        print("Robot moved successfully.")
    except Exception as e:
        print(f"Failed to move the robot: {str(e)}")
    control_relay("off")
