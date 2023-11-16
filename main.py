from fanucpy import Robot
import numpy as np
import requests
import roboticstoolbox as rtb
import spatialmath as sm


esp32_ip = "192.168.4.1"                    # ESP32 IP address
relay_on_url = f"http://{esp32_ip}/L"
relay_off_url = f"http://{esp32_ip}/H"


def control_relay(action):
    response = requests.get(relay_on_url if action == "on" else relay_off_url)
    if response.status_code == 200:
        print(f"Relay turned {action.upper()}")
    else:
        print(f"Failed to turn {action.upper()} relay")


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


lrmate = LRMate200iD4S_gen()


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

        control_relay("on")
        fanuc_robot.move("joint", vals=new_joint_positions, velocity=5, acceleration=50, cnt_val=0, linear=False)
        control_relay("off")

    except Exception as e:
        print(f"Failed to move: {str(e)}")


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
            linear=False
        )
        print(f"Moved joint {joint_index + 1} by {degrees} degrees.")
    except Exception as e:
        print(f"Failed to move joint {joint_index + 1}: {str(e)}")


def main():
    try:
        global lrmate
        print(lrmate)
        robot = Robot(robot_model="Fanuc", host="192.168.1.10", port=18735, ee_DO_type="RDO", ee_DO_num=7)
        robot.connect()         # ci si connette al robot
        robot.gripper(False)    # gripper chiuso

        current_joint_positions = robot.get_curjpos()                       # si legge la posizione dei joint
        print("Current Joint Positions:", current_joint_positions)
        current_cartesian_positions = robot.get_curpos()                    # si legge la posizione cartesiana (non conosciamo la base)
        print("Current Cartesian Positions:", current_cartesian_positions)

        # # Cinematica Diretta
        # joint_angles = np.zeros([1, 6])
        # joint_angles[0, :] = current_joint_positions
        # current_joint_positions_1 = joints_fanuc2corke(joint_angles)                                                    # trasformazione angoli joint per sfruttare il toolbox di Corke
        # current_fkine_position = lrmate.fkine(current_joint_positions_1)
        # print("Current fkine Positions:", current_fkine_position)                                                       # con la cinematica diretta si trova la posizione cartesiana secondo la base definita qui
        #
        # # Cinematica Inversa
        # euler_angles = sm.SE3.rpy(current_fkine_position, 'deg', 'zyx')                                               # si ottengono gli angoli di eulero per yaw-pitch-roll partendo dalla cinematica diretta
        # rotation_matrix = sm.SO3.RPY([euler_angles[0], euler_angles[1], euler_angles[2]], unit="deg")                 # viene definita la matrice di rotazione con gli angoli di eulero
        # try_pose = sm.SE3(rotation_matrix)                                                                              # si crea la matrice di trasformazione omogenea, vengono inseriti i valore di orientamento
        # try_pose.A[0, 3] = current_fkine_position.A[0, 3]                                                               # si inserisce la posizione cartesiana X
        # try_pose.A[1, 3] = current_fkine_position.A[1, 3]                                                               # si inserisce la posizione cartesiana Y
        # try_pose.A[2, 3] = current_fkine_position.A[2, 3]                                                               # si inserisce la posizione cartesiana Z
        # current_ikine_position = lrmate.ikine_LM(try_pose, q0=current_joint_positions)                                  # si calcola la cinematica inversa
        #
        # current_ikine_position_deg = corke2fanuc(current_ikine_position.q)                                              # trasformazione in base Fanuc (da Corke)
        # print("Current ikine values deg:", current_ikine_position_deg)
        #
        # original_value = np.array(current_ikine_position_deg)
        # rounded_value = np.round(original_value, decimals=2)
        # print(rounded_value)
        # transformed_values = rounded_value.tolist()[0]
        # new_joint_positions = transformed_values                            # si crea la nuova lista di Joint-position da fornire al robot.move()
        # print(new_joint_positions)

        # PER TEST MANUALI
        # joint_positions = new_joint_positions # DA LANCIARE MANUALMENTE PER TESTARE ROBOT.MOVE()
        # robot.move("joint", vals=joint_positions, velocity=5, acceleration=50, cnt_val=0, linear=False) # joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        while True:
            user_input = input("Enter 'on', 'off', 'move1', 'move2', 'move3' or 'q' to quit: ")
            if user_input.lower() == "on" or user_input.lower() == "off":
                control_relay(user_input.lower())
            elif user_input.lower() == "move1":
                move_robot_joint(robot, 5, 10.0)
            elif user_input.lower() == "move2":
                move_robot_joint(robot, 5, -10.0)
            elif user_input.lower() == "move3":
                move_robot_cartesian(lrmate, robot)
            elif user_input.lower() == "moveh":
                robot.move("joint", vals=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity=5, acceleration=50, cnt_val=0, linear=False)
            elif user_input.lower() == "q":
                robot.disconnect()
                break
            else:
                print("Invalid command. Enter 'on', 'off', 'move1', 'move2', 'move3' or 'q'.")

    except Exception as e:
        print("Connection to the robot was not established:", str(e))


if __name__ == "__main__":
    main()
