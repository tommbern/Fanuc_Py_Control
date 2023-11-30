from fanucpy import Robot
import sys
import RobotFunctions as Rf
import threading

lrmate = Rf.LRMate200iD4S_gen()

print(lrmate)
robot = Robot(robot_model="Fanuc", host="192.168.1.10", port=18735, ee_DO_type="RDO", ee_DO_num=7)
robot.connect()         # ci si connette al robot
robot.gripper(False)    # gripper chiuso

current_joint_positions = robot.get_curjpos()                       # si legge la posizione dei joint
print("Current Joint Positions:", current_joint_positions)
current_cartesian_positions = robot.get_curpos()                    # si legge la posizione cartesiana (non conosciamo la base)
print("Current Cartesian Positions:", current_cartesian_positions)

# PER TEST MANUALI
# joint_positions = new_joint_positions # DA LANCIARE MANUALMENTE PER TESTARE ROBOT.MOVE()
# joint_positions = [0, 0, -20, 0, 30, 0]
# # joint_positions = [75.621, 111.708, 554.819, -167.993, -0.742, -90.583]
# cart_positions = [370, 0, 200, -180, 45, 0]
# robot.move("joint", vals=joint_positions, velocity=50, acceleration=100, cnt_val=0, linear=True)
# robot.move("pose", vals=cart_positions, velocity=200, acceleration=100, cnt_val=100, linear=True)

while True:
    try:
        while True:
            user_input = input("Enter 'on', 'off', 'move1', 'move2', 'move3' or 'q' to quit: ")
            if user_input.lower() == "on" or user_input.lower() == "off":
                Rf.control_relay(user_input.lower())

            elif user_input.lower() == "go":
                joint_positions = [0, 0, 0, 0, -45, 0]
                mode = "joint"
                velocity = 20

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_robot, args=(robot, joint_positions, mode, velocity))

                relay_thread.start()
                move_thread.start()

                relay_thread.join()
                move_thread.join()

            elif user_input.lower() == "go1":
                # joint_positions = [0, 7.9, -15.5, 0, -29.5, 0]
                cart_positions = [350, 50, -10, 180, 0, 0]
                mode = "pose"
                velocity = 10

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_robot, args=(robot, cart_positions, mode, velocity))

                relay_thread.start()
                move_thread.start()

                relay_thread.join()
                move_thread.join()

            elif user_input.lower() == "go2":
                # joint_positions = [0, 30, -27, 0, -60, 0]
                cart_positions = [350, -50, -10, 180, 0, 0]
                mode = "pose"
                velocity = 10

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_robot, args=(robot, cart_positions, mode, velocity))

                relay_thread.start()
                move_thread.start()

                relay_thread.join()
                move_thread.join()

            elif user_input.lower() == "move1":
                Rf.move_robot_joint(robot, 5, 10.0)
            elif user_input.lower() == "move2":
                Rf.move_robot_joint(robot, 5, -10.0)
            elif user_input.lower() == "move3":
                Rf.move_robot_cartesian(lrmate, robot)
            elif user_input.lower() == "move4":
                x_translation_input = input("Enter X translation value in millimeters (press Enter for 0): ")
                x_translation = float(x_translation_input) / 1000 if x_translation_input else 0
                y_translation_input = input("Enter Y translation value in millimeters (press Enter for 0): ")
                y_translation = float(y_translation_input) / 1000 if y_translation_input else 0
                z_translation_input = input("Enter Z translation value in millimeters (press Enter for 0): ")
                z_translation = float(z_translation_input) / 1000 if z_translation_input else 0

                joint_positions_list = Rf.move_robot_cartesian_large_displacement(lrmate, robot, x_translation, y_translation, z_translation)
                Rf.move_robot_with_joint_positions(robot, joint_positions_list)
            elif user_input.lower() == "moveh":
                robot.move("joint", vals=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity=5, acceleration=50, cnt_val=0, linear=False)

            elif user_input.lower() == "movec1":
                Rf.control_relay("on")
                robot.move("pose", vals=[340, 0.0, 190, 0.0, -80, -180], velocity=100, acceleration=100, cnt_val=100, linear=True)
                Rf.control_relay("off")
            elif user_input.lower() == "movec2":
                Rf.control_relay("on")
                robot.move("pose", vals=[340, 0.0, 190, 0.0, -80, -180], velocity=100, acceleration=100, cnt_val=100, linear=True)
                robot.move("pose", vals=[340, 0.0, 150, 0.0, -80, -180], velocity=100, acceleration=100, cnt_val=100, linear=True)
                Rf.control_relay("off")

            elif user_input.lower() == "q":
                robot.disconnect()
                sys.exit()
            else:
                print("Invalid command. Enter 'on', 'off', 'move1', 'move2', 'move3' or 'q'.")

    except Exception as e:
        print("Connection to the robot was not established:", str(e))
#
#
# if __name__ == "__main__":
#     main()

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
