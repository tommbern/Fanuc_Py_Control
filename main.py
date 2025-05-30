from fanucpy import Robot
import sys
import RobotFunctions as Rf
import threading

lrmate = Rf.LRMate200iD4S_gen()

print(lrmate)
robot = Robot(robot_model="Fanuc", host="192.168.1.10", port=18735, ee_DO_type="RDO", ee_DO_num=7)
robot.connect()         # ci si connette al robot
# robot.gripper(False)    # gripper chiuso

current_joint_positions = robot.get_curjpos()                       # si legge la posizione dei joint
print("Current Joint Positions:", current_joint_positions)
current_cartesian_positions = robot.get_curpos()                    # si legge la posizione cartesiana (non conosciamo la base)
print("Current Cartesian Positions:", current_cartesian_positions)
#
# PER TEST MANUALI
# # #  joint_positions = [75.621, 111.708, 554.819, -167.993, -0.742, -90.583]
# cart_positions = [350, 120, 200, 90.01, -90.01, 90.01]
# # robot.move("joint", vals=joint_positions, velocity=20, acceleration=100, cnt_val=0, linear=True)
# robot.move("pose", vals=cart_positions, velocity=20, acceleration=75, cnt_val=100, linear=True)
# # # # robot.disconnect()

while True:
    try:
        while True:
            user_input = input("Enter 'on', 'off', 'move1', 'move2', 'move3' or 'q' to quit: ")
            if user_input.lower() == "on" or user_input.lower() == "off":
                Rf.control_relay(user_input.lower())

            elif user_input.lower() == "gaia":
                cart_positions = [444, 120, 175, 90.01, -90.01, 90.01]
                mode = "pose"
                velocity = 20

                robot.move("pose", vals=cart_positions, velocity=20, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_gaia, args=(robot, cart_positions, mode, velocity))

                relay_thread.start()
                move_thread.start()

                relay_thread.join()
                move_thread.join()

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
                cart_positions = [350, 80, 200, 90.01, -90.01, 90.01]

                mode = "pose"
                velocity = 20

                # relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_robot, args=(robot, cart_positions, mode, velocity))

                # relay_thread.start()
                move_thread.start()

                # relay_thread.join()
                move_thread.join()

            elif user_input.lower() == "fd": #test for focal distance
                # joint_positions = [0, 7.9, -15.5, 0, -29.5, 0]
                start_position = [455, 100, 182, 90.01, -90.01, 90.01]
                end_position = [435, 100, 182, 90.01, -90.01, 90.01]

                mode = "pose"
                velocity = 20
                robot.move("pose", vals=start_position, velocity=20, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_fd, args=(robot, start_position, end_position, mode, velocity))

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

            elif user_input.lower() == "cycle":
                # joint_positions = [0, 7.9, -15.5, 0, -29.5, 0]
                # cart_positions = [450, 150, -100, 180, 0, 0]

                mode = "pose"
                velocity = 100

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_robot_cycle, args=(robot, mode, velocity))

                relay_thread.start()
                move_thread.start()

                Rf.control_relay("off")
                relay_thread.join()
                move_thread.join()

            elif user_input.lower() == "timer":

                Rf.control_relay("on")
                Rf.time.sleep(180)
                Rf.control_relay("off")

            elif user_input.lower() == "test1":
                mode = "pose"
                velocity = 50

                position1 = [410, -50, 150, -90, 0, -90]
                position2 = [410, -50, 100, -90, 0, -90]

                position3 = [410, 0, 150, -90, 0, -90]
                position4 = [410, 0, 100, -90, 0, -90]

                position5 = [410, 50, 150, -90, 0, -90]
                position6 = [410, 50, 100, -90, 0, -90]

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_between_positions, args=(robot, mode, velocity, position1, position2, position3, position4, position5, position6))

                relay_thread.start()
                move_thread.start()

                relay_thread.join()
                # Rf.control_relay("off")
                move_thread.join()

            elif user_input.lower() == "test2": #verticale, setting:X-axis
                mode = "pose"
                velocity = 10
                iteration = 1

                position1 = [353, 35, 210, 90.01, -90.01, 90.01]
                position2 = [353, 35, 160, 90.01, -90.01, 90.01]
                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_test2, args=(robot, mode, velocity, iteration, position1, position2))

                relay_thread.start()
                move_thread.start()
                relay_thread.join()
                move_thread.join()

            elif user_input.lower() == "test22": #orizzontale, setting:Y-axis
                mode = "pose"
                velocity = 20
                iteration = 10

                position1 = [345, -100, -40, 179.99, -89, 0.01]
                position2 = [345, -190, -40, 179.99, -89, 0.01]
                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_test2, args=(robot, mode, velocity, iteration, position1, position2))

                relay_thread.start()
                move_thread.start()
                relay_thread.join()
                move_thread.join()

            elif user_input.lower() == "test3": #orizzontale, setting:Y-axis
                mode = "pose"
                velocity = 10
                iteration = 1

                position1 = [380, 30, 185, 179.99, -85, 0.01]
                position2 = [380, 120, 185, 179.99, -85, 0.01]

                position3 = [380, 30, 155, 179.99, -85, 0.01]
                position4 = [380, 120, 155, 179.99, -85, 0.01]

                position5 = [380, 30, 125, 179.99, -85, 0.01]
                position6 = [380, 120, 125, 179.99, -85, 0.01]
                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_test3, args=(robot, mode, velocity, iteration, position1, position2, position3, position4, position5, position6))

                relay_thread.start()
                move_thread.start()
                relay_thread.join()
                move_thread.join()

                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)
            elif user_input.lower() == "test33": #verticale, setting:X-axis
                mode = "pose"
                velocity = 10
                iteration = 1

                position1 = [477, 60, 200, 179.99, -85, 0.01]
                position2 = [477, 60, 110, 179.99, -85, 0.01]

                position3 = [477, 30, 200, 179.99, -85, 0.01]
                position4 = [477, 30, 110, 179.99, -85, 0.01]

                position5 = [477, 0, 200, 179.99, -85, 0.01]
                position6 = [477, 0, 110, 179.99, -85, 0.01]
                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_test33, args=(robot, mode, velocity, iteration, position1, position2, position3, position4, position5, position6))

                relay_thread.start()
                move_thread.start()
                relay_thread.join()
                move_thread.join()

                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

            elif user_input.lower() == "test34": #verticale, setting:X-axis
                mode = "pose"
                velocity = 20
                iteration = 2

                position1 = [470, 60, 100, 179.99, -85, 0.01]
                position2 = [470, 60, 10, 179.99, -85, 0.01]

                position3 = [470, 30, 100, 179.99, -85, 0.01]
                position4 = [470, 30, 10, 179.99, -85, 0.01]

                position5 = [470, 0, 100, 179.99, -85, 0.01]
                position6 = [470, 0, 10, 179.99, -85, 0.01]
                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_test34, args=(robot, mode, velocity, iteration, position1, position2, position3, position4, position5, position6))

                relay_thread.start()
                move_thread.start()
                relay_thread.join()
                move_thread.join()

                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

            elif user_input.lower() == "test35": #orizzontale, setting:Y-axis
                mode = "pose"
                velocity = 20
                iteration = 2

                position1 = [470, 75, 85, 179.99, -85, 0.01]
                position2 = [470, -15, 85, 179.99, -85, 0.01]

                position3 = [470, 75, 55, 179.99, -85, 0.01]
                position4 = [470, -15, 55, 179.99, -85, 0.01]

                position5 = [470, 75, 25, 179.99, -85, 0.01]
                position6 = [470, -15, 25, 179.99, -85, 0.01]
                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_test34, args=(robot, mode, velocity, iteration, position1, position2, position3, position4, position5, position6))

                relay_thread.start()
                move_thread.start()
                relay_thread.join()
                move_thread.join()

                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

            elif user_input.lower() == "test6":
                mode = "pose"
                velocity = 20
                iteration = 5

                position1 = [380, 15, 185, 179.99, -85, 0.01]
                position2 = [380, -165, 185, 179.99, -85, 0.01]

                position3 = [380, 15, 155, 179.99, -85, 0.01]
                position4 = [380, -165, 155, 179.99, -85, 0.01]

                position5 = [380, 15, 125, 179.99, -85, 0.01]
                position6 = [380, -165, 125, 179.99, -85, 0.01]

                position7 = [380, 15, 95, 179.99, -85, 0.01]
                position8 = [380, -165, 95, 179.99, -85, 0.01]

                position9 = [380, 15, 65, 179.99, -85, 0.01]
                position10 = [380, -165, 65, 179.99, -85, 0.01]

                position11 = [380, 15, 35, 179.99, -85, 0.01]
                position12 = [380, -165, 35, 179.99, -85, 0.01]

                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_test6, args=(robot, mode, velocity, iteration, position1, position2, position3, position4, position5, position6, position7, position8, position9, position10, position11, position12))

                relay_thread.start()
                move_thread.start()
                relay_thread.join()
                move_thread.join()

                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

            elif user_input.lower() == "test4":
                mode = "pose"
                velocity = 20

                position1 = [380, 135, 200, 179.9, -85, 0]
                position2 = [380, 105, 200, 179.9, -85, 0]
                position3 = [380, 75, 200, 179.9, -85, 0]
                position4 = [380, 45, 200, 179.9, -85, 0]
                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_test4, args=(robot, mode, velocity, position1, position2, position3, position4))

                relay_thread.start()
                move_thread.start()
                relay_thread.join()
                move_thread.join()

                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

            elif user_input.lower() == "test5":
                mode = "pose"
                velocity = 20

                position1 = [380, 150, 10, 179.9, -85, 0]
                position2 = [380, 120, 50, 179.9, -85, 0]
                position3 = [380, 90, 50, 179.9, -85, 0]
                position4 = [380, 60, 50, 179.9, -85, 0]
                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_test5, args=(robot, mode, velocity, position1, position2, position3, position4))

                relay_thread.start()
                move_thread.start()
                relay_thread.join()
                move_thread.join()

                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

            elif user_input.lower() == "test10": #verticale, setting:X-axis
                mode = "pose"
                velocity = 10
                iteration = 3

                position1 = [370, 40, 100, 90.01, -90.01, 90.01]
                position2 = [370, 40, 200, 90.01, -90.01, 90.01]
                position3 = [370, 40, 190, 90.01, -90.01, 90.01]
                position4 = [370, 40, 180, 90.01, -90.01, 90.01]
                position5 = [370, 40, 170, 90.01, -90.01, 90.01]
                position6 = [370, 40, 160, 90.01, -90.01, 90.01]
                position7 = [370, 40, 150, 90.01, -90.01, 90.01]
                position8 = [370, 40, 140, 90.01, -90.01, 90.01]
                position9 = [370, 40, 130, 90.01, -90.01, 90.01]
                position10 = [370, 40, 120, 90.01, -90.01, 90.01]
                position11 = [370, 40, 110, 90.01, -90.01, 90.01]
                # position1 = [350, 30, 230, 179.99, -89, 0.01]
                # position2 = [350, 30, -170, 179.99, -89, 0.01]
                robot.move("pose", vals=position1, velocity=50, acceleration=100, cnt_val=100, linear=True)

                relay_thread = threading.Thread(target=Rf.relay_task, args=(user_input.lower(),))
                move_thread = threading.Thread(target=Rf.move_test10, args=(robot, mode, velocity, iteration, position1,
                                                                            position2, position3, position4, position5,
                                                                            position6, position7, position8, position9,
                                                                            position10, position11))

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
                Rf.control_relay("off")
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
