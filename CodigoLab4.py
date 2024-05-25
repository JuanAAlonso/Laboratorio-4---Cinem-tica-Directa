from cmath import pi
import numpy as np
import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand
import tkinter as tk
from tkinter import ttk

# Autor: Juan David Alonso, Julian Pinzón, Rodrigo Vera

# Máximos torques para cada motor
MAX_TORQUES = [500, 400, 350, 350, 350]

# Posiciones de referencia en grados para cada caso
REF_POSITIONS_DEG = [
    [0, 0, 0, 0, 0],
    [25, 25, 20, -20, 0],
    [-35, 35, -30, 30, 0],
    [85, -20, 55, 25, 0],
    [80, -35, 55, -45, 0]
]

# Posiciones análogas desde dynamixel_wizard
REF_POSITIONS_ANALOG = [
    [514, 510, 818, 512, 512],
    [597, 597, 888, 444, 512],
    [393, 630, 716, 616, 512],
    [802, 445, 1000, 599, 512],
    [786, 395, 1000, 360, 546]
]

# Variable global para almacenar posiciones actuales
posiciones_actuales = [0, 0, 0, 0, 0]

# Función para enviar comandos a los motores Dynamixel
def enviar_comando_motor(comando, id_num, nombre_direccion, valor, retraso):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:
        comando_dynamixel = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        resultado = comando_dynamixel(comando, id_num, nombre_direccion, valor)
        rospy.sleep(retraso)
        return resultado.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

# Callback para actualizar las posiciones actuales de las articulaciones
def actualizar_posiciones(data):
    global posiciones_actuales
    posiciones_actuales = np.degrees(data.position)
    posiciones_actuales[2] -= 90

# Imprimir posiciones actuales y teóricas de las articulaciones
def imprimir_posiciones(real, teorica):
    print('\nÁngulos motores:\n')
    for i, pos in enumerate(real):
        print(f'{i + 1}: {pos:.2f}°\t', end=' ')
    error_rms = np.sqrt(np.mean((np.array(teorica) - np.array(real)) ** 2))
    print(f'\n\nError RMS: {error_rms:.2f}°\n')

# Mover gradualmente las articulaciones hacia una posición objetivo
def mover_parcialmente(indice_articulacion, posicion_objetivo, posicion_actual):
    N = 1
    delta = (posicion_objetivo - posicion_actual) / N
    for i in range(N):
        objetivo_actual = int(posicion_actual + delta * (i + 1))
        print(f'Moviendo articulación {indice_articulacion + 1} a posición {objetivo_actual}')
        enviar_comando_motor('', indice_articulacion + 1, 'Goal_Position', objetivo_actual, 0.5)

# Función para ir a la posición de home
def ir_a_home():
    for i, pos in enumerate(REF_POSITIONS_ANALOG[0]):
        print(f'Movimiento del eslabón {i + 1} a posición {pos}\n')
        enviar_comando_motor('', i + 1, 'Goal_Position', pos, 1)
        time.sleep(0.5)

# Función para ejecutar el caso seleccionado
def ejecutar_caso(caso_seleccionado):
    if caso_seleccionado == "Home":
        ir_a_home()
        print('Movimiento a la posición de inicio completado.\n')
        imprimir_posiciones(posiciones_actuales, REF_POSITIONS_DEG[0])
    else:
        caso_seleccionado = int(caso_seleccionado)
        print(f'Iniciando rutina para el caso {caso_seleccionado}\n')
        for i, pos in enumerate(REF_POSITIONS_ANALOG[caso_seleccionado]):
            print(f'Movimiento del eslabón {i + 1} a posición {pos}')
            mover_parcialmente(i, pos, REF_POSITIONS_ANALOG[0][i])
        print('Rutina finalizada.')
        imprimir_posiciones(posiciones_actuales, REF_POSITIONS_DEG[caso_seleccionado])

# Función para mostrar el menú y seleccionar un caso
def mostrar_menu():
    root = tk.Tk()
    root.title("Seleccionar caso")
    root.attributes('-fullscreen', True)  # Poner en pantalla completa

    # Añadir títulos
    title1 = tk.Label(root, text="Universidad Nacional de Colombia", font=("Helvetica", 24, "bold"))
    title1.pack(pady=10)
    title2 = tk.Label(root, text="Laboratorio 4 Robotica", font=("Helvetica", 20))
    title2.pack(pady=5)
    title3 = tk.Label(root, text="Por Juan David, Julian y Rodrigo", font=("Helvetica", 20))
    title3.pack(pady=5)

    tk.Label(root, text="Seleccione el caso a ejecutar:", font=("Helvetica", 18)).pack(pady=20)
    
    caso_var = tk.StringVar()
    caso_combobox = ttk.Combobox(root, textvariable=caso_var, font=("Helvetica", 16))
    caso_combobox['values'] = ['Home', '1', '2', '3', '4']
    caso_combobox.pack(pady=10)

    # Label para mostrar la referencia de posición correspondiente
    pos_ref_label = tk.Label(root, text="", font=("Helvetica", 16))
    pos_ref_label.pack(pady=20)
    
    def on_select(event):
        caso_seleccionado = caso_var.get()
        if caso_seleccionado == "Home":
            pos_ref_label.config(text="Posiciones de referencia en grados para la posición de inicio: ")
        else:
            caso_seleccionado = int(caso_seleccionado)
            pos_ref_label.config(text=f"Posiciones de referencia en grados para el caso {caso_seleccionado}: {REF_POSITIONS_DEG[caso_seleccionado]}")
        ejecutar_caso(caso_seleccionado)
    
    caso_combobox.bind("<<ComboboxSelected>>", on_select)

    # Botón de salida
    salir_button = tk.Button(root, text="Salir", font=("Helvetica", 16), command=root.quit)
    salir_button.pack(pady=20)

    root.mainloop()

# Inicializar el nodo de ROS
rospy.init_node('joint_listener', anonymous=True)
rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, actualizar_posiciones)

# Configurar los límites de torque de los motores
for i, torque in enumerate(MAX_TORQUES):
    enviar_comando_motor('', i + 1, 'Torque_Limit', torque, 0)

mostrar_menu()