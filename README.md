# Laboratorio-4---Cinem-tica-Directa

Por Juan David Alonso, Julián Pinzón y Rodrigo Vera

## Descripcion General

El objetivo principal de esta practica se puede resumir al rededor de comprender y resolver el problema de la cinematica directa, buscando determinar y controlar la posición y orientación de la herramienta de un manipulador robótico en función de sus variables articulares. Para esto se debe implementar el modelo cinematico directo en un manipulador fisico con dimnensiones y variables finitas como lo es el Phantom X y a través del entorno de Robot Operating System (ROS), matlab y linux calcular los modelos geometricos y tener modelos digitales que permitan la comparación de los resultados reales experimentales con aquellos simulados.

### Solución planteada

#### Parametros de DH
Lo primero para comenzar a dar solución a la practica de laboratorio fue centrarse en el desarrollo del espacio de las articulaciones del Robot, para ello se realizó un diagrama de cinematica directa dando como resultado:
![Screenshot of a comment on a GitHub issue showing an image, added in the Markdown, of an Octocat smiling and raising a tentacle.](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/DH.jpg)

Desde donde se desarrollo la siguiente tabla resumen que tambien incluye todos los angulos y desfaces dentre los sistemas coordenados teniendo en cuanta que se midieron en radianes para hacer un mejor manejo en Matlab:
![Screenshot of a comment on a GitHub issue showing an image, added in the Markdown, of an Octocat smiling and raising a tentacle.](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/ParametrosDH.JPG)

#### Codigo de solución

##### Matlab
En base a estos insumos, el primer codigo a desarrollar fue el de matlab donde se calculo las matrices de transformación homogénea para cada una de las articulaciones del robot haciendo uso de la siguiente función desarrollada

```matlab
function MTH = DhtoMat(theta,d,a,alpha)
theta = rad2deg(theta);
alpha = rad2deg(alpha);
f1 = [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta)];
f2 = [sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta)];
f3 = [0 sind(alpha) cosd(alpha) d];
f4 = [0 0 0 1];
MTH = [f1;f2;f3;f4];
end
```
Una vez calculadas cada una de las transformaciones sucesivas de la articulación 0 hasta la número 4 de manera consecutiva, fue posible realizar el calculo del mdoelo cinematico directo que dio como resultado la siguiente matriz:


![Screenshot of a comment on a GitHub issue showing an image, added in the Markdown, of an Octocat smiling and raising a tentacle.](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/MatrizCD.JPG)

```latex
\mathrm{}\\
\mathrm{where}\\
\mathrm{}\\
\;\;\sigma_1 =\sigma_7 \,\sigma_{10} \,\sin \left(\Theta_1 \right)-\sin \left(\Theta_1 \right)\,\sigma_8 \,\sigma_9 \\
\mathrm{}\\
\;\;\sigma_2 =\sigma_7 \,\sin \left(\Theta_1 \right)\,\sigma_9 +\sigma_{10} \,\sin \left(\Theta_1 \right)\,\sigma_8 \\
\mathrm{}\\
\;\;\sigma_3 =\cos \left(\Theta_1 \right)\,\sigma_7 \,\sigma_{10} -\cos \left(\Theta_1 \right)\,\sigma_8 \,\sigma_9 \\
\mathrm{}\\
\;\;\sigma_4 =\cos \left(\Theta_1 \right)\,\sigma_7 \,\sigma_9 +\cos \left(\Theta_1 \right)\,\sigma_{10} \,\sigma_8 \\
\mathrm{}\\
\;\;\sigma_5 =\sigma_8 \,\sigma_9 -\sigma_7 \,\sigma_{10} \\
\mathrm{}\\
\;\;\sigma_6 =\sigma_7 \,\sigma_9 +\sigma_{10} \,\sigma_8 \\
\mathrm{}\\
\;\;\sigma_7 =\cos \left(\Theta_2 +\frac{\pi }{2}\right)\\
\mathrm{}\\
\;\;\sigma_8 =\sin \left(\Theta_2 +\frac{\pi }{2}\right)\\
\mathrm{}\\
\;\;\sigma_9 =\sin \left(\Theta_3 +\frac{\pi }{2}\right)\\
\mathrm{}\\
\;\;\sigma_{10} =\cos \left(\Theta_3 +\frac{\pi }{2}\right)
\end{array}$
```

Una vez teniendo esto, se altera esta matriz que representa la cinematica directo con 5 posiciones objetivo R1,R2,R3,R4 y R5 que representan cada uno de las posiciones angulares de los motores exigidos en el laboratorio para acontinuación hacer uso del modelo seriallink y con esto poder visualizar cada uno de las graficas digitales expuestas en la comparación final de este laboratorio

##### Python
A continuación se desarrollo el codigo en Python con el cual se buscaba controlar los motores del Robot a su vez que se creaba una interfaz que permitiese en tiempo real, variar entre las cinco posiciones predeterminadas mientras que se visualizaba los angulos esperados para cada motor. en primer lugar para el codigo en Python se importaron varias librerias o modulos principales:

   - `numpy`: Para operaciones matemáticas.
   - `rospy`: Para interactuar con ROS (Robot Operating System) que es el sistema operativo de código abierto para robots.
   - `time`: Para manejar el tiempo.
   - `tkinter`: Para crear una interfaz gráfica de usuario (GUI).
   - `std_msgs.msg` y `sensor_msgs.msg`: Para mensajes estándar de ROS.
   - `DynamixelCommand` de `dynamixel_workbench_msgs.srv`: Para enviar comandos a los motores Dynamixel.

Para acontinuación realizar la configuración del robot, para esto se definieron los valores angulares que tendria cada arreglo de posiciones tanto en grados como en forma analogica para poder ser procesada y enviada a los motores así como los torques maximos que estos podrían controlar:

```Python
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
```

El movimiento de estos robots se realiza de manera progresiva y realizando una pequeña pausa entre cada movimiento, para controlarlo se creo la función "Mostrar menú" donde se quería que con una lista desplegable se pudiera hacer el cambio de cada uno de los cinco casos, mientras que se mostraba el nombre de la universidad, la asignatura y los autores del codigo así como la posicion, en grados para cada uno de los cinco motores, esto si, de manera continua sin detener el movimiento de los motores en ningún momento. 

```Python
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
```

### Interfaz HM
Finalmente, con el codigo anterior se tiene como resultado la siguiente interfaz, que permite variar, en tiempo real entre las cinco posicines solicitadas mientras a su vez muestra el arreglo de angulos en los que se encuentran los 5 motores.
![Screenshot of a comment on a GitHub issue showing an image, added in the Markdown, of an Octocat smiling and raising a tentacle.](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/Interfaz.jpeg)

### Comparación de Poses con gráficas digitales

| Poses | Gráficas digitales |
|---|---|
| ![Pose número 1 o Home](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/Pose1.JPG) | ![Gráficas digitales](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/RealPose1.jpg) |
| ![Pose número 2](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/Pose2.JPG) | ![Gráficas digitales](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/RealPose2.jpg) |
| ![Pose número 3](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/Pose3.JPG) | ![Gráficas digitales](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/RealPose3.jpg) |
| ![Pose número 4](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/Pose4.JPG) | ![Gráficas digitales](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/RealPose4.jpg) |
| ![Pose número 5](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/Pose5.JPG) | ![Gráficas digitales](https://github.com/JuanAAlonso/Laboratorio-4---Cinem-tica-Directa/blob/main/Imagenes/RealPose5.jpg) |

### Video de funcionamiento

>https://drive.google.com/file/d/1nCXHmTs9sp2uDxF6b5eKDhNIj0-YeXRV/view?usp=sharing
