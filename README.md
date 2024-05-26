# Laboratorio-4---Cinem-tica-Directa

Por Juan David Alonso, Julián Pinzón y Rodrigo Vera

## Descripcion General

### Solución planteada

#### Parametros de DH

#### Codigo de solución

##### Matlab

##### Python
A continuacion se explica el funcionamiento del codigo de Python Utilizado:

1. **Importaciones**: Al comienzo del código, se importan varios módulos necesarios para el funcionamiento del programa. Estos módulos incluyen:
   - `numpy`: Para operaciones matemáticas.
   - `rospy`: Para interactuar con ROS (Robot Operating System), un sistema operativo de código abierto para robots.
   - `time`: Para manejar el tiempo.
   - `tkinter`: Para crear una interfaz gráfica de usuario (GUI).
   - `std_msgs.msg` y `sensor_msgs.msg`: Para mensajes estándar de ROS.
   - `DynamixelCommand` de `dynamixel_workbench_msgs.srv`: Para enviar comandos a los motores Dynamixel.

2. **Configuración de parámetros**: A continuación, se definen algunos parámetros importantes, como los máximos torques permitidos para cada motor y las posiciones de referencia del brazo robótico en grados y en valores analógicos.

3. **Funciones de control de motores Dynamixel**: El código define varias funciones para controlar los motores Dynamixel, como enviar comandos, actualizar las posiciones actuales, mover gradualmente las articulaciones hacia una posición objetivo, ir a la posición de inicio y ejecutar un caso específico seleccionado por el usuario.

4. **Interfaz gráfica de usuario (GUI)**: Se utiliza la biblioteca `tkinter` para crear una interfaz gráfica de usuario simple que permite al usuario seleccionar un caso de movimiento del brazo robótico.

5. **Inicialización de ROS**: Se inicia un nodo de ROS para comunicarse con el sistema robótico y se suscribe a un tema que proporciona las posiciones actuales de las articulaciones.

6. **Configuración de los límites de torque de los motores Dynamixel**: Se establecen los límites de torque para los motores Dynamixel, lo que garantiza un funcionamiento seguro del brazo robótico.

7. **Llamada a la función `mostrar_menu()`**: Finalmente, se llama a la función `mostrar_menu()` para iniciar el programa y mostrar la interfaz gráfica de usuario al usuario, donde puede seleccionar y ejecutar diferentes movimientos del brazo robótico.

En resumen, este código es una herramienta completa para controlar y ejecutar movimientos predefinidos en un brazo robótico utilizando ROS y una interfaz gráfica de usuario.
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
