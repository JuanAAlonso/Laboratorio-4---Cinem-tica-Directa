# Laboratorio-4---Cinem-tica-Directa

Por Juan David Alonso, Julián Pinzón y Rodrigo Vera

## Descripcion General

### Solución planteada

#### Parametros de DH

#### Codigo de solución

##### Matlab
A continuacion se explica paso a paso l codigo de Mathlab utilizado para el laboratorio:

1. **Inicialización del entorno**: Se eliminan todas las figuras y variables existentes en el entorno de MATLAB.

2. **Definición de los valores de Denavit-Hartenberg (DH)**: Esta parte del código no está escrita y parece ser una sección del código que falta o se debe completar con los parámetros DH del manipulador.

3. **Cálculo de las matrices de transformación homogénea**: Utilizando los valores DH y las variables simbólicas para los ángulos de las articulaciones (Theta1, Theta2, Theta3, Theta4), se calculan las matrices de transformación homogénea T01, T12, T23, T34 y T04 que representan las transformaciones de cada articulación y la transformación completa del sistema.

4. **Creación de matrices de rotación para diferentes posiciones**: Se definen diferentes configuraciones (R1, R2, R3, R4, R5) que representan diferentes posiciones de las articulaciones del robot. Luego, se calculan las matrices de transformación homogénea correspondientes a estas posiciones.

5. **Cálculo de las posiciones de los extremos de las herramientas**: Se extraen las posiciones cartesianas y los ángulos de Euler de las matrices de transformación homogénea para cada configuración del robot.

6. **Definición del enlace serial**: Se define el enlace del robot utilizando la función `Link` de la biblioteca Robotics Toolbox. Cada enlace se define como una articulación de revolución con sus respectivos parámetros DH y límites articulares.

7. **Configuración del robot y visualización**: Se configura el robot utilizando la función `SerialLink` y se le asigna un nombre. Luego, se visualiza la configuración inicial del robot en una nueva figura.

8. **Visualización de la posición de inicio**: Se visualiza la posición inicial del robot y se muestra un sistema de coordenadas de referencia en la misma figura.

En resumen, este código de MATLAB define un manipulador robótico de 4 grados de libertad, calcula diferentes configuraciones para las articulaciones del robot y visualiza estas configuraciones en una figura tridimensional.
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
