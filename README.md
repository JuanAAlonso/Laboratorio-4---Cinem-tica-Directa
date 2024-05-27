# Laboratorio-4---Cinem-tica-Directa

Por Juan David Alonso, Julián Pinzón y Rodrigo Vera

## Descripcion General

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

```latex
$\begin{array}{l}
\left(\begin{array}{cccc}
\cos \left(\Theta_4 \right)\,\sigma_3 -\sin \left(\Theta_4 \right)\,\sigma_4  & \sin \left(\Theta_1 \right) & \cos \left(\Theta_4 \right)\,\sigma_4 +\sin \left(\Theta_4 \right)\,\sigma_3  & \frac{21\,\cos \left(\Theta_1 \right)\,\sigma_7 }{2}+\frac{13\,\cos \left(\Theta_4 \right)\,\sigma_3 }{2}-\frac{13\,\sin \left(\Theta_4 \right)\,\sigma_4 }{2}+\frac{21\,\cos \left(\Theta_1 \right)\,\sigma_7 \,\sigma_{10} }{2}-\frac{21\,\cos \left(\Theta_1 \right)\,\sigma_8 \,\sigma_9 }{2}\\
\cos \left(\Theta_4 \right)\,\sigma_1 -\sin \left(\Theta_4 \right)\,\sigma_2  & -\cos \left(\Theta_1 \right) & \cos \left(\Theta_4 \right)\,\sigma_2 +\sin \left(\Theta_4 \right)\,\sigma_1  & \frac{13\,\cos \left(\Theta_4 \right)\,\sigma_1 }{2}-\frac{13\,\sin \left(\Theta_4 \right)\,\sigma_2 }{2}+\frac{21\,\sigma_7 \,\sin \left(\Theta_1 \right)}{2}+\frac{21\,\sigma_7 \,\sigma_{10} \,\sin \left(\Theta_1 \right)}{2}-\frac{21\,\sin \left(\Theta_1 \right)\,\sigma_8 \,\sigma_9 }{2}\\
\cos \left(\Theta_4 \right)\,\sigma_6 -\sin \left(\Theta_4 \right)\,\sigma_5  & 0 & \cos \left(\Theta_4 \right)\,\sigma_5 +\sin \left(\Theta_4 \right)\,\sigma_6  & \frac{21\,\sigma_8 }{2}+\frac{13\,\cos \left(\Theta_4 \right)\,\sigma_6 }{2}-\frac{13\,\sin \left(\Theta_4 \right)\,\sigma_5 }{2}+\frac{21\,\sigma_7 \,\sigma_9 }{2}+\frac{21\,\sigma_{10} \,\sigma_8 }{2}+\frac{17}{2}\\
0 & 0 & 0 & 1
\end{array}\right)\\
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
