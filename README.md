# Tutorial de OpenCV en ROS

## Descripción general
Tutorial clásico de OpenCV en C++ en ROS con la intensión de facilitar el trabajo a los nuevos programadores.

## Pre-requisitos
Se considera que la computadora del usuario ya tiene correctamente instalado ROS, GIT y que ya tiene la carpeta de `catkin_ws` correctamente inicializada.
Para asegurarse que se tiene un dispositivo de video, ejecutar en una Terminal:
```
$ cheese
```
Deberá de ver la imagen de la cámara. En caso contrario significa que no tiene una cámara de video correctamente instalada. 

Para asegurarse que se tiene correctamente instalado OpenCV, ejecutar la siguiente instrucción en una Terminal:
```
$ pkg-config –-modversion opencv
```
Debe responder con la versión de OpenCV. En caso que no tenga instalado OpenCV, ejecutar:
```
sudo apt-get install ros-kinetic-vision-opencv libopencv-dev python-opencv
```

## Proceso de instalación
En una Terminal ejecutar las siguientes instrucciones:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/aaceves/opencv_tutorial.git
$ cd ~/catkin_ws
$ catkin build
$ source devel/setup.bash
```
El proceso de compilación debe terminar sin errores. 

Se habrán dado de alta los siguientes nodos

| node name | Descripción | file.cpp |
| --- | --- | --- |
| showimage | Lee un archivo jpg y lo pasa a grises | toGray.cpp | 
| shoeimage1 | Segmenta la imagen de la webcam en HSV | simpleColorSegmentation.cpp |


## Ejemplo de ejecución

En dos Terminales diferentes ejecutar cada una de las siguientes lineas:
```
$ roscore
$ rosrun opencv_tutorial <nodo>
```


## Autores y colaboradores
Este paquete está originalmente presentado en el wiki de ROS en la sección de Beginners Level. Sin embargo fue ajustado ligeramente por el Dr. Alejandro Aceves-López para que sea más comprensible a los programadores nuevos de ROS.

## Referencias
1. Sergio Cantu, “OpenCV with Python 3: Tutorial 1-34”, [Online]. Available: https://pysource.com/author/admin, [Accessed: 05-Jun-2018].
2. Anónimo, "OpenCV Tutorials ", [Online]. Available: https://docs.opencv.org/3.2.0/d9/df8/tutorial_root.html, [Accessed: 05-Jun-2018].
3. Fernando Shermal, "OpenCV Tutorial C++ ", [Online]. Available: https://www.opencv-srf.com/p/introduction.html, [Accessed: 05-Jun-2018].
4. OpenCV Team, “OpenCV - Open Source Computer Vision Library”, [Online]. Available: https://opencv.org/, [Accessed: 05-Jun-2018].
5. Alexander Mordvintsev and Abid Rahman K, “OpenCV-Python Tutorials”, [Online]. Available: https://docs.opencv.org/3.1.0/d6/d00/tutorial_py_root.html, [Accessed: 18-Dic-2018].
