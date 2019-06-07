# Tutorial de OpenCV en ROS

## Descripción general
Tutorial de OpenCV escrito en C++ dentro de ROS con la intensión de facilitar el trabajo a los nuevos programadores.

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

Se habrán dado de alta los siguientes nodos:

| <node_name> | Descripción | file.cpp |
| --- | --- | --- |
| showGray | Lee un archivo jpg y lo pasa a grises | toGray.cpp | 
| showSegmentation | Segmenta la imagen de la webcam en HSV | simpleColorSegmentation.cpp |
| showEdges | Busca bordes de la imagen de la webcam| detectEdges.cpp |
| showDrawings | Dibuja geometrías sobre una imagen| drawOnImage.cpp |


## Ejemplo de ejecución

En dos Terminales diferentes ejecutar cada una de las siguientes lineas:
```
$ roscore
$ rosrun opencv_tutorial <node_name>
```


## Autores y colaboradores
Este paquete fue desarrollado a partir de programas de OpenCV disponibles en Internet, pero ajustados ligeramente por el Dr. Alejandro Aceves-López para que sean más comprensibles a los programadores nuevos de ROS.

## Referencias

1. Anónimo, "OpenCV Tutorials ", [Online]. Available: https://docs.opencv.org/3.2.0/d9/df8/tutorial_root.html, [Accessed: 05-Jun-2018].
2. Fernando Shermal, "OpenCV Tutorial C++ ", [Online]. Available: https://www.opencv-srf.com/p/introduction.html, [Accessed: 05-Jun-2018].
3. Sergio Cantu, “Loading images – OpenCV 3.4 with python 3 Tutorial 1-37”, [Online]. Available: https://pysource.com/2018/01/20/loading-images-opencv-3-4-with-python-3-tutorial-1/#, [Accessed: 05-Jun-2019].
4. Sergio Cantu, “Computer Vision Made Simple”, [Online]. Available: https://pysource.com/, [Accessed: 05-Jun-2019].
5. OpenCV Team, “OpenCV - Open Source Computer Vision Library”, [Online]. Available: https://opencv.org/, [Accessed: 05-Jun-2018].
