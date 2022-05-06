# robot_warehouse
En este proyecto se trabajará en programar y simular un robot diferencial el cual se busca sea una representación de un robot real que seria utilizado dentro de un pequeño almacén, el robot busca volver más eficientes los procesos, incrementando así la producción y reduciendo los tiempos permitiendo también que los trabajadores ya no sean asignados a tareas repetitivas o peligrosas y puedan ser aprovechados en tareas con mayor relevancia y que no pongan en riesgo su salud física.
# requerimientos
Para poder realizar este proyecto fue necesario:
-    Contar con un sistema con Ubuntu 20.04
-    Contar con el programa de ROS noetic http://wiki.ros.org/noetic/Installation/Ubuntu
-    Contar con el programa de SolidWorks
-    Contar con el programa de Gazebo https://classic.gazebosim.org/tutorials?tut=install_ubuntu
-    Contar con el programa de RVIZ https://howtoinstall.co/es/rviz
# funcionamiento
## solidworks
Para el diseño de este robot fue necesario dividirlo en varias partes, la primera fue la base ya que con esta pieza tomamos medidas para poder tener el resto de las piezas a esa escala, luego diseñamos una tapa para la base ya que era necesario dejar un espacio para todo el circuito y las tarjetas que nuestro robot necesita para su funcionamiento, luego diseñamos una cinta transportadora para lanzar las cajas y por último las ruedas y la rueda loca.
## URDF
Para las partes en URDF se simplificó el cuerpo para un rectángulo de 80mx60cm, ponemos las inercias y los parámetros de colisión a la descripción del urdf,Las ruedas son solo cilindros unidos a la carrocería con una junta continua, para el sensor se utilizó una cámara, que consiste en una pequeña caja unida a la carrocería, para la función del sensor pondremos un código en la sección de simulación, para el transportador se utilizó una articulación prismática, que tiene el cuerpo de un rectángulo.
## Gazebo
Después de lanzar el robot en gazebo, podemos interactuar con las articulaciones y empezar a moverse alrededor del robot en el mundo, también podríamos poner cajas de cartón de los objetos de gazebo que ya se descargan cuando se instala gazebo, podemos interactuar con la cámara dentro del mundo de gazebo y no solo en rviz

![image](https://user-images.githubusercontent.com/99926615/167060398-28599098-4419-448d-98b3-0cac9c0da528.png)

Para mover un robot en simulación, éste debe tener un controlador que publique las posiciones de sus articulaciones. Para el control del movimiento de las ruedas se utiliza un controlador ya existente en el plugin de gazebo, diff-controller.

![image](https://user-images.githubusercontent.com/99926615/167060448-c289ca48-72b9-445a-a461-4ae7a43d275e.png)

Con este comando se puede mover el robot en todas las direcciones que se necesiten.

![image](https://user-images.githubusercontent.com/99926615/167060491-6f9c046b-2091-49e9-bbf7-c60d92078b71.png)

Utilizando el comando se está controlando la velocidad en todas las direcciones con las teclas de nuestro teclado, esto facilita el control del robot, y hace más fácil la simulación.

Controlador de articulación:
Para la articulación del transportador, se necesita un controlador, usamos ros control para esta instancia, necesitamos crear un archivo roslaunch en el cual lanzamos el controlador de la articulación, el tipo que usamos fue transmisión de esfuerzo. Para ello primero declaramos la transmisión de la articulación que queremos mover.

![image](https://user-images.githubusercontent.com/99926615/167060576-e7dc2b6d-17f8-46c5-9c09-011a22e82b81.png)
![image](https://user-images.githubusercontent.com/99926615/167060624-35844cd7-6671-4a08-af7f-8ac843b3e5e3.png)

En este archivo. yaml se ponen todos los parámetros del controlador, como la tasa de publicación, el nombre del publicador que vamos a publicar o suscribir y qué articulación queremos controlar, también las ganancias del control PID, ponemos 1 en la ganancia porque no queremos cambiar el código y afectar a la simulación
Para la última parte se necesita lanzar el controlador, porque si no lo hacemos el control no será capaz de publicar datos en el nodo.

![image](https://user-images.githubusercontent.com/99926615/167060681-5402fa7c-3f96-4546-b5c8-3c8ffbb16627.png)

Este comando va dentro del archivo de lanzamiento que se puede ver en el apéndice de archivos, Este comando toma los parámetros del archivo. yaml y deja que el controlador se lance con todos los archivos que necesitamos lanzar al mismo tiempo, con el comando roslaunch.

![image](https://user-images.githubusercontent.com/99926615/167060718-0d9a625c-8dcd-401d-acae-1b3ca9b76412.png)

## RVIZ
Para simular el robot en rviz, en nuestro archivo de lanzamiento tenemos que haber especificado que necesitamos mostrar el robot en rviz, este programa nos permite mostrar el modelo del robot, los sensores, y también puede suscribirse a un nodo y hacer un gui para controlar las articulaciones. Esta es la forma más fácil de simular en gazebo, pero también podríamos hacer archivos que hagan un gui para controlar las articulaciones, como este ejemplo.

![image](https://user-images.githubusercontent.com/99926615/167060163-c727975f-6585-49c6-87c2-aa6de117e58e.png)

Para la cámara necesitamos hacer un controlador para obtener la imagen que la cámara está publicando, este controlador va dentro del archivo de descripción del gazebo, y el controlador se lanza en el archivo de lanzamiento

![image](https://user-images.githubusercontent.com/99926615/167060336-01d05558-d385-4dc1-9586-704406fa1ecc.png)

# Resultados
Las siguientes imágenes muestran lo obtenido en Gazeboo: 

![image](https://user-images.githubusercontent.com/99926615/167060827-6a5c9ceb-ace2-41ef-9e0f-c9c0b6ffc56e.png)
![image](https://user-images.githubusercontent.com/99926615/167060844-dd79d724-b3ba-448c-8877-942fa5ccd07e.png)

