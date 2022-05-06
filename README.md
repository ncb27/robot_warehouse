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

## RVIZ

# instrucciones
