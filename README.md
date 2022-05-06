# robot_warehouse
En este proyecto se trabajará en programar y simular un robot diferencial el cual se busca sea una representación de un robot real que seria utilizado dentro de un pequeño almacén, el robot busca volver más eficientes los procesos, incrementando así la producción y reduciendo los tiempos permitiendo también que los trabajadores ya no sean asignados a tareas repetitivas o peligrosas y puedan ser aprovechados en tareas con mayor relevancia y que no pongan en riesgo su salud física.
# requerimientos
Para poder realizar este proyecto fue necesario:
-    Contar con un sistema con Ubuntu 20.04
-    Contar con el programa de ROS noetic
-    Contar con el programa de SolidWorks
-    Contar con el programa de Gazebo
-    Contar con el programa de RVIZ
# funcionamiento
## solidworks

## URDF
para las partes en URDF se simplificó el cuerpo para un rectángulo de 80mx60cm, ponemos las inercias y los parámetros de colisión a la descripción del urdf,Las ruedas son solo cilindros unidos a la carrocería con una junta continua, para el sensor se utilizó una cámara, que consiste en una pequeña caja unida a la carrocería, para la función del sensor pondremos un código en la sección de simulación, para el transportador se utilizó una articulación prismática, que tiene el cuerpo de un rectángulo.
## RVIZ
