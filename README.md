RESUMEN:
Proyecto final de la cátedra de Control y Sistemas de la Universidad Nacional de Cuyo. El objetivo es 
aplicar las habilidaddes adquiridas para realizar el modelado, análisis y simulación del control de 
posición para el accionamiento electromecánico de la primera articulación del robot Universal UR10 
figura 1, un robot industrial tipo serie de 6 grados de libertad.
El robot se modela como un sistema lineal variante en el tiempo (LTI), en donde se toma al
momento de inercia como un parámetro variable de entrada al modelo de la planta, y se utiliza
un tipo de controlador PID junto con la técnica de planificación de ganancias (en inglés: gain
scheduling) para lograr el correcto seguimiento de consignas y rechazo de perturbaciones.
Además, se incluye ruido de proceso en las variables de estado de la planta, y el modelado de
un encoder absoluto ruidoso a la salida del sistema. Y luego se utiliza el filtro de Kalman con el
objetivo de mitigar el ruido y obtener la estimación más óptima posible de la posición angular de
salida que se retroalimenta al controlador.
Finalmente el controlador PID, junto con el sensor y el filtro de Kalman, son modelados en el
tiempo discreto con representación en punto fijo.

![image](https://github.com/user-attachments/assets/0bc8e18c-ce03-4214-888e-d07b95264292)


[PROYECTO en Latex.pdf](https://github.com/user-attachments/files/18777277/PROYECTO.en.Latex.pdf)
