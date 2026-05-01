#!/usr/bin/env python3

# Importamos rclpy, que es la librería principal de ROS 2 para Python.
# Permite inicializar ROS, crear nodos, publicadores, timers, etc.
import rclpy

# Importamos random para generar el ruido aleatorio del sensor.
import random

# Importamos el tipo de mensaje que pide la actividad:
# sensor_msgs/msg/RelativeHumidity
from sensor_msgs.msg import RelativeHumidity


def main(args=None):
    """
    Función principal del nodo.
    En esta versión NO usamos clases porque la condición del profesor
    es programar el nodo de forma funcional.
    """

    # Inicializa la comunicación con ROS 2.
    # Antes de esta línea, el programa es solo Python normal.
    # Después de esta línea, ya se pueden crear nodos ROS 2.
    rclpy.init(args=args)

    # Creamos un nodo llamado 'pub_sensor'.
    # Este nombre debe coincidir con lo pedido en el enunciado.
    node = rclpy.create_node('pub_sensor')

    # Creamos un publicador.
    # Publicará mensajes RelativeHumidity en el tópico /humi.
    # El número 10 es el tamaño de la cola de mensajes.
    publisher = node.create_publisher(
        RelativeHumidity,
        'humi',
        10
    )

    # Valor base de humedad relativa pedido por la actividad.
    # El mensaje RelativeHumidity usa valores entre 0.0 y 1.0.
    # Entonces 0.8 representa 80% de humedad relativa.
    base_humidity = 0.8
    """
    Esta función se ejecuta periódicamente gracias al timer.
    Cada vez que se ejecuta:
    1. Crea un mensaje de humedad.
    2. Genera ruido aleatorio positivo o negativo.
    3. Suma el ruido a la humedad base.
    4. Publica el mensaje en el tópico humi.
    """


    def timer_callback():

        # Creamos un mensaje vacío del tipo RelativeHumidity.
        msg = RelativeHumidity()

        # Agregamos tiempo al mensaje.
        # Esto indica en qué instante se generó la medición.
        msg.header.stamp = node.get_clock().now().to_msg()

        # Nombre del marco de referencia del sensor.
        # Para esta actividad no es crítico, pero es buena práctica.
        msg.header.frame_id = 'humidity_sensor'

        # Decidimos aleatoriamente si el ruido será positivo o negativo.
        # random.random() genera un número entre 0.0 y 1.0.
        # Si sale menor que 0.5, usamos ruido positivo.
        # Si sale mayor o igual que 0.5, usamos ruido negativo.
        if random.random() < 0.5:
            # Ruido positivo entre 0.01 y 0.05
            noise = random.uniform(0.01, 0.05)
        else:
            # Ruido negativo entre -0.06 y -0.01
            noise = random.uniform(-0.06, -0.01)

        # Calculamos la medición final.
        # Ejemplo:
        # 0.8 + 0.03 = 0.83
        # 0.8 - 0.04 = 0.76
        humidity_value = base_humidity + noise

        # Guardamos la humedad calculada dentro del mensaje.
        msg.relative_humidity = humidity_value

        # La varianza representa incertidumbre de la medición.
        # Como la actividad no pide calcularla, la dejamos en 0.0.
        msg.variance = 0.0

        # Publicamos el mensaje en el tópico humi.
        publisher.publish(msg)

        # Mostramos en la terminal el valor publicado.
        node.get_logger().info(
            f'Humedad publicada: {humidity_value:.3f}'
        )

    # Creamos un timer que ejecuta timer_callback cada 0.5 segundos.
    # Eso equivale a publicar 2 mediciones por segundo.
    node.create_timer(0.5, timer_callback)

    # Mantiene vivo el nodo.
    # Sin esta línea, el programa terminaría inmediatamente.
    try:
        rclpy.spin(node)

    # Permite cerrar el nodo con Ctrl + C sin mostrar un error innecesario.
    except KeyboardInterrupt:
        pass

    # Limpieza final del nodo.
    finally:
        node.destroy_node()

        # Apagamos rclpy solo si todavía está activo.
        if rclpy.ok():
            rclpy.shutdown()


# Esta condición permite que el archivo se ejecute como programa principal.
# Cuando haces ros2 run lab3 pub_sensor, se llama a main().
if __name__ == '__main__':
    main()