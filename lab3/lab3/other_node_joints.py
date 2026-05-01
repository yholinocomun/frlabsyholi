#!/usr/bin/env python3

# rclpy es la librería principal de ROS 2 para Python.
import rclpy

# JointState es el tipo de mensaje usado para publicar posiciones articulares.
from sensor_msgs.msg import JointState


def main(args=None):
    """
    Nodo other_node_joints.

     Publica mensajes sensor_msgs/msg/JointState en el tópico /joint_states.

    Se usa con el robot móvil mobile.urdf, no con robot.urdf.
    """

    # Inicializa ROS 2 en Python.
    rclpy.init(args=args)

    # Crea el nodo con el nombre pedido other_node_joints
    node = rclpy.create_node('other_node_joints')

    # Crea un publicador en el tópico /joint_states.
    # robot_state_publisher escucha este tópico.
    publisher = node.create_publisher(
        JointState,
        '/joint_states',
        10
    )

    # Posiciones elegidas para las ruedas.
    # Como los joints son continuous, estos valores están dentro de un rango válido.
    # Están en radianes.
    right_wheel_position = 1.57
    left_wheel_position = -1.57

    def timer_callback():
        """
        Esta función se ejecuta periódicamente.
        Publica una posición articular fija para las dos ruedas del robot móvil.
        """

        # Creamos un mensaje JointState vacío.
        msg = JointState()

        # Agregamos el tiempo actual.
        # robot_state_publisher necesita timestamps correctos para procesar los estados.
        msg.header.stamp = node.get_clock().now().to_msg()

        # Nombres EXACTOS de las articulaciones en mobile.urdf.
        # Si estos nombres no coinciden con el URDF, el robot no se moverá.
        msg.name = [
            'base_link_to_right_wheel_link_joint',
            'base_link_to_left_wheel_link_joint'
        ]

        # Posiciones articulares correspondientes a cada nombre.
        # El orden debe coincidir con msg.name.
        msg.position = [
            right_wheel_position,
            left_wheel_position
        ]

        # Publicamos el mensaje en /joint_states.
        publisher.publish(msg)

        # Mensaje informativo en terminal.
        node.get_logger().info(
            f'Publicado /joint_states: right={right_wheel_position:.2f}, left={left_wheel_position:.2f}'
        )

    # Ejecuta el callback cada 0.5 segundos.
    node.create_timer(0.5, timer_callback)

    # Mantiene vivo el nodo.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()