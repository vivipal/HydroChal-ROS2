#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Int32
import pigpio


def servo_callback(msg, pi, pin):
    # Contrôler le servo en fonction de la valeur du message reçu
    cmd = msg.data
    print("cmd : ",cmd)
    pi.set_servo_pulsewidth(pin, cmd)
    print("read : ",pi.read(pin))

def main():
    rclpy.init()
    node = rclpy.create_node('rudder_controller')

    # Configuration du GPIO 18 en mode de sortie PWM
    NUM_GPIO = 18
    MIN_WIDTH=1000
    MAX_WIDTH=2000

    pi = pigpio.pi()

    # Créer un abonné pour le topic "/cmd_safran"
    subscription = node.create_subscription(Int32, '/cmd_safran', lambda msg: servo_callback(msg, pi, NUM_GPIO), 10)

    try:
        print("yoy1")
        rclpy.spin(node)
        print("yoy2")
    except KeyboardInterrupt:
        pass

    # Arrêter le PWM et nettoyer
    pi.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
