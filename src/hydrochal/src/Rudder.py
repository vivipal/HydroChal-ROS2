#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float32
import pigpio


def servo_callback(msg, pi, pin):
    # Contrôler le servo en fonction de la valeur du message reçu
    global last_angle_sent  # en deg
    angle_safran = max(-55, min(55, msg.data))
    print("cmd received : ", msg.data)
    cmd = int(1497 +8.17*angle_safran -0.0187*angle_safran**2)
    cmd = max(1000, min(2000, cmd))
    if (abs(angle_safran-last_angle_sent)>2.5):     # minimum de mouvement de 5deg
        print("cmd sent : ",cmd)
        pi.set_servo_pulsewidth(pin, cmd)
        last_angle_sent = angle_safran

# def servo_callback(msg, pi, pin):
#     # Contrôler le servo en fonction de la valeur du message reçu
#     cmd = max(1000, min(2000, msg.data))
#     print("cmd sent : ",cmd)
#     pi.set_servo_pulsewidth(pin, cmd)

def main():
    rclpy.init()
    node = rclpy.create_node('rudder_controller')

    # Configuration du GPIO 18 en mode de sortie PWM
    NUM_GPIO = 18
    MIN_WIDTH=1000
    MAX_WIDTH=2000

    pi = pigpio.pi()
    global last_angle_sent # en deg
    pi.set_servo_pulsewidth(NUM_GPIO, 0)
    last_angle_sent = 0

    # Créer un abonné pour le topic "/cmd_safran" en deg
    subscription = node.create_subscription(Float32, '/cmd_safran', lambda msg: servo_callback(msg, pi, NUM_GPIO), 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Arrêter le PWM et nettoyer
    pi.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
