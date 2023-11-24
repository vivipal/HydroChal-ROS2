#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO

global pwm

def servo_callback(msg):
    # Contrôler le servo en fonction de la valeur du message reçu
    servo_value = msg.data

    # Exemple de logique pour contrôler le servo en fonction de la valeur
    # Assurez-vous d'adapter cette logique en fonction de vos besoins
    # Vous devrez peut-être ajuster les plages de valeurs en fonction de votre servo.
    if servo_value < 0:
        servo_value = 0
    elif servo_value > 100:
        servo_value = 100

    # Conversion de la valeur de commande en une valeur de PWM
    duty_cycle = 2.5 + (12.5 * servo_value / 100)
    pwm.start(duty_cycle)

def main():
    rclpy.init()
    node = rclpy.create_node('servo_controller')

    # Configuration du GPIO 18 en mode de sortie PWM
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)
    pwm = GPIO.PWM(18, 50)  # Fréquence de 50 Hz

    # Créer un abonné pour le topic "/cmd_safran"
    subscription = node.create_subscription(Int32, '/cmd_safran', servo_callback, 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Arrêter le PWM et nettoyer
    pwm.stop()
    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
