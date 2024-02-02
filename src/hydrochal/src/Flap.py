#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float32
import serial
import sys

def flap_callback(msg, serial_port):
    # Contrôler le flap en fonction de la valeur du message reçu
    # print("cmd_flap received: ", msg.data)
    cmd_flap = int(1500 - 500*(msg.data/30.0))
    cmd_flap = str(max(1000, min(2000, cmd_flap)))+ "\n"
    serial_port.write(cmd_flap.encode('utf-8'))

def main():
    rclpy.init()
    node = rclpy.create_node('flap_controller')
    if len(sys.argv) != 2:
        port = '/dev/ttyUSB0'
        print("No port specified, using default port: /dev/ttyUSB0")
    else:
        port = sys.argv[1]
        print("Using port: ", port)

    # Configurer la communication série
    serial_port = serial.Serial(port, baudrate=57600, timeout=1)

    # Créer un abonné pour le topic "/cmd_flap"
    subscription = node.create_subscription(Float32, '/cmd_flap', lambda msg: flap_callback(msg, serial_port), 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Fermer la communication série et nettoyer
    serial_port.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
