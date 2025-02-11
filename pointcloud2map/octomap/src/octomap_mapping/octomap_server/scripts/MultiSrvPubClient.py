#!/usr/bin/env python

import sys
import time
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from octomap_msgs.srv import BoundingBoxQuery

import rclpy
from rclpy.node import Node


class MultiServicePublishClient(Node):

    def __init__(self):
        super().__init__('multi_service_publish_client')
        self.declare_parameter("drone_id", "UNDEFINED")
        self.drone_id = 'DT1'
        # Définitions des services serveurs existants
        service_toggle_pub = 'color_octomap_server_' + self.drone_id + '/toggle_marker_publishing'
        service_pub_std = 'color_octomap_server_' + self.drone_id + '/standard_publish_area'
        service_pub_zone = 'color_octomap_server_' + self.drone_id + '/set_publish_area'
        service_clear_zone = 'color_octomap_server_' + self.drone_id + '/clear_bbox'
        service_reset = 'color_octomap_server_' + self.drone_id + '/reset'

        # Création des clients pour chaque service
        self.cli_toggle_pub = self.create_client(SetBool, service_toggle_pub)
        self.cli_pub_std = self.create_client(Empty, service_pub_std)
        self.cli_pub_zone = self.create_client(BoundingBoxQuery, service_pub_zone)
        self.cli_clear_zone = self.create_client(BoundingBoxQuery, service_clear_zone)
        self.cli_reset_octo = self.create_client(Empty, service_reset)

        # Attendre que les services soient disponibles
        while not self.cli_toggle_pub.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service toggle_publish non disponible, attente...')
        while not self.cli_pub_std.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service publish_standard non disponible, attente...')
        while not self.cli_pub_zone.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service set_publish_srv non disponible, attente...')
        while not self.cli_clear_zone.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service clear_zone non disponible, attente...')
        while not self.cli_reset_octo.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service reset non disponible, attente...')

    # Fonction existante pour activer/désactiver le publishing
    def call_toggle_publish(self, enable):
        request = SetBool.Request()
        request.data = enable
        future = self.cli_toggle_pub.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        """ if future.result() is not None:
            self.get_logger().info('Connecté au service toggle_marker_publishing')
        else:
            self.get_logger().error('Échec de l\'appel du service toggle_marker_publishing') """
    
    # Ajout du toggle publish en true directement
    def call_pub_std(self):
        self.call_toggle_publish(True)
        request = Empty.Request()
        future = self.cli_pub_std.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        """ if future.result() is not None:
            self.get_logger().info('Connecté au service standard_publish_area')
        else:
            self.get_logger().error('Échec de l\'appel du service standard_publish_area') """

    def call_pub_zone(self, min, max):
        self.call_toggle_publish(True)
        request = BoundingBoxQuery.Request()
        request.min = Point(x=min[0], y=min[1], z=min[2])
        request.max = Point(x=max[0], y=max[1], z=max[2])
        future = self.cli_pub_zone.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        """ if future.result() is not None:
            self.get_logger().info('Connecté au service set_publish_area')
        else:
            self.get_logger().error('Échec de l\'appel du service set_publish_area') """

    def call_clear_zone(self, min, max):
        self.call_toggle_publish(True)
        request = BoundingBoxQuery.Request()
        request.min = Point(x=min[0], y=min[1], z=min[2])
        request.max = Point(x=max[0], y=max[1], z=max[2])
        future = self.cli_clear_zone.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        """ if future.result() is not None:
            self.get_logger().info('Connecté au service clear_bbox')
        else:
            self.get_logger().error('Échec de l\'appel du service clear_bbox') """
    
    def call_reset_octo(self):
        request = Empty.Request()
        future = self.cli_reset_octo.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        """ if future.result() is not None:
            self.get_logger().info('Connecté au service reset')
        else:
            self.get_logger().error('Échec de l\'appel du service reset') """

    # Nouvelle fonctionnalité : publier une grille de carrés de 5x5 sur une zone globale de 15x15
    def call_grid_publish(self):
        """
        Active le publish global puis publie, en partant du coin supérieur gauche,
        une grille de carrés de 10x10 sur une zone globale de 30x30.
        """
        self.get_logger().info("Activation du publish global pour grid publish.")
        self.call_toggle_publish(True) #Activation du publish global pour grid publish.
        square_size = 10.0
        global_size = 30.0
        z_min = -1.0
        z_max = 40.0
        rows = int(global_size // square_size)  # 6 lignes
        cols = int(global_size // square_size)   # 6 colonnes
        for i in range(rows):
            for j in range(cols):
                # Calcul des coordonnées pour centrer la grille
                x_min = -global_size/2 + j * square_size
                x_max = x_min + square_size
                y_max = global_size/2 - i * square_size
                y_min = y_max - square_size
                self.call_pub_zone([x_min, y_min, z_min], [x_max, y_max, z_max])
                time.sleep(0.3)
                self.get_logger().info(f"Publication de la zone: min({x_min}, {y_min}, {z_min}) max({x_max}, {y_max}, {z_max})")
        
        self.call_pub_std()

        self.call_toggle_publish(False) #Desactivation du publish global pour grid publish.

    # Nouvelle fonctionnalité : effacer en grille des carrés de 5x5 sur une zone globale de 15x15
    def call_grid_clear(self):
        """
        Active le publish global puis efface, en partant du coin supérieur gauche,
        une grille de carrés de 10x10 sur une zone globale de 30x30.
        """
        self.get_logger().info("Activation du publish global pour grid publish.")
        self.call_toggle_publish(True) #Activation du publish global pour grid publish.
        square_size = 10.0
        global_size = 30.0
        z_min = -1.0
        z_max = 40.0
        rows = int(global_size // square_size)  # 6 lignes
        cols = int(global_size // square_size)   # 6 colonnes
        for i in range(rows):
            for j in range(cols):
                x_min = -global_size/2 + j * square_size
                x_max = x_min + square_size
                y_max = global_size/2 - i * square_size
                y_min = y_max - square_size
                self.call_clear_zone([x_min, y_min, z_min], [x_max, y_max, z_max])
                time.sleep(0.3)
                self.get_logger().info(f"Effacement de la zone: min({x_min}, {y_min}, {z_min}) max({x_max}, {y_max}, {z_max})")
        
        self.call_pub_std()
            
        self.call_toggle_publish(False) #Desactivation du publish global pour grid publish.

def main(args=None):
    rclpy.init(args=args)
    client = MultiServicePublishClient()

    try:
        while True:
            print("\nMenu des fonctionnalités :")
            print("1. Toggle Publish (Activer/Désactiver)")
            print("2. Publier la zone standard")
            print("3. Publier sur une zone parametrable")
            print("4. Effacer une zone parametrable")
            print("5. Reset de la map entière")
            print("6. Test qui publie une grille de carrés parametrables fondu petit a petit")
            print("7. Test qui efface une grille de carrés parametrables fondu petit a petit")
            print("8. Quitter")

            choice = input("Choisissez une option (1-8) : ").strip()

            if choice == '1':
                enable = input("Activer (true) ou Désactiver (false) ? ").strip().lower()
                if enable in ['true', 'false']:
                    client.call_toggle_publish(enable == 'true')
                else:
                    print("Valeur invalide. Utilisez 'true' ou 'false'.")
            elif choice == '2':
                client.call_pub_std()
            elif choice == '3':
                try:
                    min_x = float(input("Min_x : "))
                    min_y = float(input("Min_y : "))
                    min_z = float(input("Min_z : "))
                    max_x = float(input("Max_x : "))
                    max_y = float(input("Max_y : "))
                    max_z = float(input("Max_z : "))
                    client.call_pub_zone([min_x, min_y, min_z], [max_x, max_y, max_z])
                except ValueError:
                    print("Valeur invalide. Veuillez entrer des nombres.")
            elif choice == '4':
                try:
                    min_x = float(input("Min_x : "))
                    min_y = float(input("Min_y : "))
                    min_z = float(input("Min_z : "))
                    max_x = float(input("Max_x : "))
                    max_y = float(input("Max_y : "))
                    max_z = float(input("Max_z : "))
                    client.call_clear_zone([min_x, min_y, min_z], [max_x, max_y, max_z])
                except ValueError:
                    print("Valeur invalide. Veuillez entrer des nombres.")
            elif choice == '5':
                client.call_reset_octo()
            elif choice == '6':
                client.call_grid_publish()
            elif choice == '7':
                client.call_grid_clear()
            elif choice == '8':
                break
            else:
                print("Option invalide. Veuillez choisir un nombre entre 1 et 8.")
    except KeyboardInterrupt:
        print("\nArrêt du programme.")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
