#!/usr/bin/env python

# Python code pour tester les fonctionnalités pour paramétrer le publish
# de la map 3D de voxels

import sys

from geometry_msgs.msg import Point  # Importer le message Point

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
        """ self.drone_id = (
            self.get_parameter("drone_id").get_parameter_value().string_value
        )
        if self.drone_id == "UNDEFINED":
            self.get_logger().error("Drone ID not defined")
            raise ValueError("Drone ID not defined") """

        # Définitions des services serveurs existants
        service_toggle_pub = 'color_octomap_server_'+self.drone_id+'/toggle_marker_publishing'
        service_pub_std = 'color_octomap_server_'+self.drone_id+'/standard_publish_area'
        service_pub_zone = 'color_octomap_server_'+self.drone_id+'/set_publish_area'
        service_clear_zone = 'color_octomap_server_'+self.drone_id+'/clear_bbox'
        service_reset='color_octomap_server_'+self.drone_id+'/reset'

        # Clients pour chaque service
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
        

    # Fonctions de gestion du publish utilisateur
    def call_toggle_publish(self, enable):
        request = SetBool.Request()
        request.data = enable
        future = self.cli_toggle_pub.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Connect to service toggle_marker_publishing')
        else:
            self.get_logger().error('Échec de l\'appel du service toggle_marker_publishing')
    
    def call_pub_std(self):
        request = Empty.Request()
        future = self.cli_pub_std.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Connect to service standard_publish_area')
        else:
            self.get_logger().error('Échec de l\'appel du service standard_publish_area')


    def call_pub_zone(self, min, max):
        request = BoundingBoxQuery.Request()

        # Créer des objets Point pour min et max
        request.min = Point(x=min[0], y=min[1], z=min[2])
        request.max = Point(x=max[0], y=max[1], z=max[2])

        future = self.cli_pub_zone.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Connect to service set_publish_area')
        else:
            self.get_logger().error('Échec de l\'appel du service set_publish_area')


    def call_clear_zone(self, min, max):
        request = BoundingBoxQuery.Request()

        # Créer des objets Point pour min et max
        request.min = Point(x=min[0], y=min[1], z=min[2])
        request.max = Point(x=max[0], y=max[1], z=max[2])

        future = self.cli_clear_zone.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Connect to service clear_bbox')
        else:
            self.get_logger().error('Échec de l\'appel du service clear_bbox')
    
    def call_reset_octo(self):
        request = Empty.Request()
        future = self.cli_reset_octo.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Connect to service standard_publish_area')
        else:
            self.get_logger().error('Échec de l\'appel du service standard_publish_area')


def main(args=None):
    rclpy.init(args=args)
    client = MultiServicePublishClient()

    try:
        while True:
            # Afficher le menu
            print("\nMenu des fonctionnalités :")
            print("1. Toggle Publish (Activer/Désactiver)")
            print("2. Publier la zone standard")
            print("3. Définir une zone de publication")
            print("4. Effacer une zone")
            print("5. Reset la map entière")
            print("6. Quitter")

            # Saisie utilisateur
            choice = input("Choisissez une option (1-6) : ").strip()

            # Gestion des choix
            if choice == '1':
                enable = input("Activer (true) ou Désactiver (false) ? ").strip().lower()  # .strip() pour enlever les espaces

                if enable in ['true', 'false']:  # Comparaison après la conversion en minuscule
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
                client.call_reset_octo();
            elif choice == '6':
                break
            else:
                print("Option invalide. Veuillez choisir un nombre entre 1 et 6.")
    except KeyboardInterrupt:
        print("\nArrêt du programme.")
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
