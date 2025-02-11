#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from pympler.asizeof import asizeof
import time
import importlib
import subprocess
from collections import defaultdict

# Calcul sur une fenêtre de temps plus longues de 10s pour des mesures plus précises et stables

class DynamicTopicSubscriber(Node):
    def __init__(self):
        super().__init__("dynamic_topic_subscriber")
        self.topic_subscriptions = {}  # To store subscriptions
        self.message_sizes = defaultdict(int)  # Track message sizes
        self.message_counts = defaultdict(int)  # Track message counts
        self.start_times = defaultdict(lambda: time.time())  # Start times for each topic
        self.last_received_times = defaultdict(lambda: time.time())  # Last time a message was received
        self.selected_topics = []  # Store selected topics to display
        self.last_frequencies = defaultdict(lambda: 0.0)  # Store last frequency for topics
        self.reset_interval = 10  # Interval to reset statistics (in seconds)

        # Periodically discover topics and print bandwidth
        self.create_timer(1.0, self.discover_topics)
        self.create_timer(1.0, self.print_bandwidth)

    def discover_topics(self):
        topic_names_and_types = self.get_topic_names_and_types()
        for item in topic_names_and_types:
            if len(item) != 2:
                self.get_logger().warn(f"Skipping topic due to unexpected format: {item}")
                continue

            topic_name, topic_types = item
            if not topic_types:
                self.get_logger().warn(f"No types found for topic {topic_name}, skipping...")
                continue

            if topic_name in self.topic_subscriptions:
                continue

            topic_type = topic_types[0].replace("/msg/", ".")  # Convertir 'pkg/msg/Type' en 'pkg.msg.Type'

            if "." not in topic_type:
                self.get_logger().error(f"Invalid message type format: {topic_type}")
                continue

            try:
                module_name, message_name = topic_type.rsplit(".", 1)
                
                full_module_name = f"{module_name}.msg"  # Ajout explicite du sous-module `.msg`
                msg_module = importlib.import_module(full_module_name)  # Importe geometry_msgs.msg, sensor_msgs.msg, etc.
                msg_class = getattr(msg_module, message_name)  # Récupère Twist, Image, etc.
                
                qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.VOLATILE)
                subscription = self.create_subscription(
                    msg_class,
                    topic_name,
                    lambda msg, t=topic_name: self.message_callback(msg, t),
                    qos_profile
                )

                self.topic_subscriptions[topic_name] = subscription
                self.get_logger().info(f"Subscribed to topic: {topic_name} ({topic_type})")

            except ModuleNotFoundError:
                self.get_logger().error(f"Module not found: {full_module_name}")
            except AttributeError:
                self.get_logger().error(f"Message type not found: {message_name} in {full_module_name}")
            except Exception as e:
                self.get_logger().error(f"Failed to subscribe to topic {topic_name}: {e}")
    
    def message_callback(self, msg, topic_name):
        # Vérifier si le message est "vide" ou non. Cela dépend du type de message.
        # Si le message a un champ "data", "pose", ou autre, vérifiez-le.
        if not msg:  # Si le message est vide
            return
        
        # Si le message n'est pas vide, on procède
        msg_size = asizeof(msg)
        self.message_sizes[topic_name] += msg_size
        self.message_counts[topic_name] += 1

        # Calculer la fréquence de publication
        current_time = time.time()
        last_received_time = self.last_received_times[topic_name]
        frequency = 1 / (current_time - last_received_time) if (current_time - last_received_time) > 0 else 0
        self.last_frequencies[topic_name] = frequency
        self.last_received_times[topic_name] = current_time

    def print_bandwidth(self):
        for topic_name in self.selected_topics:
            if topic_name not in self.message_sizes:
                continue  # Skip topics not being tracked
            elapsed_time = time.time() - self.start_times[topic_name]
            
            # Calculez la bande passante uniquement après une période suffisante (par exemple 10 secondes)
            if elapsed_time > self.reset_interval:
                total_size = self.message_sizes[topic_name]
                total_messages = self.message_counts[topic_name]
                bandwidth = total_size / elapsed_time if total_size > 0 else 0
                avg_msg_size = total_size / total_messages if total_messages > 0 else 0

                # Afficher en MB/s si nécessaire
                bandwidth_in_mb = bandwidth / (1024 * 1024)
                avg_msg_size_in_mb = avg_msg_size / (1024 * 1024)
                
                # Afficher aussi la fréquence de publication
                frequency = self.last_frequencies[topic_name]
                
                self.get_logger().info(
                    f"Topic: {topic_name}\n"
                    f"Bandwidth: {bandwidth_in_mb:.2f} MB/s\n"
                    f"Avg Msg Size: {avg_msg_size_in_mb:.2f} MB\n"
                    f"Msg Count: {total_messages}\n"
                    f"Publication Frequency: {frequency:.2f} Hz\n"  # Afficher la fréquence
                )
                
                # Réinitialisation des compteurs après l'affichage
                self.message_sizes[topic_name] = 0
                self.message_counts[topic_name] = 0
                self.start_times[topic_name] = time.time()

    def get_available_topics(self):
        """Récupère la liste des topics disponibles via la commande ros2 topic list."""
        result = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE)
        topics = result.stdout.decode('utf-8').strip().split('\n')
        return topics

    def display_and_select_topics(self, topics):
        """Affiche les topics disponibles et permet à l'utilisateur de choisir."""
        print("Liste des topics disponibles :\n")
        
        # Afficher les topics avec numéros
        for i, topic in enumerate(topics, 1):
            print(f"{i}. {topic}")
        
        print("\nVeuillez entrer les numéros des topics que vous souhaitez afficher, séparés par des espaces.")
        print("Exemple : 1 3 5 pour afficher les topics 1, 3 et 5.")
        
        # Demander à l'utilisateur de saisir les numéros des topics
        selected_numbers = input("Numéros des topics : ")
        
        # Convertir la saisie en une liste de numéros
        try:
            selected_indexes = [int(num) - 1 for num in selected_numbers.split()]
            
            # Afficher les topics sélectionnés
            print("\nVous avez choisi les topics suivants :\n")
            for index in selected_indexes:
                if index < 0 or index >= len(topics):
                    print(f"Numéro invalide : {index + 1}")
                else:
                    self.selected_topics.append(topics[index])
                    print(f"- {topics[index]}")
        except ValueError:
            print("Erreur : Veuillez entrer des numéros valides.")

def main():
    rclpy.init()

    # Créer une instance de notre node ROS2
    node = DynamicTopicSubscriber()

    # Récupérer la liste des topics
    topics = node.get_available_topics()
    
    if not topics:
        print("Aucun topic disponible.")
    else:
        # Afficher les topics et permettre à l'utilisateur de choisir
        node.display_and_select_topics(topics)

    # Démarrer l'exécution de ROS2
    rclpy.spin(node)
    
    # Arrêter proprement le node ROS2 après la fin
    rclpy.shutdown()

if __name__ == "__main__":
    main()
