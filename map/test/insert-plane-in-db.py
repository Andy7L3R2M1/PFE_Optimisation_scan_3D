import psycopg2
from psycopg2.extras import execute_values
import numpy as np
import math

# Connexion à la base de données PostgreSQL
conn = psycopg2.connect(
    dbname="ros_db",
    user="postgres",
    password="pwd",
    host="localhost",
    port="5432"
)

# Génération des points spatiaux
x_range = 2000
y_range = 2000
x_spacing = 5
y_spacing = 5

points = []
for x in range(-x_range, x_range, x_spacing):
    for y in range(-y_range, y_range, y_spacing):
        print("point: x "+str(x)+" y "+str(y))
        z = 0
        color_r = np.random.randint(0, 256)
        color_g = np.random.randint(0, 256)
        color_b = np.random.randint(0, 256)
        color_a = np.random.randint(0, 256)
        timestamp = np.random.random()
        nb_records = 0
        points.append((x, y, z, color_r, color_g, color_b, color_a, timestamp, nb_records))

print("Insert data in db")

# Insertion des points dans la table
SQL = """
INSERT INTO spatial_point (x, y, z, color_r, color_g, color_b, color_a, timestamp, nb_records)
VALUES %s
"""

execute_values(
    conn.cursor(),
    SQL,
    points
)

conn.commit()

# Fermeture de la connexion
conn.close()

print("Données insérées avec succès.")