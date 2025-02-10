import psycopg2
import numpy as np
import pymeshlab

# Database connection details
conn = psycopg2.connect(
    dbname="ros_db",
    user="postgres",
    password="pwd",
    host="localhost",
    port="5432"
)

# Read points from the database
points = []
with conn.cursor() as cursor:
    cursor.execute("SELECT x, y, z, color_r, color_g, color_b, color_a FROM spatial_point")
    rows = cursor.fetchall()
    for row in rows:
        points.append((row[0], row[1], row[2], row[3], row[4], row[5], row[6]))

# Close the database connection
conn.close()

print("Data acquired, number of vertices: " + str(len(points)))

# Create a new mesh using pymeshlab
ms = pymeshlab.MeshSet()
vertices = np.array([(p[0], p[1], p[2]) for p in points])
colors = np.array([(p[3], p[4], p[5], p[6]) for p in points])

print("Starting surface reconstruction")

# Create a new mesh with the vertices and colors
mesh = pymeshlab.Mesh(vertex_matrix=vertices, v_color_matrix=colors)
ms.add_mesh(mesh, 'point_cloud')

# Apply Ball Pivoting Algorithm (BPA) to reconstruct the surface
ms.apply_filter("generate_surface_reconstruction_ball_pivoting")

print("Surface reconstruction completed")

# Save the result to a PLY file
output_ply_file = 'reconstructed_surface.ply'
ms.save_current_mesh(output_ply_file)

print(f"PLY file saved as {output_ply_file}")