import sys
import os
import psycopg2
import numpy as np
import pymeshlab

grid_cell_size = int(os.environ.get("GRID_CELL_SIZE", 500))

def main(grid_cell_index_x, grid_cell_index_y):

    ply_filename = f"{grid_cell_index_x}_{grid_cell_index_y}.ply"

    if not os.path.exists(f"./ply/asked/{ply_filename}"):
        print("Error task not in queue")
        sys.exit(1)

    os.remove(f"./ply/asked/{ply_filename}")
    open(f"./ply/pending/{ply_filename}", "x")

    # Database connection details
    conn = psycopg2.connect(
        user=os.getenv("DB_USER"),
        password=os.getenv("DB_PASSWORD"),
        host=os.getenv("DB_HOST"),
        port=os.getenv("DB_PORT"),
        database=os.getenv("DB_NAME"),

    )

    # Read points from the database
    rows = []
    with conn.cursor() as cursor:
        SQL = "SELECT x, y, z, color_r, color_g, color_b, color_a FROM spatial_point WHERE x >= %(x_min)s AND x <= %(x_max)s AND y >= %(y_min)s AND y <= %(y_max)s"
        data = {
            "x_min": grid_cell_index_x * grid_cell_size,
            "x_max": grid_cell_index_x * grid_cell_size + grid_cell_size,
            "y_min": grid_cell_index_y * grid_cell_size,
            "y_max": grid_cell_index_y * grid_cell_size + grid_cell_size,
        }
        print(f"Fetching grid: x_min {data['x_min']} x_max {data['x_max']} y_min {data['y_min']} y_max {data['y_max']}")
        cursor.execute(SQL, data)
        rows = cursor.fetchall()

    colored_points = rows

    if len(colored_points) == 0:
        os.remove(f"./ply/pending/{ply_filename}")
        # TODO: create an empty ply
        sys.exit(f"No points in the cell: x {grid_cell_index_x} y {grid_cell_index_y}")

    # Close the database connection
    conn.close()

    print("Data acquired, number of vertices: " + str(len(colored_points)))

    vertices = np.array([(p[0]/100.0, p[1]/100.0, p[2]/100.0) for p in colored_points])
    colors = np.array([(p[3], p[4], p[5], p[6]) for p in colored_points])

    print("Numpy array created")

    # Create a new mesh using pymeshlab
    ms = pymeshlab.MeshSet()
    print("Mesh Object created")

    # Create a new mesh with the vertices and colors
    mesh = pymeshlab.Mesh(vertex_matrix=vertices, v_color_matrix=colors)
    ms.add_mesh(mesh, 'point_cloud')
    print("Pointcloud mesh created")

    print("Starting surface reconstruction")
    ms.apply_filter("generate_surface_reconstruction_ball_pivoting")
    print("Surface reconstruction completed")

    ms.save_current_mesh(f"./ply/done/{ply_filename}")
    print(f"PLY file saved as {ply_filename}")
    os.remove(f"./ply/pending/{ply_filename}")

if __name__ == "__main__":
    if len(sys.argv)<3:
        sys.exit("Usage: app.py <x> <y> (with x and y 2 integers)")
    try:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    except ValueError:
        sys.exit("Usage: app.py <x> <y> (with x and y 2 integers)")
    main(x, y)
