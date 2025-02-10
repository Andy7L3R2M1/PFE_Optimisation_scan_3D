from flask import Flask, request, jsonify, send_from_directory
from markupsafe import escape
import os
from os import listdir
from os.path import isfile, join

grid_cell_size = int(os.environ.get("GRID_CELL_SIZE", 500))

app = Flask(__name__)

@app.route('/')
def manual():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>PLY File Management API</title>
    </head>
    <body>
        <h1>PLY File Management API</h1>
        <p>Welcome to the PLY File Management API. Below is the list of available endpoints, their descriptions, and the corresponding HTTP status codes:</p>

        <h2>Endpoints</h2>
        <ul>
            <li>
                <strong>GET /</strong>
                <p>Returns this help manual.</p>
                <p><strong>HTTP Status Codes:</strong></p>
                <ul>
                    <li><code>200 OK</code>: Successfully returns the help manual.</li>
                </ul>
            </li>
            <li>
                <strong>GET /ply/files</strong>
                <p>Lists all PLY files categorized as <code>done</code>, <code>pending</code>, or <code>asked</code>.</p>
                <p><strong>HTTP Status Codes:</strong></p>
                <ul>
                    <li><code>200 OK</code>: Successfully returns the list of files.</li>
                    <li><code>500 Internal Server Error</code>: Required directories are missing or cannot be accessed.</li>
                </ul>
            </li>
            <li>
                <strong>GET /ply/done</strong>
                <p>Lists all PLY files in the <code>done</code> directory.</p>
                <p><strong>HTTP Status Codes:</strong></p>
                <ul>
                    <li><code>200 OK</code>: Successfully returns the list of files.</li>
                    <li><code>500 Internal Server Error</code>: An error occurred while accessing the directory.</li>
                </ul>
            </li>
            <li>
                <strong>GET /ply/done/&lt;filename&gt;</strong>
                <p>Retrieves a specific PLY file from the <code>done</code> directory.</p>
                <p><strong>HTTP Status Codes:</strong></p>
                <ul>
                    <li><code>200 OK</code>: Successfully retrieves the requested file.</li>
                    <li><code>404 Not Found</code>: The specified file does not exist in the <code>done</code> directory.</li>
                </ul>
            </li>
            <li>
                <strong>POST /ply/request</strong>
                <p>Request a new PLY file by providing grid coordinates.</p>
                <p><strong>Parameters:</strong></p>
                <ul>
                    <li><code>x</code> (integer): The x-coordinate</li>
                    <li><code>y</code> (integer): The y-coordinate</li>
                </ul>
                <p><strong>HTTP Status Codes:</strong></p>
                <ul>
                    <li><code>202 Accepted</code>: The request is accepted, and the PLY file will be created.</li>
                    <li><code>201 Created</code>: The requested PLY file is already completed and available.</li>
                    <li><code>208 Already Reported</code>: The requested PLY file has already been requested.</li>
                    <li><code>400 Bad Request</code>: Invalid or missing parameters (e.g., <code>x</code> or <code>y</code> not provided or not integers).</li>
                </ul>
            </li>
            <li>
                <strong>GET /flask-health-check</strong>
                <p>Returns <code>success</code> if the API is running.</p>
                <p><strong>HTTP Status Codes:</strong></p>
                <ul>
                    <li><code>200 OK</code>: The API is running successfully.</li>
                </ul>
            </li>
        </ul>

        <h2>Environment Variables</h2>
        <ul>
            <li><strong>GRID_CELL_SIZE:</strong> Defines the grid cell size. Default value is <code>500</code>.</li>
        </ul>

        <h2>Directory Structure</h2>
        <p>The following directories are required:</p>
        <ul>
            <li><code>ply/done</code>: Stores completed PLY files.</li>
            <li><code>ply/pending</code>: Stores PLY files that are in progress.</li>
            <li><code>ply/asked</code>: Stores PLY file requests.</li>
        </ul>

        <p>Note: Missing directories are created automatically when requests are made.</p>

        <h2>Usage Examples</h2>
        <p>Here are some examples of how to use this API:</p>

		<h3>Inserting a Plane in the Database</h3>
		<pre><code>
		python3 test/insert-plane-in-db.py
		</code></pre>

        <h3>Requesting a New PLY File</h3>
        <pre><code>
        curl -X POST -F "x=-10" -F "y=10" http://<host>/ply/request
        </code></pre>
        
        <h3>Listing All Files</h3>
        <pre><code>
        curl http://<host>/ply/files
        </code></pre>

        <h3>Fetching a Completed File</h3>
        <pre><code>
        curl http://<host>/ply/done/-1_0.ply -O
        </code></pre>
    </body>
    </html>
    """

@app.route('/ply/files', methods=['GET'])
def get_ply_files():
    # Ensure directories exist
    base_dirs = ["ply/done", "ply/pending", "ply/asked"]
    for directory in base_dirs:
        if not os.path.exists(directory):
            return jsonify({"error": f"Directory '{directory}' does not exist."}), 500

    done = [f for f in listdir("ply/done") if isfile(join("ply/done", f))]
    pending = [f for f in listdir("ply/pending") if isfile(join("ply/pending", f))]
    asked = [f for f in listdir("ply/asked") if isfile(join("ply/asked", f))]

    return jsonify({
        "done": done,
        "pending": pending,
        "asked": asked
    }), 200

@app.route('/ply/done', methods=['GET'])
def list_done_files():
    try:
        files = [f for f in listdir("ply/done") if isfile(join("ply/done", f))]
        return jsonify(files), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/ply/done/<filename>')
def serve_done_file(filename):
    try:
        return send_from_directory('ply/done', filename)
    except FileNotFoundError:
        return jsonify({"error": f"File '{filename}' not found in 'done' directory."}), 404


@app.route('/ply/request', methods=['POST'])
def request_ply_file():
    # Handle POST request for creating a PLY file
    x = request.form.get('x')
    y = request.form.get('y')

    if x is None or y is None:
        return jsonify({"error": "Please provide 'x' and 'y' parameters as integers."}), 400

    try:
        x = int(x)
        y = int(y)
    except ValueError:
        return jsonify({"error": "Parameters 'x' and 'y' must be integers."}), 400

    # Calculate grid cell indices
    grid_cell_index_x = x // grid_cell_size
    grid_cell_index_y = y // grid_cell_size
    ply_filename = f"{grid_cell_index_x}_{grid_cell_index_y}.ply"

    # Ensure directories exist
    base_dirs = ["ply/done", "ply/pending", "ply/asked"]
    for directory in base_dirs:
        if not os.path.exists(directory):
            os.makedirs(directory)

    # Load file lists
    done = [f for f in listdir("ply/done") if isfile(join("ply/done", f))]
    pending = [f for f in listdir("ply/pending") if isfile(join("ply/pending", f))]
    asked = [f for f in listdir("ply/asked") if isfile(join("ply/asked", f))]

    # Check file existence and respond with appropriate status
    if ply_filename in done:
        return jsonify({"message": f"PLY file '{ply_filename}' is already created.", "status": "done"}), 201
    elif ply_filename in asked or ply_filename in pending:
        return jsonify({"message": f"PLY file '{ply_filename}' is already requested.", "status": "already_reported"}), 208
    else:
        # Safely create the requested file
        asked_file_path = f"./ply/asked/{ply_filename}"
        with open(asked_file_path, "x") as f:
            pass

        return jsonify({
            "message": f"Request accepted. PLY file '{ply_filename}' will be created.",
            "grid_coordinates": {"x": grid_cell_index_x, "y": grid_cell_index_y},
            "status": "accepted"
        }), 202

@app.route('/flask-health-check')
def flask_health_check():
	return "success"