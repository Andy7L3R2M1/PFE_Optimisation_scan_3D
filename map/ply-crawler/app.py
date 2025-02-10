import os
import time
import subprocess
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class OnMyWatch:
    # Set the directory on watch
    watchDirectory = "./ply/asked"

    def __init__(self):
        self.observer = Observer()
        print("Ply crawler initialized")

    def run(self):
        if not os.path.exists(self.watchDirectory):
            print(f"Directory {self.watchDirectory} does not exist.")
            return
        
        event_handler = Handler()
        self.observer.schedule(event_handler, self.watchDirectory)
        self.observer.start()
        print("Ply crawler started")
        
        try:
            while True:
                time.sleep(5)
        except KeyboardInterrupt:
            self.observer.stop()
            print("Observer Stopped")

        self.observer.join()

class Handler(FileSystemEventHandler):
    @staticmethod
    def on_any_event(event):
        if event.is_directory:
            return None
        elif event.event_type == 'created':
            # Event is created, you can process it now
            print("Watchdog received created event - % s." % event.src_path)
            filename = os.path.basename(event.src_path)
            parts = filename.split('_')
            grid_cell_index_x = int(parts[0])
            grid_cell_index_y = int((parts[1].split('.'))[0])
            subprocess.run(["python", "generator.py", str(grid_cell_index_x), str(grid_cell_index_y)])

if __name__ == '__main__':
    print("Starting app.py")
    watch = OnMyWatch()
    watch.run()