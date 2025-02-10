from psycopg2.extras import execute_values
from pol_db_management.DB_Connector import DB_Connector
from .SpatialPoint import SpatialPoint


class SpatialData_DB_Connector(DB_Connector):
    __instance = None

    def __init__(self):
        super().__init__()
        if SpatialData_DB_Connector.__instance is not None:
            raise Exception(
                "This class is a singleton! Use SpatialData_DB_Connector.instance() to get the instance."
            )

    @staticmethod
    def instance():
        if SpatialData_DB_Connector.__instance is None:
            SpatialData_DB_Connector.__instance = SpatialData_DB_Connector()
        return SpatialData_DB_Connector.__instance

    def store_point(self, point: SpatialPoint):
        if not self.cursor:
            print("Database cursor not available")
            return
        # If the point does not exist, insert it, otherwise update it
        SQL = "INSERT INTO spatial_point (x, y, z, color_r, color_g, color_b, color_a, timestamp, nb_records) VALUES (%(x)s, %(y)s, %(z)s, %(r)s, %(g)s, %(b)s, %(a)s, %(ts)s, 1) ON CONFLICT (x, y, z) DO UPDATE SET color_r = %(r)s, color_g = %(g)s, color_b = %(b)s, color_a = %(a)s, timestamp = %(ts)s"
        data = {
            "x": point.x,
            "y": point.y,
            "z": point.z,
            "r": point.color.r,
            "g": point.color.g,
            "b": point.color.b,
            "a": point.color.a,
            "ts": point.timestamp,
        }
        self.cursor.execute(SQL, data)

    def store_points(self, points: list[SpatialPoint]):
        if not self.cursor:
            print("Database cursor not available")
            return
        SQL = "INSERT INTO spatial_point (x, y, z, color_r, color_g, color_b, color_a, timestamp, nb_records) VALUES %s"
        execute_values(
            self.cursor,
            SQL,
            [
                (
                    point.x,
                    point.y,
                    point.z,
                    point.color.r,
                    point.color.g,
                    point.color.b,
                    point.color.a,
                    point.timestamp,
                    1,
                )
                for point in points
            ],
        )
