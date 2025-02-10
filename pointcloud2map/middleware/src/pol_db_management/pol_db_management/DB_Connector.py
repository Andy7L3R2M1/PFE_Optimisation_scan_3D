#!/usr/bin/env python

import os
import sys
import time
import psycopg2


class DB_Connector:

    def __init__(self):
        self.connection = None
        self.cursor = None
        self.connect()

    def connect(self):
        while self.connection is None:
            try:
                print("Attempting to connect to the database...")
                sys.stdout.flush()
                self.connection = psycopg2.connect(
                    user=os.getenv("DB_USER"),
                    password=os.getenv("DB_PASSWORD"),
                    host=os.getenv("DB_HOST"),
                    port=os.getenv("DB_PORT"),
                    database=os.getenv("DB_NAME"),
                )
                self.connection.autocommit = True
                self.cursor = self.connection.cursor()
            except Exception as e:
                print(f"Error connecting to database: {e}", file=sys.stderr)
                sys.stdout.flush()
                self.connection = None
                time.sleep(5)
        print("Database connection established")
        sys.stdout.flush()

    def __del__(self):
        self.cursor.close()
        self.connection.close()
        print("Database connection closed")
