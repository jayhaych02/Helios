import csv
import io
import math
import socket
import threading
import unittest

# Helper functions
def parse_csv_data(csv_data):
    """Parse CSV content (as string) and return a list of dictionaries."""
    f = io.StringIO(csv_data)
    reader = csv.DictReader(f)
    return list(reader)

def is_within_range(coord1, coord2, tolerance=1.0):
    """
    Compute Euclidean distance between two coordinates (tuples)
    and return True if the distance is within the given tolerance.
    """
    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(coord1, coord2)))
    return distance <= tolerance

def is_valid_navigation_mode(mode):
    """Return True if the navigation mode is one of the accepted values."""
    return mode in ["LiDAR", "GNSS"]

def obstacle_cleared(initial_robot, final_robot, obstacle):
    """
    Given one-dimensional positions:
    - initial_robot: robot’s initial x-coordinate
    - final_robot: robot’s final x-coordinate after moving
    - obstacle: x-coordinate of the obstacle
    The robot is assumed to be approaching the obstacle from behind.
    We define clearance as:
       clearance = obstacle - robot_position.
    If initially clearance > 0 (i.e. the robot is behind the obstacle)
    and finally clearance < 0 (i.e. the robot has passed it),
    then the obstacle is considered cleared.
    """
    initial_diff = obstacle - initial_robot
    final_diff = obstacle - final_robot
    return initial_diff > 0 and final_diff < 0

def sensor_status_on(status):
    """Return True if the sensor status (a string) indicates it is 'on'."""
    return status.lower() == "on"


# Unit tests using CSV data
class TestRobotSystemCSV(unittest.TestCase):
    def setUp(self):
        # Simulated CSV content for several robot entries.
        # Columns: robot_id, x, y, z, navigation_mode, obstacle_x, obstacle_y, obstacle_z, sensor_status
        self.csv_content = """robot_id,x,y,z,navigation_mode,obstacle_x,obstacle_y,obstacle_z,sensor_status
1,0,0,0,GNSS,10,0,0,on
2,0.5,0,0,LiDAR,1,0,0,off
3,0,0,0,GNSS,0,0,0,on
4,1,1,1,LiDAR,0,0,0,on
"""
        self.data = parse_csv_data(self.csv_content)
    
    def test_coordinate_within_range(self):
        """
        Check that for a given target (e.g. (0,0,0)), the robot's position is within 1m.
        In this example, robot 2 is within range while robot 4 is not.
        """
        target = (0, 0, 0)
        for row in self.data:
            robot_coord = (float(row['x']), float(row['y']), float(row['z']))
            if row['robot_id'] == '2':
                self.assertTrue(is_within_range(robot_coord, target, tolerance=1.0),
                                "Robot 2 should be within 1m of the target.")
            if row['robot_id'] == '4':
                self.assertFalse(is_within_range(robot_coord, target, tolerance=1.0),
                                 "Robot 4 should NOT be within 1m of the target.")
    
    def test_navigation_mode_valid(self):
        """Verify that the navigation mode is either 'LiDAR' or 'GNSS'."""
        for row in self.data:
            mode = row['navigation_mode']
            self.assertTrue(is_valid_navigation_mode(mode),
                            f"Robot {row['robot_id']} has an invalid navigation mode: {mode}")
    
    def test_obstacle_clearance(self):
        """
        Simulate a one-dimensional movement along x.
        For each robot, assume it moves 2 units forward.
        Use the x-coordinate and the obstacle_x value to determine clearance.
        """
        for row in self.data:
            initial_robot_x = float(row['x'])
            final_robot_x = initial_robot_x + 2  # simulate a forward move of 2 units
            obstacle_x = float(row['obstacle_x'])
            cleared = obstacle_cleared(initial_robot_x, final_robot_x, obstacle_x)
            
            if row['robot_id'] == '1':
                # Robot 1: 0 -> 2, obstacle at 10; not cleared.
                self.assertFalse(cleared, "Robot 1 did not clear the obstacle.")
            elif row['robot_id'] == '2':
                # Robot 2: 0.5 -> 2.5, obstacle at 1; clearance changes from 0.5 to -1.5 => cleared.
                self.assertTrue(cleared, "Robot 2 should have cleared the obstacle.")
            elif row['robot_id'] == '3':
                # Robot 3: 0 -> 2, obstacle at 0; initial_diff = 0 so clearance condition is not met.
                self.assertFalse(cleared, "Robot 3 did not clear the obstacle (initial difference is 0).")
            elif row['robot_id'] == '4':
                # Robot 4: 1 -> 3, obstacle at 0; already past obstacle but initial_diff is negative.
                self.assertFalse(cleared, "Robot 4 did not meet the clearance criteria as defined.")
    
    def test_sensor_status(self):
        """Check that the sensor status is 'on' for some robots and 'off' for others."""
        for row in self.data:
            status = row['sensor_status']
            if row['robot_id'] in ['1', '3', '4']:
                self.assertTrue(sensor_status_on(status),
                                f"Robot {row['robot_id']} should have the sensor on.")
            elif row['robot_id'] == '2':
                self.assertFalse(sensor_status_on(status),
                                 f"Robot {row['robot_id']} should have the sensor off.")


# Unit tests simulating TCP data input.
class TestRobotSystemTCP(unittest.TestCase):
    def setUp(self):
        # Define similar CSV content to be sent over TCP.
        self.csv_content = """robot_id,x,y,z,navigation_mode,obstacle_x,obstacle_y,obstacle_z,sensor_status
1,0,0,0,GNSS,10,0,0,on
2,0.5,0,0,LiDAR,1,0,0,off
"""
        self.host = '127.0.0.1'
        self.port = 0  # Let the OS choose an ephemeral port

        # Set up a simple TCP server to send the CSV content.
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        self.port = self.server.getsockname()[1]
        self.server_running = True

        def handle_client(conn):
            with conn:
                conn.sendall(self.csv_content.encode('utf-8'))

        def server_loop():
            while self.server_running:
                try:
                    self.server.settimeout(1.0)
                    conn, addr = self.server.accept()
                    handle_client(conn)
                except socket.timeout:
                    continue

        self.server_thread = threading.Thread(target=server_loop)
        self.server_thread.start()

    def tearDown(self):
        self.server_running = False
        self.server.close()
        self.server_thread.join()

    def test_tcp_data_retrieval(self):
        """Connect to the TCP server, receive the CSV content, parse it, and verify its contents."""
        with socket.create_connection((self.host, self.port), timeout=2) as sock:
            received = sock.recv(1024).decode('utf-8')
        data = parse_csv_data(received)
        self.assertEqual(len(data), 2, "Should have received two CSV rows.")
        # Check sample data for first robot.
        self.assertEqual(data[0]['robot_id'], '1')
        self.assertEqual(data[0]['navigation_mode'], 'GNSS')
        # Verify that the second robot has the sensor turned off.
        self.assertEqual(data[1]['sensor_status'], 'off')


if __name__ == '__main__':
    unittest.main()
