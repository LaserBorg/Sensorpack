'''
receive BNO055 sensor data from USB
'''

import serial
import numpy as np
import open3d as o3d
import time


class IMU_Visualizer:
    def __init__(self):
        # Initialize Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(width=640, height=480)

        # Create a 3D box to represent the sensor
        self.box = o3d.geometry.TriangleMesh.create_box(width=1, height=1, depth=1)
        self.box.compute_vertex_normals()
        self.box.paint_uniform_color([0.5, 0.5, 0.5])  # Gray color
        self.vis.add_geometry(self.box)

        # Create a coordinate frame
        self.frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        self.vis.add_geometry(self.frame)

        # Set the camera parameters
        ctr = self.vis.get_view_control()
        ctr.set_zoom(1.5)  # Adjust this value to move the camera further away

        self.prev_acceleration = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])
        self.euler = np.array([0, 0, 0])
        self.dt = 0.01 


    def integrate_acceleration(self, acceleration):
        acceleration = np.array(acceleration)
        self.velocity += 0.5 * (acceleration + self.prev_acceleration) * self.dt
        self.position += self.velocity * self.dt
        self.prev_acceleration = acceleration
        return self.position
    

    def update_visualization(self, pos=(0,0,0), euler=(0,0,0)):
        # swap the axes to match the Open3D coordinate system
        euler = (euler[2], -euler[0], -euler[1])

        euler = np.array(euler)
        delta_euler = euler - self.euler
        self.euler = euler

        R = self.box.get_rotation_matrix_from_xyz(np.radians(delta_euler))
        self.box.rotate(R, center=self.box.get_center())

        self.box.translate(pos, relative=False)
        
        # Update the visualizer
        self.vis.update_geometry(self.box)
        self.vis.poll_events()
        self.vis.update_renderer()


def read_BNO055_from_serial(ser):
    package_len = 23 * 4  # 23 floats, each 4 bytes

    # Read bytes until we find the frame marker
    while True:
        data = ser.read(2)
        if data == FRAME_MARKER:
            break

    # Preallocate a numpy array for the frame data
    data = np.empty(package_len * 4, dtype=np.uint8)

    # Read the frame data into the numpy array
    for i in range(package_len):
        data[i] = ser.read(1)[0]

    # Convert the bytes to a numpy array of floats
    values = np.frombuffer(data, dtype=np.float32)

    # Decode the values
    temperature = values[0]
    acceleration = tuple(values[1:4])
    magnetic = tuple(values[4:7])
    gyro = tuple(values[7:10])
    euler = tuple(values[10:13])
    quaternion = tuple(values[13:17])
    linear_acceleration = tuple(values[17:20])
    gravity = tuple(values[20:23])
    
    return temperature, acceleration, magnetic, gyro, euler, quaternion, linear_acceleration, gravity


if __name__ == "__main__":
    
    port = 'COM3'
    FRAME_MARKER = b'\xAA\xAA'

    imu_visualizer = IMU_Visualizer()

    with serial.Serial(port, 921600, timeout=1) as ser:
        while True:
            values = read_BNO055_from_serial(ser)
            
            if values is not None:
                temperature, acceleration, magnetic, gyro, euler, quaternion, linear_acceleration, gravity = values

                # print("Temperature:", temperature)
                # print("Acceleration:", acceleration, "Length:", len(acceleration))
                # print("Magnetic:", magnetic, "Length:", len(magnetic))
                # print("Gyro:", gyro, "Length:", len(gyro))
                # print("Euler:", euler, "Length:", len(euler))
                # print("Quaternion:", quaternion, "Length:", len(quaternion))
                # print("Linear acceleration:", linear_acceleration, "Length:", len(linear_acceleration))
                # print("Gravity:", gravity, "Length:", len(gravity))


                # # Integrate the acceleration to get the position
                # position = imu_visualizer.integrate_acceleration(acceleration)
                # print("Position:", position)

                imu_visualizer.update_visualization(euler=euler)
                time.sleep(imu_visualizer.dt)

            else:
                print("Resynchronizing...")
