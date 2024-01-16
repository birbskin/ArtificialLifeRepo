import dm_control.mujoco 
import mujoco.viewer 
import time

m = dm_control.mujoco.MjModel.from_xml_path('example.xml')
d = dm_control.mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.azimuth = 180  # Azimuthal angle (in degrees)
    viewer.cam.elevation = -20  # Elevation angle (in degrees)
    viewer.cam.distance = 3.0  # Distance from the camera to the target
    viewer.cam.lookat[0] = 0.0  # X-coordinate of the target position
    viewer.cam.lookat[1] = 0.0  # Y-coordinate of the target position
    viewer.cam.lookat[2] = 0.75  # Z-coordinate of the target position

    for i in range(10000):
        motor_names = ["arm1", "arm2", "leg"]
        for motor in motor_names:
            i = dm_control.mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, motor)
            d.ctrl[i] = 1.0

        dm_control.mujoco.mj_step(m, d)
        viewer.sync()
        time.sleep(0.01)

    viewer.close()
