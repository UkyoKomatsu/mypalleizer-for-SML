# ==========================================================
# arm_controller.py
# Part 3-1
# ==========================================================

import math
import time
import os

from pymycobot import MyPalletizerSocket

import config
from camera_controller import CameraController


# ==========================================================
# PID
# ==========================================================

class PIDController:

    def __init__(
        self,
        kp,
        ki,
        kd
    ):

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):

        self.integral = 0.0
        self.prev_error = 0.0

    def update(
        self,
        error
    ):

        self.integral = max(
            -config.PID_INTEGRAL_LIMIT,
            min(config.PID_INTEGRAL_LIMIT, self.integral + error)
        )

        derivative = (
            error -
            self.prev_error
        )

        output = (

            self.kp * error +

            self.ki * self.integral +

            self.kd * derivative

        )

        self.prev_error = error

        return output


# ==========================================================
# Arm Controller
# ==========================================================

class ArmController:

    def __init__(self):

        # --------------------------------------
        # Camera
        # --------------------------------------

        self.camera = CameraController()

        # --------------------------------------
        # Robot
        # --------------------------------------

        self.mc = MyPalletizerSocket(

            os.getenv("MYPALLETIZER_HOST", "192.168.0.232"),

            9000

        )

        # MyPalletizerSocket connects in its constructor.

        # --------------------------------------
        # PID
        # --------------------------------------

        self.pid_x = PIDController(

            config.PID_X_KP,

            config.PID_X_KI,

            config.PID_X_KD

        )

        self.pid_y = PIDController(

            config.PID_Y_KP,

            config.PID_Y_KI,

            config.PID_Y_KD

        )

    # ======================================================
    # Safe API
    # ======================================================

    def get_coords_safe(
        self,
        retry=100
    ):

        for i in range(retry):

            coords = self.mc.get_coords()

            print(
                f"get_coords[{i}] =",
                coords
            )

            if isinstance(
                coords,
                (
                    list,
                    tuple
                )
            ):

                return coords

            time.sleep(0.2)

        raise Exception(
            "get_coords failed"
        )

    def get_angles_safe(
        self,
        retry=100
    ):

        for i in range(retry):

            angles = self.mc.get_angles()

            print(
                f"get_angles[{i}] =",
                angles
            )

            if isinstance(
                angles,
                (
                    list,
                    tuple
                )
            ):

                return angles

            time.sleep(0.2)

        raise Exception(
            "get_angles failed"
        )
    
    def coords_range_checker(self, coords):
        radius = math.hypot(coords[0], coords[1])
        if radius > config.ARM_RADIUS:
            raise ValueError(
                f"XY target radius {radius:.1f} mm exceeds {config.ARM_RADIUS} mm"
            )
        if not config.Z_COORD_LOWER <= coords[2] <= config.Z_COORD_UPPER:
            raise ValueError(
                f"Z target {coords[2]:.1f} mm is outside "
                f"{config.Z_COORD_LOWER}..{config.Z_COORD_UPPER} mm"
            )
    # def coodrs_range_checker(self,coords):
    #     if coords[0] > config.X_COORD_UPPER:
    #         print("OUT OF RANGE(X_UPPER)")
    #         return "move_x+"
    #     if coords[0] < config.X_COORD_LOWER:
    #         print("OUT OF RANGE(X_LOWER)")
    #         return "move_x-"
        
    #     if coords[1] > config.Y_COORD_UPPER:
    #         print("OUT OF RANGE(Y_UPPER)")
    #         return "move_y+"
    #     if coords[1] < config.Y_COORD_LOWER:
    #         print("OUT OF RANGE(Y_LOWER)")
    #         return "move_y-"
        
    #     if coords[2] > config.Z_COORD_UPPER:
    #         print("OUT OF RANGE(Z_UPPER)")
    #         return 
    #     if coords[2] < config.Z_COORD_LOWER:
    #         print("OUT OF RANGE(Z_LOWER)")
    #         return 
        
    # def anlges_range_checker(self,angles):
    #     if angles[0] > config.J1_ANGLE_UPPER:
    #         print("OUT OF RANGE(J1_UPPER)")
    #         return 
    #     if angles[0] < config.J1_ANGLE_LOWER:
    #         print("OUT OF RANGE(J1_LOWER)")
    #         return 
        
    #     if angles[1] > config.J2_ANGLE_UPPER:
    #         print("OUT OF RANGE(j2_UPPER)")
    #         return 
    #     if angles[1] < config.J2_ANGLE_LOWER:
    #         print("OUT OF RANGE(j2_LOWER)")
    #         return 
        
    #     if angles[2] > config.J3_ANGLE_UPPER:
    #         print("OUT OF RANGE(j2_UPPER)")
    #         return 
    #     if angles[2] < config.J3_ANGLE_LOWER:
    #         print("OUT OF RANGE(j3_LOWER)")
    #         return 
        
        
        
    # ======================================================
    # Motion
    # ======================================================

    def move_home(self):

        self.mc.sync_send_angles(

            config.HOME_ANGLES,

            config.SEARCH_SPEED

        )

    def move_search_pose(self):

        self.mc.sync_send_coords(

            config.SEARCH_COORDS,

            config.SEARCH_SPEED,

            15

        )

        if os.getenv("SIM_CAMERA") == "1":
            actual = self.get_coords_safe()
            error = math.dist(actual[:3], config.SEARCH_COORDS[:3])
            print(f"Search pose actual: {actual}; position error: {error:.1f} mm", flush=True)
            if error > 25:
                raise RuntimeError(
                    "Search pose was not reached in Gazebo. Check that arm_controller is active "
                    "and inspect docker compose logs sim."
                )

    # ======================================================
    # Camera
    # ======================================================

    def start_camera(self):

        self.camera.start()

    def stop_camera(self):

        self.camera.stop()

# ==========================================================
# arm_controller.py
# Part 3-2
# ==========================================================

    # ======================================================
    # Visual Servo
    # ======================================================

    def visual_servo(
        self,
        target_color,
        target_shape
    ):

        self.pid_x.reset()
        self.pid_y.reset()

        sim_mode = os.getenv("SIM_CAMERA") == "1"
        started = time.monotonic()
        next_scan = started + config.SIM_SEARCH_SCAN_INTERVAL_SEC
        scan_offsets = (-config.SIM_SEARCH_SWEEP_DEG,
                        config.SIM_SEARCH_SWEEP_DEG, 0)
        scan_index = 0
        search_base = self.get_angles_safe()[0] if sim_mode else 0
        last_status = ""
        stalled_moves = 0

        while True:

            if sim_mode and time.monotonic() - started >= config.SIM_SERVO_TIMEOUT_SEC:
                raise RuntimeError("Visual servo did not converge within the simulation time limit")

            target = self.camera.update(

                target_color,

                target_shape

            )

            if target is None:

                if sim_mode:
                    now = time.monotonic()
                    status = self.camera.detection_status
                    if status != last_status:
                        print(f"Searching: {status}", flush=True)
                        last_status = status
                    if now - started >= config.SIM_SEARCH_TIMEOUT_SEC:
                        if self.camera.color_image is not None:
                            import cv2
                            cv2.imwrite("/opt/sim/search_failure.png", self.camera.color_image)
                        raise RuntimeError(
                            f"No {target_color} {target_shape} after "
                            f"{config.SIM_SEARCH_TIMEOUT_SEC}s ({status}). "
                            "Copy /opt/sim/search_failure.png from the container to inspect the view."
                        )
                    if now >= next_scan:
                        angles = self.get_angles_safe()
                        angles[0] = search_base + scan_offsets[scan_index]
                        print(f"Search sweep: base joint to {angles[0]:.1f} degrees", flush=True)
                        self.mc.sync_send_angles(angles, config.SEARCH_SPEED)
                        scan_index = (scan_index + 1) % len(scan_offsets)
                        next_scan = time.monotonic() + config.SIM_SEARCH_SCAN_INTERVAL_SEC

                continue

            error_x = (

                target["cx"]

                -

                config.IMAGE_CENTER_X

            )

            error_y = (

                target["cy"]

                -

                config.IMAGE_CENTER_Y

            )

            print("--------------------------------")

            print("Target")

            print({key: target[key] for key in ("cx", "cy", "angle", "depth", "color", "shape")})

            print()

            print(

                "Error X :",

                error_x

            )

            print(

                "Error Y :",

                error_y

            )

            # --------------------------------------
            # Finish
            # --------------------------------------

            if (

                abs(error_x)

                <=

                config.CENTER_TOLERANCE

                and

                abs(error_y)

                <=

                config.CENTER_TOLERANCE

            ):

                print("Visual Servo Finished")

                return target

            # --------------------------------------
            # PID
            # --------------------------------------

            move_x = self.pid_x.update(

                error_x

            )

            move_y = self.pid_y.update(

                error_y

            )

            if sim_mode:
                limit = config.SIM_SERVO_MAX_STEP_MM
                move_x = max(-limit, min(limit, move_x))
                move_y = max(-limit, min(limit, move_y))

            coords = self.get_coords_safe()

            # --------------------------------------
            # Camera Coordinate
            #
            # Image
            #
            # +X →
            # +Y ↓
            #
            # Robot
            #
            # +X Forward
            # +Y Left
            #
            # Image Down
            #      ↓
            # Robot +X
            #
            # Image Right
            #      →
            # Robot -Y
            # --------------------------------------
            if coords[2] <= 110:
                coords[2] = 150
            target_coords = [

                coords[0] - move_y,

                coords[1] - move_x,

                coords[2],

                coords[3]

            ]

            self.coords_range_checker(target_coords)
            print()

            print("Before")

            print(coords)

            print()

            print("After")

            print(target_coords)

            self.mc.sync_send_coords(

                target_coords,

                config.MOVE_SPEED,

                15

            )
            if sim_mode:
                time.sleep(0.8)
                actual = self.get_coords_safe()
                moved = math.dist(actual[:3], coords[:3])
                stalled_moves = stalled_moves + 1 if moved < config.SIM_SERVO_STALL_MM else 0
                if stalled_moves >= config.SIM_SERVO_STALL_LIMIT:
                    raise RuntimeError(
                        f"Visual servo requested motion but Gazebo did not move "
                        f"(last request {target_coords}, actual {actual})."
                    )
            else:
                time.sleep(0.05)

# ==========================================================
# arm_controller.py
# Part 3-3
# ==========================================================

    # ======================================================
    # Camera -> Gripper Alignment
    # ======================================================

    def align_gripper(self):

        angles = self.get_angles_safe()

        depth = self.camera.get_center_depth()

        # --------------------------------------
        # Camera pitch compensation
        # --------------------------------------

        pitch_offset = (

            depth *

            math.tan(

                config.CAMERA_PITCH_RAD

            )

        )

        camera_x = (

            config.CAMERA_OFFSET_X

            -

            pitch_offset

        )

        camera_y = config.CAMERA_OFFSET_Y

        joint1 = math.radians(

            angles[0]

        )

        offset_x = (

            camera_x *

            math.cos(joint1)

            -

            camera_y *

            math.sin(joint1)

        )

        offset_y = (

            camera_x *

            math.sin(joint1)

            +

            camera_y *

            math.cos(joint1)

        )
        print("depth =", depth)
        print("pitch_offset =", pitch_offset)
        print("camera_x =", camera_x)
        print("camera_y =", camera_y)
        print("offset_x =", offset_x)
        print("offset_y =", offset_y)
        coords = self.get_coords_safe()

        

        
        target = [

            coords[0] + offset_x,

            coords[1] + offset_y,

            coords[2],

            coords[3]

        ]
        self.coords_range_checker(target)
        print()

        print("Camera Alignment")

        print(target)

        self.mc.sync_send_coords(

            target,

            config.SEARCH_SPEED,

            15

        )

    # ======================================================
    # Rotate Gripper
    # ======================================================

    def rotate_gripper(
        self,
        lego_angle
    ):

        angles = self.get_angles_safe()

        angles[3] = lego_angle + config.GRIPPER_ANGLE_OFFSET

        print()

        print("Rotate")

        print(lego_angle)

        self.mc.sync_send_angles(

            angles,

            config.MOVE_SPEED

        )

    # ======================================================
    # Open Gripper
    # ======================================================

    def open_gripper(self):

        self.mc.set_gripper_state(

            config.GRIPPER_OPEN,

            config.GRIPPER_SPEED

        )

        time.sleep(1.5)

    # ======================================================
    # Close Gripper
    # ======================================================

    def close_gripper(self):

        self.mc.set_gripper_value(

            config.GRIPPER_CLOSE,

            config.GRIPPER_SPEED

        )

        time.sleep(2)
# ==========================================================
# arm_controller.py
# Part 3-4 (Final)
# ==========================================================

    # ======================================================
    # Descend
    # ======================================================

    def descend_and_grasp(self):

        if config.USE_DEPTH:

            depth = self.camera.get_center_depth()

        else:

            depth = config.DEFAULT_DEPTH

        tool_offset = (
            config.SIM_TOOL_OFFSET if os.getenv("SIM_CAMERA") == "1"
            else config.TOOL_OFFSET
        )

        descend = (

            depth

            -

            config.LEGO_HEIGHT

            -

            tool_offset

        )

        coords = self.get_coords_safe()

        target = [

            coords[0],

            coords[1],

            coords[2] - descend,

            coords[3]

        ]

        print()

        print("Descend")

        print(target)

        if descend <= 0:
            raise ValueError(
                f"Camera depth {depth:.1f} mm is too short for tool offset "
                f"{tool_offset:.1f} mm; descent would move upward"
            )
        self.coords_range_checker(target)

        # 開く
        self.open_gripper()

        self.mc.sync_send_coords(

            target,

            config.MOVE_SPEED,

            15

        )

        time.sleep(1)

        # 掴む
        self.close_gripper()

    # ======================================================
    # Lift
    # ======================================================

    def lift(self):

        coords = self.get_coords_safe()

        target = [

            coords[0],

            coords[1],

            coords[2] + 100,

            coords[3]

        ]
        self.coords_range_checker(target)
        print()

        print("Lift")

        print(target)

        self.mc.sync_send_coords(

            target,

            config.SEARCH_SPEED,

            15

        )

    # ======================================================
    # Pick Sequence
    # ======================================================

    def pick(

        self,

        target_color,

        target_shape

    ):

        print()

        print("==============================")

        print("START PICK")

        print("==============================")

        target = self.visual_servo(

            target_color,

            target_shape

        )

        print()

        print("Align Gripper")
        

        self.align_gripper()

        print()

        print("Rotate Gripper")

        self.rotate_gripper(

            target["angle"]

        )

        print()

        print("Descend")

        self.descend_and_grasp()

        print()

        print("Lift")

        self.lift()

        print()

        print("==============================")

        print("FINISH PICK")

        print("==============================")

        return target

    # ======================================================
    # Shutdown
    # ======================================================

    def shutdown(self):

        try:

            self.camera.stop()

        except:

            pass

        try:
            self.mc.sync_send_coords([150,0,80,0],10,15)
            time.sleep(1)
            # self.mc.release_all_servos()
            self.mc.set_gripper_state(10,100)
            

        except:

            pass

        print("Shutdown Complete")
        
