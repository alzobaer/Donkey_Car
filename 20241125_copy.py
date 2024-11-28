# Standard library imports
# import datetime
import os
import time
from math import degrees
# from datetime import datetime
# now = datetime.now()

# Third-party library imports
import cv2
import numpy as np
import rospy
from cv2 import aruco
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import euler_from_quaternion

# Local module imports
#from IlluminanceSensor import IlluminanceSensorS
from RealSense import RealSense

# Global variables
# global GLOBAL_YAW_DEGREES
global GLOBAL_YAW_DEGREES
GLOBAL_YAW_DEGREES = 0

MARKER_SIZE_ACTUAL = 0.07            # Actual side length of the marker in meters
X3, Z3             = 400000, 400000  # Coordinates for visualizing near the charging station
X4, Z4             = 400000, 400000

# Directory and file settings for saving images in catkin_ws/src/test/src
OUTPUT_DIRECTORY   = "/home/zobaerpi/catkin_ws/src/test/src/picture"
FILE_PREFIX        = "captured_image_"   # Prefix for saved image files
OUTPUT_GRAY_IMAGE  = "/home/zobaerpi/catkin_ws/src/test/src/gray_image"
FILE_PREFIX_GRAY   = "gray_image_"       # Prefix for saved gray image files

# Constants
DISTANCE_THRESHOLD_MARKER_1    = 80  # Distance threshold for Marker 1 (in cm)
DISTANCE_THRESHOLD_MARKER_3    = 300 # Distance threshold for Marker 3 (in cm)
DISTANCE_THRESHOLD_MARKER_4    = 300 # Distance threshold for Marker 4 (in cm)
DISTANCE_THRESHOLD_MARKER_5    = 80  # Distance threshold for Marker 5 (in cm)
TURN_ANGLE_MARKER_1            = 80  # Turn angle for Marker 1 (in degrees)
TURN_ANGLE_MARKER_2            = -80  # Turn angle for Marker 1 (in degrees)
TURN_ANGLE_MARKER_5            = 170 # Turn angle for Marker 5 (in degrees)
RIGHT_TURN_DURATION            = 0.8 # Duration of right turn for adjusting visibility (in seconds)
LEFT_TURN_DURATION             = 0.8 # Duration of left turn for adjusting visibility (in seconds)
BACKWARD_DURATION              = 0.3 # Duration of backward movement (in seconds)
BACKWARD_DURATION_A_SECOND     = 1.0 # Duration of backward movement (in seconds)
WALL_DETECTION_DELAY           = 0.2 # Delay for wall detection operations (in seconds)

EXPECTED_DATA_LENGTH = 2  # Expected number of items (frame, depth) from camera data

UNDETECTABLE_DISTANCE      = 40000 # Distance value set when marker is not detected
ANGLE_ADJUSTMENT_THRESHOLD = 0.5   # Threshold for left/right angle adjustment
ANGLE_ADJUSTMENT_DURATION  = 0.3   # Duration for angle adjustment in seconds

PADDING             = 10   # Margin for angle difference when checking turn completion
LEFT_TURN_SPEED     = 1300 # Motor speed for left turn
RIGHT_TURN_SPEED    = 1700 # Motor speed for right turn
TURN_SLEEP_DURATION = 0.3  # Delay between turn updates in seconds

RIGHT_TURN_SPEED          = 1700 # Motor speed for right turn
TURN_SLEEP_DURATION       = 0.1  # Delay between publishing commands during turning
STOP_DURATION_BEFORE_TURN = 0.5  # Duration to stop before starting the turn
STOP_DURATION_AFTER_TURN  = 0.3  # Duration to stop after finishing the turn
FINAL_STOP_DURATION       = 0.5  # Final delay after completing the turn

BACKWARD_DURATION      = 0.5   # Duration for backward movement in seconds
LEFT_TURN_DURATION     = 0.5   # Duration for left turn in seconds
RIGHT_TURN_DURATION    = 0.5   # Duration for right turn in seconds
MAX_SEARCH_ATTEMPTS    = 3     # Maximum attempts before going backward if marker is not found
DISTANCE_THRESHOLD_MIN = 300   # Minimum distance to marker in mm to stop (converted to mm from cm)
DISTANCE_THRESHOLD_MAX = 500   # Maximum distance to marker in mm to stop (converted to mm from cm)
UNDETECTABLE_DISTANCE  = 40000 # Placeholder value when distance cannot be detected

DEPTH_THRESHOLD_NEAR          = 300  # Threshold for near obstacles in mm
DEPTH_THRESHOLD_FAR           = 1000 # Threshold for obstacles requiring a slight turn in mm
DEPTH_THRESHOLD_HARD          = 250  # Threshold for obstacles requiring a significant turn in mm
DEPTH_THRESHOLD_BOTTOM_CENTER = 200  # Threshold for bottom-center obstacles (e.g., ledges) in mm
BACKWARD_DURATION             = 1.5  # Duration for backward movement in seconds
TURN_DURATION_SLIGHT          = 0.5  # Duration for slight turns in seconds
TURN_DURATION_LARGE           = 1.0  # Duration for large turns in seconds
FORWARD_SPEED                 = 1500 # Forward movement speed
REVERSE_SPEED                 = 1850 # Reverse movement speed
CAPTURE_INTERVAL              = 0.5  # Time delay between capture intervals

LEFT_TURN_SPEED           = 1300     # Motor speed for left turn
TURN_STOP_DURATION_BEFORE = 0.5      # Duration to stop before starting the turn
TURN_STOP_DURATION_AFTER  = 0.3      # Duration to stop after finishing the turn
TURN_PUBLISH_INTERVAL     = 0.1      # Time interval between publishing commands during turn
FINAL_STOP_DURATION       = 1.0      # Final delay after completing the turn

STOP_DISTANCE_MARKER_0       = 185   # Stopping distance threshold for marker ID 0 in mm
STOP_DISTANCE_MARKER_19      = 1000  # Stopping distance threshold for marker ID 19 in mm
CENTER_LEFT_THRESHOLD        = 340   # Threshold for slight left adjustment
CENTER_RIGHT_THRESHOLD       = 390   # Threshold for slight right adjustment
CHARGE_ADJUST_DURATION_RIGHT = 0.1   # Duration for minor right adjustment
CHARGE_ADJUST_DURATION_LEFT  = 0.3   # Duration for minor left adjustment
FORWARD_DURATION             = 0.5   # Duration for forward movement
STOP_DURATION                = 0.1   # Duration for stop between adjustments
THRESHOLD_ILLUMINANCE        = 30    # Threshold for detecting charge completion

LOOP_RATE_HZ           = 100  # Frequency of the loop (100 Hz)
SERVO_CHANNEL_FORWARD  = 1    # Forward channel number for servo control
SERVO_CHANNEL_REVERSE  = 3    # Reverse channel number for servo control
FORWARD_VALUE          = 1800 # Servo value for forward movement
SLOW_FORWARD_VALUE     = 1700 # Servo value for slow forward movement
REVERSE_VALUE          = 1500 # Servo value for neutral (stop)
BACK_VALUE             = 1200 # Servo value for backward movement
LEFT_TURN_VALUE        = 1200 # Servo value for 90-degree left turn
SLOW_LEFT_TURN_VALUE   = 1300 # Servo value for slow left turn
RIGHT_TURN_VALUE       = 1800 # Servo value for 90-degree right turnS
SLOW_RIGHT_TURN_VALUE  = 1700 # Servo value for slow right turn
MIN_DISTANCE_TO_MARKER = 500  # Minimum distance to the marker in mm

YAW_OFFSET_DEGREES = 180   # Offset to adjust yaw angle into the 0-360 range

PUBLISH_INTERVAL_BACK    = 0.1  # Interval between each publish in seconds
FINAL_STOP_DURATION_BACK = 0.5  # Duration to pause after the backward movement

STOP_PUBLISH_INTERVAL    = 0.05  # Interval between each publish in seconds

RIGHT_MOTOR_ADJUST_SPEED     = 1650  # Speed for adjusting only the right motor
ADJUST_PUBLISH_INTERVAL      = 0.1   # Interval between each publish in seconds
FINAL_STOP_DURATION_ADJUST   = 0.5   # Duration to pause after adjustment

LEFT_MOTOR_ADJUST_SPEED      = 1650  # Speed for adjusting only the left motor
ADJUST_PUBLISH_INTERVAL      = 0.1   # Interval between each publish in seconds
FINAL_STOP_DURATION_ADJUST   = 0.5   # Duration to pause after adjustment

DEFAULT_DISTANCE_X     = 40000         # Default x value when marker is not detected
DEFAULT_DISTANCE_Z     = 40000         # Default z value when marker is not detected
MARKER_SIZE_CM         = 7.0           # Marker size in centimeters
CONVERSION_TO_CM       = 100           # Conversion factor from meters to centimeters
Z_CORRECTION_SLOPE     = 0.89081077    # Correction slope for z-axis distance
Z_CORRECTION_INTERCEPT = 3.89294886    # Correction intercept for z-axis distance

BACK_DURATION            = 0.2  # Duration for backward movement in seconds when marker is not detected
MARKER_STOP_INTERVAL     = 0.1  # Interval between each marker check in seconds
IMAGE_CENTER_LEFT_RATIO  = 3/8  # Ratio to calculate left center of the image
IMAGE_CENTER_RIGHT_RATIO = 5/8  # Ratio to calculate right center of the image
MARKER_ID_TO_STOP        = 0    # Marker ID to look for in stop routine

IMAGE_WIDTH               = 640
IMAGE_HEIGHT              = 300
CENTER_BOTTOM_Y           = 250
BLUE_POINT_POSITION       = (320, 275)
BACKGROUND_COLOR          = (192, 192, 192)  # Gray background color
POINT_COLOR_BLUE          = (255, 0, 0)      # Blue for center-bottom point
POINT_COLOR_GREEN         = (0, 255, 0)      # Green for another reference point
LINE_COLOR_LIGHT_BLUE     = (255, 255, 0)    # Light blue for lines
THETA_TEXT_POSITION       = (50, 250)
THETA_FONT_SCALE          = 1.0
THETA_TEXT_COLOR          = (255, 255, 0)
THETA_TEXT_THICKNESS      = 2
OUTPUT_DIRECTORY_GRAY     = OUTPUT_GRAY_IMAGE  # Output directory for gray images

DEFAULT_ORTHOGONAL_VECTOR = np.array([1, 0, 0])  # Default orthogonal vector if AB is zero

DEPTH_THRESHOLD_NEAR      = 600   # Threshold for detecting near objects in upper and center regions (in mm)
DEPTH_THRESHOLD_FAR       = 1000  # Threshold for detecting safe distance in upper and center regions (in mm)
DEPTH_THRESHOLD_BOTTOM    = 200   # Threshold for detecting near objects in bottom region (in mm)
SAFE_DEPTH_BOTTOM         = 200   # Safe depth threshold for bottom regions (in mm)

IMAGE_WIDTH     = 640  # Width of the image in pixels
IMAGE_HEIGHT    = 300  # Height of the image in pixels
IMAGE_OFFSET_Y  = 50   # Offset to adjust y-axis origin
POINT_RADIUS    = 5    # Radius of the point to draw
COLOR_RED       = (0, 0, 255)  # Color for the red point in BGR format

IMAGE_WIDTH      = 640             # Width of the image in pixels
IMAGE_MAX_Y      = 250             # Maximum y-coordinate to fit within display area
POINT_RADIUS     = 5               # Radius of the point to draw
COLOR_LIGHT_BLUE = (255, 255, 0)   # Light blue color in BGR format

RIGHT_FORWARD_MOTOR_SPEED_LEFT  = 1650  # Speed for left motor when moving forward right
RIGHT_FORWARD_MOTOR_SPEED_RIGHT = 1700  # Speed for right motor when moving forward right
ADJUSTMENT_DURATION             = 0.5   # Duration for forward adjustment in seconds

LEFT_FORWARD_MOTOR_SPEED_LEFT  = 1350  # Speed for left motor when moving forward left
LEFT_FORWARD_MOTOR_SPEED_RIGHT = 1700  # Speed for right motor when moving forward left
ADJUSTMENT_DURATION            = 0.5   # Duration for forward adjustment in seconds

DEPTH_THRESHOLD_VERY_CLOSE       = 100  # Threshold for very close obstacles in upper regions (in mm)
DEPTH_THRESHOLD_OBSTACLE_BOTTOM  = 70   # Threshold for obstacles in bottom regions (in mm)
DEPTH_THRESHOLD_OBSTACLE_MIN     = 40   # Minimum threshold for obstacles in bottom regions (in mm)

IMAGE_WIDTH        = 640               # Width of the image in pixels
IMAGE_HEIGHT       = 300               # Height of the image in pixels
IMAGE_CENTER_Y     = 250               # Y-coordinate for the bottom center position
COLOR_GRAY         = (192, 192, 192)   # Background gray color in BGR
COLOR_BLUE         = (255, 0, 0)       # Blue color for the bottom center point
COLOR_GREEN        = (0, 255, 0)       # Green color for the secondary reference point
FILE_PREFIX_GRAY   = "gray_image_"     # Prefix for saved gray images
OUTPUT_GRAY_IMAGE  = "/home/zobaerpi/catkin_ws/src/test/src/gray_image"  # Directory to save gray images

RIGHT_TURN_SPEED          = 1650  # Motor speed for right turn
REVERSE_SPEED             = 1500  # Neutral speed to stop motors
INITIAL_STOP_DURATION     = 0.5   # Initial stop duration before turning (in seconds)
TURN_SLEEP_DURATION       = 0.1   # Delay between publishing commands during turn (in seconds)
FINAL_STOP_DURATION       = 0.3   # Duration to stop after completing the turn
POST_TURN_STOP_DURATION   = 0.5   # Final delay after the entire turn

FORWARD_SPEED       = 1450  # Motor speed for forward movement
NEUTRAL_SPEED       = 1500  # Neutral speed to stop motors
REVERSE_SPEED       = 1700  # Speed for reverse movement
POST_MOVE_STOP      = 0.2   # Duration to stop after movement (in seconds)
MOVE_SLEEP_DURATION = 0.1   # Delay between publishing commands during movement (in seconds)
FINAL_STOP_DURATION = 0.2   # Final delay after completing the movement (in seconds)

LEFT_TURN_SPEED        = 1300  # Speed for left turn
RIGHT_TURN_SPEED       = 1700  # Speed for right turn
NEUTRAL_SPEED          = REVERSE_VALUE  # Neutral speed for stopping the vehicle
TURN_PADDING           = 10    # Padding for checking turn completion
TURN_SLEEP_DURATION    = 0.3  # Delay between motor updates during turns
STOP_DURATION          = 0.5  # Duration to stop after the turn
FORWARD_SPEED          = 1900  # Forward speed for longer distance movements
TURN_CHECK_DELAY       = 0.5  # Delay between turning and checking for markers

DISTANCE_THRESHOLD_1      = 65   # Threshold for greater distance than 65cm
DISTANCE_THRESHOLD_2      = 55   # Threshold for 55cm <= distance <= 65cm
MOVEMENT_SCALING_FACTOR_1 = 0.5  # Scaling factor for movement calculation
MOVEMENT_SCALING_FACTOR_2 = 0.5  # Scaling factor for movement calculation in 50-60cm range
MOVEMENT_MIN_TIME         = 1.0  # Minimum movement time for distances close to the target

FORWARD_1900_SPEED  = 1475 # Motor speed for forward movement in the forward_1900 function
REVERSE_SPEED       = 1500 # Neutral speed to stop motors
MOVE_SLEEP_DURATION = 0.1  # Delay between publishing commands during movement (in seconds)
STOP_DURATION       = 0.1  # Duration to stop after movement (in seconds)
FINAL_STOP_DURATION = 0.1 # Final stop duration after the move

LEFT_TURN_DURATION  = 0.5  # Duration for left turn adjustments (in seconds)
RIGHT_TURN_DURATION = 0.5  # Duration for right turn adjustments (in seconds)
TURN_RETRY_DELAY    = 0.5  # Delay between retries in case no marker is found

# Initialize instances for RealSense and IlluminanceSensor
camera             = RealSense()
# illuminance_sensor = IlluminanceSensor()

# Initialize the ArUco marker detector
DICTIONARY         = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
PARAMETERS         = aruco.DetectorParameters()
DETECTOR           = aruco.ArucoDetector(DICTIONARY, PARAMETERS)

# Initialize ROS node
rospy.init_node('forward_and_turn')

def imu_callback(data):
    global GLOBAL_YAW_DEGREES
    GLOBAL_YAW_DEGREES = 0

    # Calculate yaw angle from geomagnetic data
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    # Convert radians to degrees
    GLOBAL_YAW_DEGREES = degrees(yaw)
    GLOBAL_YAW_DEGREES += 180
    print(GLOBAL_YAW_DEGREES)

# Publishers and Subscribers
pub     = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
imu_pub = rospy.Publisher('/mavros/imu/data', Imu, queue_size=10)
mag_pub = rospy.Publisher('/mavros/imu/mag', MagneticField, queue_size=10)
ori_sub = rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)

# Message Objects
msg     = OverrideRCIn()
imu_msg = Imu()
mag_msg = MagneticField()

# Set loop rate
rate = rospy.Rate(LOOP_RATE_HZ)

# Record start time
start_time = time.time()

# Function to stop the vehicle for a specified duration
def stop(duration):
    """
    Stops the vehicle for a specified duration by publishing neutral values to the channels.
    """
    start_time = time.time()
    # Create a message with stop/neutral values for the channels
    msg = OverrideRCIn()
    msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_VALUE
    msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
    # Publish the stop message for the specified duration
    while time.time() - start_time < duration:
        pub.publish(msg)
        time.sleep(STOP_PUBLISH_INTERVAL)

# Function to move the vehicle backward for a specified duration
def back(turn_time):
    """
    Moves the vehicle backward for a specified duration.
    """
    # Set backward movement values for the channels
    msg = OverrideRCIn()
    msg.channels[0] = REVERSE_VALUE
    msg.channels[2] = BACK_VALUE
    # Start backward movement
    start_time = time.time()
    while time.time() - start_time < turn_time:
        pub.publish(msg)
        time.sleep(PUBLISH_INTERVAL_BACK)
    # Pause briefly after completing the backward movement
    time.sleep(FINAL_STOP_DURATION_BACK)

# Callback function to update the vehicle's current angle
# def imu_callback(data):
#     """
#     Callback function to retrieve the vehicle's orientation in degrees.
#     """
#     global GLOBAL_YAW_DEGREES
#     # Extract orientation from IMU data and convert quaternion to euler angles
#     orientation_q = data.orientation
#     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#     (_, _, yaw) = euler_from_quaternion(orientation_list)
#     # Convert yaw from radians to degrees and adjust with offset
#     GLOBAL_YAW_DEGREES = degrees(yaw) + YAW_OFFSET_DEGREES
#     # Ensure yaw angle is within the 0-360 range
#     GLOBAL_YAW_DEGREES %= 360

# Function to create a grayscale image with marked points
def _create_grayscale_image_2(x1, z1):
    """
    Generates a grayscale image with specific points marked for visualization.
    Adjusts the x-coordinate and draws red, blue, and green points on the image.
    Saves the image with a timestamped filename in the specified directory.
    """
    # Create gray background image
    gray_image = np.full((IMAGE_HEIGHT, IMAGE_WIDTH, 3), COLOR_GRAY, dtype=np.uint8)
    # Calculate bottom center position for blue point
    center_bottom = (IMAGE_WIDTH // 2, IMAGE_CENTER_Y)
    # Draw the blue and green reference points
    cv2.circle(gray_image, center_bottom, 5, COLOR_BLUE, -1)  # Draw blue circle
    cv2.circle(gray_image, (320, 275), 5, COLOR_GREEN, -1)    # Draw green circle as secondary reference
    # Adjust x1 to image scale and draw red point if within bounds
    if -1 <= x1 <= 1:
        x1_scaled = int(x1 * 320 + 320)  # Scale x1 to image dimensions
        _draw_red_point(gray_image, x1_scaled, int(z1))  # Draw red point at (x1, z1)
    # Get current timestamp for filename
    # timestamp = now().strftime('%Y%m%d_%H%M%S')
    filename = f"{FILE_PREFIX_GRAY}.jpg"
    filepath = os.path.join(OUTPUT_GRAY_IMAGE, filename)
    # Save the image
    cv2.imwrite(filepath, gray_image)
    return 0

# Function to turn the vehicle slightly to the right for charging alignment
def _charge_turn_right(turn_time):
    """
    Slightly rotates the vehicle to the right by controlling motor channels.
    Includes initial and post-turn stops to stabilize the movement.
    """
    # Initial stop before starting the turn
    msg = OverrideRCIn()
    msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_SPEED
    msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_SPEED
    pub.publish(msg)
    time.sleep(INITIAL_STOP_DURATION)
    # Right turn for specified duration
    start_time = time.time()
    while time.time() - start_time < turn_time:
        msg.channels[SERVO_CHANNEL_FORWARD - 1] = RIGHT_TURN_SPEED
        msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_SPEED
        pub.publish(msg)
        time.sleep(TURN_SLEEP_DURATION)
    # Stop briefly after completing the turn
    start_time = time.time()
    while time.time() - start_time < FINAL_STOP_DURATION:
        msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_SPEED
        msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_SPEED
        pub.publish(msg)
        time.sleep(TURN_SLEEP_DURATION)
    # Final post-turn delay
    time.sleep(POST_TURN_STOP_DURATION)

# Function to move the vehicle forward for a specified time
def _forward(turn_time):
    """
    Moves the vehicle forward for the specified duration by controlling motor channels.
    After the movement, stops the vehicle briefly before returning.
    """
    # Move forward for the specified time
    start_time = time.time()
    while time.time() - start_time < turn_time:
        msg = OverrideRCIn()
        msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_SPEED
        msg.channels[SERVO_CHANNEL_REVERSE - 1] = FORWARD_SPEED
        pub.publish(msg)
        time.sleep(MOVE_SLEEP_DURATION)
    # Stop briefly after moving forward
    start_time = time.time()
    while time.time() - start_time < POST_MOVE_STOP:
        msg.channels[SERVO_CHANNEL_FORWARD - 1] = NEUTRAL_SPEED
        msg.channels[SERVO_CHANNEL_REVERSE - 1] = NEUTRAL_SPEED
        pub.publish(msg)
        time.sleep(MOVE_SLEEP_DURATION)
    # Final stop delay
    time.sleep(FINAL_STOP_DURATION)

# Function to stop at marker and adjust alignment based on marker position
def marker_stop(msg, frame, depth, corners, list_ids, marker_id, image_right_center_x, image_left_center_x, marker_center_x):
    """
    Stops the vehicle at the specified marker and adjusts alignment based on the marker position.
    """
    try:
        # Find the index of the specified marker ID and get its corners
        marker_index   = np.where(list_ids == marker_id)[0][0]
        marker_corners = corners[marker_index][0]
        capture_image(frame)
        # Calculate the center of the marker
        marker_center = np.mean(marker_corners, axis=0)
        # Get marker distance
        _, marker0_z  = _distances(0)
        _create_grayscale_image_2(marker_center_x, marker0_z)
        if not np.isnan(marker0_z):
            marker_distance = marker0_z * 10  # Convert to mm
            # Stop if the marker distance is below the defined thresholds
            if marker_id == 0 and marker_distance < 170:
                msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_VALUE
                msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
                pub.publish(msg)
                time.sleep(4.0)
                # Check for charge completion based on illuminance
                if illuminance_sensor.is_above_threshold(threshold=THRESHOLD_ILLUMINANCE):
                    raise ValueError("Charging complete confirmed.")
                else:
                    print("Charging complete not confirmed. Continuing operation.")
                    back(5.0)
                    return
            elif marker_id == 19 and marker_distance < STOP_DISTANCE_MARKER_19:
                msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_VALUE
                msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
                pub.publish(msg)
                exit()
            # Alignment adjustments based on marker center position
            if marker_center_x > image_right_center_x:
                cv2.circle(frame, (int(marker_center[0]), int(marker_center[1])), 10, (0, 0, 255), -1)
                cv2.line(frame, (int(image_right_center_x), 0), (int(image_right_center_x), frame.shape[0]), (0, 0, 255), 2)
                capture_image(frame)
                _create_grayscale_image_2(int(marker_center_x), int(marker0_z))
                _charge_turn_right(CHARGE_ADJUST_DURATION_RIGHT)
                _forward(FORWARD_DURATION)
            elif marker_center_x < image_left_center_x:
                cv2.circle(frame, (int(marker_center[0]), int(marker_center[1])), 10, (0, 0, 255), -1)
                cv2.line(frame, (int(image_left_center_x), 0), (int(image_left_center_x), frame.shape[0]), (0, 0, 255), 2)
                capture_image(frame)
                _create_grayscale_image_2(int(marker_center_x), int(marker0_z))
                turn_left(CHARGE_ADJUST_DURATION_LEFT)
                _forward(FORWARD_DURATION)
            elif marker_center_x < CENTER_LEFT_THRESHOLD:
                cv2.circle(frame, (int(marker_center[0]), int(marker_center[1])), 10, (0, 0, 255), -1)
                cv2.line(frame, (CENTER_LEFT_THRESHOLD, 0), (CENTER_LEFT_THRESHOLD, frame.shape[0]), (0, 0, 255), 2)
                capture_image(frame)
                _create_grayscale_image_2(int(marker_center_x), int(marker0_z))
                turn_left(CHARGE_ADJUST_DURATION_LEFT)
                _forward(FORWARD_DURATION)
            elif marker_center_x > CENTER_RIGHT_THRESHOLD:
                cv2.circle(frame, (int(marker_center[0]), int(marker_center[1])), 10, (0, 0, 255), -1)
                cv2.line(frame, (CENTER_RIGHT_THRESHOLD, 0), (CENTER_RIGHT_THRESHOLD, frame.shape[0]), (0, 0, 255), 2)
                capture_image(frame)
                _create_grayscale_image_2(int(marker_center_x), int(marker0_z))
                _charge_turn_right(CHARGE_ADJUST_DURATION_RIGHT)
                _forward(FORWARD_DURATION)
            else:
                msg = OverrideRCIn()
                msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_VALUE
                msg.channels[SERVO_CHANNEL_REVERSE - 1] = FORWARD_VALUE
                pub.publish(msg)
                time.sleep(FORWARD_DURATION)
            stop(STOP_DURATION)
    except IndexError:
        print(f"Marker ID {marker_id} not found.")
        return None, None

# Function to calculate the center coordinates of a specified marker
def calc_marker_center(frame, corners, list_ids, marker_id):
    """
    Calculates the center coordinates of the specified marker.
    """
    try:
        # Get the index of the specified marker ID
        marker_index = np.where(list_ids == marker_id)[0][0]
        marker_corners = corners[marker_index][0]
        # Calculate the center coordinates of the marker
        marker_center_x = np.mean(marker_corners[:, 0])
        marker_center_y = np.mean(marker_corners[:, 1])
        return marker_center_x, marker_center_y
    except IndexError:
        print(f"Marker ID {marker_id} not found.")
        return None, None

# Function to turn the vehicle to the left for a specified duration
def turn_left(turn_time):
    """
    Turns the vehicle to the left for the specified duration.
    """
    # Initial stop before starting the left turn
    msg = OverrideRCIn()
    msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_VALUE
    msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
    pub.publish(msg)
    time.sleep(TURN_STOP_DURATION_BEFORE)
    # Start left turn for the specified turn_time
    start_time = time.time()
    while time.time() - start_time < turn_time:
        msg = OverrideRCIn()
        # msg.channels[SERVO_CHANNEL_FORWARD - 1] = LEFT_TURN_SPEED
        msg.channels[SERVO_CHANNEL_FORWARD - 1] = 1150
        msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
        pub.publish(msg)
        # time.sleep(TURN_PUBLISH_INTERVAL)
        time.sleep(0.01)
    # Brief stop after completing the left turn
    start_time = time.time()
    while time.time() - start_time < TURN_STOP_DURATION_AFTER:
        msg = OverrideRCIn()
        msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_VALUE
        msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
        pub.publish(msg)
        time.sleep(TURN_PUBLISH_INTERVAL)
    # Final delay to ensure smooth transition
    time.sleep(FINAL_STOP_DURATION)

# Function to adjust vehicle angle by moving forward to the left
def _adjust_angle_forward_left():
    """
    Adjusts the vehicle's angle by moving it forward to the left.
    Sets the left and right motor speeds to turn the vehicle slightly to the left.

    This function is useful for fine-tuning the vehicle's angle to avoid obstacles
    or adjust alignment based on sensor input.
    """
    print("Adjusting angle by moving forward to the left...")
    msg = OverrideRCIn()
    msg.channels[0] = LEFT_FORWARD_MOTOR_SPEED_LEFT
    msg.channels[2] = LEFT_FORWARD_MOTOR_SPEED_RIGHT
    pub.publish(msg)
    time.sleep(ADJUSTMENT_DURATION)

# Function to adjust vehicle angle by moving forward to the right
def _adjust_angle_forward_right():
    """
    Adjusts the vehicle's angle by moving it forward to the right.
    Sets the left and right motor speeds to turn the vehicle slightly to the right.

    This function is useful for fine-tuning the vehicle's angle to avoid obstacles
    or adjust alignment based on sensor input.
    """
    print("Adjusting angle by moving forward to the right...")
    msg = OverrideRCIn()
    msg.channels[0] = RIGHT_FORWARD_MOTOR_SPEED_LEFT
    msg.channels[2] = RIGHT_FORWARD_MOTOR_SPEED_RIGHT
    pub.publish(msg)
    time.sleep(ADJUSTMENT_DURATION)

# Safety confirmation function to detect obstacles and adjust movement accordingly
def _confirm_safe(frame, depth):
    """
    Analyzes depth data from specific regions of the camera frame to detect obstacles
    and adjusts the vehicle's position if obstacles are detected within set thresholds.

    Draws rectangles and labels on the frame when obstacles are detected in the left or right regions.
    The function also initiates movement adjustments to avoid collisions.
    """
    # Extract depth information from defined regions of the image
    depth_top_left_hard     = depth[:int(frame.shape[0] / 4), int(frame.shape[1] / 13 * 2):int(frame.shape[1] / 13 * 3)]
    depth_top_right_hard    = depth[:int(frame.shape[0] / 4), int(frame.shape[1] / 13 * 8):int(frame.shape[1] / 13 * 11)]
    depth_top_center        = depth[:int(frame.shape[0] / 4), int(frame.shape[1] / 3):int(frame.shape[1] / 3 * 2)]
    depth_bottom_left_hard  = depth[int(frame.shape[0] / 4 * 3):, int(frame.shape[1] / 13 * 2):int(frame.shape[1] / 13 * 5)]
    depth_bottom_right_hard = depth[int(frame.shape[0] / 4 * 3):, int(frame.shape[1] / 13 * 8):int(frame.shape[1] / 13 * 11)]
    depth_bottom_center     = depth[int(frame.shape[0] / 4 * 3):, int(frame.shape[1] / 5 * 2):int(frame.shape[1] / 3 * 5)]

    # Calculate average depth for each region
    avg_depth_top_left_hard     = np.mean(depth_top_left_hard)
    avg_depth_top_right_hard    = np.mean(depth_top_right_hard)
    avg_depth_top_center        = np.mean(depth_top_center)
    avg_depth_bottom_left_hard  = np.mean(depth_bottom_left_hard)
    avg_depth_bottom_right_hard = np.mean(depth_bottom_right_hard)
    avg_depth_bottom_center     = np.mean(depth_bottom_center)

    # Check for obstacles in the top-center region and adjust accordingly
    if avg_depth_top_center != 0 and avg_depth_top_center < DEPTH_THRESHOLD_VERY_CLOSE:
        back(1.5)
        turn_left(1.0)

    # Check for obstacles in the bottom-center region and adjust accordingly
    elif DEPTH_THRESHOLD_OBSTACLE_MIN <= avg_depth_bottom_center < DEPTH_THRESHOLD_OBSTACLE_BOTTOM:
        back(1.5)
        turn_left(1.5)

    # Check for obstacles in the left regions and adjust angle
    if avg_depth_top_left_hard < DEPTH_THRESHOLD_VERY_CLOSE or avg_depth_bottom_left_hard < DEPTH_THRESHOLD_VERY_CLOSE:
        cv2.rectangle(frame, (0, 0), (int(frame.shape[1] / 3), int(frame.shape[0] / 2)), (0, 0, 255), 2)
        cv2.putText(frame, "LEFT WALL", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), thickness=2)
        capture_image(frame)
        turn_right(0.5)

    # Check for obstacles in the right regions and adjust angle
    if avg_depth_top_right_hard < DEPTH_THRESHOLD_VERY_CLOSE or avg_depth_bottom_right_hard < DEPTH_THRESHOLD_VERY_CLOSE:
        cv2.rectangle(frame, (int(frame.shape[1] / 3 * 2), 0), (frame.shape[1], int(frame.shape[0] / 2)), (0, 0, 255), 2)
        cv2.putText(frame, "RIGHT WALL", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), thickness=2)
        capture_image(frame)
        turn_left(0.5)

# Function to adjust vehicle angle based on proximity data from depth sensors
def _decide_how_long_adjust():
    """
    Continuously retrieves depth data from different regions of the camera frame,
    evaluates the proximity of nearby obstacles, and adjusts the vehicle's angle accordingly.
    Stops when all regions meet the safe depth thresholds.
    """
    while True:
        print("Adjusting angle based on proximity data...")
        # Capture frame and depth data
        frame, depth, _, _ = get_camera()

        # Define regions and extract depth information
        depth_top_left      = depth[:int(frame.shape[0] / 2), int(frame.shape[1] / 6):int(frame.shape[1] / 6) * 2]
        depth_top_right     = depth[:int(frame.shape[0] / 2), int(frame.shape[1] / 6 * 4):int(frame.shape[1] / 6 * 5)]
        depth_bottom_left   = depth[int(frame.shape[0] / 4 * 3):, int(frame.shape[1] / 13 * 2):int(frame.shape[1] / 13 * 5)]
        depth_bottom_right  = depth[int(frame.shape[0] / 4 * 3):, int(frame.shape[1] / 13 * 8):int(frame.shape[1] / 13 * 11)]
        depth_center_left   = depth[int(frame.shape[0] / 3):int(frame.shape[0] / 3 * 2), int(frame.shape[1] / 6):int(frame.shape[1] / 6 * 2)]
        depth_center_right  = depth[int(frame.shape[0] / 3):int(frame.shape[0] / 3 * 2), int(frame.shape[1] / 6 * 4):int(frame.shape[1] / 6 * 5)]

        # Calculate average depth for each region
        avg_depth_top_left      = np.mean(depth_top_left)
        avg_depth_top_right     = np.mean(depth_top_right)
        avg_depth_bottom_left   = np.mean(depth_bottom_left)
        avg_depth_bottom_right  = np.mean(depth_bottom_right)
        avg_depth_center_left   = np.mean(depth_center_left)
        avg_depth_center_right  = np.mean(depth_center_right)

        # Safety confirmation using frame and depth data
        _confirm_safe(frame, depth)
        print("decide")
        # Check each region for proximity and adjust angle accordingly
        if avg_depth_top_left < DEPTH_THRESHOLD_NEAR:
            _adjust_angle_forward_right()
        elif avg_depth_top_right < DEPTH_THRESHOLD_NEAR:
            _adjust_angle_forward_left()

        if avg_depth_center_left < DEPTH_THRESHOLD_NEAR:
            _adjust_angle_forward_right()
        elif avg_depth_center_right < DEPTH_THRESHOLD_NEAR:
            _adjust_angle_forward_left()

        if avg_depth_bottom_right < DEPTH_THRESHOLD_BOTTOM:
            _adjust_angle_forward_left()
        elif avg_depth_bottom_left < DEPTH_THRESHOLD_BOTTOM:
            _adjust_angle_forward_right()
        # Check if all regions meet safe depth thresholds
        if (avg_depth_center_left > DEPTH_THRESHOLD_FAR and
            avg_depth_center_right > DEPTH_THRESHOLD_FAR and
            avg_depth_bottom_left > SAFE_DEPTH_BOTTOM and
            avg_depth_bottom_right > SAFE_DEPTH_BOTTOM and
            avg_depth_top_left > DEPTH_THRESHOLD_FAR and
            avg_depth_top_right > DEPTH_THRESHOLD_FAR):
            return 0
        return 0

def wall_detection(frame, depth):
    """
    Detects walls and obstacles based on depth information and adjusts the angle accordingly.
    """
    # Define depth regions for obstacle detection
    depth_top_left       = depth[:int(frame.shape[0] / 4), int(frame.shape[1] / 13 * 2):int(frame.shape[1] / 13 * 5)]
    depth_top_left_hard  = depth[:int(frame.shape[0] / 4), int(frame.shape[1] / 13 * 2):int(frame.shape[1] / 13 * 3)]
    depth_top_right      = depth[:int(frame.shape[0] / 4), int(frame.shape[1] / 13 * 8):int(frame.shape[1] / 13 * 11)]
    depth_top_right_hard = depth[:int(frame.shape[0] / 4), int(frame.shape[1] / 13 * 10):int(frame.shape[1] / 13 * 11)]
    depth_top_center     = depth[:int(frame.shape[0] / 3), int(frame.shape[1] / 3):int(frame.shape[1] / 3 * 2)]
    # Compute average depths for each region
    avg_depth_top_left       = np.mean(depth_top_left)
    avg_depth_top_right      = np.mean(depth_top_right)
    avg_depth_top_left_hard  = np.mean(depth_top_left_hard)
    avg_depth_top_right_hard = np.mean(depth_top_right_hard)
    avg_depth_top_center     = np.mean(depth_top_center)
    # Bottom, center, and left/right depth regions
    depth_bottom_left_hard  = depth[int(frame.shape[0] / 4 * 3):, int(frame.shape[1] / 13 * 2):int(frame.shape[1] / 13 * 5)]
    depth_bottom_right_hard = depth[int(frame.shape[0] / 4 * 3):, int(frame.shape[1] / 13 * 8):int(frame.shape[1] / 13 * 11)]
    depth_center_left_hard  = depth[int(frame.shape[0] / 3):int(frame.shape[0] / 3 * 2), int(frame.shape[1] / 13 * 2):int(frame.shape[1] / 13 * 5)]
    depth_center_right      = depth[int(frame.shape[0] / 3):int(frame.shape[0] / 3 * 2), int(frame.shape[1] / 13 * 8):int(frame.shape[1] / 13 * 11)]
    depth_center_center     = depth[int(frame.shape[0] / 3):int(frame.shape[0] / 3 * 2), int(frame.shape[1] / 3):int(frame.shape[1] / 3 * 2)]
    depth_bottom_center     = depth[int(frame.shape[0] / 4 * 3):, int(frame.shape[1] / 5 * 2):int(frame.shape[1] / 5 * 3)]
    # Compute average depths for bottom and center regions
    avg_depth_bottom_left_hard  = np.mean(depth_bottom_left_hard)
    avg_depth_bottom_right_hard = np.mean(depth_bottom_right_hard)
    avg_depth_center_right      = np.mean(depth_center_right)
    avg_depth_center_left_hard  = np.mean(depth_center_left_hard)
    avg_depth_center_center     = np.mean(depth_center_center)
    avg_depth_bottom_center     = np.mean(depth_bottom_center)

    print(avg_depth_bottom_left_hard)
    
    # Check for obstacles directly in front
    if avg_depth_top_center < DEPTH_THRESHOLD_NEAR:
        back(BACKWARD_DURATION_A_SECOND)
        turn_left(TURN_DURATION_SLIGHT)
        print("a")
    elif avg_depth_bottom_center < DEPTH_THRESHOLD_BOTTOM_CENTER:
        back(BACKWARD_DURATION_A_SECOND)
        turn_left(TURN_DURATION_SLIGHT)
        print("b")
    elif avg_depth_center_center < DEPTH_THRESHOLD_NEAR:
        back(BACKWARD_DURATION_A_SECOND)
        turn_left(TURN_DURATION_SLIGHT)
        print("c")
    # Check for obstacles to the left and adjust angle if needed
    elif avg_depth_top_left < 500:
        capture_image(frame)
        _decide_how_long_adjust()
        print("d")
    elif avg_depth_bottom_left_hard < 100:
        capture_image(frame)
        _decide_how_long_adjust()
        print("e")
    elif avg_depth_center_left_hard < 500:
        capture_image(frame)
        _decide_how_long_adjust()
    # Check for obstacles to the right and adjust angle if needed
    elif avg_depth_top_right < 500:
        capture_image(frame)
        _decide_how_long_adjust()
    elif avg_depth_bottom_right_hard < 100:
        capture_image(frame)
        _decide_how_long_adjust()
    elif avg_depth_center_right < DEPTH_THRESHOLD_NEAR:
        capture_image(frame)
        _decide_how_long_adjust()
    # Check for hard obstacles on the left requiring a larger turn
    elif avg_depth_top_left_hard < DEPTH_THRESHOLD_HARD or avg_depth_bottom_left_hard < DEPTH_THRESHOLD_HARD:
        capture_image(frame)
        turn_right(TURN_DURATION_LARGE)
    # Check for hard obstacles on the right requiring a larger turn
    elif avg_depth_top_right_hard < DEPTH_THRESHOLD_HARD or avg_depth_bottom_right_hard < DEPTH_THRESHOLD_HARD:
        capture_image(frame)
        turn_left(TURN_DURATION_LARGE)
    # Check for ledges or steps
    if avg_depth_bottom_center < DEPTH_THRESHOLD_BOTTOM_CENTER:
        back(2.0)
    # Publish forward movement after adjustments
    msg = OverrideRCIn()
    msg.channels[SERVO_CHANNEL_FORWARD - 1] = 1460
    msg.channels[SERVO_CHANNEL_REVERSE - 1] = 1900
    pub.publish(msg)

# Function to draw a light blue point on the image at specified coordinates
def _draw_light_blue_point(image, x, y):
    """
    Draws a light blue point on the given image at specified (x, y) coordinates
    if they fall within the defined image boundaries. Adjusts y-coordinate to
    fit the display within a 250-pixel height viewable area.
    """
    # Check x-coordinate bounds
    if x < 0 or x > IMAGE_WIDTH:
        return 0
    # Adjust y-coordinate to account for offset
    y = IMAGE_MAX_Y - y
    # Check if y is within display bounds after adjustment
    if y < 0 or y > IMAGE_MAX_Y:
        return 0
    # Draw the light blue circle at (x, y) on the image
    cv2.circle(image, (x, y), POINT_RADIUS, COLOR_LIGHT_BLUE, -1)  # Draw filled light blue circle

# Function to draw a red point on the image at specified coordinates
def _draw_red_point(image, x, y):
    """
    Draws a red point on the given image at specified (x, y) coordinates
    if they fall within the image boundaries. Adjusts y-coordinate to
    fit the display within a 250-pixel height viewable area.
    """
    # Check x-coordinate bounds
    if x < 0 or x > IMAGE_WIDTH:
        return 0
    # Adjust y-coordinate to account for offset
    y = IMAGE_OFFSET_Y - y
    # Check y-coordinate bounds
    if y < 0 or y > IMAGE_OFFSET_Y:
        return 0
    # Draw the red circle at (x, y) on the image
    cv2.circle(image, (x, y), POINT_RADIUS, COLOR_RED, -1)  # Draw filled red circle

# Function to create a grayscale visualization image near the charging station
def _create_grayscale_image(x1, z1, x2, z2, X3, Z3, X4, Z4):
    """
    Generates a grayscale image with markers to visualize positions near the charging station.
    """
    # Initialize grayscale background
    gray_image = np.full((IMAGE_HEIGHT, IMAGE_WIDTH, 3), BACKGROUND_COLOR, dtype=np.uint8)
    # Calculate the center-bottom point and draw it in blue
    center_bottom = (IMAGE_WIDTH // 2, 275)
    cv2.circle(gray_image, center_bottom, 5, POINT_COLOR_BLUE, -1)
    cv2.circle(gray_image, BLUE_POINT_POSITION, 5, POINT_COLOR_GREEN, -1)
    # Convert coordinates to image space if within the range -1 to 1
    if -1 <= x1 <= 1:
        x1 = int(x1 * IMAGE_WIDTH / 2 + IMAGE_WIDTH / 2)  # Scale x to image width
        _draw_red_point(gray_image, x1, int(z1))
    if -1 <= x2 <= 1:
        x2 = int(x2 * IMAGE_WIDTH / 2 + IMAGE_WIDTH / 2)
        _draw_red_point(gray_image, x2, int(z2))
    # If both points exist, draw points and lines between them
    if x1 is not None and x2 is not None:
        # Adjust X3 and X4 to shift them for visualization
        new_X3 = X3 - 20
        new_X4 = X4 - 20
        _draw_light_blue_point(gray_image, int(new_X3), int(Z3))
        _draw_light_blue_point(gray_image, int(new_X4), int(Z4))
        # Draw line between new_X3 and center-bottom if within bounds
        # Z4は真ん中
        if 0 <= new_X3 <= IMAGE_WIDTH:
            new_Z3 = CENTER_BOTTOM_Y - Z3
            new_Z4 = CENTER_BOTTOM_Y - Z4
            cv2.line(gray_image, (int(new_X4), int(new_Z4)), center_bottom, LINE_COLOR_LIGHT_BLUE, 1)
            # Calculate angle theta between the two points
            len_x = new_X4 - center_bottom[0]
            len_y = CENTER_BOTTOM_Y - new_Z4
            tan_theta = len_x / len_y
            theta = np.degrees(np.arctan(tan_theta))
            # Adjust theta for quadrant
            if len_y < 0 and len_x > 0:
                theta = 180 + theta
            elif len_y < 0 and len_x < 0:
                theta = theta - 180
            # Draw theta value on the image
            cv2.putText(gray_image, f"theta: {theta:.2f}", THETA_TEXT_POSITION, cv2.FONT_HERSHEY_SIMPLEX, THETA_FONT_SCALE, THETA_TEXT_COLOR, THETA_TEXT_THICKNESS)
    # Create output directory if it doesn't exist
    if not os.path.exists(OUTPUT_DIRECTORY_GRAY):
        os.makedirs(OUTPUT_DIRECTORY_GRAY)
    # Get the file path for saving the image
    files = os.listdir(OUTPUT_DIRECTORY_GRAY)
    file_number = len(files)
    file_name = FILE_PREFIX + str(file_number) + ".jpg"
    file_path = os.path.join(OUTPUT_DIRECTORY_GRAY, file_name)
    # Save the image
    cv2.imwrite(file_path, gray_image)
    return theta

# Function to find the midpoint and points perpendicular to the line segment AB
def _find_point_P(A, B, d):
    """
    Finds the midpoint M between two points A and B, and calculates two points (P_up and P_down)
    that are distance `d` away from M, perpendicular to the line segment AB.
    """
    # Calculate the midpoint M
    M = np.add(A, B) / 2
    # Calculate the vector AB and its orthogonal vector
    AB = np.subtract(B, A)
    orthogonal_vector = np.array([-AB[1], AB[0], 0])
    # Check if AB is zero; if so, use a default orthogonal vector
    if np.linalg.norm(orthogonal_vector) == 0:
        orthogonal_vector = DEFAULT_ORTHOGONAL_VECTOR
    # Normalize and scale the orthogonal vector by distance d
    orthogonal_vector = (orthogonal_vector / np.linalg.norm(orthogonal_vector)) * d
    # Calculate points P_up and P_down
    P_up = np.add(M, orthogonal_vector)
    P_down = np.subtract(M, orthogonal_vector)
    # Return results as tuples for easy handling
    return tuple(M), tuple(P_up), tuple(P_down)


# Function to calculate intermediate points between two 3D coordinates
def _goal_point(x1, z1, x2, z2, d):
    """
    Calculates the midpoint and two additional points (P_up and P_down) between
    two given points in 3D space using the `find_point_P` function.
    """
    # Define points A and B as 3D coordinates (x, z, 0) in space
    A = np.array([x1, z1, 0])
    B = np.array([x2, z2, 0])
    # Use find_point_P to get the midpoint and additional points
    M, P_up, P_down = _find_point_P(A, B, d)
    return M, P_up, P_down

# Function to repeatedly check for a marker and stop the vehicle if detected
def _marker_stop_repeat():
    """
    Repeatedly checks for a specific marker and stops the vehicle if the marker is detected.
    If the marker is not detected, the vehicle moves backward slightly.
    """
    while True:
        # Capture frame and detect markers
        frame, depth, corners, list_ids = get_camera()
        capture_image(frame)
        # Calculate image center x-coordinates for left and right references
        image_left_center_x  = frame.shape[1] * IMAGE_CENTER_LEFT_RATIO
        image_right_center_x = frame.shape[1] * IMAGE_CENTER_RIGHT_RATIO
        # Check if the target marker is detected
        if MARKER_ID_TO_STOP in list_ids:
            # Calculate the center coordinates of the detected marker
            marker_center_x, marker_center_y = calc_marker_center(frame, corners, list_ids, MARKER_ID_TO_STOP)
            # Stop the vehicle at the marker's position
            marker_stop(msg, frame, depth, corners, list_ids, MARKER_ID_TO_STOP, image_right_center_x, image_left_center_x, marker_center_x)
        else:
            # Move backward if the marker is not detected
            back(BACK_DURATION)
        # Wait briefly before the next check
        time.sleep(MARKER_STOP_INTERVAL)

# Function to calculate the forward movement time based on marker distances
def _cal_forward(z1, z2):
    """
    Calculate the movement time required to reach the target position based on marker distances.
    """
    # Calculate the average distance from both markers
    distance = (z1 + z2) / 2
    if distance > DISTANCE_THRESHOLD_1:  # If the distance is greater than 65cm
        point_to_target = distance - 50
        movement        = point_to_target * MOVEMENT_SCALING_FACTOR_1
        movement_time   = movement / 10
        return movement_time
    elif distance >= DISTANCE_THRESHOLD_2:  # If the distance is between 55cm and 65cm
        point_to_target = distance - 50
        movement        = point_to_target * MOVEMENT_SCALING_FACTOR_2
        movement_time   = (movement + 0.5) / 4
        if movement_time < MOVEMENT_MIN_TIME:
            return MOVEMENT_MIN_TIME
        return MOVEMENT_MIN_TIME # SAME!!!
    else:  # For distances 50cm or less
        # If the distance is less than or equal to 50cm, set a fixed movement time
        return MOVEMENT_MIN_TIME

# Function to move the vehicle forward for the specified time with the 1900 speed setting
def _forward_1900(turn_time):
    """
    Moves the vehicle forward for a specified duration using the forward_1900 speed setting.
    After moving forward, stops the vehicle briefly before returning.
    """
    # Move forward for the specified time
    start_time = time.time()
    while time.time() - start_time < turn_time:
        msg = OverrideRCIn()
        msg.channels[SERVO_CHANNEL_FORWARD - 1] = FORWARD_1900_SPEED
        msg.channels[SERVO_CHANNEL_REVERSE - 1] = 1880
        pub.publish(msg)
        time.sleep(MOVE_SLEEP_DURATION)
    # Stop briefly after moving forward
    start_time = time.time()
    while time.time() - start_time < STOP_DURATION:
        msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_SPEED
        msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_SPEED
        pub.publish(msg)
        time.sleep(MOVE_SLEEP_DURATION)
    # Final stop delay after the entire operation
    time.sleep(FINAL_STOP_DURATION)

# Function to rotate until a marker is detected
def _turn_until_find_marker(turn_type: str):
    """
    Rotates the vehicle in a specified direction (left or right) until it finds one of the target markers (1, 3, or 4).
    """
    # Capture the frame and get marker data
    frame, _ = camera.get_frame()
    _, ids, _ = DETECTOR.detectMarkers(frame)
    list_ids = np.ravel(ids)
    # If markers 1, 3, or 4 are not detected, continue turning
    if 1 not in list_ids and 3 not in list_ids and 4 not in list_ids:
        if turn_type == "left":
            turn_left(LEFT_TURN_DURATION)  # Turn left
            time.sleep(TURN_RETRY_DELAY)  # Wait before retrying
            _turn_until_find_marker("left")  # Recursively try again
        elif turn_type == "right":
            turn_right(RIGHT_TURN_DURATION)  # Turn right
            time.sleep(TURN_RETRY_DELAY)  # Wait before retrying
            _turn_until_find_marker("right")  # Recursively try again
        else:
            raise ValueError("turn_type must be either 'left' or 'right'.")
    else:
        return 0  # Marker found, exit the function

# Function to turn the vehicle until it reaches the target theta (in degrees)
def _turn_until_theta(theta):
    """
    Turns the vehicle by a specified angle (theta) while checking for markers.
    The vehicle adjusts its position based on the distance and angle to the markers.
    """
    # Get the current distances for markers 3 and 4
    _, z1 = _distances(3)
    _, z2 = _distances(4)
    Z3 = (z1 + z2) / 2  # Average distance for markers 3 and 4
    run_time = _cal_forward(z1, z2)
    global GLOBAL_YAW_DEGREES
    initial_yaw_degrees = GLOBAL_YAW_DEGREES
    # Adjust target angle based on the direction of turn (left or right)
    if theta >= 0:  # Left turn
        target_yaw_degrees = (GLOBAL_YAW_DEGREES + theta) % 360
        print(f"Turning left to target yaw: {target_yaw_degrees}")
        # Turn left while checking the vehicle's yaw angle
        while True:
            # if (target_yaw_degrees >= GLOBAL_YAW_DEGREES and GLOBAL_YAW_DEGREES < target_yaw_degrees - TURN_PADDING) or (target_yaw_degrees < GLOBAL_YAW_DEGREES and GLOBAL_YAW_DEGREES > target_yaw_degrees + TURN_PADDING):
            #     break
            if (target_yaw_degrees >= initial_yaw_degrees and (GLOBAL_YAW_DEGREES > target_yaw_degrees or GLOBAL_YAW_DEGREES < initial_yaw_degrees - PADDING)):
                break
            elif (target_yaw_degrees < initial_yaw_degrees and target_yaw_degrees <= GLOBAL_YAW_DEGREES <= (initial_yaw_degrees - PADDING)):
                break
            print(GLOBAL_YAW_DEGREES)
            msg = OverrideRCIn()
            msg.channels[SERVO_CHANNEL_FORWARD - 1] = 1325
            msg.channels[SERVO_CHANNEL_REVERSE - 1] = NEUTRAL_SPEED
            pub.publish(msg)
            print(GLOBAL_YAW_DEGREES)
            time.sleep(TURN_SLEEP_DURATION)
        stop(STOP_DURATION)  # Stop after turning
        # Adjust movement after turn depending on the distance
        if Z3 > 65:
            print("前進")
            _forward_1900(run_time)
            time.sleep(TURN_CHECK_DELAY)
            _turn_until_find_marker("right")
        elif 55 <= Z3 <= 65:
            _forward(run_time)
            time.sleep(TURN_CHECK_DELAY)
            _turn_until_find_marker("right")
        else:
            _forward(2.0)
            time.sleep(TURN_CHECK_DELAY)
            _turn_until_find_marker("right")
        return 0
    else:  # Right turn
        target_yaw_degrees = (GLOBAL_YAW_DEGREES + theta) % 360

        # Turn right while checking the vehicle's yaw angle
        print(f"Turning right to target yaw: {target_yaw_degrees}")
        while True:
            if (target_yaw_degrees <= initial_yaw_degrees and (GLOBAL_YAW_DEGREES < target_yaw_degrees or GLOBAL_YAW_DEGREES > initial_yaw_degrees + PADDING)):
                break
            elif (target_yaw_degrees > initial_yaw_degrees and (initial_yaw_degrees + PADDING) < GLOBAL_YAW_DEGREES < target_yaw_degrees):
                break
            msg = OverrideRCIn()
            msg.channels[SERVO_CHANNEL_FORWARD - 1] = 1675
            msg.channels[SERVO_CHANNEL_REVERSE - 1] = NEUTRAL_SPEED
            pub.publish(msg)
            print(GLOBAL_YAW_DEGREES)
            time.sleep(TURN_SLEEP_DURATION)
        stop(STOP_DURATION)  # Stop after turning
        # Adjust movement after turn depending on the distance
        if Z3 > 65:
            _forward_1900(run_time)
            time.sleep(TURN_CHECK_DELAY)
            _turn_until_find_marker("left")
        elif 55 <= Z3 <= 65:
            _forward(run_time)
            time.sleep(TURN_CHECK_DELAY)
            _turn_until_find_marker("left")
        else:
            _forward(1.5)
            time.sleep(TURN_CHECK_DELAY)
            _turn_until_find_marker("left")
        return 0

def station_detection(frame, depth, corners, list_ids):
    """
    Detects the charging station and adjusts the vehicle's position and orientation for docking.
    """
    global Z3, Z4
    search_attempts = 0
    while True:
        frame, _ = camera.get_frame()
        corners, ids, _ = DETECTOR.detectMarkers(frame)
        capture_image(frame)
        list_ids = np.ravel(ids)
        # Check for detected markers
        if len(list_ids) != 0:
            z0 = z1 = 0  # Initialize distances
            if 0 in list_ids and 3 not in list_ids and 4 not in list_ids:
                back(BACKWARD_DURATION)
            if 3 not in list_ids:
                turn_left(LEFT_TURN_DURATION)
                search_attempts += 1
                if search_attempts == MAX_SEARCH_ATTEMPTS:
                    back(BACKWARD_DURATION)
                    search_attempts = 0
                continue
            if 4 not in list_ids:
                turn_right(RIGHT_TURN_DURATION)
                search_attempts += 1
                if search_attempts == MAX_SEARCH_ATTEMPTS:
                    back(BACKWARD_DURATION)
                    search_attempts = 0
                continue
            for marker_id in [3, 4]:
                marker_index = np.where(list_ids == marker_id)[0][0]
                marker_corners = corners[marker_index][0]
                # Calculate marker center coordinates
                marker_center_x = np.mean(marker_corners[:, 0])
                marker_center_y = np.mean(marker_corners[:, 1])
                # Calculate the distance to the marker
                _, distance_01 = _distance_aruco(marker_id)
                if not np.isnan(distance_01):
                    distance_01 *= 10  # Convert to mm
                    if marker_id == 3:
                        x0, y0, z0 = marker_center_x, marker_center_y, distance_01
                    elif marker_id == 4:
                        x1, y1, z1 = marker_center_x, marker_center_y, distance_01
                else:
                    continue
                # Check if the marker is within stopping range
                if (DISTANCE_THRESHOLD_MIN < z0 < DISTANCE_THRESHOLD_MAX) or (DISTANCE_THRESHOLD_MIN < z1 < DISTANCE_THRESHOLD_MAX):
                    msg = OverrideRCIn()
                    msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_VALUE
                    msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
                    pub.publish(msg)
                    time.sleep(1)
                    _marker_stop_repeat()
                # Draw marker details on the frame
                cv2.circle(frame, (int(marker_center_x), int(marker_center_y)), 10, (0, 0, 255), -1)
                cv2.putText(frame, f"{marker_id}", (int(marker_center_x), int(marker_center_y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.polylines(frame, [marker_corners.astype(np.int32)], True, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.circle(frame, (int(marker_center_x), int(marker_center_y)), 10, (0, 0, 255), -1)
        # Convert frame to grayscale and detect markers again
        _ = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Retrieve distance data for grayscale processing
        if 3 in list_ids:
            X3, Z3 = _distances(3)
            new_Z3 = Z3 * 3.5
        else:
            X3, new_Z3 = UNDETECTABLE_DISTANCE, UNDETECTABLE_DISTANCE
        if 4 in list_ids:
            X4, Z4 = _distances(4)
            new_Z4 = Z4 * 3.5
        else:
            X4, new_Z4 = UNDETECTABLE_DISTANCE, UNDETECTABLE_DISTANCE
        # Calculate target point and adjust orientation if both markers are visible
        if 3 in list_ids and 4 in list_ids:
            X33 = X3 * 320 + 320 + 20  # Shift to adjust right bias
            X44 = X4 * 320 + 320 + 20
            _, _, P_down = _goal_point(X33, new_Z3, X44, new_Z4, 125)
            _, _, P_down2 = _goal_point(X33, new_Z3, X44, new_Z4, 150)
            theta = _create_grayscale_image(X3, new_Z3, X4, new_Z4, P_down[0], P_down[1], P_down2[0], P_down2[1])
            theta *= -1  # Reverse to match left-turning convention
            if Z3 != UNDETECTABLE_DISTANCE and Z4 != UNDETECTABLE_DISTANCE:
                print("Theta:", theta)
                _turn_until_theta(theta)
                return 0
        else:
            theta = _create_grayscale_image(X3, Z3, X4, Z4, UNDETECTABLE_DISTANCE, UNDETECTABLE_DISTANCE, UNDETECTABLE_DISTANCE, UNDETECTABLE_DISTANCE)
            return 0

# Function to turn the vehicle to the right for a specified duration
def turn_right(turn_time):
    """
    Turns the vehicle to the right for the specified duration.
    """
    # Initial stop before starting the right turn
    msg = OverrideRCIn()
    msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_VALUE
    msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
    pub.publish(msg)
    time.sleep(STOP_DURATION_BEFORE_TURN)
    # Start right turn for the specified turn_time
    start_time = time.time()
    while time.time() - start_time < turn_time:
        msg = OverrideRCIn()
        msg.channels[SERVO_CHANNEL_FORWARD - 1] = RIGHT_TURN_SPEED
        msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
        pub.publish(msg)
        time.sleep(TURN_SLEEP_DURATION)
    # Brief stop after completing the right turn
    start_time = time.time()
    while time.time() - start_time < STOP_DURATION_AFTER_TURN:
        msg = OverrideRCIn()
        msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_VALUE
        msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
        pub.publish(msg)
        time.sleep(TURN_SLEEP_DURATION)
    # Final delay to ensure smooth transition
    time.sleep(FINAL_STOP_DURATION)

def only_turn_until_theta(theta):
    """
    Function to turn the vehicle until it reaches the specified angle (theta).
    Turns left if theta is positive, right if theta is negative.
    """
    global GLOBAL_YAW_DEGREES  # Current vehicle angle in degrees
    print(GLOBAL_YAW_DEGREES)

    initial_yaw_degrees = GLOBAL_YAW_DEGREES  # Initial vehicle angle before the turn
    target_yaw_degrees  = (initial_yaw_degrees + theta) % 360  # Target angle after the turn

    if theta >= 0:
        # Left turn
        while True:
            print(GLOBAL_YAW_DEGREES)
            # Check if the target angle is reached, considering wrapping around at 360 degrees
            if (target_yaw_degrees >= initial_yaw_degrees and (GLOBAL_YAW_DEGREES > target_yaw_degrees or GLOBAL_YAW_DEGREES < initial_yaw_degrees - PADDING)):
                break
            elif (target_yaw_degrees < initial_yaw_degrees and target_yaw_degrees <= GLOBAL_YAW_DEGREES <= (initial_yaw_degrees - PADDING)):
                break
            # Publish the left turn command
            msg = OverrideRCIn()
            msg.channels[SERVO_CHANNEL_FORWARD - 1] = LEFT_TURN_SPEED
            msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
            pub.publish(msg)
            time.sleep(TURN_SLEEP_DURATION)
    else:
        # Right turn
        while True:
            print(GLOBAL_YAW_DEGREES)
            # Check if the target angle is reached, considering wrapping around at 360 degrees
            if (target_yaw_degrees <= initial_yaw_degrees and (GLOBAL_YAW_DEGREES < target_yaw_degrees or GLOBAL_YAW_DEGREES > initial_yaw_degrees + PADDING)):
                break
            elif (target_yaw_degrees > initial_yaw_degrees and (initial_yaw_degrees + PADDING) < GLOBAL_YAW_DEGREES < target_yaw_degrees):
                break
            # Publish the right turn command
            msg = OverrideRCIn()
            msg.channels[SERVO_CHANNEL_FORWARD - 1] = RIGHT_TURN_SPEED
            msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
            pub.publish(msg)
            time.sleep(TURN_SLEEP_DURATION)

# Function to calculate the distance and x-coordinate of a specified ArUco marker
def _distance_aruco(marker_id):
    """
    Calculates the distance and x-coordinate of a specified ArUco marker ID using the camera frame.
    """
    # Load camera calibration parameters
    camera_matrix = np.load("mtx.npy")
    distortion_coeff = np.load("dist.npy")
    # Get frame and depth data from the camera
    frame, _ = camera.get_frame()
    # Detect ArUco markers in the frame
    corners, ids, _ = DETECTOR.detectMarkers(frame)
    # Check if any markers were detected
    if ids is not None:
        for i, id in enumerate(ids):
            if id == marker_id:
                # Extract the corner points of the specified marker
                corner = corners[i]
                # Estimate the pose of the marker
                _, tvec, _ = aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE_ACTUAL, camera_matrix, distortion_coeff)
                # Extract and convert the translation vector to get x and z
                tvec = np.squeeze(tvec)
                x = tvec[0]
                z = tvec[2] * CONVERSION_TO_CM  # Convert from meters to centimeters
                # Apply a calibration correction to z to improve accuracy
                z = Z_CORRECTION_SLOPE * z + Z_CORRECTION_INTERCEPT
                return x, z
    # Return default values if the marker is not detected
    return DEFAULT_DISTANCE_X, DEFAULT_DISTANCE_Z

# Function to get the distance and x-coordinate of a specified marker ID
def _distances(id):
    """
    Retrieves the x-coordinate and distance to a specified marker ID.
    """
    # Call distance_aruco once and check if the marker is detected
    marker_data = _distance_aruco(id)
    if marker_data is not None:
        x, z = marker_data
        return x, z
    else:
        return 40000,40000  # Return None values if marker is not detected

# Function to adjust the angle to the right by moving only the left motor
def _adjust_angle_only_left_motor(turn_time):
    """
    Adjusts the angle to the right by activating only the left motor for a specified duration.
    """
    # Create message to activate only the left motor
    msg = OverrideRCIn()
    msg.channels[0] = LEFT_MOTOR_ADJUST_SPEED
    msg.channels[2] = LEFT_MOTOR_ADJUST_SPEED
    # Start adjustment
    start_time = time.time()
    while time.time() - start_time < turn_time:
        pub.publish(msg)
        time.sleep(ADJUST_PUBLISH_INTERVAL)
    # Brief stop after completing the adjustment
    time.sleep(FINAL_STOP_DURATION_ADJUST)

# Function to adjust the angle to the left by moving only the right motor
def _adjust_angle_only_right_motor(turn_time):
    """
    Adjusts the angle to the left by activating only the right motor for a specified duration.
    """
    # Create message to activate only the right motor
    msg = OverrideRCIn()
    msg.channels[0] = RIGHT_MOTOR_ADJUST_SPEED
    msg.channels[2] = RIGHT_MOTOR_ADJUST_SPEED
    # Start adjustment
    start_time = time.time()
    while time.time() - start_time < turn_time:
        pub.publish(msg)
        time.sleep(ADJUST_PUBLISH_INTERVAL)
    # Brief stop after completing the adjustment
    time.sleep(FINAL_STOP_DURATION_ADJUST)

# Function to check if the distance to the marker is less than a specified threshold
def distance_check(marker_id, distance_threshold):
    """
    Checks if the distance to the specified marker is less than the given threshold.
    """
    x_distance, z_distance = _distances(marker_id)
    # Check if the marker is undetectable
    if z_distance == UNDETECTABLE_DISTANCE:
        return False
    # Check if the marker is within the desired distance
    if 0 < z_distance < distance_threshold:
        return True
    else:
        # Determine if the marker is to the left or right based on x_distance
        if x_distance < -ANGLE_ADJUSTMENT_THRESHOLD:
            _adjust_angle_only_right_motor(ANGLE_ADJUSTMENT_DURATION)
        elif x_distance > ANGLE_ADJUSTMENT_THRESHOLD:
            _adjust_angle_only_left_motor(ANGLE_ADJUSTMENT_DURATION)
        return False

# Function to capture and save camera image with a timestamped filename
def capture_image(frame):
    """
    Captures and saves the camera image with a timestamped filename
    """
       # picture2というディレクトリが存在しない場合は作成
    if not os.path.exists(OUTPUT_DIRECTORY):
        os.makedirs(OUTPUT_DIRECTORY)
    # pucture2ディレクトリ内のファイル数を取得
    files = os.listdir(OUTPUT_DIRECTORY)
    # ファイル数を取得
    file_number = len(files)
    # ファイル名を作成
    file_name = str(file_number) + ".jpg"
    # ファイルのパスを作成
    file_path = os.path.join(OUTPUT_DIRECTORY, file_name)
    # 画像を保存
    cv2.imwrite(file_path, frame)
    cv2.imwrite(file_path, frame)

# Function to retrieve camera image and depth data
def get_camera():
    """
    Retrieves the camera image and depth data from the RealSense camera.
    """
    data = camera.get_frame()
    # Check if data contains both frame and depth information
    if len(data) == EXPECTED_DATA_LENGTH:
        frame, depth = data
    else:
        return None, None, None, None
    # Detect ArUco markers in the frame
    corners, ids, rejected_img_points = DETECTOR.detectMarkers(frame)
    # Convert marker IDs to a flattened list, handling the case where no markers are detected
    list_ids = np.ravel(ids) if ids is not None else []
    return frame, depth, corners, list_ids

def main():
    """
    Main function that handles marker detection and robot movement control.
    """

    msg = None
    frame, depth, corners, list_ids = get_camera()
    global GLOBAL_YAW_DEGREES

    # Define image regions for marker detection
    image_left_center_x  = frame.shape[1] / 4
    image_right_center_x = frame.shape[1] / 4 * 3
    i = 0
    global initial_yaw

    print("Starting the main loop...")
    # Main loop
    while True:
        print(f"----- Iteration {i} -----")
        i += 1
        frame, depth, corners, list_ids = get_camera()

        # Capture current frame image
        print("Capturing image...")
        capture_image(frame)

        # Marker 1: Detect and check distance for possible 180° turn
        if 1 in list_ids and len(corners) > 0:
            print("Marker 1 detected")
            if distance_check(1, DISTANCE_THRESHOLD_MARKER_1) == 1:
                print("Distance check passed for Marker 1")
                only_turn_until_theta(TURN_ANGLE_MARKER_1)
                
        if 2 in list_ids and len(corners) > 0:
            print("Marker 2 detected")
            # if distance_check(2, DISTANCE_THRESHOLD_MARKER_1) == 1:
            print("Distance check passed for Marker 2")
            # only_turn_until_theta(TURN_ANGLE_MARKER_2)
            turn_left(2.0)

        # Marker 3: Station detection and adjustments
        if 3 in list_ids and len(corners) > 0:
            print("Marker 3 detected")
            if distance_check(3, DISTANCE_THRESHOLD_MARKER_3) == 1:
                print("Distance check passed for Marker 3")
                if 4 not in list_ids:
                    print("Marker 4 not detected")
                    turn_right(RIGHT_TURN_DURATION)
                    continue

                # Stop and detect station
                msg = OverrideRCIn()
                msg.channels[SERVO_CHANNEL_FORWARD - 1] = REVERSE_VALUE
                msg.channels[SERVO_CHANNEL_REVERSE - 1] = REVERSE_VALUE
                pub.publish(msg)
                station_detection(frame, depth, corners, list_ids)

            elif distance_check(3, DISTANCE_THRESHOLD_MARKER_3) == 0:
                print("Distance check failed for Marker 3")
                wall_detection(frame, depth)
                time.sleep(WALL_DETECTION_DELAY)

        # Marker 4: Adjust for visibility of Marker 3
        elif 4 in list_ids and len(corners) > 0:
            print("Marker 4 detected")
            if distance_check(4, DISTANCE_THRESHOLD_MARKER_4) == 1:
                print("Distance check passed for Marker 4")
                turn_left(LEFT_TURN_DURATION)
            elif distance_check(4, DISTANCE_THRESHOLD_MARKER_4) == 0:
                print("Distance check failed for Marker 4")
                wall_detection(frame, depth)

        # Marker 5: Check distance and initiate 180° turn if needed
        if 5 in list_ids and len(corners) > 0:
            print("Marker 5 detected")
            if distance_check(5, DISTANCE_THRESHOLD_MARKER_5) == 1:
                print("Distance check passed for Marker 5")
                only_turn_until_theta(TURN_ANGLE_MARKER_5)

        # Marker 19: Calculate center coordinates and perform stop action
        if 19 in list_ids:
            print("Marker 19 detected")
            marker_center_x, marker_center_y = calc_marker_center(frame, corners, list_ids, 19)
            marker_stop(msg, frame, depth, corners, list_ids, 19, image_right_center_x, image_left_center_x, marker_center_x)

        # Marker 0 only: Slight backward move
        if 0 in list_ids and 3 not in list_ids and 4 not in list_ids:
            print("Marker 0 detected")
            back(BACKWARD_DURATION)

        # No markers detected: Perform wall detection and move forward
        if 0 not in list_ids and 3 not in list_ids and 4 not in list_ids:
            print("No markers detected")
            wall_detection(frame, depth)
            time.sleep(WALL_DETECTION_DELAY)
            
        else:
            print("No markers detected")
            # stop(0.5)
        print("-----------------------")

if __name__ == '__main__':
    try:
        while True:
            main()
    except KeyboardInterrupt:
        print("Ctrl+C was pressed")
    except Exception as e:
        print(e)