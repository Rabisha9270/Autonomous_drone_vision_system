import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to the drone
print("Connecting to the drone...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(target_altitude):
    """
    Arms the drone and flies to the target altitude.
    """
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Target altitude reached")
            break
        time.sleep(1)

def process_frame(frame):
    """
    Processes a single video frame to detect objects or features.
    """
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Use edge detection (Canny) as an example
    edges = cv2.Canny(gray, 50, 150)
    return edges

def main():
    # Arm and take off to 10 meters
    arm_and_takeoff(10)

    # Open a video stream (use 0 for a webcam or provide a video file path)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Unable to open video stream")
        return

    print("Processing video stream...")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            # Process the frame
            processed_frame = process_frame(frame)

            # Display the processed frame
            cv2.imshow("Drone Vision", processed_frame)

            # Example logic for object detection (e.g., stop if a feature is detected)
            if cv2.countNonZero(processed_frame) > 1000:
                print("Feature detected! Hovering...")
                vehicle.mode = VehicleMode("LOITER")
                break

            # Break on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Release resources
        cap.release()
        cv2.destroyAllWindows()

        # Land the drone
        print("Landing...")
        vehicle.mode = VehicleMode("LAND")
        while vehicle.armed:
            print("Waiting for disarming...")
            time.sleep(1)
        print("Drone disarmed")

if __name__ == "__main__":
    main()
