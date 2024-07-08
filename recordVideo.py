import cv2

def record_video(output_file, duration_sec):
    # Initialize the camera
    cap = cv2.VideoCapture(0)  # 0 represents the default camera

    # Check if the camera is opened correctly
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Get the default camera width and height
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'MP4V')  # You can use other codecs as well (e.g., 'MJPG', 'MP4V')
    out = cv2.VideoWriter(output_file, fourcc, 20.0, (width, height))

    start_time = cv2.getTickCount()  # Starting time
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Write the frame to the output video file
        out.write(frame)

        # Stop recording if duration_sec time has elapsed
        current_time = cv2.getTickCount()
        elapsed_time = (current_time - start_time) / cv2.getTickFrequency()
        if elapsed_time >= duration_sec:
            break

        # Display the recording to the screen
        cv2.imshow('Recording', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break

    # Release everything when done
    cap.release()
    out.release()
    cv2.destroyAllWindows()

# Driver Code
if __name__ == '__main__':
    output_file = 'output.avi'
    duration_sec = 10   # set duration in seconds for which you want to record the video
    record_video(output_file, duration_sec)