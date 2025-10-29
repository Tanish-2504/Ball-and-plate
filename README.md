# Interface (GUI)
Initial simulations were done on MATLAB and Simulink which have been given in the folder MATLAB_Simulink for reference. Following this we got into the physical model .
We created the 3D models of majority components required and got them 3D printed 

The file named "interface.py" contains the final python code with the GUI 
It uses first edge detection to detect the rectangle plate and refines the selection using yellow colour detection as we used yellow coloured plate. After it detects the yellow plate it trims the visible window to the size of the rectangle board with 2 pixel offset on each side 

Next comes the ball detection . We have used 3 filters to detect the ball. First we applied Hough circle transform to detect the circular ball followed by which there is a colour filter that detects the specific coloured ball and then to imrpove robustness we also added a filter that detects only perfectly round circles and that too with the appropriate size which helps in ignoring some absurd circles if any present and captured by the camera 

Once the ball is detected then its position is detected and the distance of ball from the setpoint(blue dot) is detected and sent to the PID controller which gives it the specified values as per the calculated values using the values given to each Kpx , Kpy , Kix , Kiy , Kdx , Kdy.

These calculated angles are then transferred via serial communication at a fixed baud rate to the arduino connected which calculates the required servo angles as mentioned in the file "final_arduino.py" and the motors tilt the connected rods to make sure the ball doesnt fall from the plate 

There are also 2 more buttons on the GUI tab that enable circle movement and 8 shape movement 
When enabled a red dot moves in a circle and the ball dynamically follows the red dot to trace out the desired shape 

The two images named "system image.jpeg" and "system_image.jpeg" show the image of our final built model and the video named "circle_video.mp4" shows how the ball perfectly follows the circular red dot on the plate .
