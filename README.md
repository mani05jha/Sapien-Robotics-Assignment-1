Components and Tools Used to develop this project:
1. 2 24-Volt 175RPM DC Motors with Quadrature Encoders
2. L298N Motor Driver
3. ESP32
4. 24Volt Power Supply
5. 2 Wheels with 3.5cm radius
6. A chassis with a 29.5cm width (it's a square box)
7. Micro USB Cable
8. Arduino IDE

Some Considerations:
1. I did not have 4 DC motors with me currently, so I developed everything using 2 motors with encoders.
2. I did not implement the Kalman Filter as it was hard to understand and implement in a short amount of time, but I do know that I have to implement it on encoder values only and then derive the x,y,θ,v, and ω. A 1D Kalman filter will be used here (maybe I am wrong).
3. But I am still printing the Odometry without a Kalman filter using encoder data only.
4. In this assignment, I have also implemented a PID Controller for a DC Motor to control the speed using feedback
5. I am currently still learning about the Kalman filter and will try to use it to derive the x,y,θ,v, and ω

Documentation:
1. In this code, 4 things are running in parallel
-> Serial data checker at 100Hz
-> PID Controller at 30Hz
-> Odometry Update task at 50Hz
-> Separate ISR functions for left and right encoders
3.  ISR is implemented on both the signal pins of the encoders, Signal Pin A and B
4.  The ISR function will just compare the previous and current state of Signal Pins A and B. If the state is valid, it will Increment/Decrement the current position of the encoder. This type of check will filter the invalid data and does not update     the encoder counter on invalid values
5.  I was unable to implement debouncing on encoder pins as my encoders are very high, approximately. 8000 positions in just 1 sec for 2.9 revolutions. This gives approximately 8 positions per millisecond.
6.  Now we will talk about the Serial Data checker, which is running at 100Hz or every 10 milliseconds:
    a. I am using the ArduinoJson library to parse a JSON string
    b. \"{\"vL\": \"91\", \"vR\": \"92\" }\" -> This is how our JSON string looks, which is used to control the speed of both the motors
    c. We are then extracting the raw values from the string and passing them to our PID Control Parameters.
    d. The speed is given in Ticks per PID Loop, which means how many encoder Ticks/Counts we want per PID Loop, or every 33 milliseconds.
7. Now we will talk about the PID Loop, which is running at 30Hz or every 33 milliseconds:
   a. I have declared some parameters to keep track of the values that are crucial for the output of our PID Controller
   b. Parameters are:
      i. setpoint    -> how many encoder Ticks/Counts we want per PID Loop, or every 33 milliseconds
     ii. Perror      -> corresponds to the previous error used in the calculation of the PIDerivative term per PID loop
    iii. Encoder     -> Current Encoder Value
     iv. prevEncoder -> Previous Encoder value or last encoder value
      v. Pcumlative  -> Cumulative error from the start of the PID loop to the end of the PID loop used in the calculation of PIntergalD term
     vi. output      -> Result of the PID Controller ( kp * error + Ki * Pcumlative + Kd * )
    vii. Kp          -> Proportional Component (Formula For Proportional Term is Kp * error)
   viii. Ki          -> Integral Component     (Formula For Integral Term is Ki * Pcumlative)
     ix. Kd          -> Derivative Component   (Formula For Derivative Term is Kd * error - Perror)
   c. PID will start only when the setpoint value is greater than 1
   d. How PID is working
      i. We calculate the error term using the current encoder value and the previous encoder value -> Proportinal Error
     ii. We will add the error value to Pcumlative with the current error (Pcumlative = Pcumlative + (error * 0.033) -> Integral Error
    iii. We will get our derivative error using current and previous error (derivative = (error - Perror) / 0.033; -> Derivative Error
     iv. Now we will calculate the Output of PID Controller (output = Ki * error + Kd * Pcumlative + Kd * derivative)
      v. Adjust the output so that it crosses the bounds of {-255, 255} (-255 means the motor will rotate in the reverse direction and vice versa)
     vi. Repeat the process for both motors
    vii. Do it until the stop condition occurs
8. Now we will talk about the Odometry Update Loop, which is running at 50Hz or every 20 milliseconds:
   a. This task is to output the current odometry of the Bot, which is denoted by x,y,theta,v, and w
   b. We have declared some constants and variables to keep track of the odometry
   c. Parameters are:
      i. wheel_radius
     ii. track_width           -> or chassis width
    iii. count_per_rev         -> per revolution count of the motor encoders
     iv. time_interval         -> time interval of the Odometry loop task
      v. PI
     vi. linear_displacement   -> linear displacement of the robot
    vii. angular_displacement  -> angular displacement of the robot
   viii. x                     -> Position of the robot on the X Axis
     ix. y                     -> Position of the robot on the Y Axis
      x. theta                 -> Angular Position along Z Axis
     xi. v                     -> Linear Velocity of the robot
    xii. w                     -> Angular Velocity of the robot
   d. How Odometry updates are working
      i. We will calculate the linear displacement of the robot by averaging the displacement of both the motor wheel using the formula                                                                                                                          (linear_displacement_of_single_motor = ( 2*PI*wheel_radius*(current_encoder_count - previous_encoder_count)) / count_per_rev ))
         (linear_displacement = (linear_displacement_of_left_motor + linear_displacement_of_right_motor ) / 2)
     ii. Now we will calculate the angular displacement of the robot along the z-axis by subtracting the left_motor_displacement from right_motor_displacement and then dividing it by track_width                                                               (angular_displacement = (linear_displacement_of_right_motor - linear_displacement_of_left_motor) / track_width )
    iii. Now we will store the current encoder count in previous_encoder_count for the next Odometry calculation
     iv. Now we will calculate the x and y position of the robot using the formula
         ( x_current = x_previous + cos(current_angle + (angular_displacement / 2))) ( y_current = y_previous + sin(current_angle + (angular_displacement / 2)))
      v. Now we will calculate the current angle of the robot using the formula
         theta = theta + angular_displacement
     vi. Now we will bound the theta of the robot in the range (-PI, PI)

Challenges:
1. Understanding the Kalman Filter itself is challenging
2. The motors I am using with the robot have an opposite arrangement to move the robot forward; one motor will rotate clockwise, and the other counterclockwise. This leads to an error in which the encoder counts of both motors are significantly       different
3. To solve this issue, I had to implement a PID controller to control the motor speed using Encoder TICKS per PID loop.
