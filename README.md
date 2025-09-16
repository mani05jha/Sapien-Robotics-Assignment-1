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
2. Serial data checker at 100Hz
3. PID Controller at 30Hz
4. Odometry Update task at 50Hz
5. Separate ISR functions for left and right encoders

6. ISR is implemented on both the signal pins of the encoders, Signal Pin A and B
7.  The ISR function will just compare the previous and current state of Signal Pins A and B. If the state is valid, it will Increment/Decrement the current position of the encoder. This type of check will filter the invalid data and does not update     the encoder counter on invalid values
8.  I was unable to implement debouncing on encoder pins as my encoders are very high, approximately. 8000 positions in just 1 sec for 2.9 revolutions. This gives approximately 8 positions per millisecond.

9. Now we will talk about the Serial Data checker, which is running at 100Hz or every 10 milliseconds:
10. I am using the ArduinoJson library to parse a JSON string
11. \"{\"vL\": \"91\", \"vR\": \"92\" }\" -> This is how our JSON string looks, which is used to control the speed of both the motors
12. We are then extracting the raw values from the string and passing them to our PID Control Parameters.
13. The speed is given in Ticks per PID Loop, which means how many encoder Ticks/Counts we want per PID Loop, or every 33 milliseconds.

14. Now we will talk about the PID Loop, which is running at 30Hz or every 33 milliseconds
15. I have declared some parameters to keep track of the values that are crucial for the output of our PID Controller
16. Parameters are:
17. setpoint    -> how many encoder Ticks/Counts we want per PID Loop, or every 33 milliseconds
18. Perror      -> corresponds to the previous error used in the calculation of the PIDerivative term per PID loop
19. Encoder     -> Current Encoder Value
20. prevEncoder -> Previous Encoder value or last encoder value
21. Pcumlative  -> Cumulative error from the start of the PID loop to the end of the PID loop used in the calculation of PIntergalD term
22. output      -> Result of the PID Controller ( kp * error + Ki * Pcumlative + Kd * )
23. Kp          -> Proportional Component (Formula For Proportional Term is Kp * error)
24. Ki          -> Integral Component     (Formula For Integral Term is Ki * Pcumlative)
25. Kd          -> Derivative Component   (Formula For Derivative Term is Kd * error - Perror)
26. PID will start only when the setpoint value is greater than 1
27. How PID is working
28. We calculate the error term using the current encoder value and the previous encoder value -> Proportinal Error
29. We will add the error value to Pcumlative with the current error (Pcumlative = Pcumlative + (error * 0.033) -> Integral Error
30. We will get our derivative error using current and previous error (derivative = (error - Perror) / 0.033; -> Derivative Error
31. Now we will calculate the Output of PID Controller (output = Ki * error + Kd * Pcumlative + Kd * derivative)
32. Adjust the output so that it crosses the bounds of {-255, 255} (-255 means the motor will rotate in the reverse direction and vice versa)
33. Repeat the process for both motors
34. Do it until the stop condition occurs

35. Now we will talk about the Odometry Update Loop, which is running at 50Hz or every 20 milliseconds:
36. This task is to output the current odometry of the Bot, which is denoted by x,y,theta,v, and w
37. We have declared some constants and variables to keep track of the odometry
38. Parameters are:
39. wheel_radius
40. track_width           -> or chassis width
41. count_per_rev         -> per revolution count of the motor encoders
42. time_interval         -> time interval of the Odometry loop task
43. PI
44. linear_displacement   -> linear displacement of the robot
45. angular_displacement  -> angular displacement of the robot
46. x                     -> Position of the robot on the X Axis
47. y                     -> Position of the robot on the Y Axis
48. theta                 -> Angular Position along Z Axis
49. v                     -> Linear Velocity of the robot
50. w                     -> Angular Velocity of the robot
51. How Odometry updates are working
52. We will calculate the linear displacement of the robot by averaging the displacement of both the motor wheel using the formula
53. (linear_displacement_of_single_motor = ( 2*PI*wheel_radius*(current_encoder_count - previous_encoder_count)) / count_per_rev ))
54. (linear_displacement = (linear_displacement_of_left_motor + linear_displacement_of_right_motor ) / 2)
55. Now we will calculate the angular displacement of the robot along the z-axis by subtracting the left_motor_displacement from right_motor_displacement and then dividing it by track_width
56. (angular_displacement = (linear_displacement_of_right_motor - linear_displacement_of_left_motor) / track_width )
57. Now we will store the current encoder count in previous_encoder_count for the next Odometry calculation
58. Now we will calculate the x and y position of the robot using the formula
59. ( x_current = x_previous + cos(current_angle + (angular_displacement / 2))) ( y_current = y_previous + sin(current_angle + (angular_displacement / 2)))
60. Now we will calculate the current angle of the robot using the formula
61. theta = theta + angular_displacement
62. Now we will bound the theta of the robot in the range (-PI, PI)

Challenges:
1. Understanding the Kalman Filter itself is challenging
2. The motors I am using with the robot have an opposite arrangement to move the robot forward; one motor will rotate clockwise, and the other counterclockwise. This leads to an error in which the encoder counts of both motors are significantly       different
3. To solve this issue, I had to implement a PID controller to control the motor speed using Encoder TICKS per PID loop.
