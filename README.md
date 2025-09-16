Video link for Assignment 2 ->

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
   a. Serial data checker at 100Hz
   b. PID Controller at 30Hz
   c. Odometry Update task at 50Hz
   d. Separate ISR functions for left and right encoders
2.  ISR is implemented on both the signal pins of the encoders, Signal Pin A and B
3.  The ISR function will just compare the previous and current state of Signal Pins A and B. If the state is valid, it will Increment/Decrement the current position of the encoder. This type of check will filter the invalid data and does not update     the encoder counter on invalid values
4.  I was unable to implement debouncing on encoder pins as my encoders are very high, approximately. 8000 positions in just 1 sec for 2.9 revolutions. This gives approximately 8 positions per millisecond.
5.  Now we will talk about the Serial Data checker, which is running at 100Hz or every 10 milliseconds:
    a. I am using the ArduinoJson library to parse a JSON string
    b. \"{\"vL\": \"91\", \"vR\": \"92\" }\" -> This is how our JSON string looks, which is used to control the speed of both the motors
    c. We are then extracting the raw values from the string and passing them to our PID Control Parameters.
    d. The speed is given in Ticks per PID Loop, which means how many encoder Ticks/Counts we want per PID Loop, or every 33 milliseconds.
6. Now we will talk about the PID Loop, which is running at 30Hz or every 33 milliseconds:
   a. 
