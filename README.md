Video link for Assignment 2 ->

Some Considerations:
1. I did not have 4 DC motors with me currently, so I developed everything using 2 motors with encoders.
2. I did not implement the Kalman Filter as it was hard to understand and implement in a short amount of time, but I do know that I have to implement it on encoder values only and then derive the x,y,θ,v, and ω. A 1D Kalman filter will be used here (maybe I am wrong).
3. But I am still printing the Odometry without a Kalman filter using encoder data only.
4. In this assignment, I have also implemented a PID Controller for a DC Motor to control the speed using feedback
5. I am currently still learning about the Kalman filter and will try to use it to derive the x,y,θ,v, and ω

Documentation:
1. In this code there are 3 things which is running parallely
