# LookingForward
Univeristy of British Columbia, IGEN 230 Design Project
Spring 2019 - Presented April 4th/2019
A proof-of-concept collision avoidance system implemented on a skateboard.
Inputs:   Distance from LiDAR Lite V3, Angle from Tower Pro SG90 Servo, Yaw/Pitch/Roll from MPU6050's Gyro.
Process:  Time stamped vectors pointing to nearby obstacles, with angle offsets (due to dynamic nature of a moving 
          skateboard) from gyro stored; derivative of vectors * distance = seconds to impact (SOI) to nearest object
          in current trajectory. SOI = 2.2s is set as the danger threshhold (based on research & experimention).
Outputs:  Piezo Buzzer alarming the user, with frequency mapped to increase as SOI decreases.

Link to final report: https://drive.google.com/open?id=18kKv8XsAvu9eewQbNph3zWeGGHRPOC1O
