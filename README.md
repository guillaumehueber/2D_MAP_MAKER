# Summary of our 2DMapMaker project:
- This project makes a 2D map of a board with different objects on it.
- It uses the Inertial Measurement Unit (IMU) of the E-puck2 to find its position on a board with obstacles and different objects. The robot turns on itself and finds the angle where it's headed by integrating the angular rate from the MPU9250 sensor in the IMU.
- These values are cleaned using a soft threshold technique allowing better results than a traditional hard threshold. The threshold is the standard deviation found on more than 17 000 values from the IMU when the robot is staying still. This removes a lot of the noise, we go down to around 15% values of noise remaining (which can be further reduced to 0.4% if we take the threshold to be 2 x the standard deviation)
- We can see this in the following histograms:

>### Picture 1
>Acceleration BEFORE cleaning
    <p float="left">
    <img src="pictures/Acceleration BEFORE cleaning.png" alt="drawing" width="700"/>
    </p>

>### Picture 2
>Acceleration AFTER cleaning
    <p float="left">
    <img src="pictures/Acceleration AFTER cleaning.png" alt="drawing" width="700"/>
    </p>
- In the second image (after going through the soft threshold filter) that almost all values end up at the 0 value bin.

- The noise needs to be reduced as we want the position [m] from acceleration [m/s^2] so we need to integrate twice which amplifies any noise. We use the trapezoidal rule to integrate like this: 

>### Code block 1
>```c
>//Get acceleration values, offset them and integrate twice to get position
>for(int i = 0; i < NB_AXIS; i++){
>    acc[i].clean_acc = clean_values(get_acc(i), acc[i].avg, acc[i].stdev) * STANDARD_GRAVITY * acc_raw2g;
>    // Calculate velocity using trapezoidal rule
>    acc[i].vel += (acc[i].clean_acc + acc[i].prev_acc) / 2 * (TIME_PERIOD_MS / 1000); // in m/s
>
>    // Calculate position using trapezoidal rule
>    acc[i].pos += acc[i].prev_pos + (acc[i].vel + acc[i].prev_vel) / 2 * TIME_PERIOD_MS; // in mm
>    acc[i].prev_acc = acc[i].clean_acc;
>    acc[i].prev_vel = acc[i].vel;
>}
>```

- Unfortunately, despite cleaning the values with soft threshold, when finding the position through integration, we see a shift in velocity and so a constant slope greater than 1 in the position. We can see this in the following three graphs: 

>### Picture 3
>Acceleration, velocity and position when being moved
    <p float="left">
    <img src="pictures\Acceleration, velocity and position when being moved.png" alt="drawing" width="700"/>
    </p>
    
- This is why we ended up finding the position simply from seeing when the acceleration was greater than a certain threshold and positioning the E-puck2 at precise locations.