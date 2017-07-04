# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 


## Modified files ##
kalman_filter.cpp
kalman_filter.h
FusionEKF.cpp
FusionEKF.h
tools.cpp




## results ##
result.txt record all the result in each timestamp

I found my result accuracy is relative good:  
"rmse_vx":0.330152051993931,
"rmse_vy":0.409315156283566,
"rmse_x":0.0944382255718658,
"rmse_y":0.0846791390044219

## Lessions learned ##
Just using knowledge got from class is very straightfoward.

However, two blocks I met and solved.

1. First I used atan(x_(1)/x_(0)), I had some incorrect refluxation when I have x move to negative.
Then I updated with atan2(x_(1), x_(0)), I have better results.
atan2 has better definition of diametrically opposite directions.
Teh one arg atan() function can not distigusih (1,1) and (-1,-1).

2. Second I found my result turns dramatically bad when I switch from quadrant 2 to 3, when I have (-x,y) to (-x,-y), I had big veritical change in estimated y value and vy value.
Then I noticed the atan2 value I have into phi is -3.1x, which original y has 3.x 
so, y-h turned out to be > 6 in angle calcatuion.

After I have retricted my result in (-pi, pi), I have good result coming ou.



