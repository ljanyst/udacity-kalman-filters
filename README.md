
Kalman Filters
==============

The package contains a small library implementing kalman filters and a
radar-lidar fusion using both extended and unscented filters.

Extended filter:

    ]==>  ./ExtendedKF  ../data/obj_pose-laser-radar-synthetic-input.txt ../src/ekf.txt
    Accuracy - RMSE:
    0.0973826
    0.0853209
    0.441738
    0.453757

Unscented filter:

    ]==>  ./UnscentedKF  ../data/obj_pose-laser-radar-synthetic-input.txt ../src/ukf.txt
    Accuracy - RMSE:
    0.0659867
    0.0811041
    0.277747
    0.166186

![Unscented Filter](https://github.com/ljanyst/udacity-kalman-filters/blob/master/imgs/ukf_position.png?raw=true)

![Extended Filter](https://github.com/ljanyst/udacity-kalman-filters/blob/master/imgs/ekf_position.png?raw=true)

![Diffrence against ground truth](https://github.com/ljanyst/udacity-kalman-filters/blob/master/imgs/diff.png?raw=true)
