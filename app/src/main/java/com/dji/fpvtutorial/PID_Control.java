package com.dji.fpvtutorial;

public class PID_Control {

    double REFERENCE_AREA = 300 * 300;
    double UPPER_LIMIT_Z_INT = 1200000;
    double UPPER_LIMIT_X_INT = 15000;
    double UPPER_LIMIT_Y_INT = 15000;

    double pitch_gain_crt = -0.000021;
    double pitch_gain_int = -0.0000002;
    double pitch_gain_der = -0.000022;
    //double pitch_gain_crt = 0;
    //double pitch_gain_der = 0;
    //double pitch_gain_der = 0;


    double roll_gain_crt = 0.002;
    double roll_gain_int = 0.00004;
    double roll_gain_der = 0.0025;

    double throttle_gain_crt = -0.0008;
    double throttle_gain_int = -0.000004;
    double throttle_gain_der = -0.0008;

    double z_inte_error = 0;
    double z_prev_error = 0;
    double x_inte_error = 0;
    double x_prev_error = 0;
    double y_inte_error = 0;
    double y_prev_error = 0;

    double roll_control = 0;
    double throttle_control = 0;
    double pitch_control = 0;
    double yaw_control = 0;

    public void update_control_parameter(double x_error, double y_error, double z_error){
        //Determine the movement of the drone
        z_inte_error += z_error;
        x_inte_error += x_error;
        y_inte_error += y_error;
        if (x_inte_error > UPPER_LIMIT_X_INT) x_inte_error = UPPER_LIMIT_X_INT;
        if (x_inte_error < - UPPER_LIMIT_X_INT) x_inte_error = - UPPER_LIMIT_X_INT;
        if (y_inte_error > UPPER_LIMIT_Y_INT) y_inte_error = UPPER_LIMIT_Y_INT;
        if (y_inte_error < -UPPER_LIMIT_Y_INT) y_inte_error = -UPPER_LIMIT_Y_INT;
        if (z_inte_error > UPPER_LIMIT_Z_INT) z_inte_error = UPPER_LIMIT_Z_INT;
        if (z_inte_error < -UPPER_LIMIT_Z_INT) z_inte_error = -UPPER_LIMIT_Z_INT;

        double diff_x = x_error - x_prev_error;
        double diff_y = y_error - y_prev_error;
        double diff_z = z_error - z_prev_error;

        roll_control = roll_gain_crt*x_error + roll_gain_int*x_inte_error + roll_gain_der*diff_x;
        pitch_control = pitch_gain_crt*z_error + pitch_gain_int*z_inte_error + pitch_gain_der*diff_z;
        throttle_control = throttle_gain_crt*y_error + throttle_gain_int*y_inte_error + throttle_gain_der*diff_y;

        x_prev_error = x_error;
        y_prev_error = y_error;
        z_prev_error = z_error;
    }
}
