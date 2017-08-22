

// output_armed - sends commands to the motors
// includes new scaling stability patch
void output_armed_stabilizing()
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   throttle_thrust_rpy_mix;    // partial calculation of throttle_thrust_best_rpy
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   rpy_low = 0.0f;             // lowest motor value
    float   rpy_high = 0.0f;            // highest motor value
    float   yaw_allowed = 1.0f;         // amount of yaw we can fit in
    float   unused_range;               // amount of yaw we can fit in the current channel
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
    float   throttle_thrust_hover = get_hover_throttle_as_high_end_pct();   // throttle hover thrust value, 0.0 - 1.0

    // apply voltage and air pressure compensation
    roll_thrust = _roll_in * get_compensation_gain();
    pitch_thrust = _pitch_in * get_compensation_gain();
    yaw_thrust = _yaw_in * get_compensation_gain();
    throttle_thrust = get_throttle() * get_compensation_gain();

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    throttle_thrust_rpy_mix = MAX(throttle_thrust, throttle_thrust*MAX(0.0f,1.0f-_throttle_rpy_mix)+throttle_thrust_hover*_throttle_rpy_mix);

    // calculate throttle that gives most possible room for yaw which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // calculate amount of yaw we can fit into the throttle range
    // this is always equal to or less than the requested yaw from the pilot or rate controller

    throttle_thrust_best_rpy = MIN(0.5f, throttle_thrust_rpy_mix);

    // calculate roll and pitch for each motor
    // calculate the amount of yaw input that each motor can accept
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
            if (!is_zero(_yaw_factor[i])){
                if (yaw_thrust * _yaw_factor[i] > 0.0f) {
                    unused_range = fabsf((1.0f - (throttle_thrust_best_rpy + _thrust_rpyt_out[i]))/_yaw_factor[i]);
                    if (yaw_allowed > unused_range) {
                        yaw_allowed = unused_range;
                    }
                } else {
                    unused_range = fabsf((throttle_thrust_best_rpy + _thrust_rpyt_out[i])/_yaw_factor[i]);
                    if (yaw_allowed > unused_range) {
                        yaw_allowed = unused_range;
                    }
                }
            }
        }
    }

    // todo: make _yaw_headroom 0 to 1
    yaw_allowed = MAX(yaw_allowed, (float)_yaw_headroom/1000.0f);

    if (fabsf(yaw_thrust) > yaw_allowed) {
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw = true;
    }

    // add yaw to intermediate numbers for each motor
    rpy_low = 0.0f;
    rpy_high = 0.0f;
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];

            // record lowest roll+pitch+yaw command
            if (_thrust_rpyt_out[i] < rpy_low) {
                rpy_low = _thrust_rpyt_out[i];
            }
            // record highest roll+pitch+yaw command
            if (_thrust_rpyt_out[i] > rpy_high) {
                rpy_high = _thrust_rpyt_out[i];
            }
        }
    }

    // check everything fits
    throttle_thrust_best_rpy = MIN(0.5f - (rpy_low+rpy_high)/2.0, throttle_thrust_rpy_mix);
    if (is_zero(rpy_low)){
        rpy_scale = 1.0f;
    } else {
        rpy_scale = constrain_float(-throttle_thrust_best_rpy/rpy_low, 0.0f, 1.0f);
    }

    // calculate how close the motors can come to the desired throttle
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0f){
        // Full range is being used by roll, pitch, and yaw.
        limit.roll_pitch = true;
        limit.yaw = true;
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    } else {
        if (thr_adj < -(throttle_thrust_best_rpy+rpy_low)){
            // Throttle can't be reduced to desired value
            thr_adj = -(throttle_thrust_best_rpy+rpy_low);
        } else if (thr_adj > 1.0f - (throttle_thrust_best_rpy+rpy_high)){
            // Throttle can't be increased to desired value
            thr_adj = 1.0f - (throttle_thrust_best_rpy+rpy_high);
            limit.throttle_upper = true;
        }
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = throttle_thrust_best_rpy + thr_adj + rpy_scale*_thrust_rpyt_out[i];
        }
    }

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i], 0.0f, 1.0f);
        }
    }
}
