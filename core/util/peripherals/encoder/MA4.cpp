#include "MA4.h"

MA4::MA4(const struct pwm_dt_spec *pwm, bool invert) 
    : _pwm(pwm), _invert(invert), _pulsewidth(0.0f), _period(0.0f) {

    if (!device_is_ready(_pwm->dev)) {
        // printf("ERROR: ENCODER WIRING IS WRONG(?)")
        return;
    }

    pwm_configure_capture(_pwm->dev, _pwm->channel,
                        PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_CONTINUOUS, // Setting it to capture both pulse and period 
                        MA4::capture_cb, this);

    pwm_enable_capture(_pwm->dev, _pwm->channel);                       
}

void MA4::on_capture(uint32_t period_cycles, uint32_t pulse_cycles, int status) {
    if (status != 0) { //Means that we're not ready 
        return;
    }

    uint64_t period_ns, pulse_ns;
    pwm_cycles_to_nsec(_pwm->dev, _pwm->channel, period_cycles, &period_ns);
    pwm_cycles_to_nsec(_pwm->dev, _pwm->channel, pulse_cycles, &pulse_ns);

    // Unit Conversion
    _period = period_ns / 1.0e6f;
    _pulsewidth = pulse_ns / 1.0e6f;
}

void MA4::capture_cb(const struct device *dev, uint32_t channel, uint32_t period_cycles, uint32_t pulse_cycles, int status, void *user_data) {
    static_cast<MA4 *>(user_data)->on_capture(period_cycles, pulse_cycles, status);
}


float MA4::period() {
    return _period;
}

float MA4::pulsewidth() {
    return _pulsewidth;
}

float MA4::dutycycle() {
    if (_period == 0.0f) {
        return 0.0f;  // Return 0 until first valid PWM cycle
    }
    return _pulsewidth / _period;
}

// void MA4::rise() {
//     _period = chrono::duration_cast<chrono::microseconds>(_t.elapsed_time()).count() / 1000.0f;
//     _t.reset();
// }

// void MA4::fall() {
//     _pulsewidth = chrono::duration_cast<chrono::microseconds>(_t.elapsed_time()).count() / 1000.0f;
// }

float MA4::getEncoderYawPosition() {
    static float filtered_yaw = 0.0f;
    float filter_alpha = 0.2f;  // 0.0-1.0: lower = more smoothing, higher = more responsive

    float duty_raw = dutycycle();
    float duty_min = 0.02943f;   // 2.943%
    float duty_max = 0.97058f;   // 97.058%

    //low pass filter 
    float yaw_position = (fabsf(((duty_raw - duty_min) / (duty_max - duty_min)) * 360.0f));
    filtered_yaw = filtered_yaw * (1.0f - filter_alpha) + yaw_position *  filter_alpha;
    // printf("%.2f\n",yaw_position);
    if (_invert) {
        return (360.0f - filtered_yaw);
    }
    return filtered_yaw;
}

float MA4::encoderMovingAverage() {
    const int windowSize = 20;
    static float readings[windowSize] = {0};
    static int index = 0;
    static bool filled = false;

    float newReading = getEncoderYawPosition();
    if (newReading < 0) {
        return -1.0f; // Encoder not available
    }

    readings[index] = newReading;
    index = (index + 1) % windowSize;
    if (index == 0) {
        filled = true;
    }

    float sum = 0.0;
    float result = 0.0;

    if (filled) {
        for (int i = 0; i < windowSize; i++) {
            sum += readings[i];
        }
        result = sum / windowSize;
    } else {
        for (int i = 0; i < index; i++) {
            sum += readings[i];
        }
        result = sum / index;
    }
    // printf("%.2f\n",result);
    return result;

}