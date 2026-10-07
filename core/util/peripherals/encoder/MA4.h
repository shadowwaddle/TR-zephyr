
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <util/algorithms/general_functions.h>
#ifndef MA4_H
#define MA4_H

/** PwmIn class to read PWM inputs
 * 
 * Uses InterruptIn to measure the changes on the input
 * and record the time they occur
 *
 * @note uses InterruptIn, so not available on p19/p20
 */
class MA4 {
public:
    /** Constructor
     *
     * @param pwm Struct with both dev and channel 
     * @param invert invert cw and ccw cuz ma4 is a hoe
     */ 
    MA4(const struct pwm_dt_spec *pwm, bool invert=false);
    
    /** Read the current period
     *
     * @returns the period in seconds
     */
    float period();
    
    /** Read the current pulsewidth
     *
     * @returns the pulsewidth in seconds
     */
    float pulsewidth();
    
    /** Read the current dutycycle
     *
     * @returns the dutycycle as a percentage, represented between 0.0-1.0
     */
    float dutycycle();

    /**
     * Gets the yaw position from encoder (PWM) input in degrees (0-360)
     * @return yaw position in degrees, or -1 if encoder not available
     */
    float getEncoderYawPosition();

    /**
     * A helper method to calculate the moving average of the encoder readings for yaw position
     * @return the moving average of the encoder readings for yaw position
     */
    float encoderMovingAverage();
    

protected:

    // Capture Callback
    static void capture_cb(const struct device *dev, uint32_t channel,
                            uint32_t period_cycles, uint32_t pulse_cycles,
                            int status, void *user_data);
    
    // Update _period and _pulse
    void on_capture(uint32_t period_cycles, uint32_t pulse_cycles, int status);

    // void rise();
    // void fall();
    
    // InterruptIn _p;
    const struct pwm_dt_spec *_pwm; 
    // Timer _t;
    bool _invert;
    volatile float _pulsewidth, _period; // Volatile since Zephyr does PWM stuff thru interrupts, and so we're basically telling the compiler
                                         // that we want to actually check what these are and not use any cache'd results
};

#endif
