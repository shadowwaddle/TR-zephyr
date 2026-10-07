// #include "stm32f446xx.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <util/communications/DJIRemote2.h>
#include <util/communications/CANHandler.h>
#include <util/communications/jetson/Jetson.h>
#include <util/peripherals/imu/ISM330.h>
#include <util/algorithms/PID.h>
#include <base_robot/BaseRobot.h>
#include <zephyr/dt-bindings/pwm/pwm.h>


// Robot Constants
constexpr float PITCH_LOWER_BOUND{-22.0};
constexpr float PITCH_UPPER_BOUND{25.0};

constexpr float JOYSTICK_YAW_SENSITIVITY_DPS = 300;
constexpr float JOYSTICK_PITCH_SENSITIVITY_DPS = 150;
// Mouse sensitivity initialized
constexpr float MOUSE_SENSITIVITY_YAW_DPS = 1.0;
constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 1.0;

// constexpr PID::config YAW_VEL_PID     = {181, 3.655 * 10e-3, 4.51 * 7.5, 32000, 1000};
constexpr PID::config YAW_VEL_PID     = {181,0,10, 32000, 1000};
constexpr PID::config YAW_POS_PID     = {1, 0, 0, 45, 2};
const float yaw_static_friction       = 0;//-150;       // We multiply it by dir
const float yaw_kinetic_friction      = 0;       // We multiply this by yawvelo

constexpr PID::config PITCH_VEL_PID   = {173.8994f, 4.898f * static_cast<float>(10e-6), 12.474f * static_cast<float>(10e3) * 1.5f, 16000, 2000}; //{25, 0.001, 5, 16000, 1000};
// constexpr PID::config PITCH_VEL_PID = {170, 0, 1200, 16000,2000}; //{25, 0.001, 5, 16000, 1000};
constexpr PID::config PITCH_POS_PID   = {1, 0, 0,30,2}; //{1, 0, 0, 30, 2};
const float pitch_gravity_feedforward = -1200;    // We multiply this by cos(angle)
const float pitch_static_friction     = 0;       // We multiply it by dir
const float pitch_kinetic_friction    = 0; //5.5;     // We multiply this by pitchvelo

constexpr PID::config FL_VEL_CONFIG = {3, 0, 0};
constexpr PID::config FR_VEL_CONFIG = {3, 0, 0};
constexpr PID::config BL_VEL_CONFIG = {3, 0, 0};
constexpr PID::config BR_VEL_CONFIG = {3, 0, 0};

constexpr PID::config FLYWHEEL_L_PID = {7.1849, 0.000042634, 0};
constexpr PID::config FLYWHEEL_R_PID = {7.1849, 0.000042634, 0};
constexpr PID::config INDEXER_PID_VEL = {2.7, 0.001, 0};
constexpr PID::config INDEXER_PID_POS = {0.1, 0, 0.001};


const struct device *gpio_devb = DEVICE_DT_GET(DT_NODELABEL(gpiob));
const struct device *gpio_devc= DEVICE_DT_GET(DT_NODELABEL(gpioc));

// Devices from DT

const struct device *canbus1_dev = DEVICE_DT_GET(DT_NODELABEL(can1));

const struct device *canbus2_dev = DEVICE_DT_GET(DT_NODELABEL(can2));
constexpr short yaw_id = 3;
constexpr short pitch_id = 5;

const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);


static const struct i2c_dt_spec imu_spec = I2C_DT_SPEC_GET(DT_NODELABEL(imu));

static const struct pwm_dt_spec encoderSpec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_encoder_ch));
static const struct device *controllerUart = DEVICE_DT_GET(DT_NODELABEL(usart1));
static const struct device *refUartDev = DEVICE_DT_GET(DT_NODELABEL(usart3));
// DJIRemote2 controller(controllerUart);

static const struct device *jetsonUart = DEVICE_DT_GET(DT_NODELABEL(uart5));

// Subsystem Configs
TurretSubsystem::config turret_config = {
    canbus1_dev,
    CANHandler::CANBUS_1,
    yaw_id,
    M3508,
    canbus2_dev,
    CANHandler::CANBUS_2, // TODO: Ideally we could grab canbus thru the device
    pitch_id,
    M3508,
    YAW_VEL_PID,
    YAW_POS_PID,
    PITCH_VEL_PID,
    PITCH_POS_PID,
    yaw_static_friction,
    yaw_kinetic_friction,
    pitch_gravity_feedforward,
    pitch_static_friction,
    pitch_kinetic_friction,
    1,
    1.0/3.0,
    PITCH_LOWER_BOUND,
    PITCH_UPPER_BOUND
};
ShooterSubsystem::config shooter_config = {
    canbus2_dev,
    CANHandler::CANBUS_2,
    ShooterSubsystem::BURST,
    0,
    2,
    4,
    6,
    FLYWHEEL_L_PID,
    FLYWHEEL_R_PID,
    INDEXER_PID_VEL,
    INDEXER_PID_POS,
    false
};

// State variables
ChassisSpeeds des_chassis_state;
TurretSubsystem::TurretInfo des_turret_state;
ShootState des_shoot_state;

float pitch_desired_angle = 0.0;
float yaw_desired_angle = 0.0;
float dt_global = 0.0;

IMU::EulerAngles imuAngles;

class Infantry : public BaseRobot {
  public:
    ISM330 imu_;
    MA4 encoder_;  
    // BufferedSerial jetson_raw_serial;
    Jetson jetson;

    Jetson::WriteState stm_state;
    Jetson::ReadState jetson_state;

    TurretSubsystem turret_;
    ShooterSubsystem shooter_;
    ChassisSubsystem chassis_;

    bool imu_initialized{false};

    Infantry(Config &config)
        : BaseRobot(config),
          // clang-format off
        imu_(imu_spec),
        // encoder_(PB_4),
        encoder_(&encoderSpec),

        // jetson_raw_serial(PC_12, PD_2,115200),
        // jetson(jetson_raw_serial),
        jetson(jetsonUart),
        turret_(turret_config, imu_),
        shooter_(shooter_config),

        // TODO add passing in individual PID objects for the motors
        chassis_(ChassisSubsystem::Config{
            5,      // left_front_can_id
            1,      // right_front_can_id
            2,      // left_back_can_id
            4,      // right_back_can_id
            0.22617,  // radius
            0.065,    // speed_pid_ff_ks
            35,     // yaw_initial_offset_ticks
            imu_,
            &encoder_   
        }
        )
    // clang-format on
    {
        // pin_mode(IMU_I2C_SCL, PinMode::OpenDrainPullUp);
        // pin_mode(IMU_I2C_SDA, PinMode::OpenDrainPullUp);
    }

    ~Infantry() {}

    void init() override {
        // timer = us_ticker_read();
            if (!device_is_ready(controllerUart)) {
    printf("[ERROR] controllerUart (usart1) not ready!\n");
    }
        imu_.begin(0.9, 0);
    }

    void periodic(unsigned long dt_us) override {
        // TODO this should be threaded inside imu instead
        imu_.mahonyUpdateIMU(dt_us / 1000000.0);
        imuAngles = imu_.getImuAngles();
        // TODO: use this in code correctly to drive faster
        max_linear_vel = MAX_VEL;
        des_chassis_state.vX = jy * max_linear_vel;
        des_chassis_state.vY = jx * max_linear_vel;

        // Read jetson
        jetson_state = jetson.read();
        // check if new jetson state given and if we want cv
        if (cv_enabled_ && (now_us() - jetson_state.stamp_us ) / 1000 < 500) {
            yaw_desired_angle = jetson_state.desired_yaw_rads * 180 / PI;
            pitch_desired_angle = jetson_state.desired_pitch_rads * 180 / PI;
        }

        // Turret from remote
        yaw_desired_angle -= myaw * 0.01f;
        yaw_desired_angle -= jyaw * JOYSTICK_YAW_SENSITIVITY_DPS * dt_us / 1000000;
        yaw_desired_angle = capAngle(yaw_desired_angle);
        des_turret_state.yaw_angle_degs = yaw_desired_angle;

        pitch_desired_angle -= mpitch * 0.01f;
        pitch_desired_angle -= jpitch * JOYSTICK_PITCH_SENSITIVITY_DPS * dt_us / 1000000;
        pitch_desired_angle = std::clamp(pitch_desired_angle, PITCH_LOWER_BOUND, PITCH_UPPER_BOUND);
        des_turret_state.pitch_angle_degs = pitch_desired_angle;

        // Chassis logic
        if (drive == 'u' || (drive == 'o' && remote_.getMode() == DJIRemote2::ModeSwitch::MODE_N)) {
            des_chassis_state.vOmega = 0;
            chassis_.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);
            des_turret_state.turret_mode = TurretState::AIM;
            referee_.is_aligned = false;
            referee_.is_cv_on = false;
            referee_.is_spinning = false;
        }  else if (drive == 'd' || 
                   (drive == 'o' &&
                    remote_.getMode() == DJIRemote2::ModeSwitch::MODE_S)) {
            des_chassis_state.vOmega = omega_speed;
            chassis_.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);
            des_turret_state.turret_mode = TurretState::AIM;
            referee_.is_aligned = false;
            referee_.is_cv_on = false;
            referee_.is_spinning = true;
        } else {
            chassis_.setWheelPower({0, 0, 0, 0});
            des_turret_state.turret_mode = TurretState::SLEEP;
            des_turret_state.yaw_angle_degs = turret_.getState().yaw_angle_degs;
            yaw_desired_angle = turret_.getState().yaw_angle_degs;
            des_turret_state.pitch_angle_degs = 0;
            referee_.is_aligned = false;
            referee_.is_cv_on = false;
            referee_.is_spinning = false;
        }

        // Shooter Logic 
        //REMOVED remote_.PAUSEToggled() == true && FROM THE FIRST CONDITION
        if ((remote_.PAUSEToggled() == true && remote_.TriggerPressed() == true) || remote_.getMouseL()) {
            des_shoot_state = ShootState::SHOOT;
        } else if (remote_.CUSTRPressed() == true && remote_.PAUSEToggled() == true) {
            des_shoot_state = ShootState::JAM;
        } else if (remote_.PAUSEToggled() == true || shot == 'd') {
            des_shoot_state = ShootState::FLYWHEEL;
            referee_.is_flywheel_on = true;
        } else {
            des_shoot_state = ShootState::OFF;
            referee_.is_flywheel_on = false;
        }

        turret_.setState(des_turret_state);
        shooter_.setState(des_shoot_state);

        turret_.periodic(chassis_.getChassisSpeeds().vOmega * 60 / (2 * PI));

        float lim = referee_.robot_status.chassis_power_limit;

        if (lim <= 0) {
            lim = 80;
        } 
        chassis_.power_limit = lim;
        
        
        chassis_.periodic(&imuAngles);
        shooter_.periodic(referee_.power_heat_data.shooter_17mm_1_barrel_heat,
                         referee_.robot_status.shooter_barrel_heat_limit);

        // jetson comms
        set_jetson_state();
        jetson.write(stm_state);

        // printf("time %.4f\n", dt_us / 1000000.0);

        // Debug print statements
        // printf("des: %.2f, %.2f, %.2f %d \n", jetson.read().desired_x_vel,
        // jetson.read().desired_y_vel, jetson.read().desired_angular_vel,
        // jetson.read().localization_calibration); printf("y: %.2f\n",
        // turret.getState().yaw_angle); printf("p: %.2f\n",
        // turret.getState().pitch_angle + remote_.getPitch()); printf("p:
        // %.2f\n", turret.getState().pitch_angle); printf("%d\n",
        // shooter.getState()); printf("v:%d\n",testmot>>VELOCITY); printf("cx:
        // %.2f\n", remote_.getChassisX()); printf("switch: %d\n",
        // remote_.getSwitch(Remote::Switch::RIGHT_SWITCH));
        // %.2f\n", imu.getImuAngles().yaw);
        // printf("%d\n", referee_.get_game_progress());
        // printf("yp %.2f \n", encoder_.encoderMovingAverage());
        // printf("%.2f, %.2f, %.2f\n", imuAngles.roll, imuAngles.pitch, imuAngles.yaw);
        // printf("remote state: %d\n", remote_.getMode());
        // printf("remote jx: %.2f, jy: %.2f, jpitch: %.2f, jyaw: %.2f\n", jx, jy, jpitch, jyaw);
        // remote_.printMissedPackets();
        // printf("Chassis motor speeds: %.2f, %.2f, %.2f, %.2f\n", chassis_.getMotorSpeed(ChassisSubsystem::LEFT_FRONT, ChassisSubsystem::METER_PER_SECOND), chassis_.getMotorSpeed(ChassisSubsystem::RIGHT_FRONT, ChassisSubsystem::METER_PER_SECOND), chassis_.getMotorSpeed(ChassisSubsystem::LEFT_BACK, ChassisSubsystem::METER_PER_SECOND), chassis_.getMotorSpeed(ChassisSubsystem::RIGHT_BACK, ChassisSubsystem::METER_PER_SECOND));
        // printf("Chassis speeds: %.2f, %.2f, %.2f, %.2f\n", chassis_.LF.getData(VELOCITY), chassis_.RF.getData(VELOCITY), chassis_.LB.getData(VELOCITY), chassis_.RB.getData(VELOCITY));
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop

    void set_jetson_state() {
        stm_state.activate_CV = cv_enabled_;
        stm_state.game_state = referee_.get_game_progress();
        stm_state.robot_hp = referee_.get_remain_hp();
        stm_state.team_color = referee_.is_red_or_blue();

        stm_state.chassis_x_velocity = chassis_.getChassisSpeeds().vX;
        stm_state.chassis_y_velocity = chassis_.getChassisSpeeds().vY;
        stm_state.chassis_rotation = chassis_.getChassisSpeeds().vOmega;

        stm_state.yaw_angle_rads = degreesToRadians(turret_.getState().yaw_angle_degs);
        stm_state.yaw_velocity = degreesToRadians(turret_.getState().yaw_velo_rad_s);
        stm_state.pitch_angle_rads = degreesToRadians(turret_.getState().pitch_angle_degs);
        stm_state.pitch_velocity = degreesToRadians(turret_.getState().pitch_velo_rad_s);
    }
};


int main(void)
{
    printf("HELLO\n");
    
    // I'm sure there's a cleaner way to do this but for now it's getting set in main
    BaseRobot::Config config = BaseRobot::Config{};
    config.led0_dev = &led0_dev;
    config.led1_dev = &led1_dev;
    config.led2_dev = &led2_dev;
    config.controller_uart_dev = controllerUart;
    config.referee_uart_dev = refUartDev;
    static Infantry infantry(config);




    infantry.main_loop();
    // // blocking
}
