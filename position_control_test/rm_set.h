#ifndef RM_SET
#define RM_SET

#include "Arduino.h"

#define M3508P19 0
#define M2006P36 1

class Rm_set {
    private:

    public:
        const static int MOTOR_NUM = 8;  // amount of motors supported by 1 can bus
        // const static int MAX_RPM = 15000;  // max rpm for M3508 P19
        // const static int MAX_OUTPUT = 4096; 
        const static int GEAR_RATIO = 19; // fix this later
        const static int REVOLUTION = 8192;
        const static int NOMINAL_FPS = 1000;
        const static int MAX_ACCEL = 100; //rpm / s
        const static int NOISE_THRESHOLD = 1000;
        
        Rm_set();

        int id;

        int motor_mode[MOTOR_NUM] = {0};
        int target_rpm[MOTOR_NUM] = {0};
        int target_pos[MOTOR_NUM] = {0};
        int max_rpm[MOTOR_NUM] = {0};
        int max_output[MOTOR_NUM] = {0};
        int rpm[MOTOR_NUM] = {0};
        int pos[MOTOR_NUM] = {0};
        int gearbox_pos[MOTOR_NUM] = {0};
        int can_frame_counter[MOTOR_NUM] = {0};
        int can_frame_prev_time[MOTOR_NUM] = {0};
        int can_fps[MOTOR_NUM] = {0};
        double pid_k_rpm[MOTOR_NUM][3] = {{0.0}};
        double pid_o_rpm[MOTOR_NUM][3] = {{0.0}};
        int pid_max_i_rpm[MOTOR_NUM] = {0}; // 0.5
        double pid_k_pos[MOTOR_NUM] = {0}; // 0.5
        int rpm_error[MOTOR_NUM] = {0};
        int prev_rpm_error[MOTOR_NUM] = {0};

        int max_rpm_pos[MOTOR_NUM] = {2000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

        int prev_rpm[MOTOR_NUM] = {0};
        int prev_pos[MOTOR_NUM] = {0};
        int prev_time[MOTOR_NUM] = {0};
        int output[MOTOR_NUM] = {0};
        int t = 0;
        int fs_counter;

        // og gearbox counting
        bool cur_low[MOTOR_NUM] = {false};
        bool cur_high[MOTOR_NUM] = {false};
        bool prev_low[MOTOR_NUM] = {false};
        bool prev_high[MOTOR_NUM] = {false};


        unsigned char can_msg0[8] = {0};
        unsigned char can_msg1[8] = {0};

        // internal function
        void run_rpm_pid_cycle(int id);

        // for users to use

        void init_motor(int id, int type);
        

        // manual PID settings
        void set_pid_rpm(int id, double p, double i, double d);

        // sets the motor with ID 0..7 to rpm -9333..9333
        void set_target_rpm(int id, int rpm);

        // sets the target position (1 rotation = 3591/187 * 8192 = 157,312.68)
        void set_target_pos(int id, int pos);

        // 
        void update_motor_status(int id, long micros, int pos, int rpm);
        void reset_gearbox_pos(int id);

        void autotune_pid(int id);
        void find_limit(int id, bool dir, int threshold_current);

};

#endif