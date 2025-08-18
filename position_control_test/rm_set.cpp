#include "rm_set.h"

void Rm_set::init_motor(int id, int type){
  if (type == M3508P19){
    max_output[id] = 8192;
    max_rpm[id] = 9000;
  } else {
    max_output[id] = 4096;
    max_rpm[id] = 15000;
  }
}

Rm_set::Rm_set(){
  for(int i = 0; i < this->MOTOR_NUM; i++){
    this->pid_max_i_rpm[i] = 1000;
    this->pid_k_pos[i] = 0.3;
    init_motor(i, M3508P19);
  }
}

void Rm_set::set_pid_rpm(int id, double p, double i, double d){
    this->pid_k_rpm[id][0] = p;
    this->pid_k_rpm[id][1] = i;
    this->pid_k_rpm[id][2] = d;
}
void Rm_set::set_target_rpm(int id, int rpm){
    this->motor_mode[id] = 0;
    this->target_rpm[id] = rpm;
    this->target_rpm[id] = this->target_rpm[id] > max_rpm[id] ? max_rpm[id] : (this->target_rpm[id] < -max_rpm[id] ? -max_rpm[id] : this->target_rpm[id]);
}

void Rm_set::run_rpm_pid_cycle(int id){
    this->prev_rpm_error[id] = this->rpm_error[id];
    this->rpm_error[id] = this->target_rpm[id] - this->rpm[id];
    this->rpm_error[id] = this->rpm[id] > 32768 ? this->rpm_error[id] + 65536: this->rpm_error[id];

    this->pid_o_rpm[id][0] = this->pid_k_rpm[id][0] * this->rpm_error[id];
    this->pid_o_rpm[id][1] += pid_k_rpm[id][1] * this->rpm_error[id];
    this->pid_o_rpm[id][2] = this->pid_k_rpm[id][2] * (this->rpm_error[id] - this->prev_rpm_error[id]);
    this->pid_o_rpm[id][1] = this->pid_o_rpm[id][1] > this->pid_max_i_rpm[id] ? this->pid_max_i_rpm[id] : (this->pid_o_rpm[id][1] < -this->pid_max_i_rpm[id] ? -this->pid_max_i_rpm[id] : this->pid_o_rpm[id][1]);

    this->output[id] = this->pid_o_rpm[id][0] + this->pid_o_rpm[id][1] + this->pid_o_rpm[id][2];
    this->output[id] = this->output[id] > max_output[id] ? max_output[id] : (this->output[id] < -max_output[id] ? -max_output[id] : this->output[id]);
    for (int m = 0; m < 4; m++){
      this->can_msg0[m * 2] = (unsigned char)(this->output[m] >> 8);
      this->can_msg0[m * 2 + 1] = this->output[m] & 0xff;
    }
    for (int m = 0; m < 4; m++){
      this->can_msg1[m * 2] = (unsigned char)(this->output[m + 4] >> 8);
      this->can_msg1[m * 2 + 1] = this->output[m + 4] & 0xff;
    }
}

void Rm_set::update_motor_status(int id, long micros, int pos, int rpm){
    this->prev_rpm[id] = this->rpm[id];
    this->prev_pos[id] = this->pos[id];
    this->rpm[id] = rpm;
    this->pos[id] = pos;
    int dt = micros - this->prev_time[id];
    this->prev_time[id] = micros;
    this->can_frame_counter[id] += 1;
    if (micros - this->can_frame_prev_time[id] > 100000){
      this->can_frame_prev_time[id] = micros;
      this->can_fps[id] = 10 * can_frame_counter[id];
      this->can_frame_counter[id] = 0;
    }
    if (dt > 1500){
      fs_counter += 1;
    }
    
    double ds = 0;
    // algorithm a
    /*
    if (this->rpm[id] <= 32768 && this->rpm[id] > this->NOISE_THRESHOLD && this->prev_pos[id] - 200 > this->pos[id]){  // positive but new pos < old pos
        ds += this->REVOLUTION;
    } 
    else if (this->rpm[id] > 32768 && this->rpm[id] < 65536 - this->NOISE_THRESHOLD && this->prev_pos[id] + 200 < this->pos[id]){  //negative but new pos > old pos
        ds -= this->REVOLUTION;
    }
    else if (this->rpm[id] <= this->NOISE_THRESHOLD || this->rpm[id] >= 65536 - this->NOISE_THRESHOLD){
      if (this->prev_pos[id] - 4096 > this->pos[id]){  // noise cancellation ????
        ds += this->REVOLUTION;
      }
      else if (this->prev_pos[id] + 4096 < this->pos[id]){  // noise cancellation ????
        ds -= this->REVOLUTION;
      } 
    }
    */
    // algorithm b
    
    this->prev_low[id] = this->cur_low[id];
    this->prev_high[id] = this->cur_high[id];
    this->cur_low[id] = this->pos[id] < 2048;
    this->cur_high[id] = this->pos[id] > this->REVOLUTION - 2048;
    if (this->prev_low[id] && this->cur_high[id]){
      ds -= this->REVOLUTION;
    }
    if (this->prev_high[id] && this->cur_low[id]){
      ds += this->REVOLUTION; 
    }
    
    // a+b
    /*
    this->prev_low[id] = this->cur_low[id];
    this->prev_high[id] = this->cur_high[id];
    this->cur_low[id] = this->pos[id] < 2048;
    this->cur_high[id] = this->pos[id] > this->REVOLUTION - 2048;
    if (this->rpm[id] <= 32768 && this->rpm[id] > this->NOISE_THRESHOLD && this->prev_pos[id] - 200 > this->pos[id]){  // positive but new pos < old pos
        ds += this->REVOLUTION;
    } 
    else if (this->rpm[id] > 32768 && this->rpm[id] < 65536 - this->NOISE_THRESHOLD && this->prev_pos[id] + 200 < this->pos[id]){  //negative but new pos > old pos
        ds -= this->REVOLUTION;
    }
    else if (this->rpm[id] <= this->NOISE_THRESHOLD && this->prev_high[id] && this->cur_low[id]){  // noise cancellation ????
        ds += this->REVOLUTION;
    }
    else if (this->rpm[id] >= 65536 - this->NOISE_THRESHOLD && this->prev_low[id] && this->cur_high[id]){  // noise cancellation ????
        ds -= this->REVOLUTION;
    }

    */
    ds += this->pos[id] - this->prev_pos[id];
    
    this->gearbox_pos[id] += ds;
    if (this->motor_mode[id] == 1){  // position control
      int dp = this->target_pos[id] - this->gearbox_pos[id];
      // ganchulia principle
      this->target_rpm[id] = dp * pid_k_pos[id];
      this->target_rpm[id] = constrain(this->target_rpm[id], -max_rpm_pos[id], max_rpm_pos[id]);
    }
    this->run_rpm_pid_cycle(id);
}

void Rm_set::set_target_pos(int id, int pos){
    this->motor_mode[id] = 1;
    this->target_pos[id] = pos;
}

void Rm_set::reset_gearbox_pos(int id){
    this->gearbox_pos[id] = 0;
}

void autotune_pid(){
  
}

void find_limit(int id, bool dir, int threshold_current){
  ;
}