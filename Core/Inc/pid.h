#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float last_error;

    float out_min;
    float out_max;
    float integral_min;
    float integral_max;
} PID_t;

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max,
              float integral_min,
              float integral_max);

float PID_Update(PID_t *pid, float target, float feedback, float dt);

float PID_Update_With_D(PID_t *pid,
                        float target,
                        float feedback,
                        float derivative_feedback,
                        float dt);

void PID_Reset(PID_t *pid);



#ifdef __cplusplus
}
#endif


#endif // PID_H