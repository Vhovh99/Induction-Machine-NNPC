#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H


void open_loop_voltage_control(float Vd, float Vq, float angle_rad);
void svpwm_test(void);

#endif /* __FOC_CONTROL_H */