#ifndef INC_RLS_H_
#define INC_RLS_H_

void init_rls (int sysorder); // Init RLS algorithm parameters
void init_param_calcul_teta(int sysorder);
void teta_calc (arm_matrix_instance_f32 Xk , float output_t , float lambda , int sysorder); // input & output à l'instant t
void update_X (float * X , float input , float output , int sysorder); // X parameters update after each elapsed sample period
float32_t rep_sys (void); // System's response after transfer function estimation


#endif /* INC_RLS_H_ */
