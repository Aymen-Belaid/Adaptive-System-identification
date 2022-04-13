#define ARM_MATH_CM7
#include "arm_math.h"
#include "RLS.h"
#include <stdlib.h>

arm_matrix_instance_f32 teta;
arm_matrix_instance_f32 Xk;
arm_matrix_instance_f32 Pn;

void init_rls (int sysorder)
{

	// init teta
	float32_t  * data_t = (float32_t *) malloc ((2*sysorder)*sizeof(float32_t));
	for (int i = 0 ; i < (2*sysorder) ; i++)
	{
		data_t [i] = 0;
	}
	arm_mat_init_f32 ( &teta , 2*sysorder, 1 , data_t);


	//Init Xk
	float32_t  * data_xk = (float32_t *) malloc ((2*sysorder)*sizeof(float32_t));
		for (int i = 0 ; i < (2*sysorder) ; i++)
		{
			data_xk [i] = 0;
		}
	arm_mat_init_f32 ( &Xk , 1 , 2*sysorder , data_xk);

	//Init Pn
	float32_t  * data_pn = (float32_t *) malloc (pow((2*sysorder),2)*sizeof(float32_t));

	for (int i = 0 ; i < pow((2*sysorder),2) ; i++)
	{
		data_pn [i] = 0;
	}
	for (int i = 0 ; i < 2*sysorder ; i++)
	{
		data_pn [ i*(2*sysorder + 1)] = 0.1 ;
	}
	arm_mat_init_f32 ( &Pn , 2*sysorder , 2*sysorder , data_pn);

}

arm_matrix_instance_f32 Xk_t;
arm_matrix_instance_f32  constant;
arm_matrix_instance_f32 cst_lambda;
arm_matrix_instance_f32  K ;
arm_matrix_instance_f32 Clone_K ;
arm_matrix_instance_f32 constant1;
arm_matrix_instance_f32 teta_new ;
arm_matrix_instance_f32  Pn_new ;
arm_matrix_instance_f32  cst2 ;
arm_matrix_instance_f32  cst3 ;

void init_param_calcul_teta(int sysorder)
{
    //xt
    float32_t  * data = (float32_t *) malloc ((2*sysorder)*sizeof(float32_t));
	for (int i = 0 ; i < (2*sysorder) ; i++)
	{
		data [i] = 0;
	}

	arm_mat_init_f32 ( &Xk_t , 2*sysorder , 1  , data);


    	float32_t  * data1 = (float32_t *) malloc ((2*sysorder)*sizeof(float32_t));
		for (int i = 0 ; i < (2*sysorder) ; i++)
		{
			data1 [i] = 0;
		}

	arm_mat_init_f32 (&constant , 2*sysorder , 1 , data1);



	float32_t  data_cst;
	arm_mat_init_f32 (&cst_lambda , 1 , 1 , &data_cst);


	float32_t  * data_k = (float32_t *) malloc ((2*sysorder)*sizeof(float32_t));
		for (int i = 0 ; i < (2*sysorder) ; i++)
		{
			data_k [i] = 0;
		}

	arm_mat_init_f32 ( &K ,2*sysorder , 1 , data_k);

	float32_t  data_cst1;
	arm_mat_init_f32 (&constant1 , 1 , 1 , &data_cst1);


	float32_t  * data2= (float32_t *) malloc ((2*sysorder)*sizeof(float32_t));
	for (int i = 0 ; i < (2*sysorder) ; i++)
	{
		data2 [i] = 0;
	}

	arm_mat_init_f32 ( &cst2  , 1 ,2*sysorder , data2);


	float32_t  * data3= (float32_t *) malloc ((2*sysorder*2*sysorder)*sizeof(float32_t));
	for (int i = 0 ; i < (2*sysorder*2*sysorder) ; i++)
	{
		data3 [i] = 0;
	}

	arm_mat_init_f32 ( &cst3  , 2*sysorder ,2*sysorder , data3);

    float32_t  * data_teta = (float32_t *) malloc ((2*sysorder)*sizeof(float32_t));
	for (int i = 0 ; i < (2*sysorder) ; i++)
	{
		data_teta [i] = 0;
	}
	arm_mat_init_f32 ( &teta_new , 2*sysorder, 1 , data_teta);

    	float32_t  * data_pn_new = (float32_t *) malloc (pow((2*sysorder),2)*sizeof(float32_t));

	for (int i = 0 ; i < pow((2*sysorder),2) ; i++)
	{
		data_pn_new [i] = 0;
	}
	for (int i = 0 ; i < 2*sysorder ; i++)
	{
		data_pn_new [ i*(2*sysorder + 1)] = 0.1 ;
	}
	arm_mat_init_f32 ( &Pn_new , 2*sysorder , 2*sysorder , data_pn_new);

}

void teta_calc (arm_matrix_instance_f32  Xk , float output_t , float lambda ,int sysorder )
{
	arm_status status;
	// calcul de K
	status = arm_mat_trans_f32 (&Xk , &Xk_t);

	status = arm_mat_mult_f32 (&Pn , &Xk_t , &constant); // constant == Pn*Xk'

	status = arm_mat_mult_f32 (&Xk , &constant , &cst_lambda);// cst_lambda == Xk*(Pn*Xk')

	*(cst_lambda.pData) = *(cst_lambda.pData) + (float32_t)lambda; // cst_lambda = lambda + Xk*(Pn*Xk')

	status = arm_mat_mult_f32 (&Pn , &Xk_t , &K); // K == Pn*Xk'

	for (int i = 0 ; i < 2*sysorder ; i++)
	{
		K.pData [i] = (K.pData [i])/(*(cst_lambda.pData)); /* K == (Pn * Xk')./(cst_lambda = (lambda + Xk*(Pn*Xk')))*/
	}
	Clone_K = K;

	// teta

	status = arm_mat_mult_f32 (&Xk , &teta , &constant1); // constant1 = Xk * teta

	float32_t cst = (float32_t) output_t - *(constant1.pData); // cst = ( Y(t,:) - Xk * teta)

	for (int i = 0 ; i < 2*sysorder ; i++)
		{
		Clone_K.pData [i] = (Clone_K.pData [i])*cst ;
		}


	status = arm_mat_add_f32 (&teta , &Clone_K , &teta_new);
	teta = teta_new;


	status = arm_mat_mult_f32 (&Xk , &Pn , &cst2); // cst2 = Xk*Pn


	status = arm_mat_mult_f32 (&K , &cst2 , &cst3); // cst3 == K*(Xk*Pn)


	status = arm_mat_sub_f32 (&Pn , &cst3 , &Pn_new);

	for (int i = 0 ; i < ((2*sysorder)*(2*sysorder)) ; i ++)
	{
		Pn_new.pData [i] = Pn_new.pData [i] / lambda ;
	}
	Pn= Pn_new;

}


void update_X (float * X , float input , float output , int sysorder)
{
	// Yk
	for (int i = (sysorder -1) ; i>0 ; i--)
	{
		X[i] = X[i-1];
	}
	X[0] = (-1) * output;

	// Uk
	for (int i = (2*sysorder -1) ; i > sysorder ; i--)
	{
		X[i] = X[i-1];
	}
	X[sysorder] = input;
	arm_mat_init_f32 ( &Xk , 1 , 2*sysorder , X);

}



float32_t rep_sys (void)
{
	arm_matrix_instance_f32 rep;
	float32_t  resp_output;
	arm_mat_init_f32 (&rep , 1 , 1 , &resp_output);
	arm_mat_mult_f32( &Xk, &teta , &rep);
	return (*(rep.pData));


}

