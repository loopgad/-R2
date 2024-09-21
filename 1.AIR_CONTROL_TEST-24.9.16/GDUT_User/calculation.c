#include "main.h"
#include "calculation.h"

/*!
 * \fn     Kinematic_Analysis
 * \brief  ��ȫ�����˶������?
 *          ֱ������Ŀ���ٶ�ֵ--->��λ����ÿ��
 * 			�����ڲ����㣺��ÿ��--->תÿ���ӣ����ӣ�--->תÿ���ӣ�ת�ӣ�
 * \param  [in] float Vx   #
 * \param  [in] float Vy   #
 * 
 * \retval void
 */
 	 
void Kinematic_Analysis_Inverse(void)
{

	MOTOR_REAL_INFO[0].TARGET_RPM = 19*((0 + Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);
	MOTOR_REAL_INFO[1].TARGET_RPM = 19*((-(number/2) * Robot_Chassis.Robot_V[y] - (0.5) * Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);
	MOTOR_REAL_INFO[2].TARGET_RPM = 19*(((number/2) * Robot_Chassis.Robot_V[y] - (0.5) * Robot_Chassis.Robot_V[x] + RADIUS * Robot_Chassis.Robot_V[w])*60)/(Radius_Wheel*2*PI);
}
/*!
 * \fn     Axis_analyse_for_WORLDtoROBOT
 * \brief  �����������ϵ���ٶ������������ϵ�µ��ٶ�ת
           ��
 *          
 *		   �õ������ٶ�
 * \param  [in] ROBOT_CHASSIS* ROBOT_NOW_INFO_IN_ITSELF   #
 * \param  [in] ROBOT_CHASSIS* ROBOT_NOW_INFO_IN_WORLD    #
 * 
 * \retval void
 */
void Axis_analyse_for_WORLDtoROBOT(void)
{
	/*�Ą������Ӌ��ĽǶ�ֵ*/
	// x�����ϵ�ת��
	Robot_Chassis.Robot_V[x]= cos(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[x]) + sin(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[y]);
	// y�����ϵ�ת��
	Robot_Chassis.Robot_V[y]= -sin(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180) * (Robot_Chassis.World_V[x]) + cos(-ACTION_GL_POS_DATA.REAL_YAW* PI / 180)*(Robot_Chassis.World_V[y]);
	
//	// x�����ϵ�ת��
//	Robot_Chassis.Robot_V[x]= Robot_Chassis.World_V[x];
//	// y�����ϵ�ת��.
//	Robot_Chassis.Robot_V[y]= Robot_Chassis.World_V[y];
	
	// W�����ϵ�ת��
	Robot_Chassis.Robot_V[w]= Robot_Chassis.World_V[w];
	
}
float KEEP_YAW=0;
void World_Control(void)        //ʼ�������������µ�Y����Ϊ��Y
{
	Robot_Chassis.World_V[x]=-(ROCK_L_X-1500)*0.003f;
	Robot_Chassis.World_V[y]=(ROCK_L_Y-1500)*0.003f;

	if(ROCK_R_X==1500)
	{
		 if(ROCK_L_X==1500 && ROCK_L_Y == 1500){
 			Robot_Chassis.World_V[w]= 0;
		 }
		 else{
			//YawAdjust(KEEP_YAW);
		 }
	}
	else 
	{
		Robot_Chassis.World_V[w]=(ROCK_R_X-1500)*0.01f;
		KEEP_YAW=ACTION_GL_POS_DATA.REAL_YAW;    //��ʱ��������?���д��ݣ�������ȱ��
	}
	
	
}

/*
9.21
�]��
*/

void Robot_Control(void)    //ʼ���Ի�ͷ����Ϊ��Y
{	
	static int flag = 0;//makesure only read angle when the first time entry "KEEP_YAW" 
	
	Robot_Chassis.Robot_V[x]=-(ROCK_L_X-1500)*0.03f;
	Robot_Chassis.Robot_V[y]=(ROCK_L_Y-1500)*0.03f;
	if(ROCK_R_X==1500)
	{	
		if(flag == 0){
			KEEP_YAW=ACTION_GL_POS_DATA.REAL_YAW;
			YawAdjust(KEEP_YAW);
			flag = 1;
		}

	}
	else 
	{
		Robot_Chassis.Robot_V[w]=(ROCK_R_X-1500)*0.01f;
		flag = 0;
	}
	
	
	
}
void AUTO_Control(void)
{
	//�Զ�·���滮�����ϣ������ڴ�
}
void Contor_FSM(void)
{
	if(SWD==2000)   //����D�ǿ���
	{
		switch(SWB)  //����B��ģʽ����
		{
			case 1000:
				Robot_Control();
				break;
			case 1500:
				World_Control();
			break;
			case 2000:
				AUTO_Control();
			break;
				
		}
				
				
	}
}