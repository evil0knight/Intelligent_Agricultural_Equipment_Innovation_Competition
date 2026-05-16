#include "Task_move.h"
#include "pid.h"
#include "encoder.h"
#include "mike.h"
#include "SR04.h"

/*  测试   */
float u_distance=6;
uint16_t cnt;
/*  测试   */
extern uint8_t work_off_mark;
uint8_t tx_data[2];

float start_yaw,yaw;			//初始 YAW
float speed_goal=200;

Ultra_distance ultra,ultra_F;			//超声距离*4


uint8_t Move_flag,Move_init_flag;
uint8_t M_speed_state; // 加速- -减速
/* 光电管相关参数标志 */
uint8_t pass_mark=0;
uint8_t corridor_cnt=0,corridor_mark=0;
uint8_t pes_count[4];//用于计数，三次均为0，则有效
uint8_t pes_mark[4];
/* 加/减速 函数 */
int i_speed=0;
uint8_t i_flag,end_flag;

extern void Start(void);
extern void edge_1(void);
extern void corridor_1(void);
extern void edge_2(void);
extern void corridor_2(void);
extern void edge_3(void);
extern void corridor_3(void);
extern void edge_4(void);
extern void corridor_4(void);
extern void edge_5(void);
extern void corridor_5(void);
extern void edge_6(void);
extern void corridor_6(void);

extern void Move_pid_clear(void);
extern void working_on(void);
extern void working_off(void);
extern void pes_scan(uint8_t loc);

void Move(void)
{
		switch(Move_flag)
		{
			case start: Start();break;
			
			case Fieldridge1: edge_1();break;
			case corridor1: corridor_1();break;
			
			case Fieldridge2: edge_2();break;
			case corridor2: corridor_2();break;
			
			case Fieldridge3: edge_3();break;
			case corridor3: corridor_3();break;
			
			case Fieldridge4: edge_4();break;
			case corridor4: corridor_4();break;
			
			case Fieldridge5: edge_5();break;
			case corridor5: corridor_5();break;
			
			case Fieldridge6: edge_6();break;		
			case corridor6: corridor_6();break;
			
			default: break;

			
		}
	
}


/*======================================================*/
/*====================测试函数===========================*/

void canshu(void)
{
//		float out1=0,out2=0,out3=0;
//		if(M_speed_state==0)
//		{
//				TRIG_B
//				out1=0;

//				out2=loc_pid_control(u_distance,g_odom.y);

//				out3=yaw_pid_control();
//				
//				mike_wheel(out1+0.08f*out2,out2,out3);
				
//				if(my_abs(g_odom.y)>60)/////////////////////////////////////////////////////////////////
//				{

//					Encoder_Clear();
//					pass_mark=0;
//					i_flag=0;//速度从0开始，需要标志置0

//					M_speed_state=1;
//				}	
				 

				//

//		}
		
//		else if(M_speed_state==1)
//		{
//				TRIG_B
//				out1=0;

//				out2=speed_change(5,180,180);


//				out3=yaw_pid_control();
//				
//				mike_wheel(out1+0.08f*out2,out2,out3);
//				
//				if(my_abs(g_odom.y)>5)/////////////////////////////////////////////////////////////////
//				{

//					Encoder_Clear();
//					pass_mark=0;
//					i_flag=0;//速度从0开始，需要标志置0
//					

//					Move_pid_clear();
////					Motor_current_set(0,0,0,0);
//					mike_wheel(0,0,out3);
//					M_speed_state=2;
//				}	
//		}
//		else if(M_speed_state==2)
//		{

//				out1=0;
//			
//				out3=yaw_pid_control();
//				mike_wheel(0,0,out3);
//		}
/*=================== 横移 ====================*/
//		if(M_speed_state==0)
//		{
//				TRIG_B
//				if (ultra.B<= 20 && ultra.B > 1)
//				{
//						out1=ultra_pid_control_BF(B_u,7,ultra.B);

//				}
//				else
//				{
//						out1=100;
//				}
//				if (ultra.B<= 12 && ultra.B > 1)
//				{
//						out2=speed_change(5,0,-280);

//				}
//				else
//				{
//						out2=0;
//				}
//				out3=yaw_pid_control();
//				
//				mike_wheel(out1+0.08f*out2,out2,out3);
//				
//				if(my_abs(Car_s.vy)>100)/////////////////////////////////////////////////////////////////
//				{

//					Encoder_Clear();
//					pass_mark=0;
//					i_flag=0;//速度从0开始，需要标志置0

//					M_speed_state=1;
//				}	
//				

//				//

//		}
//		
//		else if(M_speed_state==1)
//		{
//				TRIG_B
//				if (ultra.B<= 20 && ultra.B > 1)
//				{
//						out1=ultra_pid_control_BF(B_u,7,ultra.B);

//				}
//				else
//				{
//						out1=0;
//				}
//				out2=speed_change(5,-280,-180);


//				out3=yaw_pid_control();
//				
//				mike_wheel(out1+0.08f*out2,out2,out3);
//				
//				if(my_abs(Car_s.vy)>100)/////////////////////////////////////////////////////////////////
//				{

//					Encoder_Clear();
//					pass_mark=0;
//					i_flag=0;//速度从0开始，需要标志置0
//					

//					Move_pid_clear();
//					Motor_current_set(0,0,0,0);
//					M_speed_state=2;
//				}	
//		}
//		else if(M_speed_state==2)
//		{
//				//TRIG_B
//				if (ultra.B<= 20 && ultra.B > 1)
//				{
//							out1=ultra_pid_control_BF(B_u,7,ultra.B);

//				}
//				else
//				{
//							out1=0;
//				}

//				out3=yaw_pid_control();
//				mike_wheel(0,0,out3);
//		}
	

/*=================== 向前stop ====================*/

//		if(M_speed_state==0)
//		{

//			out1=speed_change(10,0,300);

//			out3=yaw_pid_control();

//			mike_wheel(out1,0,out3);
//			TRIG_F
//			if(my_abs(Car_s.vx)>50  &&ultra.F<40.0f &&ultra.F>1.0f)  
//			{
//					/* 每次改变状态必须 */
//					Encoder_Clear();
//					Move_pid_clear();				
//					M_speed_state=1;
//			}
//		}
//		else if(M_speed_state==1)
//		{
//			out1=speed_change(10,300,160);
//		
//			out3=yaw_pid_control();
//			
//			mike_wheel(out1,0,out3);
//			TRIG_F
//			if(ultra.F<25.0f&&ultra.F>1.0f) 
//			{
//					/* 作业指令 */
//					working_off();
//					/* 作业指令 */
//			}
//			if(ultra.F<16.0f&&ultra.F>1.0f)
//			{
//					Encoder_Clear();
//				
//				
//					i_flag=0;//速度从0开始，需要标志置0
//					Move_pid_clear();			
//					Motor_current_set(0,0,0,0);
//					M_speed_state=2;
//				
//			}

//		}
//		
//		else if(M_speed_state==2)
//		{
//					TRIG_F
//					if (ultra.F<= 20 && ultra.F > 1)
//					{
//							out1=ultra_pid_control_BF(F_u,6,ultra.F);

//					}
//					else
//					{
//							out1=100;
//					}
//					if (ultra.F<= 12 && ultra.F > 1)
//					{
//							out2=speed_change(5,0,-280);

//					}
//					else
//					{
//							out2=0;
//					}
//				out3=yaw_pid_control();
//				
//				mike_wheel(out1+0.08f*out2,out2,out3);
//				
//				if(my_abs(Car_s.vy)>200)/////////////////////////////////////////////////////////////////
//				{

//					Encoder_Clear();
//					pass_mark=0;
//					i_flag=0;//速度从0开始，需要标志置0
//					

//					Move_pid_clear();
//					Motor_current_set(0,0,0,0);
//					M_speed_state=3;
//				}	
//				

//				//

//		}
//		else if(M_speed_state==3)
//		{
//				//TRIG_F
//				if (ultra.F<= 20 && ultra.F > 1)
//				{
//							out1=ultra_pid_control_BF(F_u,6,ultra.F);

//				}
//				else
//				{
//							out1=0;
//				}

//				out3=yaw_pid_control();
//				mike_wheel(0,0,out3);
//		}







/*=================== 向后stop ====================*/
//		if(M_speed_state==0)
//		{

//			out1=speed_change(10,0,-300);

//			out3=yaw_pid_control();

//			mike_wheel(out1,0,out3);
//			TRIG_B
//			if(my_abs(Car_s.vx)>100  &&ultra.B<40.0f &&ultra.B>1.0f)  
//			{
//					/* 每次改变状态必须 */
//					Encoder_Clear();
//					Move_pid_clear();				
//					M_speed_state=1;
//			}
//		}
//		else if(M_speed_state==1)
//		{
//			out1=speed_change(10,-300,-200);
//		
//			out3=yaw_pid_control();
//			
//			mike_wheel(out1,0,out3);
//			TRIG_B
//			if(ultra.B<25.0f&&ultra.B>1.0f) 
//			{
//					/* 作业指令 */
//					working_off();
//					/* 作业指令 */
//			}
//			if(ultra.B<16.0f&&ultra.B>1.0f)
//			{
//					Encoder_Clear();
//				
//				
//					i_flag=0;//速度从0开始，需要标志置0
//					Move_pid_clear();			
//					Motor_current_set(0,0,0,0);
//					M_speed_state=2;
//				
//			}

//		}
//		
//		else if(M_speed_state==2)
//		{
//					TRIG_B
//					if (ultra.B<= 20 && ultra.B > 1)
//					{
//							out1=ultra_pid_control_BF(B_u,7,ultra.B);

//					}
//					else
//					{
//							out1=100;
//					}
//					if (ultra.B<= 12 && ultra.B > 1)
//					{
//							out2=speed_change(5,0,280);

//					}
//					else
//					{
//							out2=0;
//					}
//				out3=yaw_pid_control();
//				
//				mike_wheel(out1+0.08f*out2,out2,out3);
//				
//				if(my_abs(Car_s.vy)>150)/////////////////////////////////////////////////////////////////
//				{

//					Encoder_Clear();
//					pass_mark=0;
//					i_flag=0;//速度从0开始，需要标志置0
//					

//					Move_pid_clear();
//					Motor_current_set(0,0,0,0);
//					M_speed_state=3;
//				}	
//				

//				//

//		}
//		else if(M_speed_state==3)
//		{
//				TRIG_B
//				if (ultra.B<= 20 && ultra.B > 1)
//				{
//							out1=ultra_pid_control_BF(B_u,7,ultra.B);

//				}
//				else
//				{
//							out1=100;
//				}

//				out3=yaw_pid_control();
//				mike_wheel(out1,0,out3);
//		}
		
}


/*======================================================*/



void Start(void)
{
//		canshu();
	
		float out1=0,out2=0,out3=0;

		
		/* 启动 */
		if(M_speed_state==0)
		{
				
				out1=0;			
				out2=speed_change(5,0,300);			  
				out3=yaw_pid_control();
				mike_wheel(out1+0.09f*out2,out2,out3);
			
				TRIG_B
			
				if(my_abs(g_odom.y)>20&& ultra.B<50)
				{
					/* 每次改变状态必须 */
					Encoder_Clear();					
					M_speed_state=1;					
				}	
			//M_speed_state=1;

		}
		/* 进入赛道  vy加速，vx近似0 */
		else if(M_speed_state==1 )
		{
	
				TRIG_B
				if (ultra.B< 17 && ultra.B > 1)
				{
						out1=ultra_pid_control_BF(B_u,7.5,ultra.B);
				}
				else
				{
						out1=0;
				}
				out2=speed_change(10,300,300);

				out3=yaw_pid_control();

				mike_wheel(out1+0.09f*out2,out2,out3);
				
				TRIG_F
				pes_scan(L_u);
				pes_scan(F_u);
				if(pes_mark[L_u]==1 &&pes_mark[F_u]==1&& corridor_mark==0)
				{
					corridor_cnt+=1;
					corridor_mark=1;
				}
				else if(pes_mark[L_u]==0 &&pes_mark[F_u]==0&&ultra.F>30.0f&& corridor_mark==1)
				{
					corridor_mark=0;				
				}

				if(corridor_cnt==4)
				{
					/* 作业指令 */
					working_on();
					/* 作业指令 */
										
					/* 每次改变状态必须 */
					Encoder_Clear();
					Move_pid_clear();	/* 积分清零，方便减速 */
					corridor_cnt=0;
					corridor_mark=0;	
					
					M_speed_state=2;
				}
			//M_speed_state=2;
		}

		else if(M_speed_state==2)
		{
				TRIG_B			
				if (ultra.B< 17 && ultra.B >1)
				{
						out1=ultra_pid_control_BF(B_u,7.5,ultra.B);
				}
				else
				{
						out1=0;
				}
				out2=speed_change(5,300,180);
				out3=yaw_pid_control();
				
				mike_wheel(out1+0.09f*out2,out2,out3);
				pes_scan(F_u);
				pes_scan(L_u);
				if(pes_mark[F_u]==0 &&pes_mark[L_u]==0 && pass_mark==0)
				{
						Encoder_Clear();
						pass_mark=1;
				}
				if(pass_mark==1)
				{
						if(my_abs(g_odom.y)>5)/////////////////////////////////////////////////////////////////
						{

							//Encoder_Clear();
							pass_mark=0;
							i_flag=0;//速度从0开始，需要标志置0
							
							Move_pid_clear();
							mike_wheel(0,-10,out3);

							M_speed_state=3;		
						}	
				}

				//M_speed_state=3;

		}
		else if(M_speed_state==3)																							 //进入
		{

				out1=speed_change(5,0,320);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
						//out2=-10;
					out2=loc_pid_control(5,g_odom.y);
				}
				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);

				if(my_abs(g_odom.x)>40)
				{
						Encoder_Clear();
						M_speed_state=0;
	
						Move_flag=Fieldridge1;
				}

		
//				M_speed_state=0;
//				Move_flag=Fieldridge1;
		}
	
}


void edge_1(void)
{
		float out1=0,out2=0,out3=0;
		

		if(M_speed_state==0)
		{		
				out1=speed_change(10,320,320);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
						out2=0;
				}			
				out3=yaw_pid_control();
			
				mike_wheel(out1,out2,out3);
			
				TRIG_F

				if(my_abs(g_odom.x)>180  &&ultra.F<40.0f &&ultra.F>1.0f)  
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						Move_pid_clear();	/* 积分清零，方便减速 */
						M_speed_state=2;
				}
		
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{
		
				out1=speed_change(10,320,160);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
						out2=0;
				}			
				out3=yaw_pid_control();
				
				mike_wheel(out1,out2,out3);
			
				TRIG_F
				if(ultra.F<25.0f&&ultra.F>1.0f) 
				{
						/* 作业指令 */
						working_off();
						/* 作业指令 */
				}
				if(ultra.F<16.0f&&ultra.F>1.0f)
				{
						Encoder_Clear();
					
					
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();			
						mike_wheel(-10,0,out3);
						M_speed_state=3;
					
				}
			//M_speed_state=3;
		}
		else if(M_speed_state==3)  																				// 出垄
		{
					TRIG_F
					if (ultra.F<= 15 && ultra.F > 1)
					{
							out1=ultra_pid_control_BF(F_u,5,ultra.F);

					}
					else
					{
							out1=100;
					}
					if (ultra.F<= 12 && ultra.F > 1)
					{
							out2=speed_change(5,0,260);

					}
					else
					{
							out2=0;
					}


					out3=yaw_pid_control();
					mike_wheel(out1+0.09f*out2,out2,out3);
					TRIG_L
					if(my_abs(g_odom.y)>60&&ultra.L<40.0f &&ultra.L>1.0f)
					{
							Encoder_Clear();//清除累计路程
							
							M_speed_state=0;
;
							Move_flag=corridor1;
					}

//					M_speed_state=0;
//					Move_flag=corridor1;
		}
}


void corridor_1(void)
{
		float out1=0,out2=0,out3=0;
		


		if(M_speed_state==0)
		{			
				TRIG_F
				if (ultra.F<= 17 && ultra.F > 1)
				{
						out1=ultra_pid_control_BF(F_u,5,ultra.F);

				}
				else
				{
						out1=0;
				}
				out2=speed_change(5,260,180);

				out3=yaw_pid_control();
				mike_wheel(out1+0.09f*out2,out2,out3);


				TRIG_L
				if(ultra.L<17.0f&&ultra.L> 1.0f) 
				{
						/* 每次改变状态必须 */

						Encoder_Clear();
					
						i_flag=0;//速度从0开始，需要标志置0

						Move_pid_clear();
						mike_wheel(0,-10,out3);
						M_speed_state=1;
						
				}

					

				//M_speed_state=1;
				
		}
		else if(M_speed_state==1)														// 进入
		{		
				
				TRIG_L
				TRIG_F
				if(ultra.L<= 12.0f && ultra.L > 1)
				{
						out1=speed_change(5,0,-320);
				}
				else
				{
						out1=ultra_pid_control_BF(F_u,5,ultra.F);
				}
				if (ultra.L<= 13 && ultra.L > 1)
				{
							out2=ultra_pid_control(L_u,7.5,ultra.L);
				}
				else
				{
							out2=180;
				}

				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);

				if(my_abs(g_odom.x)>40)
				{
						Encoder_Clear();
						M_speed_state=0;
						Move_flag=Fieldridge2;
				}
					
//				M_speed_state=0;
//				Move_flag=Fieldridge2;
		}

}

void edge_2(void)
{
		float out1=0,out2=0,out3=0;
		

		if(M_speed_state==0)
		{		
				out1=speed_change(10,-320,-320);
			
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7.5,ultra.L);
				}
				else
				{
						out2=0;
				}
				
				out3=yaw_pid_control();
			
				mike_wheel(out1,out2,out3);
			
				TRIG_B
				if(my_abs(g_odom.x)>180  &&ultra.B<40.0f &&ultra.B>1.0f)
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						Move_pid_clear();	/* 积分清零，方便减速 */
						M_speed_state=2;
				}
			
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{

				out1=speed_change(10,-320,-160);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7.5,ultra.L);
				}
				else
				{
						out2=0;
				}	
				out3=yaw_pid_control();
				
				mike_wheel(out1,out2,out3);
			
				TRIG_B
				if(ultra.B<16.0f&&ultra.B>1.0f) 
				{
				
						/* 每次改变状态必须 */

						Encoder_Clear();
					
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(15,0,out3);
						M_speed_state=3;
					
				}
				//M_speed_state=3;
		}
		else if(M_speed_state==3)
		{

				TRIG_B
				if (ultra.B<= 20 && ultra.B > 1)
				{
						out1=ultra_pid_control_BF(B_u,7,ultra.B);

				}
				else
				{
						out1=-100;
				}
				if (ultra.B<= 12 && ultra.B > 1)
				{
						out2=speed_change(5,0,-260);

				}
				else
				{
						out2=0;
				}
				out3=yaw_pid_control();
				mike_wheel(out1+0.08f*out2,out2,out3);

				if(my_abs(g_odom.y)>15)
				{
						Encoder_Clear();//清除累计路程
						corridor_cnt=0;
						M_speed_state=0;

						Move_flag=corridor2;
				}
			
//					M_speed_state=0;
//					Move_flag=corridor2;
		}
}

void corridor_2(void)
{
		float out1=0,out2=0,out3=0;
		


		if(M_speed_state==0)
		{			
				TRIG_B
				if (ultra.B<= 17 && ultra.B > 1)
				{
						out1=ultra_pid_control_BF(B_u,7,ultra.B);

				}
				else
				{
						out1=0;
				}	
				out2=speed_change(5,-260,-260);  
				out3=yaw_pid_control();
				mike_wheel(out1+0.08f*out2,out2,out3);
				TRIG_F
				pes_scan(L_u);
				pes_scan(F_u);
				if(pes_mark[F_u]==1  &&pes_mark[L_u]==1 && corridor_mark==0)
				{
					corridor_cnt+=1;
					corridor_mark=1;
				}
				else if(pes_mark[L_u]==0 &&pes_mark[F_u]==0&&ultra.F>30.0f&& corridor_mark==1)
				{
					corridor_mark=0;				
				}
				if(corridor_cnt==1)
				{
						/* 作业指令 */
						working_on();
						
						/* 作业指令 */
				}
				if(corridor_cnt==2)
				{

					
					/* 每次改变状态必须 */
					Encoder_Clear();

					corridor_cnt=0;
					corridor_mark=0;	
					
					M_speed_state=1;
				}
						
				//M_speed_state=1;
				
		}
		else if(M_speed_state==1)
		{
				TRIG_B			
				if (ultra.B< 15 && ultra.B >1)
				{
						out1=ultra_pid_control_BF(B_u,7,ultra.B);
				}
				else
				{
						out1=0;
				}

				out2=speed_change(5,-260,-180); //////////////////////
				
				out3=yaw_pid_control();
				
				mike_wheel(out1+0.08f*out2,out2,out3);
				pes_scan(F_u);
				pes_scan(L_u);
				if(pes_mark[F_u]==0 &&pes_mark[L_u]==0 && pass_mark==0)
				{
						Encoder_Clear();
						pass_mark=1;
				}
				if(pass_mark==1)
				{
						
						if(my_abs(g_odom.y)>5.0f) /////////////////////////////////////////////////////////////////
						{

//							Encoder_Clear();
							
							pass_mark=0;
							i_flag=0;//速度从0开始，需要标志置0
							Move_pid_clear();
							mike_wheel(0,15,out3);
							M_speed_state=2;		
						}	
				}
		}
		else if(M_speed_state==2)
		{		
				out1=speed_change(5,0,320);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
							out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
							//out2=10;
					out2=loc_pid_control(-5,g_odom.y);
				}

				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);

				if(my_abs(g_odom.x)>40)
				{
						Encoder_Clear();
						M_speed_state=0;
	
						Move_flag=Fieldridge3;
				}
					
//				M_speed_state=0;
//				Move_flag=Fieldridge3;
		}

}



void edge_3(void)
{
		float out1=0,out2=0,out3=0;
		
		if(M_speed_state==0)
		{		
				out1=speed_change(10,320,320);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
						out2=0;
				}			
				out3=yaw_pid_control();
			
				mike_wheel(out1,out2,out3);
			
				TRIG_F

				if(my_abs(g_odom.x)>180  &&ultra.F<40.0f &&ultra.F>1.0f)
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						Move_pid_clear();	/* 积分清零，方便减速 */
						M_speed_state=2;
				}
			
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{

		
				out1=speed_change(10,320,160);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
						out2=0;
				}			
				out3=yaw_pid_control();
				
				mike_wheel(out1,out2,out3);
			
				TRIG_F
				if(ultra.F<25.0f&&ultra.F>1.0f) /* 判断 右侧何时为空 */
				{
						/* 作业指令 */
						working_off();
						/* 作业指令 */
				}
				if(ultra.F<16.0f&&ultra.F>1.0f) /* 判断 右侧何时为空 */
				{

						/* 每次改变状态必须 */
						Encoder_Clear();
					
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(-10,0,out3);
						M_speed_state=3;
					
				}
			//M_speed_state=3;
		}
		else if(M_speed_state==3)  																				// 出垄
		{
					TRIG_F
					if (ultra.F<= 20 && ultra.F > 1)
					{
							out1=ultra_pid_control_BF(F_u,6,ultra.F);

					}
					else
					{
							out1=100;
					}
					if (ultra.F<= 12 && ultra.F > 1)
					{
							out2=speed_change(5,0,-260);

					}
					else
					{
							out2=0;
					}

				
					out3=yaw_pid_control();
					mike_wheel(out1+0.08f*out2,out2,out3);
			
					if(my_abs(g_odom.y)>45)////////////////
					{
							Encoder_Clear();//清除累计路程
							M_speed_state=0;

							Move_flag=corridor3;
					}		

//					M_speed_state=0;
//					Move_flag=corridor3;
		}
}

void corridor_3(void)
{
		float out1=0,out2=0,out3=0;
		
		if(M_speed_state==0)
		{			
				TRIG_F
				if (ultra.F<= 17 && ultra.F > 1)
				{
						out1=ultra_pid_control_BF(F_u,6,ultra.F);

				}
				else
				{
						out1=0;
				}
				out2=speed_change(5,-260,-180);
				out3=yaw_pid_control();
				mike_wheel(out1+0.08f*out2,out2,out3);
			
			
				TRIG_B
				pes_scan(B_u);
				if( pes_mark[B_u]==0  &&ultra.B>20.0f && pass_mark==0)
				{
						Encoder_Clear();
						pass_mark=1;
				}
				if(pass_mark==1)
				{
					if(my_abs(g_odom.y)>5.0f) /////////////////////////////////////////////////////////////////
					{

//						Encoder_Clear();
						pass_mark=0;
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(0,15,out3);
						M_speed_state=1;		
					}
				}		
				//M_speed_state=1;
				
		}
		else if(M_speed_state==1)														// 进入
		{		
				out1=speed_change(5,0,-320);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
							out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
							out2=loc_pid_control(-5,g_odom.y);
							//out2=15;
							//out2=0;
				}

				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);

				if(my_abs(g_odom.x)>40)
				{
						Encoder_Clear();
						M_speed_state=0;
						Move_flag=Fieldridge4;
				}

				
//				M_speed_state=0;
//				Move_flag=Fieldridge4;
		}

}
void edge_4(void)
{
		float out1=0,out2=0,out3=0;
		

		if(M_speed_state==0)
		{		
				out1=speed_change(10,-320,-320);
			
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
						out2=0;
				}
				
				out3=yaw_pid_control();
			
				mike_wheel(out1,out2,out3);
			
				TRIG_B
				if(my_abs(g_odom.x)>180  &&ultra.B<40.0f &&ultra.B>1.0f)
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						Move_pid_clear();	/* 积分清零，方便减速 */
						M_speed_state=2;
				}
			
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{

		
				out1=speed_change(10,-320,-160);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
						out2=0;
				}	
				out3=yaw_pid_control();
				
				mike_wheel(out1,out2,out3);
			
				TRIG_B
				if(ultra.B<16.0f&&ultra.B>1.0f) 
				{
					
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(15,0,out3);
					/* 每次改变状态必须 */
						Encoder_Clear();
						M_speed_state=3;
					
				}
				//M_speed_state=3;
		}
		else if(M_speed_state==3)
		{

				TRIG_B
				if (ultra.B<= 20 && ultra.B > 1)
				{
						out1=ultra_pid_control_BF(B_u,6,ultra.B);

				}
				else
				{
						out1=-150;
				}
				if (ultra.B<= 12 && ultra.B > 1)
				{
						out2=speed_change(5,0,-260);

				}
				else
				{
						out2=0;
				}

				
				out3=yaw_pid_control();
				mike_wheel(out1+0.08f*out2,out2,out3);
				if(my_abs(g_odom.y)>10)
				{
						/* 作业指令 */
						working_on();
						
						/* 作业指令 */
					
				}
				pes_scan(L_u);
				pes_scan(F_u);
				if(pes_mark[L_u]==1 && pes_mark[F_u]==1)
				{

						Encoder_Clear();//清除累计路程

						M_speed_state=0;

						Move_flag=corridor4;
				}
//					M_speed_state=0;
//					Move_flag=corridor4;
		}
}

void corridor_4(void)
{
		float out1=0,out2=0,out3=0;
		
		if(M_speed_state==0)
		{			
				TRIG_B
				if (ultra.B<= 17 && ultra.B > 1)
				{
						out1=ultra_pid_control_BF(B_u,6,ultra.B);

				}
				else
				{
						out1=0;
				}	
				out2=speed_change(5,-260,-180);
				out3=yaw_pid_control();
				mike_wheel(out1+0.08f*out2,out2,out3);
				
				pes_scan(F_u);
				pes_scan(L_u);
				if(pes_mark[F_u]==0 &&pes_mark[L_u]==0 && pass_mark==0)
				{
						Encoder_Clear();
						pass_mark=1;
				}
				if(pass_mark==1)
				{
					if(my_abs(g_odom.y)>3.0f)/////////////////////////////////////////////////////////////////
					{
//						Encoder_Clear();
						pass_mark=0;
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(0,15,out3);
						M_speed_state=1;		
					}
				}
				//M_speed_state=1;
				
		}
		else if(M_speed_state==1)
		{		
				out1=speed_change(5,0,320);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
						out2=loc_pid_control(-5,g_odom.y);
						//out2=10;
				}

				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);

				if(my_abs(g_odom.x)>40)
				{
						Encoder_Clear();
						M_speed_state=0;
	
						Move_flag=Fieldridge5;
				}
					
//				M_speed_state=0;
//				Move_flag=Fieldridge5;
		}

}

void edge_5(void)
{
		float out1=0,out2=0,out3=0;
		
		if(M_speed_state==0)
		{		
				out1=speed_change(10,320,320);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
						out2=0;
				}			
				out3=yaw_pid_control();
			
				mike_wheel(out1,out2,out3);
			
				TRIG_F
				
				if(my_abs(g_odom.x)>180  &&ultra.F<40.0f &&ultra.F>1.0f)
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						Move_pid_clear();	/* 积分清零，方便减速 */
						M_speed_state=2;
				}
			
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{

		
				out1=speed_change(10,320,160);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,7,ultra.L);
				}
				else
				{
						out2=0;
				}			
				out3=yaw_pid_control();
				
				mike_wheel(out1,out2,out3);
			
				TRIG_F
				if(ultra.F<25.0f&&ultra.F>1.0f) 
				{
						/* 作业指令 */
						working_off();
						/* 作业指令 */
				}
				if(ultra.F<16.0f&&ultra.F>1.0f)
				{

						/* 每次改变状态必须 */
						Encoder_Clear();
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(-10,0,out3);
						M_speed_state=3;
					
				}
			//M_speed_state=3;
		}
		else if(M_speed_state==3)  																				// 出垄
		{
					TRIG_F
					if (ultra.F<= 20 && ultra.F > 1)
					{
							out1=ultra_pid_control_BF(F_u,6,ultra.F);

					}
					else
					{
							out1=100;
					}
					if (ultra.F<= 12 && ultra.F > 1)
					{
							out2=speed_change(5,0,-260);

					}
					else
					{
							out2=0;
					}


					out3=yaw_pid_control();
					mike_wheel(out1+0.08f*out2,out2,out3);
			
					if(my_abs(g_odom.y)>45)
					{
							Encoder_Clear();//清除累计路程
							
							M_speed_state=0;
					
							Move_flag=corridor5;
					}		
//					M_speed_state=0;
//					Move_flag=corridor5;
		}
}


void corridor_5(void)
{
		float out1=0,out2=0,out3=0;
		
		if(M_speed_state==0)
		{			
				TRIG_F
				if (ultra.F<= 17 && ultra.F > 1)
				{
						out1=ultra_pid_control_BF(F_u,6,ultra.F);

				}
				else
				{
						out1=0;
				}
				out2=speed_change(10,-260,-150);
				out3=yaw_pid_control();
				mike_wheel(out1+0.08f*out2,out2,out3);
			
			
				TRIG_B
				pes_scan(B_u);
				if(pes_mark[B_u]==0 && pass_mark==0&& ultra.B>20.0f)
				{
						Encoder_Clear();
						pass_mark=1;
				}
				if(pass_mark==1)
				{
					if(my_abs(g_odom.y)>5.0f)/////////////////////////////////////////////////////////////////
					{
//						Encoder_Clear();
					
						pass_mark=0;
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(0,15,out3);
						M_speed_state=1;		
					}
				}		
				//M_speed_state=1;
				
		}
		else if(M_speed_state==1)														
		{		
				out1=speed_change(5,0,-320);
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,5.5,ultra.L);
				}
				else
				{
						//out2=0;
						//out2=10;
						out2=loc_pid_control(-5,g_odom.y);
				}

				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);

				if(my_abs(g_odom.x)>40)
				{
						Encoder_Clear();
						M_speed_state=0;
						Move_flag=Fieldridge6;
				}
					
//				M_speed_state=0;
//				Move_flag=Fieldridge6;
		}

}


void edge_6(void)
{
		float out1=0,out2=0,out3=0;
		
		if(M_speed_state==0)
		{		
				out1=speed_change(10,-320,-320);
			
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,5.5,ultra.L);
				}
				else
				{
						out2=0;
				}
				
				out3=yaw_pid_control();
			
				mike_wheel(out1,out2,out3);
			
				TRIG_B
				if(my_abs(g_odom.x)>180  &&ultra.B<40.0f &&ultra.B>1.0f)
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						Move_pid_clear();	/* 积分清零，方便减速 */
						M_speed_state=2;
				}
			
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{

		
				out1=speed_change(10,-320,-160);//这里应该根据超声波位置环启动时的大小确定减到的速度
				TRIG_L
				if (ultra.L<= 17 && ultra.L > 1)
				{
						out2=ultra_pid_control(L_u,5.5,ultra.L);
				}
				else
				{
						out2=0;
				}	
				out3=yaw_pid_control();
				
				mike_wheel(out1,out2,out3);
			
				TRIG_B
				if(ultra.B<16.0f&&ultra.B>1.0f) /* 判断 右侧何时为空 */
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
					
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(15,0,out3);
						M_speed_state=3;
					
				}
				//M_speed_state=3;
		}
		else if(M_speed_state==3)
		{

				TRIG_B
				if (ultra.B<= 20 && ultra.B > 1)
				{
						out1=ultra_pid_control_BF(B_u,6,ultra.B);

				}
				else
				{
						out1=-150;
				}
				if (ultra.B<= 12 && ultra.B > 1)
				{
						out2=speed_change(10,0,-260);

				}
				else
				{
						out2=0;
				}

				out3=yaw_pid_control();
				mike_wheel(out1+0.08f*out2,out2,out3);
		
				if(my_abs(g_odom.y)>20)
				{
						Encoder_Clear();//清除累计路程
						
						M_speed_state=0;
	
						Move_flag=corridor6;
				}	
		}
}

void corridor_6(void)
{
		float out1=0,out2=0,out3=0;
		if(M_speed_state==0)
		{			
				TRIG_B
				if (ultra.B<= 17 && ultra.B > 1)
				{
						out1=ultra_pid_control_BF(B_u,6,ultra.B);

				}
				else
				{
						out1=0;
				}	
				out2=speed_change(5,-260,-260);
				out3=yaw_pid_control();
				mike_wheel(out1+0.08f*out2 ,out2,out3);
				TRIG_B
				pes_scan(B_u);
				if(pes_mark[B_u]==0 && pass_mark==0&& ultra.B>20.0f)
				{
						Encoder_Clear();
						pass_mark=1;
				}
				if(pass_mark==1)
				{
					if(my_abs(g_odom.y)>6.0f)/////////////////////////////////////////////////////////////////
					{
						Encoder_Clear();
						pass_mark=0;
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(0,0,out3);
						M_speed_state=1;		
					}
				}
				//M_speed_state=1;
				
		}
		else if(M_speed_state==1)
		{		
				mike_wheel(0,0,out3);

		}


}


void Move_init(void)
{
		/*				pid结构体				mode	   P		I	  D		 out   iout	积分抗饱和		*/
	
		PID_Init(&pid_motor_speed[0]  	,1,		19.5*1.6  ,0.52 ,0.00  , 6000, 5000,200);
		PID_Init(&pid_motor_speed[1]	,1,		19.5*1.6  ,0.53 ,0.00  , 6000, 5000,200);
		PID_Init(&pid_motor_speed[2]  	,1,		19.5*1.7  ,0.53 ,0.00  , 6000, 5000,200);
		PID_Init(&pid_motor_speed[3]	,1,		19.5*1.65 ,0.52 ,0.00  , 6000, 5000,200);


		PID_Init(&pid_yaw				,1,		0.65      ,0.0,  0.001 , 100,   100,100);

	
		PID_Init(&pid_ultra[F_u]  ,1,-21.5,0.0,-0, 120, 120,120);

	
		PID_Init(&pid_ultra[B_u]  ,1, 16  ,0.0, 0, 120, 120,120);

	
		PID_Init(&pid_ultra[L_u]  ,1,-23,0.0,0, 100, 100,100);

	
		PID_Init(&pid_ultra[R_u]  ,1,0.0,0.0,0.0, 60, 60,20);
		
		PID_Init(&pid_loc ,1,	25.0,0.0,0.0, 100, 100,100);

	
}


void Move_pid_clear(void)
{
	PID_clear(&pid_motor_speed[0]);
	PID_clear(&pid_motor_speed[1]);
	PID_clear(&pid_motor_speed[2]);
	PID_clear(&pid_motor_speed[3]);
}





void working_on(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);//uart6_tx
}

void working_off(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);//uart6_tx
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
}





float my_abs(float x)
{
	if (x>=0) return  x;
	else			return -x;
}


void pes_scan(uint8_t loc)
{
	switch(loc)
	{
		case 0: 
		{
			if(PES_F==0)
			{
				if(pes_count[F_u]<3)
				{
					pes_count[F_u]++;
				}
				else if(pes_count[F_u]==3)
				{
					pes_mark[F_u]=1;
				}
			}
			else
			{
				pes_mark[F_u]=0;
				pes_count[F_u]=0;
			}
			break;
		}
		case 1: 
		{
			if(PES_B==0)
			{
				if(pes_count[B_u]<3)
				{
					pes_count[B_u]++;
				}
				else if(pes_count[B_u]==3)
				{
					pes_mark[B_u]=1;
				}
			}
			else
			{
				pes_mark[B_u]=0;
				pes_count[B_u]=0;
			}
			break;
		}
		case 2: 
		{

			if(PES_L==0)
			{

				if(pes_count[L_u]<3)
				{
					pes_count[L_u]++;
				}
				else if(pes_count[L_u]==3)
				{
					pes_mark[L_u]=1;
				}
			}
			else
			{
				pes_mark[L_u]=0;
				pes_count[L_u]=0;
			}
			break;
		}
		default:break;
	}

}

/* s_speed 以 K 为步长 至 M_speed */
int speed_change(int k, int s_speed, int M_speed)
{
    // 初始化静态变量
    if(i_flag == 0)
    {
        i_speed = s_speed;
        i_flag = 1;
    }
    
    // 如果已经达到目标速度，直接返回
    if(i_speed == M_speed)
    {
        return i_speed;
    }
    
    // 比较当前速度与目标速度，而不是起始速度
    if(my_abs(i_speed) < my_abs(M_speed))
    {
        // 加速过程
        if(my_abs(i_speed) < my_abs(M_speed))
        {
            if(M_speed < 0)
            {
                i_speed = i_speed - k;
                // 防止超调
                if(i_speed < M_speed) i_speed = M_speed;
            }
            else 
            {
                i_speed = i_speed + k;
                // 防止超调
                if(i_speed > M_speed) i_speed = M_speed;
            }
        }
        else
        {
            i_speed = M_speed;
        }
    }
    else
    {
        // 减速过程
        if(my_abs(i_speed) > my_abs(M_speed))
        {
            if(M_speed < 0)
            {
                i_speed = i_speed + k;
                // 防止超调
                if(i_speed > M_speed) i_speed = M_speed;
            }
            else 
            {
                i_speed = i_speed - k;
                // 防止超调
                if(i_speed < M_speed) i_speed = M_speed;
            }
        }
        else
        {
            i_speed = M_speed;
        }
    }
    return i_speed;
}





float yaw_pid_control(void)
{
		pid_yaw.set=0;

		if( (start_yaw-yaw)>180 )
		{
				pid_yaw.fdb=-(start_yaw-yaw-360);
		}
		else if( (start_yaw-yaw)<-180 )
		{
				pid_yaw.fdb=-(start_yaw-yaw+360);
		}
		else
		{
				pid_yaw.fdb=-(start_yaw-yaw);
		}
		PID_Calc(&pid_yaw);
		return pid_yaw.out;
	
}



float ultra_pid_control(uint8_t i,float goal,float fd)
{
		pid_ultra[i].set=goal;
		pid_ultra[i].fdb=fd;
		PID_Calc(&pid_ultra[i]);

		return pid_ultra[i].out;
}

float ultra_pid_control_BF(uint8_t i,float goal,float fd)
{
		pid_ultra[i].set=goal;
		pid_ultra[i].fdb=fd;
		PID_Calc(&pid_ultra[i]);

		return pid_ultra[i].out;
}

float loc_pid_control(float goal,float fd)
{
		pid_loc.set=goal;
		pid_loc.fdb=fd;
		PID_Calc(&pid_loc);

		return pid_loc.out;
}

