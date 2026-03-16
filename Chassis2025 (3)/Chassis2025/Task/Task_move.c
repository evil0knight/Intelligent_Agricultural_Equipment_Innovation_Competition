#include "Task_move.h"
#include "pid.h"
#include "encoder.h"
#include "mike.h"
#include "SR04.h"

/*  测试   */
float u_distance=7;
uint16_t cnt;
/*  测试   */



float start_yaw,yaw;			//初始 YAW
float speed_goal=200;

Ultra_distance ultra,ultra_F;			//超声距离*4


uint8_t Move_flag,Move_init_flag;
uint8_t M_speed_state; // 加速- -减速
uint8_t pass_mark=0;

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



uint8_t corridor_cnt=0,corridor_mark=0;

void Start(void)
{
		float out1=0,out2=0,out3=0;

		
		/* 启动   vy加速，vx=0   */
		if(M_speed_state==0)
		{
				
				out1=0;			
				out2=speed_change(20,0,300);
				/* Y 方向运动时 yaw环 */
			  
				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);
			
				TRIG_B
				if(my_abs(Car_s.vy)>100&& ultra.B<50)
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
				if (ultra.B< 15 && ultra.B > 1)
				{
						out1=ultra_pid_control(B_u,5,ultra.B);
				}
				else
				{
						out1=0;
				}
				out2=speed_change(20,300,300);

				out3=yaw_pid_control();

				mike_wheel(out1,out2,out3);

				if(PES_F ==1 && corridor_mark==0)
				{
					corridor_cnt+=1;
					corridor_mark=1;
				}
				if(PES_F ==0 && corridor_mark==1)
				{
					corridor_mark=0;				
				}
				TRIG_L
				if(corridor_cnt>=5 &&ultra.L<40.0f &&ultra.L>3.0f)
				{
					/* 每次改变状态必须 */
					Encoder_Clear();
					Move_pid_clear();
					
					corridor_cnt=0;
					M_speed_state=2;
				}
			//M_speed_state=2;
		}
		/*   vy减速，vx近似0 */
		else if(M_speed_state==2)
		{
				TRIG_B			
				if (ultra.B< 15 && ultra.B >1)
				{
						out1=ultra_pid_control(B_u,5,ultra.B);
				}
				else
				{
						out1=0;
				}
				out2=speed_change(20,300,230);
				out3=yaw_pid_control();
				
				mike_wheel(out1,out2,out3);
			
				TRIG_L
				if(ultra.L<17.0f&&ultra.L> 1.0f) /* 判断 右侧何时为空 */
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(0,0,0);
						M_speed_state=3;
					
				}
				//M_speed_state=3;

		}
		else if(M_speed_state==3)																							 //进入
		{

				out1=speed_change(20,0,300);
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
				if(ultra.B>60 && ultra.B <200)
				{
						Encoder_Clear();//进入垄1,清除累计路程
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
				out1=speed_change(20,300,300);
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

				if(my_abs(Car_s.vx)>200  &&ultra.F<50.0f &&ultra.F>1.0f)
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						M_speed_state=2;
				}
			
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{

		
				out1=speed_change(20,300,160);
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
				if(ultra.F<17.0f&&ultra.F>1.0f) /* 判断 右侧何时为空 */
				{
						/* 每次改变状态必须 */
					
						Encoder_Clear();
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(0,0,0);
						M_speed_state=3;
					
				}
			//M_speed_state=3;
		}
		else if(M_speed_state==3)  																				// 出垄
		{
					TRIG_F
					if (ultra.F<= 17 && ultra.F > 1)
					{
							out1=ultra_pid_control(F_u,7,ultra.F);

					}
					else
					{
							out1=0;
					}
					out2=speed_change(20,0,-300);
					out3=yaw_pid_control();
					mike_wheel(out1,out2,out3);
			
					if(my_abs(Car_s.vy)>120)
					{
							Encoder_Clear();//清除累计路程
							
							M_speed_state=0;
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
						out1=ultra_pid_control(F_u,7,ultra.F);

				}
				else
				{
						out1=0;
				}
				out2=speed_change(20,-300,-200);
				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);
			
			
			
				if(PES_B ==1 && pass_mark==0)
				{
						Encoder_Clear();
						pass_mark=1;
				}
				if(my_abs(Car_s.vy)>12 && pass_mark==1)
				{
					Encoder_Clear();
					pass_mark=0;
					i_flag=0;//速度从0开始，需要标志置0
					Move_pid_clear();
					mike_wheel(0,0,0);
					M_speed_state=1;		
				}		
				//M_speed_state=1;
				
		}
		else if(M_speed_state==1)														// 进入
		{		
				out1=speed_change(20,0,-300);
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

				if(my_abs(Car_s.vx)>40)
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
				out1=speed_change(20,-300,-300);
			
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
				if(my_abs(Car_s.vx)>200  &&ultra.B<50.0f &&ultra.B>1.0f)
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						M_speed_state=2;
				}
			
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{

		
				out1=speed_change(20,-300,-160);//这里应该根据超声波位置环启动时的大小确定减到的速度
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
				if(ultra.B<17.0f&&ultra.B>1.0f) /* 判断 右侧何时为空 */
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(0,0,0);
						M_speed_state=3;
					
				}
				//M_speed_state=3;
		}
		else if(M_speed_state==3)
		{

				TRIG_B
				if (ultra.B<= 17 && ultra.B > 1)
				{
						out1=ultra_pid_control(B_u,7,ultra.B);

				}
				else
				{
						out1=0;
				}	
				out2=speed_change(20,0,-300);
				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);
		
				if(my_abs(Car_s.vy)>120)
				{
						Encoder_Clear();//清除累计路程
						
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
						out1=ultra_pid_control(B_u,7,ultra.B);

				}
				else
				{
						out1=0;
				}	
				out2=speed_change(20,-300,-200);
				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);
				if(PES_F ==1 && pass_mark==0)
				{
						Encoder_Clear();
						pass_mark=1;
				}
				if(my_abs(Car_s.vy)>10 && pass_mark==1)
				{
					Encoder_Clear();
					pass_mark=0;
					i_flag=0;//速度从0开始，需要标志置0
					Move_pid_clear();
					mike_wheel(0,0,0);
					M_speed_state=1;		
				}		
				//M_speed_state=1;
				
		}
		else if(M_speed_state==1)
		{		
				out1=speed_change(20,0,300);
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

				if(my_abs(Car_s.vx)>40)
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
				out1=speed_change(20,300,300);
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

				if(my_abs(Car_s.vx)>200  &&ultra.F<50.0f &&ultra.F>1.0f)
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						M_speed_state=2;
				}
			
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{

		
				out1=speed_change(20,300,160);
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
				if(ultra.F<17.0f&&ultra.F>1.0f) /* 判断 右侧何时为空 */
				{
						/* 每次改变状态必须 */
					
						Encoder_Clear();
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(0,0,0);
						M_speed_state=3;
					
				}
			//M_speed_state=3;
		}
		else if(M_speed_state==3)  																				// 出垄
		{
					TRIG_F
					if (ultra.F<= 17 && ultra.F > 1)
					{
							out1=ultra_pid_control(F_u,7,ultra.F);

					}
					else
					{
							out1=0;
					}
					out2=speed_change(20,0,-300);
					out3=yaw_pid_control();
					mike_wheel(out1,out2,out3);
			
					if(my_abs(Car_s.vy)>120)
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
						out1=ultra_pid_control(F_u,7,ultra.F);

				}
				else
				{
						out1=0;
				}
				out2=speed_change(20,-300,-200);
				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);
			
			
			
				if(PES_B ==1 && pass_mark==0)
				{
						Encoder_Clear();
						pass_mark=1;
				}
				if(my_abs(Car_s.vy)>12 && pass_mark==1)
				{
					Encoder_Clear();
					pass_mark=0;
					i_flag=0;//速度从0开始，需要标志置0
					Move_pid_clear();
					mike_wheel(0,0,0);
					M_speed_state=1;		
				}		
				//M_speed_state=1;
				
		}
		else if(M_speed_state==1)														// 进入
		{		
				out1=speed_change(20,0,-300);
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

				if(my_abs(Car_s.vx)>40)
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
				out1=speed_change(20,-300,-300);
			
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
				if(my_abs(Car_s.vx)>200  &&ultra.B<50.0f &&ultra.B>1.0f)
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						M_speed_state=2;
				}
			
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{

		
				out1=speed_change(20,-300,-160);//这里应该根据超声波位置环启动时的大小确定减到的速度
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
				if(ultra.B<17.0f&&ultra.B>1.0f) /* 判断 右侧何时为空 */
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(0,0,0);
						M_speed_state=3;
					
				}
				//M_speed_state=3;
		}
		else if(M_speed_state==3)
		{

				TRIG_B
				if (ultra.B<= 17 && ultra.B > 1)
				{
						out1=ultra_pid_control(B_u,7,ultra.B);

				}
				else
				{
						out1=0;
				}	
				out2=speed_change(20,0,-300);
				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);
		
				if(my_abs(Car_s.vy)>120)
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
						out1=ultra_pid_control(B_u,7,ultra.B);

				}
				else
				{
						out1=0;
				}	
				out2=speed_change(20,-300,-200);
				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);
				if(PES_F ==1 && pass_mark==0)
				{
						Encoder_Clear();
						pass_mark=1;
				}
				if(my_abs(Car_s.vy)>12 && pass_mark==1)
				{
					Encoder_Clear();
					pass_mark=0;
					i_flag=0;//速度从0开始，需要标志置0
					Move_pid_clear();
					mike_wheel(0,0,0);
					M_speed_state=1;		
				}		
				//M_speed_state=1;
				
		}
		else if(M_speed_state==1)
		{		
				out1=speed_change(20,0,300);
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

				if(my_abs(Car_s.vx)>40)
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
				out1=speed_change(20,300,300);
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

				if(my_abs(Car_s.vx)>200  &&ultra.F<50.0f &&ultra.F>1.0f)
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						M_speed_state=2;
				}
			
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{

		
				out1=speed_change(20,300,160);
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
				if(ultra.F<17.0f&&ultra.F>1.0f) /* 判断 右侧何时为空 */
				{
						/* 每次改变状态必须 */
					
						Encoder_Clear();
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(0,0,0);
						M_speed_state=3;
					
				}
			//M_speed_state=3;
		}
		else if(M_speed_state==3)  																				// 出垄
		{
					TRIG_F
					if (ultra.F<= 17 && ultra.F > 1)
					{
							out1=ultra_pid_control(F_u,7,ultra.F);

					}
					else
					{
							out1=0;
					}
					out2=speed_change(20,0,-300);
					out3=yaw_pid_control();
					mike_wheel(out1,out2,out3);
			
					if(my_abs(Car_s.vy)>120)
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
						out1=ultra_pid_control(F_u,7,ultra.F);

				}
				else
				{
						out1=0;
				}
				out2=speed_change(20,-300,-200);
				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);
			
			
			
				if(PES_B ==1 && pass_mark==0)
				{
						Encoder_Clear();
						pass_mark=1;
				}
				if(my_abs(Car_s.vy)>12 && pass_mark==1)
				{
					Encoder_Clear();
					pass_mark=0;
					i_flag=0;//速度从0开始，需要标志置0
					Move_pid_clear();
					mike_wheel(0,0,0);
					M_speed_state=1;		
				}		
				//M_speed_state=1;
				
		}
		else if(M_speed_state==1)														// 进入
		{		
				out1=speed_change(20,0,-300);
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

				if(my_abs(Car_s.vx)>40)
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
				out1=speed_change(20,-300,-300);
			
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
				if(my_abs(Car_s.vx)>200  &&ultra.B<50.0f &&ultra.B>1.0f)
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						M_speed_state=2;
				}
			
				//M_speed_state=2;
		}
		
		else if(M_speed_state==2)
		{

		
				out1=speed_change(20,-300,-160);//这里应该根据超声波位置环启动时的大小确定减到的速度
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
				if(ultra.B<17.0f&&ultra.B>1.0f) /* 判断 右侧何时为空 */
				{
						/* 每次改变状态必须 */
						Encoder_Clear();
						i_flag=0;//速度从0开始，需要标志置0
						Move_pid_clear();
						mike_wheel(0,0,0);
						M_speed_state=3;
					
				}
				//M_speed_state=3;
		}
		else if(M_speed_state==3)
		{

				TRIG_B
				if (ultra.B<= 17 && ultra.B > 1)
				{
						out1=ultra_pid_control(B_u,7,ultra.B);

				}
				else
				{
						out1=0;
				}	
				out2=speed_change(20,0,-300);
				out3=yaw_pid_control();
				mike_wheel(out1,out2,out3);
		
				if(my_abs(Car_s.vy)>180)
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
		
		Encoder_Clear();
		pass_mark=0;
		i_flag=0;//速度从0开始，需要标志置0
		Move_pid_clear();
		mike_wheel(out1,out2,out3);

}


void Move_init(void)
{
		/*													P		I		D		out	iout			*/
	
		PID_Init(&pid_motor_speed[0]  	,1,36,0.18,0 	, 6000, 3000,3000);
		PID_Init(&pid_motor_speed[1]		,1,32,0.16,0	, 6000, 3000,3000);
		PID_Init(&pid_motor_speed[2]  	,1,40,0.20,0	, 6000, 3000,3000);
		PID_Init(&pid_motor_speed[3]		,1,32,0.16,0	, 6000, 3000,3000);
	

		PID_Init(&pid_yaw					,1,0.45,0.0,0.35, 30, 10,10);

	
		PID_Init(&pid_ultra[F_u]  ,1,-10,0.0,-1, 30, 30,30);

	
		PID_Init(&pid_ultra[B_u]  ,1, 22,0.0,21, 30, 30,30);

	
		PID_Init(&pid_ultra[L_u]  ,1,-27,0.0,-3, 30, 30,30);

	
		PID_Init(&pid_ultra[R_u]  ,1,0.0,0.0,0.0, 30, 30,30);

	
}


void Move_pid_clear(void)
{
	PID_clear(&pid_motor_speed[0]);
	PID_clear(&pid_motor_speed[1]);
	PID_clear(&pid_motor_speed[2]);
	PID_clear(&pid_motor_speed[3]);
}


//void Move_pid_clear(void)
//{
//	
//		switch(Move_init_flag)
//			
//		{
//			//case start: break;
//			
//			
//			/* 左过道结束 */
//			case Fieldridge1: PID_clear(&pid_ultra[L_u]);
//												PID_clear(&pid_u_speed[L_u]);
//			
//												PID_clear(&pid_yaw[Y_y]);
//												PID_clear(&pid_y_speed[Y_y]);
//													
//												Move_flag=Fieldridge1;
//												
//												break;
//			
//			case corridor1:   PID_clear(&pid_ultra[F_u]);
//												PID_clear(&pid_u_speed[F_u]);
//			
//												PID_clear(&pid_yaw[X_y]);
//												PID_clear(&pid_y_speed[X_y]);
//												Move_flag=corridor1;
//												
//												break;
//			/* 右过道结束 */
//			case Fieldridge2: PID_clear(&pid_ultra[R_u]);
//												PID_clear(&pid_u_speed[R_u]);
//			
//												PID_clear(&pid_yaw[Y_y]);
//												PID_clear(&pid_y_speed[Y_y]);
//												Move_flag=Fieldridge2;
//												
//												break;
//												
//			case corridor2:		PID_clear(&pid_ultra[F_u]);
//												PID_clear(&pid_u_speed[F_u]);
//			
//												PID_clear(&pid_yaw[X_y]);
//												PID_clear(&pid_y_speed[X_y]);
//												Move_flag=corridor2;
//												
//												break;
//			/* 左过道结束 */
//			case Fieldridge3: PID_clear(&pid_ultra[R_u]);
//												PID_clear(&pid_u_speed[R_u]);
//			
//												PID_clear(&pid_yaw[Y_y]);
//												PID_clear(&pid_y_speed[Y_y]);
//												Move_flag=Fieldridge3;
//												
//												break;
//			
//			case corridor3: 	PID_clear(&pid_ultra[F_u]);
//												PID_clear(&pid_u_speed[F_u]);
//			
//												PID_clear(&pid_yaw[X_y]);
//												PID_clear(&pid_y_speed[X_y]);
//												Move_flag=corridor3;
//												
//												break;
//			/* 右过道结束 */
//			case Fieldridge4: PID_clear(&pid_ultra[R_u]);
//												PID_clear(&pid_u_speed[R_u]);
//			
//												PID_clear(&pid_yaw[Y_y]);
//												PID_clear(&pid_y_speed[Y_y]);
//												Move_flag=Fieldridge4;
//												
//												break;
//			
//			case corridor4: 	PID_clear(&pid_ultra[F_u]);
//												PID_clear(&pid_u_speed[F_u]);
//			
//												PID_clear(&pid_yaw[X_y]);
//												PID_clear(&pid_y_speed[X_y]);
//												Move_flag=corridor4;
//												
//												break;
//			/* 左过道结束 */
//			case Fieldridge5: PID_clear(&pid_ultra[R_u]);
//												PID_clear(&pid_u_speed[R_u]);
//			
//												PID_clear(&pid_yaw[Y_y]);
//												PID_clear(&pid_y_speed[Y_y]);
//												Move_flag=Fieldridge5;
//												
//												break;
//			
//			case corridor5:   PID_clear(&pid_ultra[F_u]);
//												PID_clear(&pid_u_speed[F_u]);
//			
//												PID_clear(&pid_yaw[X_y]);
//												PID_clear(&pid_y_speed[X_y]);
//												Move_flag=corridor5;
//												
//												break;
//			
//			default: break;
//			
//		}
//	
//	
//}










float my_abs(float x)
{
	if (x>=0) return  x;
	else			return -x;
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





