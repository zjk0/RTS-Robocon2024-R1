/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "RobotControl.h"

/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/
const double robot_length=0.613f;
const double robot_width=0.613f;
const double L=0.273f;
const double  theta1=PI/3;
const double  theta2=PI/6;
float Vx,Vy,Vw;

/*
*********************************************************************************************************
*                                            	变量定义
*********************************************************************************************************
*/

Robot_INFO Robot;

Nav_Point Bezier_Nav[404]={0};  //本次导航点

int Nav_i=0;                    //0~101,标记走到了那个点
int Point_i=0;                  //0~101,用于数据导入
int data_flag=0;                //0-进行数据导入，1-数据导入完成

float Now_projection;           //当前点在路径上的投影
float Dis_erro;                 //当前点在速度方向上的位移差
float Comp_v;                   //PID计算后补偿的速度

//                       P   I   D
PIDType Car_Poin_PID =  {5, 0.1, 0,0,0,0,0};//位置->速度PID参数
  
/*
*********************************************************************************************************
*                                               函数实现
*********************************************************************************************************
*/

/**
 * @brief 机器人速度转轮速
 * @param none 
 * @return None
 * @note 机器人在自身坐标系下速度转到轮速
 */
void Robot_Velocity_To_Wheel_Speed()
{
    //float Vx,Vy,Vw;
    Vx=Robot.Robot_in_self.velocity_exp.Vx;
    Vy=Robot.Robot_in_self.velocity_exp.Vy;
    Vw=Robot.Robot_in_self.velocity_exp.Vw;
    //Robot.Wheel_Speed[1]=sqrt(2)*Vx+sqrt(2)*Vy+Vw*(robot_length+robot_width)/2;
    //Robot.Wheel_Speed[2]=sqrt(2)*Vx-sqrt(2)*Vy+Vw*(robot_length+robot_width)/2;
    //Robot.Wheel_Speed[3]=-sqrt(2)*Vx-sqrt(2)*Vy+Vw*(robot_length+robot_width)/2;
   


    Robot.Wheel_Speed[1] = Vx+L*Vw;
    Robot.Wheel_Speed[2] = (-cos(theta1)*Vx-sin(theta1)*Vy+L*Vw);
    Robot.Wheel_Speed[3] = -sin(theta2)*Vx+cos(theta2)*Vy+L*Vw;
    
}
/**
 * @brief 轮速转电机
 * @param none
 * @return None
 * 
 */
void Wheel_Speed_To_Motor()
{
    M3508[4].ExpSpeed=Robot.Wheel_Speed[1]*60*19.5/(2*PI*WHEEL_R);
    M3508[5].ExpSpeed=Robot.Wheel_Speed[2]*60*19.5/(2*PI*WHEEL_R);
    M3508[6].ExpSpeed=Robot.Wheel_Speed[3]*60*19.5/(2*PI*WHEEL_R);
   // M3508[4].ExpSpeed=Robot.Wheel_Speed[4]*60*19.5/(2*PI*WHEEL_R);
}
/**
 * @brief 机器人世界坐标系转自身坐标系速度
 * @param none
 * @return None
 */
void Robot_Velocity_World_To_Self()
{
    float Vx,Vy,Vw;
    Vx=Robot.Robot_in_world.velocity_exp.Vx;
    Vy=Robot.Robot_in_world.velocity_exp.Vy;
    Vw=Robot.Robot_in_world.velocity_exp.Vw;
    Robot.Robot_in_self.velocity_exp.Vx=Vx*cos(Robot.Robot_in_self.position_now.yaw)+Vy*sin(Robot.Robot_in_self.position_now.yaw);
    Robot.Robot_in_self.velocity_exp.Vy=-Vx*sin(Robot.Robot_in_self.position_now.yaw)+Vy*cos(Robot.Robot_in_self.position_now.yaw);
    Robot.Robot_in_self.velocity_exp.Vw=Vw;
}
/**
 * @brief 导航函数
 *
 * @param[in] step 第几个过程
 *  
 * @return Projection 向量<x1,y1>在向量<x2,y2>上的投影
 *
 * @note 用于计算目标点点的投影。注意！！！不适用于车体旋转！！！
 *
 */
void Navigation (int step)                           //????PID????
{
  if(data_flag == 0)
  {
    for(Point_i = 0; Point_i<=100; Point_i++)
    {
      Bezier_Nav[Point_i].Pos_x   = datas[step][Point_i];
      Bezier_Nav[Point_i].Pos_y   = datas[step][101+Point_i];
      Bezier_Nav[Point_i].v       = datas[step][202+Point_i];
      Bezier_Nav[Point_i].tan_v   = datas[step][303+Point_i];
      Bezier_Nav[Point_i].Projection = Get_Projection(datas[step][Point_i]-datas[step][0], datas[step][100+Point_i]-datas[step][101], datas[step][100]-datas[step][0], datas[step][201]-datas[step][101]);
    }
    data_flag++;
  }
  if(Nav_i<100)
  {
    Now_projection = Get_Projection(Robot.Robot_in_self.position_now.x-datas[step][0], Robot.Robot_in_self.position_now.y-datas[step][101], datas[step][100]-datas[step][0], datas[step][201]-datas[step][101]);
    if(Now_projection < Bezier_Nav[Nav_i].Projection)
    {
      //当前点到 过目标点 且 斜率为目标速度方向 的直线的距离
      Dis_erro = Get_Distance(Robot.Robot_in_self.position_now.x,Robot.Robot_in_self.position_now.y, Bezier_Nav[Nav_i].Pos_x, Bezier_Nav[Nav_i].Pos_y, Bezier_Nav[Nav_i].tan_v);
      Comp_v = PIDCal(&Car_Poin_PID, Dis_erro);
      Robot.Robot_in_world.velocity_exp.Vx = (Comp_v*(Bezier_Nav[Nav_i].tan_v/sqrt(Bezier_Nav[Nav_i].tan_v*Bezier_Nav[Nav_i].tan_v+1)) + Bezier_Nav[Nav_i].v*(1/sqrt(Bezier_Nav[Nav_i].tan_v*Bezier_Nav[Nav_i].tan_v+1)))/2;
      Robot.Robot_in_world.velocity_exp.Vy = (Comp_v*(1/sqrt(Bezier_Nav[Nav_i].tan_v*Bezier_Nav[Nav_i].tan_v+1)) + Bezier_Nav[Nav_i].v*(Bezier_Nav[Nav_i].tan_v/sqrt(Bezier_Nav[Nav_i].tan_v*Bezier_Nav[Nav_i].tan_v+1)))/2;
      Robot.Robot_in_world.velocity_exp.Vw = 0;
    }
    else// if(Now_projection > Bezier_Nav[Nav_i].Projection)
    {
      Nav_i++;
    }
  }
  else
  {
    Robot.Robot_in_world.velocity_exp.Vx = 0;
    Robot.Robot_in_world.velocity_exp.Vy = 0;
    Robot.Robot_in_world.velocity_exp.Vw =  0;
    data_flag = 0;
    Point_i = 0;
  }
}
/**
 * @brief 向量投影函数
 *
 * @param[in] x1 待投影向量x坐标
 * @param[in] y1 待投影向量y坐标
 * @param[in] x2 方向向量x坐标
 * @param[in] y2 方向向量y坐标
 *            
 * @return Projection 向量<x1,y1>在向量<x2,y2>上的投影
 *
 * @note 用于计算目标点点的投影。
 *
 */
float Get_Projection(float x1, float y1, float x2, float y2)
{
  float Projection = (x1*x2 + y1*y2)/sqrt(x2*x2 + y2*y2);
  return Projection;
}
/**
 * @brief 点到直线距离函数
 *
 * @param[in] x1 目标点x坐标
 * @param[in] y1 目标点y坐标
 * @param[in] x2 直线所过点x坐标
 * @param[in] y2 直线所过点y坐标
 * @param[in] k  直线斜率
 *            
 * @return Projection 点(x1,y1)到过(x2,y2)斜率为k的直线的距离
 *
 * @note 用于计算点到直线的距离。
 *
 */
float Get_Distance(float x1, float y1, float x2, float y2, float k)
{
  float Distance = abs((k*x2 - y2 - k*x1 + y1)/sqrt(k*k + 1));
  return Distance;
}
