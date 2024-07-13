/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "RobotControl.h"
// #define PI 3.1415926
/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/
const double robot_length = 0.613f;
const double robot_width = 0.613f;
const double L = 0.273f;
const double theta1 = PI / 3;
const double theta2 = PI / 6;
// const double wheel_diameter = 101.34f;  // mm

float Vx = 0, Vy = 0, Vw = 0;
float Vx_pre = 0, Vy_pre = 0, Vw_pre = 0;

// ********************* the parameters to note the previous value *********************
float x_pre = 0;   // previous x
float y_pre = 0;   // previous y
float yaw_pre = 0; // previous yaw
// ********************* the parameters to note the previous value *********************

// ********************* the correct parameters of straight line motion *********************
float x_correct = 2;    // the correct parameter in x axis, when the straight line motion in y axis is running
float y_correct = 2;    // the correct parameter in y axis, when the straight line motion in x axis is running
float yaw_correct = 15; // the correct parameter to correct the direction of yaw
// ********************* the correct parameters of straight line motion *********************

// ********************* the parameters in the motion of rotating an angle *********************
float yaw_start = 0;        // the value of yaw when it starts to rotate
float yaw_now = 0;          // the real-time value of yaw
float delta_yaw = 0;        // the change value of yaw based on yaw_start
uint8_t yaw_start_flag = 0; // analyse whether yaw_start is given value: 0->no, 1->yes

float real;

float Speed_Limit = 100;
float Speed_Limit_w = 100;

#define Speed_Limit_x_Far 3 
#define Speed_Limit_x_Close 2 
#define Speed_Limit_y_Far 2.8 
#define Speed_Limit_y_Close 2 

//#define Speed_Limit_x_Far 1.6
//#define Speed_Limit_x_Close 1.6
//#define Speed_Limit_y_Far 2
//#define Speed_Limit_y_Close 1.6

//#define KP_x_Far 2.8
//#define KP_x_Close 1.7 + 0.05 + 0.3
//#define KP_y_Far 2.8
//#define KP_y_Close 1.6 + 0.05 + 0.3

#define KP_x_Far 3
#define KP_x_Close 4//1.7 
#define KP_y_Far 3
#define KP_y_Close 4//1.7

#define MAX_SPEED_Area2_x_Far 1.5
#define MAX_SPEED_Area2_x_Close 1.5
#define MAX_SPEED_Area2_y_Far 1.8
#define MAX_SPEED_Area2_y_Close 1.6


//#define KP_x_Far 2.5
//#define KP_x_Close 1.7 + 0.05
//#define KP_y_Far 2.5
//#define KP_y_Close 1.6 + 0.05

//#define KP_x_Far 2.1
//#define KP_x_Close 1.7 + 0.05
//#define KP_y_Far 1.9
//#define KP_y_Close 1.6 + 0.05
#define x_close 0.6
#define y_close 0.6

//#define Vx_delta 0.01
//#define Vy_delta 0.01
//#define Vw_delta 1

#define Vx_delta 100
#define Vy_delta 100
#define Vw_delta 100

// the PID struct of this method to control the paramters of PID
// PIDType PID_Vx_move_distance = {0.7, 1.2, 1.3, 0, 0, 0, 0, 0,0.1};
// PIDType PID_Vy_move_distance = {0.7, 1.2, 1.3, 0, 0, 0, 0, 0,0.1};
PIDType PID_Vx_move_distance = {2.5, 0, 2.3, 0, 0, 0, 0, 0, 1.3};
PIDType PID_Vy_move_distance = {2.5, 0, 10, 0, 0, 0, 0, 0, 1.3}; // {2.5, 0.00025, 0.2, 0, 0, 0, 0, 0, 1.3}
PIDType PID_w_move_distance = {1.6, 0.0004, 3, 0, 0, 0, 0, 0, 1.5};    // 最后一个是w的限幅

PIDType PID_Vx_move_distance_2 = {2.5, 0.00025, 0.22, 0, 0, 0, 0, 0, 1.3};
PIDType PID_Vy_move_distance_2 = {2.5, 0.00001, 0.22, 0, 0, 0, 0, 0, 1.3};

// PIDType PID_Vx_move_distance_little = {0.7, 1.5, 1.7, 0, 0, 0, 0, 0,0};
// PIDType PID_Vy_move_distance_little = {0.7, 1.5, 1.7, 0, 0, 0, 0, 0,0};
// PIDType PID_w_move_distance_little = {1.7, 2.3, 1.1, 0, 0, 0, 0, 0,0};
//  ********************* the parameters in the motion of rotating an angle *********************

uint8_t status = 0; // the status of process, which can help robot understand different status
uint32_t jishu_0_x = 0;
uint32_t jishu_0_y = 0;
uint32_t jishu_0_w = 0;

float Laser_Data_1 = 0;
float Laser_Data_2 = 0;
float Laser_Data_3 = 0;
float Laser_Data_4 = 0;
float Laser_Data_1_pre = 0;
float Laser_Data_2_pre = 0;

float IMU_yaw = 0;
float IMU_Vw = 0;

float Action_x = 0;
float Action_y = 0;

uint8_t laser_status = 0;

float yaw_pre_laser = 0;
uint8_t Release_Seed_Flag = 0;
uint8_t Enable_flag = 0;
uint8_t upper_command = 0;

// double Exparg_90 = 211866.614861;

/*
*********************************************************************************************************
*                                            	变量定义
*********************************************************************************************************
*/

Robot_INFO Robot;

Nav_Point Bezier_Nav[404] = {0}; // 本次导航点

int Nav_i = 0;     // 0~101,标记走到了那个点
int Point_i = 0;   // 0~101,用于数据导入
int data_flag = 0; // 0-进行数据导入，1-数据导入完成

float Now_projection; // 当前点在路径上的投影
float Dis_erro;       // 当前点在速度方向上的位移差
float Comp_v;         // PID计算后补偿的速度

//                       P   I   D
PIDType Car_Poin_PID = {5, 0.1, 0, 0, 0, 0, 0, 0}; // 位置->速度PID参数

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
    Vx = Robot.Robot_in_self.velocity_exp.Vx;
    Vy = -Robot.Robot_in_self.velocity_exp.Vy;
    Vw = Robot.Robot_in_self.velocity_exp.Vw;
    
//    if(Vx - Vx_pre > Vx_delta){
//       Vx = Vx_pre + Vx_delta;
//    }
//    else if(Vx - Vx_pre < -Vx_delta){
//       Vx = Vx_pre - Vx_delta;
//    }
//    
//    if(Vy - Vy_pre > Vy_delta){
//        Vy = Vy_pre + Vy_delta;
//    }
//    else if(Vy - Vy_pre < -Vy_delta){
//        Vy = Vy_pre - Vy_delta;
//    }
//    
//    if(Vw - Vw_pre > Vw_delta){
//        Vw = Vw_pre + Vw_delta;
//    }
//    else if(Vw - Vw_pre < -Vw_delta){
//        Vw = Vw_pre - Vw_delta;
//    }
    
    if (fabs(Vx) > Speed_Limit)
    {
        Vx = Speed_Limit * Vx / fabs(Vx);
    }
    if (fabs(Vy) > Speed_Limit)
    {
        Vy = Speed_Limit * Vy / fabs(Vy);
    }
    if (fabs(Vw) > Speed_Limit_w)
    {
        Vw = Speed_Limit_w * Vw / fabs(Vw);
    }
    Robot.Wheel_Speed[1] = Vx + L * Vw;
    Robot.Wheel_Speed[2] = (-cos(theta1) * Vx - sin(theta1) * Vy + L * Vw);
    Robot.Wheel_Speed[3] = -sin(theta2) * Vx + cos(theta2) * Vy + L * Vw;
    Vx_pre = Vx;
    Vy_pre = Vy;
    Vw_pre = Vw;

}
/**
 * @brief 轮速转电机
 * @param none
 * @return None
 *
 */
void Wheel_Speed_To_Motor()
{
    M3508[5].ExpSpeed = (int16_t)(Robot.Wheel_Speed[1] * 60 * 19.5 / (2 * PI * WHEEL_R)); // 车头
    M3508[4].ExpSpeed = (int16_t)(Robot.Wheel_Speed[2] * 60 * 19.5 / (2 * PI * WHEEL_R));
    M3508[6].ExpSpeed = (int16_t)(Robot.Wheel_Speed[3] * 60 * 19.5 / (2 * PI * WHEEL_R));
    //    M3508[6].ExpSpeed=Robot.Wheel_Speed[1]*60*19.5/(2*PI*WHEEL_R);//车头
    //    M3508[4].ExpSpeed=Robot.Wheel_Speed[2]*60*19.5/(2*PI*WHEEL_R);
    //    M3508[5].ExpSpeed=Robot.Wheel_Speed[3]*60*19.5/(2*PI*WHEEL_R);
    // M3508[4].ExpSpeed=Robot.Wheel_Speed[4]*60*19.5/(2*PI*WHEEL_R);
}
/**
 * @brief 机器人世界坐标系转自身坐标系速度
 * @param none
 * @return None
 */
void Robot_Velocity_World_To_Self()
{
    float Vx, Vy, Vw;
    Vx = Robot.Robot_in_world.velocity_exp.Vx;
    Vy = Robot.Robot_in_world.velocity_exp.Vy;
    Vw = Robot.Robot_in_world.velocity_exp.Vw;
    Robot.Robot_in_self.velocity_exp.Vx = Vx * cos(Robot.Robot_in_self.position_now.yaw) + Vy * sin(Robot.Robot_in_self.position_now.yaw);
    Robot.Robot_in_self.velocity_exp.Vy = -Vx * sin(Robot.Robot_in_self.position_now.yaw) + Vy * cos(Robot.Robot_in_self.position_now.yaw);
    Robot.Robot_in_self.velocity_exp.Vw = Vw;
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
/*void Navigation (int step)                           //????PID????
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
}*/
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
    float Projection = (x1 * x2 + y1 * y2) / sqrt(x2 * x2 + y2 * y2);
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
    float Distance = abs((int16_t)((k * x2 - y2 - k * x1 + y1) / sqrt(k * k + 1))) * 1.0;
    // float Distance = abs((k*x2 - y2 - k*x1 + y1)/sqrt(k*k + 1));
    return Distance;
}






void Move_Distance_Laser(float distance_x, float distance_y, float distance_w)
{                                 
    PID_w_move_distance.KP = 5; // 可以给大点
   // PID_w_move_distance.KD = 0;
    // x-aixs
    if (laser_status == 7)
    {
        if (distance_x - Laser_Data_2 >= -0.02 && distance_x - Laser_Data_2 <= 0.02)
        {
            PID_Vx_move_distance.SumErr = 0;
            PID_Vx_move_distance.Sumlimit = 0.3;
            Robot.Robot_in_world.velocity_exp.Vx = 0;
            jishu_0_x++;
        }
        else
        {
            jishu_0_x = 0;
            if (fabs(distance_x - Laser_Data_2) > x_close)
            {
                PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Far;
                PID_Vx_move_distance.KP = KP_x_Far;
                //PID_Vx_move_distance.KI = 0;
                Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, distance_x - Laser_Data_2);
            }
            else
            {
                //PID_w_move_distance.KD = 0;
                PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Close;
                PID_Vx_move_distance.KP = KP_x_Close;
                //PID_Vx_move_distance.KI = /*0.00012*/ 0.00014;
                ////PID_Vx_move_distance.KD = 0.23;
                Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, distance_x - Laser_Data_2);
            }
        }

        // y-axis
        if (distance_y - Laser_Data_1 >= -0.02 && distance_y - Laser_Data_1 <= 0.02)
        {
            PID_Vy_move_distance.SumErr = 0;
            PID_Vy_move_distance.Sumlimit = 0.3;
            Robot.Robot_in_world.velocity_exp.Vy = 0;
            jishu_0_y++;
        }
        else
        {
            jishu_0_y = 0;
            if (fabs(distance_y - Laser_Data_1) > y_close)
            {
                // //PID_Vx_move_distance.KI = 0;
                PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Far;
                PID_Vy_move_distance.KP = KP_y_Far;
                //PID_Vy_move_distance.KI = 0.00011;

                // //PID_Vy_move_distance.KI = 0;
                Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, distance_y - Laser_Data_1);
            }
            else
            {
                PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Close;
                PID_Vy_move_distance.KP = KP_y_Close;
                //PID_Vy_move_distance.KI = /*0.00012*/ 0.0001;
                //PID_Vy_move_distance.KD = 0.23;
                Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, distance_y - Laser_Data_1);
            }
        }
    }
    else if (laser_status == 12)
    {
        if (distance_x - Laser_Data_2 >= -0.02 && distance_x - Laser_Data_2 <= 0.02)
        {
            PID_Vx_move_distance.SumErr = 0;
            PID_Vx_move_distance.Sumlimit = 0.3;
            Robot.Robot_in_world.velocity_exp.Vx = 0;
            jishu_0_x++;
        }
        else
        {
            jishu_0_x = 0;
            if (fabs(distance_x - Laser_Data_2) > x_close)
            {
                PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Far;
                PID_Vx_move_distance.KP = KP_x_Far;
                //PID_Vx_move_distance.KI = 0;
                Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, distance_x - Laser_Data_2);
            }
            else
            {
                //PID_w_move_distance.KD = 0;
                PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Close;
                PID_Vx_move_distance.KP = KP_x_Close;
                //PID_Vx_move_distance.KI = /*0.00012*/ 0.000142;
                //PID_Vx_move_distance.KD = 0.23;
                Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, distance_x - Laser_Data_2);
            }
        }

        // y-axis
        if (distance_y - Laser_Data_1 >= -0.02 && distance_y - Laser_Data_1 <= 0.02)
        {
            PID_Vy_move_distance.SumErr = 0;
            PID_Vy_move_distance.Sumlimit = 0.3;
            Robot.Robot_in_world.velocity_exp.Vy = 0;
            jishu_0_y++;
        }
        else
        {
            jishu_0_y = 0;
            if (fabs(distance_y - Laser_Data_1) > y_close)
            {
                // //PID_Vx_move_distance.KI = 0;
                PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Far;
                PID_Vy_move_distance.KP = KP_y_Far;
                //PID_Vy_move_distance.KI = 0.00011;

                // //PID_Vy_move_distance.KI = 0;
                Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, distance_y - Laser_Data_1);
            }
            else
            {
                PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Close;
                PID_Vy_move_distance.KP = KP_y_Close;
                //PID_Vy_move_distance.KI = /*0.00012*/ 0.000142;
                //PID_Vy_move_distance.KD = 0.23;
                Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, distance_y - Laser_Data_1);
            }
        }
    }
    else
    {
        if (distance_x - Laser_Data_2 >= -0.02 && distance_x - Laser_Data_2 <= 0.02)
        {
            PID_Vx_move_distance.SumErr = 0;
            PID_Vx_move_distance.Sumlimit = 0.3;
            Robot.Robot_in_world.velocity_exp.Vx = 0;
            jishu_0_x++;
        }
        else
        {
            jishu_0_x = 0;
            if (fabs(distance_x - Laser_Data_2) > x_close)
            {
                PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Far;
                PID_Vx_move_distance.KP = KP_x_Far;
                //PID_Vx_move_distance.KI = 0;
                Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, distance_x - Laser_Data_2);
            }
            else
            {
                //PID_w_move_distance.KD = 0;
                PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Close;
                PID_Vx_move_distance.KP = KP_x_Close;
                //PID_Vx_move_distance.KI = /*0.00012*/ 0.000075;
                //PID_Vx_move_distance.KD = 0.3298;
                Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, distance_x - Laser_Data_2);
            }
        }

        // y-axis
        if (distance_y - Laser_Data_1 >= -0.02 && distance_y - Laser_Data_1 <= 0.02)
        {
            PID_Vy_move_distance.SumErr = 0;
            PID_Vy_move_distance.Sumlimit = 0.3;
            Robot.Robot_in_world.velocity_exp.Vy = 0;
            jishu_0_y++;
        }
        else
        {
            jishu_0_y = 0;
            if (fabs(distance_y - Laser_Data_1) > y_close)
            {
                // //PID_Vx_move_distance.KI = 0;
                PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Far;
                PID_Vy_move_distance.KP = KP_y_Far;
                //PID_Vy_move_distance.KI = 0.00011;

                // //PID_Vy_move_distance.KI = 0;
                Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, distance_y - Laser_Data_1);
            }
            else
            {
                PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Close;
                PID_Vy_move_distance.KP = KP_y_Close;
                //PID_Vy_move_distance.KI = /*0.00012*/ 0.000145;
                //PID_Vy_move_distance.KD = 0.2298;
                Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, distance_y - Laser_Data_1);
            }
        }
    }

    if (distance_w - Robot.Robot_in_self.position_now.yaw >= -0.02 && distance_w - Robot.Robot_in_self.position_now.yaw <= 0.02)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
    }
    else if (distance_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        distance_w = -distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - distance_w > PI)
    {
        distance_w = 2 * PI + distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    Robot_Velocity_World_To_Self();
}






void Move_Certain_W_PID(float distance_w)
{
    // x-aixs
    PID_w_move_distance.KP = 2.5;
    PID_w_move_distance.KD = 0.3;
    if (distance_w - Robot.Robot_in_self.position_now.yaw >= -0.02 && distance_w - Robot.Robot_in_self.position_now.yaw <= 0.02)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
        jishu_0_x++;
        jishu_0_y++;
    }
    else if (distance_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        distance_w = -distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
        jishu_0_x = 0;
        jishu_0_y = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - distance_w > PI)
    {
        distance_w = 2 * PI + distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
        jishu_0_x = 0;
        jishu_0_y = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
        jishu_0_x = 0;
        jishu_0_y = 0;
    }
    Robot.Robot_in_world.velocity_exp.Vy = 0;
    Robot.Robot_in_world.velocity_exp.Vx = 0;
    Robot_Velocity_World_To_Self();
}






void Move_Distance_x_Laser_y_action(float distance_x, float distance_y, float distance_w, float error_act)
{                                 // ����һ�����������ξ���ʽpid��������ȥpre���ڦ�����ô��εĽ������pre�Ӧ�������ֱ�ӵ���pidcla����¼pre
    PID_w_move_distance.KP = 2.3; // 可以给大点
    PID_w_move_distance.KD = 0;
    // x-aixs

    if (distance_x - Laser_Data_3 >= -0.01 && distance_x - Laser_Data_3 <= 0.01)
    {
		PID_Vx_move_distance.SumErr = 0;
        PID_Vx_move_distance.Sumlimit = 0.3;
        Robot.Robot_in_world.velocity_exp.Vx = 0;
        jishu_0_x++;
    }
    else
    {
        jishu_0_x = 0;
        if (fabs(distance_x - Laser_Data_3) > x_close)
        {
            // PID_Vx_move_distance.KP = 1.2;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance_2, distance_x - Laser_Data_3);
        }
        else
        {
            PID_w_move_distance.KD = 0;
            // //PID_Vx_move_distance.KI = 0.0001;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance_2, distance_x - Laser_Data_3);
        }
    }

    // y-axis
    if (distance_y - (Robot.Robot_in_self.position_now.y + error_act) >= -0.01 && distance_y - (Robot.Robot_in_self.position_now.y + error_act) <= 0.01)
    {
        Robot.Robot_in_world.velocity_exp.Vy = 0;
        jishu_0_y++;
    }
    else
    {
        jishu_0_y = 0;
        if (fabs(distance_y - (Robot.Robot_in_self.position_now.y + error_act)) > y_close)
        {
            // //PID_Vy_move_distance.KI = 1.2;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance_2, distance_y - (Robot.Robot_in_self.position_now.y + error_act));
        }
        else
        {
            // PID_w_move_distance.KD = 0;
            // PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Close;
            /// //PID_Vy_move_distance.KI = 0.0001;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance_2, distance_y - (Robot.Robot_in_self.position_now.y + error_act));
        }
    }

    if (distance_w - Robot.Robot_in_self.position_now.yaw >= -0.01 && distance_w - Robot.Robot_in_self.position_now.yaw <= 0.01)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
    }
    else if (distance_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        distance_w = -distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - distance_w > PI)
    {
        distance_w = 2 * PI + distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    Robot_Velocity_World_To_Self();
}






void Move_Distance_y_Laser_x_action(float distance_x, float distance_y, float distance_w, float error_act)
{                                 // ����һ�����������ξ���ʽpid��������ȥpre���ڦ�����ô��εĽ������pre�Ӧ�������ֱ�ӵ���pidcla����¼pre
    PID_w_move_distance.KP = 2.3; // 可以给大点
    PID_w_move_distance.KD = 0;
    // x-aixs

    if (distance_x - (Robot.Robot_in_self.position_now.x + error_act) >= -0.01 && distance_x - (Robot.Robot_in_self.position_now.x + error_act) <= 0.01)
    {
        Robot.Robot_in_world.velocity_exp.Vx = 0;
        jishu_0_x++;
    }
    else
    {
        jishu_0_x = 0;
        if (fabs(distance_x - (Robot.Robot_in_self.position_now.x + error_act)) > x_close)
        {
            // PID_Vx_move_distance.KP = 1.2;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance_2, distance_x - (Robot.Robot_in_self.position_now.x + error_act));
        }
        else
        {
            PID_w_move_distance.KD = 0;
            // //PID_Vx_move_distance.KI = 0.0001;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance_2, distance_x - (Robot.Robot_in_self.position_now.x + error_act));
        }
    }

    // y-axis
    if (distance_y - Laser_Data_4 >= -0.01 && distance_y - Laser_Data_4 <= 0.01)
    {
        Robot.Robot_in_world.velocity_exp.Vy = 0;
        jishu_0_y++;
    }
    else
    {
        jishu_0_y = 0;
        if (fabs(distance_y - Laser_Data_4) > y_close)
        {
            // //PID_Vy_move_distance.KI = 1.2;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance_2, distance_y - Laser_Data_4);
        }
        else
        {
            // PID_w_move_distance.KD = 0;
            // PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Close;
            /// //PID_Vy_move_distance.KI = 0.0001;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance_2, distance_y - Laser_Data_4);
        }
    }

    if (distance_w - Robot.Robot_in_self.position_now.yaw >= -0.01 && distance_w - Robot.Robot_in_self.position_now.yaw <= 0.01)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
    }
    else if (distance_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        distance_w = -distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - distance_w > PI)
    {
        distance_w = 2 * PI + distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    Robot_Velocity_World_To_Self();
}






void Move_Distance_Laser_2(float distance_x, float distance_y, float distance_w){                                 
    PID_w_move_distance.KP = 2.3; // 可以给大点
    PID_w_move_distance.KD = 0;
    // x-aixs

    if (distance_x - Laser_Data_3 >= -0.01 && distance_x - Laser_Data_3 <= 0.01)
    {
		PID_Vx_move_distance.SumErr = 0;
		PID_Vx_move_distance.Sumlimit = 0.3;
		Robot.Robot_in_world.velocity_exp.Vx = 0;
        jishu_0_x++;
    }
    else
    {
        jishu_0_x = 0;
        if (fabs(distance_x - Laser_Data_3) > x_close)
        {
			PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Far;
			PID_Vx_move_distance.KP = KP_x_Far;
			//PID_Vx_move_distance.KI = 0;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, distance_x - Laser_Data_3);
        }
        else
        {
			PID_w_move_distance.KD = 0;
			PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Close;
			PID_Vx_move_distance.KP = KP_x_Close;
			//PID_Vx_move_distance.KI = /*0.00012*/ 0.000145;
			//PID_Vx_move_distance.KD = 0.2298;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, distance_x - Laser_Data_3);
        }
    }

    // y-axis
    if (distance_y - Laser_Data_4 >= -0.01 && distance_y - Laser_Data_4 <= 0.01)
    {
		PID_Vy_move_distance.SumErr = 0;
		PID_Vy_move_distance.Sumlimit = 0.3;
		Robot.Robot_in_world.velocity_exp.Vy = 0;
        jishu_0_y++;
    }
    else
    {
        jishu_0_y = 0;
        if (fabs(distance_y - Laser_Data_4) > y_close)
        {
			PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Far;
			PID_Vy_move_distance.KP = KP_y_Far;
			//PID_Vy_move_distance.KI = 0.00011;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, distance_y - Laser_Data_4);
        }
        else
        {
			PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Close;
			PID_Vy_move_distance.KP = KP_y_Close;
			//PID_Vy_move_distance.KI = /*0.00012*/ 0.000145;
			//PID_Vy_move_distance.KD = 0.2298;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, distance_y - Laser_Data_4);
        }
    }

    if (distance_w - Robot.Robot_in_self.position_now.yaw >= -0.02 && distance_w - Robot.Robot_in_self.position_now.yaw <= 0.02)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
    }
    else if (distance_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        distance_w = -distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - distance_w > PI)
    {
        distance_w = 2 * PI + distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    Robot_Velocity_World_To_Self();
}


void PID_Vy_Laser_area1 (float position_x, float position_y, float position_w) {
	jishu_0_x = 10000;
    if (position_y - Laser_Data_1 >= -0.01 && position_y - Laser_Data_1 <= 0.01)
    {
		PID_Vy_move_distance.SumErr = 0;
		PID_Vy_move_distance.Sumlimit = 0.3;
		Robot.Robot_in_world.velocity_exp.Vy = 0;
        jishu_0_y++;
    }
    else
    {
        jishu_0_y = 0;
        if (fabs(position_y - Laser_Data_1) > y_close)
        {
			PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Far;
			PID_Vy_move_distance.KP = 3;
			//PID_Vy_move_distance.KI = 0.00011;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, position_y - Laser_Data_1);
        }
        else
        {
			PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Close;
			PID_Vy_move_distance.KP = 3;
			//PID_Vy_move_distance.KI = /*0.00012*/ 0.000145;
			//PID_Vy_move_distance.KD = 0.2298;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, position_y - Laser_Data_1);
        }
    }
	PID_w_move_distance.KP = 3;
    if (position_w - Robot.Robot_in_self.position_now.yaw >= -0.015 && position_w - Robot.Robot_in_self.position_now.yaw <= 0.015)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
    }
    else if (position_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        position_w = -position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - position_w > PI)
    {
        position_w = 2 * PI + position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    Robot_Velocity_World_To_Self();
}

void PID_Vy_Laser_area2 (float position_x, float position_y, float position_w) {
	jishu_0_x = 10000;
    if (position_y - Laser_Data_4 >= -0.01 && position_y - Laser_Data_4 <= 0.01)
    {
		PID_Vy_move_distance.SumErr = 0;
		PID_Vy_move_distance.Sumlimit = 0.3;
		Robot.Robot_in_world.velocity_exp.Vy = 0;
        jishu_0_y++;
    }
    else
    {
        jishu_0_y = 0;
        if (fabs(position_y - Laser_Data_4) > y_close)
        {
			PID_Vy_move_distance.Sumlimit = MAX_SPEED_Area2_y_Close;
			PID_Vy_move_distance.KP = 3;
			//PID_Vy_move_distance.KI = 0.00011;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, position_y - Laser_Data_4);
        }
        else
        {
			PID_Vy_move_distance.Sumlimit = MAX_SPEED_Area2_y_Close;
			PID_Vy_move_distance.KP = 3;
			//PID_Vy_move_distance.KI = /*0.00012*/ 0.000145;
			//PID_Vy_move_distance.KD = 0.2298;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, position_y - Laser_Data_4);
        }
    }
	PID_w_move_distance.KP = 3;
    if (position_w - Robot.Robot_in_self.position_now.yaw >= -0.015 && position_w - Robot.Robot_in_self.position_now.yaw <= 0.015)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
    }
    else if (position_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        position_w = -position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - position_w > PI)
    {
        position_w = 2 * PI + position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    Robot_Velocity_World_To_Self();
}



void PID_Vx_Laser_area2 (float position_x, float position_y, float position_w) {
	jishu_0_y = 10000;
	PID_w_move_distance.KP = 3;
    if (position_x - Laser_Data_3 >= -0.02 && position_x - Laser_Data_3 <= 0.02)
    {
        PID_Vx_move_distance.SumErr = 0;
		PID_Vx_move_distance.Sumlimit = 0.3;
		Robot.Robot_in_world.velocity_exp.Vx = 0;
        jishu_0_x++;
    }
    else
    {
        jishu_0_x = 0;
        if (fabs(position_x - Laser_Data_3) > x_close)
        {
			PID_Vx_move_distance.Sumlimit = MAX_SPEED_Area2_x_Far;
			PID_Vx_move_distance.KP = 2.5;
			//PID_Vx_move_distance.KI = 0;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, position_x - Laser_Data_3);
        }
        else
        {
			PID_w_move_distance.KD = 0;
			PID_Vx_move_distance.Sumlimit = MAX_SPEED_Area2_x_Close;
			PID_Vx_move_distance.KP = 2.5;
			//PID_Vx_move_distance.KI = /*0.00012*/ 0.000145;
			//PID_Vx_move_distance.KD = 0.2298;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, position_x - Laser_Data_3);
        }
    }

    if (position_w - Robot.Robot_in_self.position_now.yaw >= -0.015 && position_w - Robot.Robot_in_self.position_now.yaw <= 0.015)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
    }
    else if (position_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        position_w = -position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - position_w > PI)
    {
        position_w = 2 * PI + position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    Robot_Velocity_World_To_Self();
}

void PID_Vx_Laser_area1 (float position_x, float position_y, float position_w) {
	jishu_0_y = 10000;
	PID_w_move_distance.KP = 3;
    if (position_x - Laser_Data_2 >= -0.02 && position_x - Laser_Data_2 <= 0.02)
    {
                PID_Vx_move_distance.SumErr = 0;
		PID_Vx_move_distance.Sumlimit = 0.3;
		Robot.Robot_in_world.velocity_exp.Vx = 0;
        jishu_0_x++;
    }
    else
    {
        jishu_0_x = 0;
        if (fabs(position_x - Laser_Data_2) > x_close)
        {
			PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Far;
			PID_Vx_move_distance.KP = 3;
			//PID_Vx_move_distance.KI = 0;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, position_x - Laser_Data_2);
        }
        else
        {
			//PID_w_move_distance.KD = 0;
			PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Close;
			PID_Vx_move_distance.KP = 3;
			//PID_Vx_move_distance.KI = /*0.00012*/ 0.000145;
			//PID_Vx_move_distance.KD = 0.2298;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, position_x - Laser_Data_2);
        }
    }

    if (position_w - Robot.Robot_in_self.position_now.yaw >= -0.015 && position_w - Robot.Robot_in_self.position_now.yaw <= 0.015)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
    }
    else if (position_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        position_w = -position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - position_w > PI)
    {
        position_w = 2 * PI + position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    Robot_Velocity_World_To_Self();
}

void PID_Vx_Vy_action (float distance_x, float distance_y, float position_w) {
//	Robot.Robot_in_world.velocity_exp.Vx = 0;
//	jishu_0_x++;
	PID_w_move_distance.KP = 2.2;
    if (distance_x - (Robot.Robot_in_self.position_now.x - x_pre) >= -0.02 && distance_x - (Robot.Robot_in_self.position_now.x - x_pre) <= 0.02) {
        PID_Vx_move_distance.SumErr = 0;
		PID_Vx_move_distance.Sumlimit = 0.3;
        Robot.Robot_in_world.velocity_exp.Vx = 0;
        jishu_0_x++;
    }
    else {
        jishu_0_x = 0;
        PID_w_move_distance.KD = 0;
        PID_Vx_move_distance.Sumlimit = MAX_SPEED_Area2_x_Close;
        PID_Vx_move_distance.KP = KP_x_Close;
        //PID_Vx_move_distance.KI = /*0.00012*/ 0.000145;
        //PID_Vx_move_distance.KD = 0.2298;
        Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, distance_x - (Robot.Robot_in_self.position_now.x - x_pre));
    }

    if (distance_y - (Robot.Robot_in_self.position_now.y - y_pre) >= -0.02 && distance_y - (Robot.Robot_in_self.position_now.y - y_pre) <= 0.02) {
        PID_Vy_move_distance.SumErr = 0;
		PID_Vy_move_distance.Sumlimit = 0.3;
        Robot.Robot_in_world.velocity_exp.Vy = 0;
        jishu_0_y++;
    }
    else {
        jishu_0_y = 0;
        PID_Vy_move_distance.Sumlimit = MAX_SPEED_Area2_y_Close;
        PID_Vy_move_distance.KP = KP_y_Close;
        //PID_Vy_move_distance.KI = /*0.00012*/ 0.000145;
        //PID_Vy_move_distance.KD = 0.2298;
        Robot.Robot_in_world.velocity_exp.Vy = PIDCal_pos(&PID_Vy_move_distance, distance_y - (Robot.Robot_in_self.position_now.y - y_pre));
    }

    if (position_w - Robot.Robot_in_self.position_now.yaw >= -0.01 && position_w - Robot.Robot_in_self.position_now.yaw <= 0.01)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
    }
    else if (position_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        position_w = -position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - position_w > PI)
    {
        position_w = 2 * PI + position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    Robot_Velocity_World_To_Self();
}



void Move_Distance_Laser_3(float distance_x, float distance_y, float distance_w)
{                                 
    PID_w_move_distance.KP = 2.5; // 可以给大点
    PID_w_move_distance.KD = 0;
    // x-aixs

    if (distance_x - Laser_Data_4 >= -0.01 && distance_x - Laser_Data_4 <= 0.01)
    {
		PID_Vx_move_distance.SumErr = 0;
		PID_Vx_move_distance.Sumlimit = 0.3;
		Robot.Robot_in_world.velocity_exp.Vx = 0;
        jishu_0_x++;
    }
    else
    {
        jishu_0_x = 0;
        if (fabs(distance_x - Laser_Data_4) > x_close)
        {
			PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Far;
			PID_Vx_move_distance.KP = KP_x_Far;
			//PID_Vx_move_distance.KI = 0;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, distance_x - Laser_Data_4);
        }
        else
        {
			PID_w_move_distance.KD = 0;
			PID_Vx_move_distance.Sumlimit = Speed_Limit_x_Close;
			PID_Vx_move_distance.KP = KP_x_Close;
			//PID_Vx_move_distance.KI = /*0.00012*/ 0.000145;
			//PID_Vx_move_distance.KD = 0.2298;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, distance_x - Laser_Data_4);
        }
    }

    // y-axis
    if (distance_y - Laser_Data_3 >= -0.01 && distance_y - Laser_Data_3 <= 0.01)
    {
		PID_Vy_move_distance.SumErr = 0;
		PID_Vy_move_distance.Sumlimit = 0.3;
		Robot.Robot_in_world.velocity_exp.Vy = 0;
        jishu_0_y++;
    }
    else
    {
        jishu_0_y = 0;
        if (fabs(distance_y - Laser_Data_3) > y_close)
        {
			PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Far;
			PID_Vy_move_distance.KP = KP_y_Far;
			//PID_Vy_move_distance.KI = 0.00011;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, distance_y - Laser_Data_3);
        }
        else
        {
			PID_Vy_move_distance.Sumlimit = Speed_Limit_y_Close;
			PID_Vy_move_distance.KP = KP_y_Close;
			//PID_Vy_move_distance.KI = /*0.00012*/ 0.000145;
			//PID_Vy_move_distance.KD = 0.2298;
            Robot.Robot_in_world.velocity_exp.Vy = -PIDCal_pos(&PID_Vy_move_distance, distance_y - Laser_Data_3);
        }
    }

    if (distance_w - Robot.Robot_in_self.position_now.yaw >= -0.01 && distance_w - Robot.Robot_in_self.position_now.yaw <= 0.01)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
    }
    else if (distance_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        distance_w = -distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - distance_w > PI)
    {
        distance_w = 2 * PI + distance_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, distance_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    Robot_Velocity_World_To_Self();
}

void PID_Vx_Laser4_chouxiang (float position_x, float position_y, float position_w) {
	jishu_0_y = 10000;
	PID_w_move_distance.KP = 3;
    if (position_x - Laser_Data_4 >= -0.02 && position_x - Laser_Data_4 <= 0.02)
    {
        PID_Vx_move_distance.SumErr = 0;
		PID_Vx_move_distance.Sumlimit = 0.3;
		Robot.Robot_in_world.velocity_exp.Vx = 0;
        jishu_0_x++;
    }
    else
    {
        jishu_0_x = 0;
        if (fabs(position_x - Laser_Data_4) > x_close)
        {
			PID_Vx_move_distance.Sumlimit = MAX_SPEED_Area2_x_Far;
			PID_Vx_move_distance.KP = 3;
			//PID_Vx_move_distance.KI = 0;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, position_x - Laser_Data_4);
        }
        else
        {
			PID_w_move_distance.KD = 0;
			PID_Vx_move_distance.Sumlimit = MAX_SPEED_Area2_x_Close;
			PID_Vx_move_distance.KP = 3;
			//PID_Vx_move_distance.KI = /*0.00012*/ 0.000145;
			//PID_Vx_move_distance.KD = 0.2298;
            Robot.Robot_in_world.velocity_exp.Vx = PIDCal_pos(&PID_Vx_move_distance, position_x - Laser_Data_4);
        }
    }

    if (position_w - Robot.Robot_in_self.position_now.yaw >= -0.015 && position_w - Robot.Robot_in_self.position_now.yaw <= 0.015)
    {
        PID_w_move_distance.SumErr = 0;
        Robot.Robot_in_world.velocity_exp.Vw = 0;
        jishu_0_w++;
    }
    else if (position_w - Robot.Robot_in_self.position_now.yaw > PI)
    {
        position_w = -position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else if (Robot.Robot_in_self.position_now.yaw - position_w > PI)
    {
        position_w = 2 * PI + position_w;
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    else
    {
        Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, position_w - Robot.Robot_in_self.position_now.yaw);
        jishu_0_w = 0;
    }
    Robot_Velocity_World_To_Self();
}