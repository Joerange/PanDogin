//
// Created by Joerange on 2024/1/18.
//
#include "main.h"
#include "Jump_Task.h"


void ExecuteJump(uint8_t JumpType,float JumpAngle)
{
    if (JumpType == Standard_Jump)//标准跳（对绝大多数地面都有较好的适应性，兼具高度和远度）
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 800;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time = 200;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time = 200;      //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time = 1000;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_flylegheight = LegStandLenth; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(6.0f, 0, 0, 0);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(JumpAngle + 15, stance_height, prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(15.0f);//速度拉满
        ChangeGainOfPID(19.0f, 0, 0, 0);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(JumpAngle, jump_extension, launch_time);
        /*
        高刚度的实现：
        pos_kp很大：
                可以立刻输出一个很大的值给速度环，其值乘以角度差即为输出的速度目标值。3508速度上限8900，角度差按最小10度计算，则pos_kp可以设置为890。
        sp_kp很大：
                可以在当前速度与目标速度差别不是很大的时候也能输出很大的电流，更何况速度差别较大的时候。从而可以使电机的速度保持较大。
                C620电调电流上限值16384（对应20A），该值明显在数量级上小于通常所需要的速度值。
                我们的控制过程是需要将速度从8900快速变为0，速度值为250时已经很慢了，以它为标准，配置sp_kp为8已经够用了，因此速度环的PID是比较普适的，一个8走天下。
        因此，总的控制效果近似由下式决定：Speed=Current=angle*pos_kp。因此，若我们希望在临界目标角度10度前电机能始终保持最大速度，那么，我们可以由下式计算：
        pos_kp = CurrentMAX/angle_Thresh = 8900/10 = 890。10度已经是比较小了，angle_Thresh越小，越容易超调，从而造成危害。因此不建议小于10度。
        高刚度表现为，转动很小的角度都很费力，即很难转动电机。
        */
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(8.0f, 0, 0, 0);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(-25, jump_flylegheight, fall_time);
        /*
        低刚度：
            根据上述公式：pos_kp*sp_kp = CurrentMAX/angle_Thresh，低刚度意味着我们要选择一个略微大一点的angle_Thresh，从而我们可以小角度内比较容易摆动电机，而到一定角度则掰不动了。
            比如我们选择30度，则有：
            pos_kp = CurrentMAX/angle_Thresh = 8900/30 ≈ 300。故，为此可以配置pos_kp为300。
            低刚度表现为，在一定角度范围内比较容易转动，但角度越大越费力，达到临界阈值角度很难继续转动，并且持续时间越久越费力（因为有默认的I）。
        */
        //脚用力准备站起来
        ChangeGainOfPID(8.0f, 0, 0, 0);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(-70, jump_landlegheight, strech_time);
        //差不多站好了，执行
        gpstate = 1;
    }
    else if(JumpType == High_Jump)//简单原地跳个高（任何地面都行）
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 300;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time = 200;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time = 100;      //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time = 300;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_flylegheight = LegStandLenth; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(8.0f,0.0f,0,0);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(JumpAngle,stance_height,prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(15.0f);//速度拉满
        ChangeGainOfPID(19.0f,0,0,0);// 使用高刚度和低阻尼执行跳转
        SetPolarPositionAll_Delay(JumpAngle,jump_extension,launch_time);
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(5.0f,0.0f,0,0);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionAll_Delay(-25,jump_flylegheight,fall_time);
        //脚用力准备站起来
        ChangeGainOfPID(6,0,0.0f,0.0f);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionAll_Delay(-88,jump_landlegheight,strech_time);
        //差不多站好了，执行完毕
        gpstate = 1;
    }
}