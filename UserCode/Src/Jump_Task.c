//
// Created by Joerange on 2024/1/18.
//
#include "main.h"
#include "Jump_Task.h"

extern float times;
extern uint8_t reverse_move_flag;

void ExecuteJump(uint8_t JumpType,float JumpAngle)
{
    if (JumpType == Standard_Jump)//标准跳（对绝大多数地面都有较好的适应性，兼具高度和远度）
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 1000;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time = 300;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time = 200;      //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time = 650;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_flylegheight = LegStandLenth; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(6.0f, 0.5f, 0, 0);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(JumpAngle + 13, stance_height, prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(SpeedMode_VERYEX);//速度拉满
        ChangeGainOfPID(35.0f,0.23f,0, 0);//使用刚度小，阻尼大的增益0
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
        ChangeGainOfPID(8.0f, 0.3f, 0, 0);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(-25, jump_flylegheight, fall_time);
        /*
        低刚度：
            根据上述公式：pos_kp*sp_kp = CurrentMAX/angle_Thresh，低刚度意味着我们要选择一个略微大一点的angle_Thresh，从而我们可以小角度内比较容易摆动电机，而到一定角度则掰不动了。
            比如我们选择30度，则有：
            pos_kp = CurrentMAX/angle_Thresh = 8900/30 ≈ 300。故，为此可以配置pos_kp为300。
            低刚度表现为，在一定角度范围内比较容易转动，但角度越大越费力，达到临界阈值角度很难继续转动，并且持续时间越久越费力（因为有默认的I）。
        */
        //脚用力准备站起来
        ChangeGainOfPID(8.0f, 0.18f, 0, 0);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(-80, jump_landlegheight, strech_time);
        //差不多站好了，执行
        gpstate = 1;
    }
    else if(JumpType == High_Jump)//简单原地跳个高（任何地面都行）
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 300;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time = 200;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time = 100;      //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time = 150;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_flylegheight = LegStandLenth + 4; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(8.0f,0.0f,0,0);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(JumpAngle,stance_height,prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(30.0f);//速度拉满
        ChangeGainOfPID(30.0f,0,0,0);// 使用高刚度和低阻尼执行跳转
        SetPolarPositionAll_Delay(JumpAngle,jump_extension,launch_time);
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(5.0f,5.0f,0,0);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionAll_Delay(-55,jump_flylegheight,fall_time);
        //脚用力准备站起来
        ChangeGainOfPID(6,0,0.0f,0.0f);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionAll_Delay(-88,jump_landlegheight,strech_time);
        //差不多站好了，执行完毕
        gpstate = HALT;
    }
    else if(JumpType == Far_Jump)//简单跳个远（要求地面摩擦较大）
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 300;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time = 200;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time = 150;      //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time = 250;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_flylegheight = LegStandLenth-2; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,0.0f,0.0f);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(JumpAngle,stance_height,prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(30);//速度拉满
        ChangeGainOfPID(30.0f,0.1f,0.0f,0.0f);
        SetPolarPositionAll_Delay(JumpAngle,jump_extension,launch_time);
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(9  ,2.1f,0.0f,0.0f);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionAll_Delay(-55,jump_flylegheight,fall_time);
        //脚用力准备站起来
        ChangeGainOfPID(9,2.1f,0.0f,0.0f);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionAll_Delay(-80,jump_landlegheight,strech_time);
        //差不多站好了，执行完毕
        gpstate = HALT_IMU;
    }
    else if(JumpType == Leap_Jump)//非常极限的四足同时跳。
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 400;  //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time=160;  //伸展腿的持续时间                 [s]  0.2
        const uint16_t fall_time = 400;  //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time=500;  //落地并用力支撑的时间             [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;//跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthExtremeMax;//伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_landlegheight = LegStandLenth;//落地时腿长度 [cm]，理论上应等于LegStandLenth 18.0f
        const float jump_angle = JumpAngle;
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(6.0f,0.2f,0.26f,25,0.22f);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//速度拉满
        ChangeAllGainOfPID(8,0.1f,3.0f,200,0.1f);// 使用高刚度和低阻尼执行跳转
        SetPolarPositionFB_Delay(Leg_Front,jump_angle,jump_extension,0);
        SetPolarPositionFB_Delay(Leg_Back,jump_angle,jump_extension,launch_time);
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(5,0.1f,0.8f,30,2.0f);//要求具有非常好的响应速度和调节速度
        SetPolarPositionFB_Delay(Leg_Front,10,LegSquatLenth,0);
        SetPolarPositionFB_Delay(Leg_Back,-65,LegLenthMin,fall_time/2);
        SetPolarPositionFB_Delay(Leg_Front,-70,LegStandLenth,fall_time/2);
        //脚用力准备站起来
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        FBLegsPID_Set(Leg_Front,6,0.1f,0.08f,35,1.1f);
        FBLegsPID_Set(Leg_Back,6,0.1f,0.1f,130,1.1f);
        SetPolarPositionFB_Delay(Leg_Front,-70,jump_landlegheight,0);
        SetCartesianPositionFB_Delay(Leg_Back,0.2f,LegSquatLenth,strech_time);
        //差不多站好了，执行完毕
        gpstate = HALT;
    }
}
//阶梯专用分段跳跃函数（三段跳解决问题）
void StairJump(uint8_t stage)
{
    if(stage == 0)//正常情况下可以一次性跳两级台阶
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 300;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time =200;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time =250;      //在空中飞翔的时间                  [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time =300;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_flylegheight = LegStandLenth+5; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        const float jump_angle = 68;// 向前跳跃的角度[°]
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,70,0.22f);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//速度拉满
        ChangeAllGainOfPID(7,0.1f,2.5f,200,0.1f);//使用高刚度和低阻尼执行跳转
        SetPolarPositionAll_Delay(jump_angle,jump_extension,launch_time);
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8,0.1f,70.0f,0);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionAll_Delay(-22,jump_flylegheight,fall_time);//前倾
        //脚用力准备站起来
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(5,0.1f,0.7f,25,2.0f);//站立态PID
        SetPolarPositionFB_Delay(Leg_Back,-80,LegSquatLenth,0);
        SetPolarPositionFB_Delay(Leg_Front,-70,jump_landlegheight,strech_time);
        //差不多站好了，执行完毕
        gpstate = HALT;
    }
    if(stage == 1)//正常情况下可以后腿落到阶梯顶层
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 300;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time =200;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time =350;      //在空中飞翔的时间                  [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time =300;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_flylegheight = LegStandLenth+7.2f; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        const float jump_angle = 55;// 向前跳跃的角度[°]
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,70,0.22f);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//速度拉满
        ChangeAllGainOfPID(7,0.1f,2.5f,200,0.1f);//使用高刚度和低阻尼执行跳转
        SetPolarPositionAll_Delay(jump_angle,jump_extension,launch_time);
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8,0.1f,70.0f,0);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionAll_Delay(-22,jump_flylegheight,fall_time);//前倾
        //脚用力准备站起来
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(5,0.1f,0.7f,25,2.0f);//站立态PID
        SetPolarPositionFB_Delay(Leg_Back,80,LegSquatLenth,0);
        SetPolarPositionFB_Delay(Leg_Front,-70,jump_landlegheight,strech_time);
        //差不多站好了，执行完毕
        gpstate = HALT;
    }
    else if(stage == 2)//正常情况可以后腿落到最后一级台阶。
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 300;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time =200;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time =450;      //在空中飞翔的时间                  [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time =300;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_flylegheight = LegStandLenth; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        const float jump_angle = 85;// 向前跳跃的角度[°] 该变量决定了向前跳跃的角度，从而决定前跳的距离。角度基准是地面，而不是垂直地面，因此越小越远。不建议小于40度。
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,70,0.22f);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//速度拉满
        ChangeAllGainOfPID(7,0.1f,0.2f,190,0.1f);//使用高刚度和低阻尼执行跳转
        SetPolarPositionAll_Delay(jump_angle,jump_extension,launch_time);
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8,0.1f,70.0f,0);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionFB_Delay(Leg_Back,80,LegSquatLenth,0);
        SetPolarPositionFB_Delay(Leg_Front,-25,jump_flylegheight,fall_time);//前倾
        //脚用力准备站起来
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(6,0.1f,60.0f,0.3f);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionFB_Delay(Leg_Back,80,LegSquatLenth,0);
        SetPolarPositionFB_Delay(Leg_Front,-70,jump_landlegheight,strech_time);
        //差不多站好了，执行完毕
        gpstate = HALT;
    }
    else if(stage == 3)
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 300;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time =200;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time =500;      //在空中飞翔的时间                  [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time =300;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_flylegheight = LegStandLenth+5; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        const float jump_angle = 60;// 向前跳跃的角度[°] 该变量决定了向前跳跃的角度，从而决定前跳的距离。角度基准是地面，而不是垂直地面，因此越小越远。不建议小于40度。
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,70,0.22f);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//速度拉满
        ChangeAllGainOfPID(7.38f,0.1f,1.5f,222.5f,0.1f);// 使用高刚度和低阻尼执行跳转
        SetPolarPositionAll_Delay(jump_angle,jump_extension,launch_time);
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8,0.1f,70.0f,0);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionFB_Delay(Leg_Back,65,LegLenthMin,0);
        SetPolarPositionFB_Delay(Leg_Front,-25,jump_flylegheight,fall_time);//前倾
        //脚用力准备站起来
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(6,0.1f,35.0f,0.3f);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionAll_Delay(-70,jump_landlegheight,strech_time);
        //差不多站好了，执行完毕
        gpstate = HALT;
    }
}
//跷跷板专用分段跳跃函数（三段跳解决问题）
void SeesawJump(uint8_t stage)
{
    if(stage == 0)
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 400;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time=160;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time = 200;      //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time=500;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthExtremeMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        const float jump_angle = 60;
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(6.0f,0.2f,0.26f,25,0.22f);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//速度拉满
        ChangeAllGainOfPID(8,0.1f,3.0f,200,0.1f);// 使用高刚度和低阻尼执行跳转
        SetPolarPositionFB_Delay(Leg_Front,jump_angle,jump_extension,0);
        SetPolarPositionFB_Delay(Leg_Back,jump_angle,jump_extension,launch_time);
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(5,0.1f,0.8f,30,2.0f);//要求具有非常好的响应速度和调节速度
        TargetAngle1=71.5f;
        TargetAngle2=180.0 - 59.1;
        SetCoupledThetaPositionAll();
        osDelay(fall_time);
        //脚用力准备站起来
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        FBLegsPID_Set(Leg_Front,6,0.1f,0.08f,35,1.1f);
        FBLegsPID_Set(Leg_Back,6,0.1f,0.1f,130,1.1f);
        SetPolarPositionAll_Delay(70,jump_landlegheight,strech_time);
        //差不多站好了，执行完毕
        gpstate = HALT;
    }
    else if(stage == 1)
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 400;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time=160;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time = 200;      //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time=500;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthExtremeMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        const float jump_angle = 55;
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(6.0f,0.2f,0.26f,25,0.22f);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//速度拉满
        ChangeAllGainOfPID(8,0.1f,3.0f,200,0.1f);// 使用高刚度和低阻尼执行跳转
        SetPolarPositionFB_Delay(Leg_Front,jump_angle,jump_extension,0);
        SetPolarPositionFB_Delay(Leg_Back,jump_angle,jump_extension,launch_time);
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(5,0.1f,0.8f,30,2.0f);//要求具有非常好的响应速度和调节速度
        TargetAngle1=71.5f;
        TargetAngle2=180-59.1;
        SetCoupledThetaPositionAll();
        osDelay(fall_time);
        //脚用力准备站起来
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        FBLegsPID_Set(Leg_Front,6,0.1f,0.08f,35,1.1f);
        FBLegsPID_Set(Leg_Back,6,0.1f,0.1f,130,1.1f);
        SetPolarPositionAll_Delay(70,jump_landlegheight,strech_time);
        //差不多站好了，执行完毕
        gpstate = HALT;
    }
    else
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 300;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
        const uint16_t launch_time =200;    //伸展腿的持续时间                  [s]  0.2
        const uint16_t fall_time =250;      //在空中飞翔的时间                  [s]  0.25（这个时间最好设置的小点）
        const uint16_t strech_time =300;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
        const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
        const float jump_flylegheight = LegStandLenth; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
        const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        const float jump_angle = 85;// 向前跳跃的角度[°] 该变量决定了向前跳跃的角度，从而决定前跳的距离。角度基准是地面，而不是垂直地面，因此越小越远。不建议小于40度。
        //下蹲，准备起跳，持续时间为prep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,70,0.22f);//使用刚度小，阻尼大的增益
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //芜湖起飞（核心），持续时间为launch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//速度拉满
        ChangeAllGainOfPID(7.38f,0.1f,0.4f,222.5f,0.1f);// 使用高刚度和低阻尼执行跳转
        SetPolarPositionAll_Delay(jump_angle,jump_extension,launch_time);
        //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8,0.1f,70.0f,0);//使用低刚度和大量的阻尼来处理下降
        SetPolarPositionAll_Delay(-20,jump_flylegheight,fall_time);//前倾
        //脚用力准备站起来
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(6,0.1f,60.0f,0.3f);//使用低刚度和大量的阻尼来处理下降（PID计算速度2ms，6,0.1,60的PID参数具有很好的缓冲能力！）
        SetPolarPositionAll_Delay(-70,jump_landlegheight,strech_time);
        //差不多站好了，执行完毕
        gpstate = HALT;
    }
}
//双木桥专用跳跃命令（跳上双木桥，然后边走边微调IMU即可快速通过，也可以尝试稳健的跳跃通过）
void Bridge_Jump(uint8_t stage)
{
    switch(stage)
    {
        case 0:
        {
            /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
            const uint16_t prep_time = 300;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
            const uint16_t launch_time=200;    //伸展腿的持续时间                  [s]  0.2
            const uint16_t fall_time = 130;      //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
            const uint16_t strech_time=250;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
            /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
            const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
            const float jump_extension = LegLenthMax-1.0f; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
            const float jump_flylegheight = LegStandLenth-2; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
            const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
            const float jump_angle = 75;
            //下蹲，准备起跳，持续时间为prep_time
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            ChangeGainOfPID(8.0f,0.1f,0.0f,0);//使用刚度小，阻尼大的增益
            SetPolarPositionAll_Delay(jump_angle + IMU_EulerAngle.EulerAngle[Pitch],stance_height,prep_time);
            //芜湖起飞（核心），持续时间为launch_time
            AllLegsSpeedLimit(30);//速度拉满
            ChangeGainOfPID(30.0f,0.1f,0.0f,0);// 使用高刚度和低阻尼执行跳转
            SetPolarPositionFB_Delay(Leg_Front,jump_angle + IMU_EulerAngle.EulerAngle[Pitch],jump_extension,0);
            SetPolarPositionFB_Delay(Leg_Back,jump_angle + IMU_EulerAngle.EulerAngle[Pitch],jump_extension,launch_time);
            //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            ChangeGainOfPID(8,0.1f,0.0f,0);//使用低刚度和大量的阻尼来处理下降
            SetPolarPositionAll_Delay(-45,jump_flylegheight,fall_time);
            //脚用力准备站起来
            AllLegsSpeedLimit(SpeedMode_FAST);
            ChangeGainOfPID(8,0.1f,0.0f,0);//使用低刚度和大量的阻尼来处理下降
            FBLegsPID_Set(Leg_Front,6,0.1f,0.08f,35,1.1f);
            FBLegsPID_Set(Leg_Back,6,0.1f,0.1f,130,1.1f);
            SetPolarPositionAll_Delay(-62,jump_landlegheight,strech_time);
            //差不多站好了，执行完毕
            gpstate = HALT_IMU;
            break;
        }
        case 1:
        {
            /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
            const uint16_t prep_time = 300;       //准备时间，即收缩退准备起跳的时间  [s]  0.4
            const uint16_t launch_time=200;    //伸展腿的持续时间                  [s]  0.2
            const uint16_t fall_time = 300;      //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
            const uint16_t strech_time=500;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
            /*跳跃的姿态把控（调节时，可按0.1的整数倍进行加减调整，如（LegSquatLenth-0.4））*/
            const float stance_height = LegLenthMin;  //跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f，这里测试跳跃时可以使用LegLenthMin 10.7f
            const float jump_extension = LegLenthMax; //伸展跳跃的最大伸腿长度      [cm]，理论上应等于LegLenthMax 28
            const float jump_flylegheight = LegStandLenth-2; //飞翔时腿长度   [cm]，经验上应介于LegLenthMax 28与LegStandLenth 18.0f之间，是一个适中的值。
            const float jump_landlegheight = LegStandLenth; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
            const float jump_angle = 56;
            //下蹲，准备起跳，持续时间为prep_time
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            ChangeAllGainOfPID(6.0f,0.2f,0.26f,25,0.22f);//使用刚度小，阻尼大的增益
            SetPolarPositionFB_Delay(Leg_Back,jump_angle,stance_height,0);
            SetPolarPositionFB_Delay(Leg_Front,jump_angle+6,stance_height,prep_time);
            //芜湖起飞（核心），持续时间为launch_time
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//速度拉满
            ChangeAllGainOfPID(6,0.1f,2.0f,200,0.1f);// 使用高刚度和低阻尼执行跳转
            SetPolarPositionFB_Delay(Leg_Front,jump_angle,jump_extension+1.0f,0);
            SetPolarPositionFB_Delay(Leg_Back,jump_angle,jump_extension,launch_time);
            //飞翔过程（也即降落过程）中的姿态（核心），持续时间为fall_time
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            ChangeGainOfPID(5,0.1f,35.0f,0);//使用低刚度和大量的阻尼来处理下降
            SetPolarPositionAll_Delay(-25,jump_flylegheight,fall_time);
            //脚用力落地
            AllLegsSpeedLimit(SpeedMode_FAST);
            FBLegsPID_Set(Leg_Back,5,0.1f,0.20f,20,2.0f);
            FBLegsPID_Set(Leg_Back,5,0.1f,0.20f,20,2.0f);
            SetPolarPositionAll_Delay(-70,jump_landlegheight,strech_time);
            //站起来
            ChangeAllGainOfPID(5,0.1f,0.55f,25,2.0f);//站立态PID
            AllLegsSpeedLimit(SpeedMode_VERYSLOW);
            TargetAngle1=0;TargetAngle2=180;
            SetCoupledThetaPositionAll();
            osDelay(600);
            //差不多站好了，执行完毕
            gpstate = HALT;
            break;
        }

    }
}
/***
//前空翻跳跃，即后腿先跳，随后是前腿。
//该跳跃有不同的模式，包括正常前空翻跳、侧旋跳、双足直立跳等（跳跃效果乃至成功与否与地面材质密切相关）。
//执行该跳跃时请做好保护措施，否则容易造成损失。
***/
void FrontFlipJump(uint8_t mode)
{
    //模式0：前空翻跳（成功跳过高栏）
    if(mode==0)
    {
        uint8_t SeconTime=0;//判断是否是第二次到达
        uint32_t timedelay=0;//非阻塞延时，让跳跃与IMU结合更合理
        //IMU到达特定角度范围跳跃的控制变量
        uint8_t imu_wait_lock = 1;//IMU锁定
        float imu_fullscale_correction = 3;//即为理论满量程（90度）与实际满量程之差。
        float takeoff_inclination = 67-imu_fullscale_correction;//核心参数 （inclination：倾向）
        float imu_angle_half_range = 1.5f;//可以满足使用
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 850;            //准备时间，即收缩退准备起跳的时间 [ms]
        const uint16_t backleg_jump_time  = 150;   //后退跳的持续时间				   [ms]
        const uint16_t frontleg_jump_time = 180;   //前腿跳的持续时间                [ms]
        /*跳跃的姿态把控*/
        const float stance_height = LegLenthMin+1.0f;//跳跃之前腿的高度  [cm]
        const float jump_extension = LegLenthExtremeMax;//最大是LegLenthExtremeMax
        const float backleg_jump_angle  =  81.0f;//理论上前腿后移主要由该角度不合适引起
        const float frontleg_squat_angle = 60.0f;
        const float delta_angle = 10;
        /***********************************************************/
        /*下蹲准备起跳，持续时间为prep_time*/
        ChangeAllGainOfPID(5,0.1f,0.7f,25,2.0f);//站立态PID//ChangeAllGainOfPID(8,0.1,0.26,200,0.22);
        AllLegsSpeedLimit(SpeedMode_SLOW);
        //后腿下蹲
        SetPolarPositionFB_Delay(Leg_Back,backleg_jump_angle,stance_height,0);
        //前腿“倾斜”下蹲
        SetPolarPositionFB_Delay(Leg_Front,frontleg_squat_angle,stance_height,prep_time);
        /***********************************************************/
        /*后腿起跳，前腿立刻变化一段（这种情况下，若前腿变化合适，理论上转动的支点不再是足端，而应当看成是髋关节），持续时间为launch_time*/
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//速度拉满
        LegPID_Set(1,8,0.1f,5.5f,255,0.1f);LegPID_Set(3,8,0.1f,5.5f,255,0.1f);//使用高刚度和低阻尼执行跳转
        /*********后腿起跳**********/
        SetPolarPositionFB_Delay(Leg_Back,backleg_jump_angle,jump_extension,0);
        /*********前腿立刻再向前一点**********/
        LegPID_Set(0,5,0.1f,0.7f,25,2.0f);//ChangeAllGainOfPID(5,0.1,0.7,25,2.0);//站立态PID
        LegPID_Set(2,5,0.1f,0.7f,25,2.0f);
        LegSpeedLimit(0,4000);
        LegSpeedLimit(2,4000);
        SetPolarPositionFB_Delay(Leg_Front,frontleg_squat_angle+delta_angle,stance_height,0);
        /***********************************************************/
        //前腿逐渐进入水平态（核心，与IMU反馈的pitch角配合执行跳跃），后腿迅速卧倒
        while(imu_wait_lock)
        {
            /***********后腿等待倒立的时机************/
            timedelay++;
            if(timedelay == backleg_jump_time/5) //非阻塞延时，约5ms一次。
            {
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******后腿慢慢倒立态（必要，否则会卡住）******/
                LegSpeedLimit(1,SpeedMode_SLOW + 150);
                LegSpeedLimit(3,SpeedMode_SLOW + 150);
                //后腿反向转
                TargetAngle1=-180;
                TargetAngle2=360;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            /***********前腿慢慢水平************/
            LegPID_Set(0,7.5f,0.1f,0.8f,180,0.1f);
            LegPID_Set(2,7.5f,0.1f,0.8f,180,0.1f);
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            SetCartesianPositionFB_Delay(Leg_Front,LegSquatLenth,-0.15f,0);
            //一直到角度合适，然后切换到下一个步态。
            if(SeconTime==0 && IMU_EulerAngle.EulerAngle[Pitch]> (takeoff_inclination-imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< (takeoff_inclination+imu_angle_half_range) )
            {
                SeconTime=2;
            }
            else if(SeconTime==2 && IMU_EulerAngle.EulerAngle[Pitch]> (88-imu_fullscale_correction) && IMU_EulerAngle.EulerAngle[Pitch]< (90-imu_fullscale_correction) )//当pitch角度出现系统误差时，可能角度范围达不到这里，因此无法前翻，必须先在imu.c接收中断中修正系统误差。
            {
                SeconTime=1;
            }
            else if(SeconTime==1 && IMU_EulerAngle.EulerAngle[Pitch]> (takeoff_inclination-imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< (takeoff_inclination+imu_angle_half_range) )
            {
                imu_wait_lock=0;
                //避免过快翻转导致后腿没来得及卧倒，故这里强制再进行一次卧倒。
                FBLegsPID_Set(Leg_Back,6,0.1f,0.04f,35,0.1f);
                FBLegsSpeedLimit(Leg_Back,SpeedMode_SLOW + 150);
                //后腿反向转
                TargetAngle1=-180;TargetAngle2=360;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            osDelay(5);//每次延时5ms，保证其它任务正常执行的同时，也不会有因为延时过高而错过IMU角度，同时也方便进行计数计时。
        }
        /***********************************************************/
        //前腿跳跃
        LegSpeedLimit(0,SpeedMode_JUMPPEDAL);//速度拉满
        LegSpeedLimit(2,SpeedMode_JUMPPEDAL);//速度拉满
        ChangeAllGainOfPID(8,0.1f,5.5f,255,0.1f);//使用高刚度和低阻尼执行跳转
        SetCartesianPositionFB_Delay(Leg_Front,jump_extension,-0.15f,frontleg_jump_time);
        /***********************************************************/
        //前腿正站立
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(8.0f,0.1f,0.25f,100,0.1f);
//		ChangeAllGainOfPID(5,0.1,0.7,25,2.0);//站立态PID
        TargetAngle1=0;TargetAngle2=180;
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(300);
        /***********************************************************/
        //最后阶段（前、后腿均变为四脚朝天态，但转动的方向相反防止机械卡位）
        ChangeAllGainOfPID(8.0f,0.1f,0.25f,100,0.1f);//使用低刚度和大量的阻尼来处理下降
        //后腿反方向（向后）转动
        ReverseMoveOpen();
        SetCartesianPositionFB_Delay(Leg_Back,0,-LegStandLenth,0);
        ReverseMoveClose();
        //前腿正常（向前）转动
        SetCartesianPositionFB_Delay(Leg_Front,0,-LegStandLenth,0);
        gpstate = STOP;//回到停止态
    }
        //模式1：空中侧旋跳
    else if(mode == 1)
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const float prep_time = 0.4f;       //准备时间，即收缩退准备起跳的时间   [s]  0.4
        const float launch_time = 0.12f;     //伸展腿的持续时间                  [s]  0.2
        const float fall_time = 0.3f;       //在空中飞翔的时间                  [s]  0.25（这个时间最好设置的小点）
        const float strech_time = 0.17f;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        const float shrink_time = 0.21f;
        /*跳跃的姿态把控*/
        const float stance_height = LegLenthMin+1.0f;//跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax;//最大是LegLenthExtremeMax
        const float jump_landlegheight = LegLenthMin+3.0f; //落地时腿长度  [cm]，理论上应等于LegStandLenth 18.0f
        const float jump_angle = 88;// 向前跳跃的角度[°] 该变量决定了向前跳跃的角度，从而决定前跳的距离。角度基准是地面，而不是垂直地面，因此越小越远。不建议小于40度。
        /*心跳控制*/
        static uint32_t firt_execute=0;//每个执行周期的启动标志
        if(firt_execute == 0 )
        {
            /*强行控制心跳为最初*/
            times=0;
        }
        firt_execute=1;
        float tt=times*5/1000;
        /*跳跃过程*/
        //下蹲准备起跳，持续时间为prep_time
        if (tt < prep_time)
        {
            ChangeGainOfPID(8,0.1f,140,0.22f);//恢复正常PD
            AllLegsSpeedLimit(SpeedMode_FAST);
            x = -stance_height * cos(jump_angle * PI/180);//要转换为弧度再带入三角函数计算
            y =  stance_height * sin(jump_angle * PI/180);
            CartesianToTheta();
            SetCoupledThetaPositionAll();
        }
            //后腿起跳，持续时间为launch_time
        else if (tt >= prep_time && tt < (prep_time + launch_time))
        {
            //速度拉满
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //使用高刚度和低阻尼执行跳转
            ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD不要动了，主要是调节I值。
            /*********后腿起跳**********/
            x = -jump_extension * cos(jump_angle * PI/180);
            y =  jump_extension * sin(jump_angle * PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
        }
            //前腿逐渐进入倒立态，后腿站立
        else if (tt >= (prep_time + launch_time) && tt < (prep_time + launch_time + fall_time))
        {
            AllLegsSpeedLimit(SpeedMode_FAST);
            ChangeAllGainOfPID(8.0f,0.1f,2.5f,200,0.1f);
            /******后腿（正）站立态******/
            TargetAngle2=-180;
            TargetAngle1=0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /*******前腿逐渐倒立*********/
            x=0;
            y=-LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //前腿跳跃
        else if (tt >= (prep_time + launch_time + fall_time) && tt < (prep_time + launch_time + fall_time + strech_time))
        {
            /*********前腿跳跃********/
            //速度拉满
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //使用高刚度和低阻尼执行跳转
            ChangeAllGainOfPID(8,0.1f,6.0f,222,0.1f);//PD不要动了，主要是调节I值。
            y=-0.15f;
            x=jump_extension+0.5f;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //站立
        else if (tt >= (prep_time + launch_time + fall_time + strech_time) && tt < (prep_time + launch_time + fall_time + strech_time + shrink_time))
        {
            x = jump_landlegheight * cos(75 * PI/180);
            y = jump_landlegheight * sin(75 * PI/180);
            CartesianToTheta();
            //使用低刚度和大量的阻尼来处理下降
            ChangeGainOfPID(6,0.1f,35.0f,0.3f);
            SetCoupledThetaPositionAll();
        }
            //落地前一瞬间
        else
        {
            firt_execute=0;//回归函数的初始态
            gpstate = HALT;//回到站立态
        }
    }
        //模式2：双足直立跳
    else if(mode == 2)
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const float prep_time = 0.4f;        //准备时间，即收缩退准备起跳的时间   [s]  0.4
        const float launch_time = 0.12f;     //伸展腿的持续时间                  [s]  0.2
        const float fall_time = 0.25f;        //在空中飞翔的时间                  [s]  0.25（这个时间最好设置的小点）
        const float strech_time = 0.17f;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        const float shrink_time = 0.3f;
        /*跳跃的姿态把控*/
        const float stance_height = LegLenthMin+1.0f;//跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax;//最大是LegLenthExtremeMax
        const float jump_angle = 88;// 向前跳跃的角度[°] 该变量决定了向前跳跃的角度，从而决定前跳的距离。角度基准是地面，而不是垂直地面，因此越小越远。不建议小于40度。
        /*心跳控制*/
        static uint32_t firt_execute=0;//每个执行周期的启动标志
        if(firt_execute == 0 )
        {
            /*强行控制心跳为最初*/
            times=0;
        }
        firt_execute=1;
        float tt=times*5/1000;
        /*跳跃过程*/
        //下蹲准备起跳，持续时间为prep_time
        if (tt < prep_time)
        {
            ChangeGainOfPID(8,0.1f,140,0.22f);//恢复正常PD
            AllLegsSpeedLimit(SpeedMode_FAST);
            x = -stance_height * cos(jump_angle * PI/180);//要转换为弧度再带入三角函数计算
            y =  stance_height * sin(jump_angle * PI/180);
            CartesianToTheta();
            SetCoupledThetaPositionAll();
        }
            //后腿起跳，持续时间为launch_time（核心，要与前腿进入水平态的时间配合好）
        else if (tt >= prep_time && tt < (prep_time + launch_time))
        {
            //速度拉满
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //使用高刚度和低阻尼执行跳转
            ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD不要动了，主要是调节I值。
            /*********后腿起跳**********/
            x = -jump_extension * cos(jump_angle * PI/180);
            y =  jump_extension * sin(jump_angle * PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
        }
            //前腿进入水平态（核心，其延时要与跳跃点配合好），后腿卧倒
        else if (tt >= (prep_time + launch_time) && tt < (prep_time + launch_time + fall_time))
        {
            AllLegsSpeedLimit(3500);
            ChangeAllGainOfPID(8.0f,0.1f,4.5f,210,0.2f);
            /******后腿卧倒态（必要，否则会卡住）******/
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /************前腿水平************/
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //前腿跳跃
        else if (tt >= (prep_time + launch_time + fall_time) && tt < (prep_time + launch_time + fall_time + strech_time))
        {
            //速度拉满
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //使用高刚度和低阻尼执行跳转
            ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD不要动了，主要是调节I值。
            y=-0.15f;
            x=jump_extension;//跳跃伸腿
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //后腿快速卧倒
        else if (tt >= (prep_time + launch_time + fall_time + strech_time) && tt < (prep_time + launch_time + fall_time + strech_time + shrink_time))
        {
            /******后腿继续快速卧倒态（必要，否则会卡住）******/
            ChangeAllGainOfPID(7,0.1f,1.1f,150,0.1f);
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
        }
            //落地前一瞬间
        else
        {
            //倒立站起
            x=0;
            y=-LegStandLenth;
            CartesianToTheta();
            //使用低刚度和大量的阻尼来处理下降
            ChangeAllGainOfPID(8.0f,0.1f,0.6f,100,0.1f);
            SetCoupledThetaPositionAll();
            //跳出函数
            firt_execute=0;//回归函数的初始态
            gpstate = STOP;//回到停止态
            ChangeGainOfPID(8,0.1f,70,0.22f);//恢复正常PD
        }
    }
        //模式3：快速向前翻
    else if(mode == 3)
    {
        uint8_t SeconTime=0;//判断是否是第二次到达
        uint32_t timedelay=0;//非阻塞延时，让跳跃与IMU结合更合理
        //IMU到达特定角度范围跳跃的控制变量
        uint8_t imu_wait_lock = 1;//IMU锁定
        float imu_fullscale_correction = 5;//满量程变化时进行修正
        float takeoff_inclination = 80-imu_fullscale_correction;//核心参数
        float imu_angle_half_range = 1;//可以满足使用
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 500;            //准备时间，即收缩退准备起跳的时间 [s]
        const uint16_t backleg_jump_time  = 150;    //后退跳的持续时间

        /*跳跃的姿态把控*/
        const float stance_height = LegLenthMin+1.0f;//跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax-1;//最大是LegLenthExtremeMax
        const float backleg_jump_angle  =  81.0f;//理论上前腿后移主要由该角度不合适引起
        const float frontleg_squat_angle = 60.0f;
        const float delta_angle = 10;

        /***********************************************************/
        /*下蹲准备起跳，持续时间为prep_time*/
        ChangeGainOfPID(8,0.1f,140,0.22f);
        AllLegsSpeedLimit(SpeedMode_SLOW);
        //后腿下蹲
        x = -stance_height*cos(backleg_jump_angle*PI/180);
        y =  stance_height*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        //前腿“倾斜”下蹲
        x = -stance_height*cos(frontleg_squat_angle*PI/180);
        y =  stance_height*sin(frontleg_squat_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(prep_time);//跳跃前进行短暂延时

        /***********************************************************/
        /*后腿起跳，前腿立刻变化一段（这种情况下，若前腿变化合适，理论上转动的支点不再是足端，而应当看成是髋关节），持续时间为launch_time*/
        //速度拉满
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
        //使用高刚度和低阻尼执行跳转
        LegPID_Set(1,8,0.1f,5.5f,255,0.1f);
        LegPID_Set(3,8,0.1f,5.5f,255,0.1f);
        /*********后腿起跳**********/
        x = -jump_extension*cos(backleg_jump_angle*PI/180);
        y =  jump_extension*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        /*********前腿立刻再向前一点**********/
        x =  -stance_height*cos((frontleg_squat_angle+delta_angle)*PI/180);
        y =   stance_height*sin((frontleg_squat_angle+delta_angle)*PI/180);
        CartesianToTheta();
        LegPID_Set(0,7,0.1f,1.5f,222,0.1f);
        LegPID_Set(2,7,0.1f,1.5f,222,0.1f);
        LegSpeedLimit(0,3000);
        LegSpeedLimit(2,3000);
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);

        /***********************************************************/
        //前腿逐渐进入水平态（核心，与IMU反馈的pitch角配合执行跳跃），后腿迅速卧倒
        while(imu_wait_lock)
        {
            timedelay++;
            if(timedelay == backleg_jump_time/5) //非阻塞延时，约5ms一次。
            {
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******后腿慢慢倒立态（必要，否则会卡住）******/
                LegSpeedLimit(1,SpeedMode_SLOW + 150);
                LegSpeedLimit(3,SpeedMode_SLOW + 150);
                //后腿反向转
                TargetAngle1=-180;
                TargetAngle2=360;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            /***********前腿慢慢水平************/
            LegPID_Set(0,7.5f,0.1f,1.1f,200,0.1f);
            LegPID_Set(2,7.5f,0.1f,1.1f,200,0.1f);
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
            //一直到角度合适，然后切换到下一个步态。
            if(SeconTime==0 && IMU_EulerAngle.EulerAngle[Pitch]> (takeoff_inclination-imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< (takeoff_inclination+imu_angle_half_range) )
            {
                SeconTime=2;
            }
            else if(SeconTime==2 && IMU_EulerAngle.EulerAngle[Pitch]> (88-imu_fullscale_correction) && IMU_EulerAngle.EulerAngle[Pitch]< (90-imu_fullscale_correction) )//当pitch角度出现系统误差时，可能角度范围达不到这里，因此无法前翻，必须先在imu.c接收中断中修正系统误差。
            {
                SeconTime=1;
            }
            else if(SeconTime==1 && IMU_EulerAngle.EulerAngle[Pitch]> (takeoff_inclination-imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< (takeoff_inclination+imu_angle_half_range) )
            {
                imu_wait_lock=0;
                //避免过快翻转导致后腿没来得及卧倒，故这里强制再进行一次卧倒。
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******后腿慢慢倒立态（必要，否则会卡住）******/
                LegSpeedLimit(1,SpeedMode_SLOW + 150);
                LegSpeedLimit(3,SpeedMode_SLOW + 150);
                //后腿反向转
                TargetAngle1=-180;
                TargetAngle2=360;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            osDelay(5);//每次延时5ms，保证其它任务正常执行的同时，也不会有因为延时过高而错过IMU角度，同时也方便进行计数计时。
        }
        //最后阶段（前、后腿均变为四脚朝天态，但转动的方向相反防止机械卡位）
        x=0;
        y=-LegStandLenth;
        //使用低刚度和大量的阻尼来处理下降
        ChangeAllGainOfPID(8.0f,0.1f,0.25f,40,0.1f);
        //后腿反方向（向后）转动
        reverse_move_flag=1;//方向反向控制
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        //前腿正常（向前）转动
        reverse_move_flag=0;//恢复（其实不写也没事儿）
        x=0;
        y=-LegSquatLenth;
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        //延时一会儿，等前腿落差不多
        osDelay(200);
        //前腿也站立起来
        x=0;
        y=-LegStandLenth;
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        gpstate = STOP;//回到停止态
        ChangeGainOfPID(8,0.1f,70,0.22f);//恢复正常PD
    }
        //模式4：双足跪直立
    else if(mode == 4)
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const float prep_time = 0.4f;        //准备时间，即收缩退准备起跳的时间   [s]  0.4
        const float launch_time = 0.12f;     //伸展腿的持续时间                  [s]  0.2
        const float fall_time = 0.25f;        //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
        const float strech_time = 0.17f;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        const float shrink_time = 0.3f;
        /*跳跃的姿态把控*/
        const float stance_height = LegLenthMin+1.0f;//跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax;//最大是LegLenthExtremeMax
        /*心跳控制*/
        static uint32_t firt_execute=0;//每个执行周期的启动标志
        if(firt_execute == 0 )
        {
            /*强行控制心跳为最初*/
            times=0;
        }
        firt_execute=1;
        float tt=times*5/1000;
        /*跳跃过程*/
        //下蹲准备起跳，持续时间为prep_time
        if (tt < prep_time)
        {
            ChangeGainOfPID(8,0.1,140,0.22);
            AllLegsSpeedLimit(SpeedMode_FAST);
            //后腿垂直下蹲
            x = -stance_height*cos(88*PI/180);
            y =  stance_height*sin(88*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            //前腿“倾斜”下蹲
            x =  -stance_height*cos(75*PI/180);
            y =  stance_height*sin(75*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //后腿起跳，前腿迅速回到垂直态，持续时间为launch_time（核心，要与前腿进入水平态的时间配合好）
        else if (tt >= prep_time && tt < (prep_time + launch_time))
        {
            //速度拉满
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //使用高刚度和低阻尼执行跳转
            ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD不要动了，主要是调节I值
            /*********后腿起跳**********/
            x = -jump_extension*cos(66*PI/180);
            y =  jump_extension*sin(66*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /*********前腿立刻再向前一点**********/
            x = -stance_height*cos(88*PI/180);
            y =  stance_height*sin(88*PI/180);
            CartesianToTheta();
            LegPID_Set(0,7,0.1f,1.5f,222,0.1f);
            LegPID_Set(2,7,0.1f,1.5f,222,0.1f);
            LegSpeedLimit(0,SpeedMode_EXTREME);
            LegSpeedLimit(2,SpeedMode_EXTREME);
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //前腿逐渐进入水平态（核心，其延时要与跳跃点配合好），后腿卧倒
        else if (tt >= (prep_time + launch_time) && tt < (prep_time + launch_time + fall_time))
        {
            ChangeAllGainOfPID(8.0f,0.1f,1.5f,200,0.2f);
            /******后腿迅速卧倒态（必要，否则会卡住）******/
            LegSpeedLimit(1,SpeedMode_EXTREME);
            LegSpeedLimit(3,SpeedMode_EXTREME);
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /***********前腿慢慢水平************/
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //前腿跳跃
        else if (tt >= (prep_time + launch_time + fall_time) && tt < (prep_time + launch_time + fall_time + strech_time))
        {
//				//速度拉满
//				AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
//				//使用高刚度和低阻尼执行跳转
//				ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD不要动了，主要是调节I值。
//				y=-0.15f;
//				x=jump_extension;//跳跃伸腿
//				CartesianToTheta();
//				SetCoupledThetaPosition(0);
//				SetCoupledThetaPosition(2);
        }
            //后腿快速卧倒
        else if (tt >= (prep_time + launch_time + fall_time + strech_time) && tt < (prep_time + launch_time + fall_time + strech_time + shrink_time))
        {
            /******后腿继续快速卧倒态（必要，否则会卡住）******/
            ChangeAllGainOfPID(7,0.1f,1.1f,150,0.1f);
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
        }
            //落地前一瞬间
        else
        {
            //倒立站起
            x=0;
            y=-LegStandLenth;
            CartesianToTheta();
            //使用低刚度和大量的阻尼来处理下降
            ChangeAllGainOfPID(8.0f,0.1f,0.6f,100,0.1f);
            SetCoupledThetaPositionAll();
            //跳出函数
            firt_execute=0;//回归函数的初始态
            gpstate = STOP;//回到停止态
            ChangeGainOfPID(8,0.1f,70,0.22f);//恢复正常PD
        }
    }
        //模式5：站起再倒立（慢速向前翻滚）
    else if(mode == 5)
    {
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const float prep_time = 0.4f;        //准备时间，即收缩退准备起跳的时间   [s]  0.4
        const float launch_time = 0.12f;     //伸展腿的持续时间                  [s]  0.2
        const float fall_time = 0.25f;        //在空中飞翔的时间                 [s]  0.25（这个时间最好设置的小点）
        const float strech_time = 0.17f;  //落地并用力支撑的时间              [s]  0.3（这个时间结束后就会立刻进入站立态了）
        const float shrink_time = 0.3f;
        /*跳跃的姿态把控*/
        const float stance_height = LegLenthMin+1.0f;//跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax;//最大是LegLenthExtremeMax
        /*心跳控制*/
        static uint32_t firt_execute=0;//每个执行周期的启动标志
        if(firt_execute == 0 )
        {
            /*强行控制心跳为最初*/
            times=0;
        }
        firt_execute=1;
        float tt=times*5/1000;
        /*跳跃过程*/
        //下蹲准备起跳，持续时间为prep_time
        if (tt < prep_time)
        {
            ChangeGainOfPID(8,0.1f,140,0.22f);
            AllLegsSpeedLimit(SpeedMode_FAST);
            //后腿垂直下蹲
            x = -stance_height*cos(88*PI/180);
            y =  stance_height*sin(88*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            //前腿“倾斜”下蹲
            x =  -stance_height*cos(75*PI/180);
            y =  stance_height*sin(75*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //后腿起跳，前腿迅速回到垂直态，持续时间为launch_time（核心，要与前腿进入水平态的时间配合好）
        else if (tt >= prep_time && tt < (prep_time + launch_time))
        {
            //速度拉满
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //使用高刚度和低阻尼执行跳转
            ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD不要动了，主要是调节I值
            /*********后腿起跳**********/
            x = -jump_extension*cos(75*PI/180);
            y =  jump_extension*sin(75*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /*********前腿立刻再向前一点**********/
            x = -stance_height*cos(88*PI/180);
            y =  stance_height*sin(88*PI/180);
            CartesianToTheta();
            LegPID_Set(0,7,0.1f,1.5f,222,0.1f);
            LegPID_Set(2,7,0.1f,1.5f,222,0.1f);
            LegSpeedLimit(0,SpeedMode_EXTREME);
            LegSpeedLimit(2,SpeedMode_EXTREME);
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //前腿逐渐进入水平态（核心，其延时要与跳跃点配合好），后腿卧倒
        else if (tt >= (prep_time + launch_time) && tt < (prep_time + launch_time + fall_time))
        {
            ChangeAllGainOfPID(8.0f,0.1f,1.5f,200,0.2f);
            /******后腿迅速卧倒态（必要，否则会卡住）******/
            LegSpeedLimit(1,SpeedMode_EXTREME);
            LegSpeedLimit(3,SpeedMode_EXTREME);
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /***********前腿慢慢水平************/
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //前腿跳跃
        else if (tt >= (prep_time + launch_time + fall_time) && tt < (prep_time + launch_time + fall_time + strech_time))
        {
            //什么都不做
        }
            //后腿快速卧倒
        else if (tt >= (prep_time + launch_time + fall_time + strech_time) && tt < (prep_time + launch_time + fall_time + strech_time + shrink_time))
        {
            /******后腿继续快速卧倒态（必要，否则会卡住）******/
            ChangeAllGainOfPID(7,0.1f,1.1f,150,0.1f);
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
        }
            //落地前一瞬间
        else
        {
            //倒立站起
            x=0;
            y=-LegStandLenth;
            CartesianToTheta();
            //使用低刚度和大量的阻尼来处理下降
            ChangeAllGainOfPID(8.0f,0.1f,0.6f,100,0.1f);
            SetCoupledThetaPositionAll();
            //跳出函数
            firt_execute=0;//回归函数的初始态
            gpstate = STOP;//回到停止态
            ChangeGainOfPID(8,0.1f,70,0.22f);//恢复正常PD
        }
    }
        //模式7：稳定半90度自恢复
    else if(mode == 7)
    {
        uint8_t SeconTime=0;//判断是否是第二次到达
        uint32_t timedelay=0;
        //IMU到达特定角度范围跳跃的控制变量
        uint8_t imu_wait_lock = 1;
        float takeoff_inclination = 80;
        float imu_angle_half_range = 1;
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 500;        //准备时间，即收缩退准备起跳的时间 [s]
        const uint16_t strech_time = 210;  //前腿跳的持续时间                 [s]
        /*跳跃的姿态把控*/
        const float stance_height = LegLenthMin+1.0f;//跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax+0.4f;//最大是LegLenthExtremeMax
        const float backleg_jump_angle  =  80;//理论上前腿后移主要由该角度不合适引起
        const float frontleg_squat_angle = 70;
        const float delta_angle = 10;

        /***********************************************************/
        /*下蹲准备起跳，持续时间为prep_time*/
        ChangeGainOfPID(8,0.1f,140,0.22f);
        AllLegsSpeedLimit(SpeedMode_FAST);
        //后腿下蹲
        x = -stance_height*cos(backleg_jump_angle*PI/180);
        y =  stance_height*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        //前腿“倾斜”下蹲
        x = -stance_height*cos(frontleg_squat_angle*PI/180);
        y =  stance_height*sin(frontleg_squat_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(prep_time);//跳跃前进行短暂延时

        /***********************************************************/
        /*后腿起跳，前腿立刻变化一段（这种情况下，若前腿变化合适，理论上转动的支点不再是足端，而应当看成是髋关节），持续时间为launch_time*/
        //速度拉满
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
        //使用高刚度和低阻尼执行跳转
        LegPID_Set(1,8,0.1f,5.5f,255,0.1f);
        LegPID_Set(3,8,0.1f,5.5f,255,0.1f);
        /*********后腿起跳**********/
        x = -jump_extension*cos(backleg_jump_angle*PI/180);
        y =  jump_extension*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        /*********前腿立刻再向前一点**********/
        x =  -stance_height*cos((frontleg_squat_angle+delta_angle)*PI/180);
        y =   stance_height*sin((frontleg_squat_angle+delta_angle)*PI/180);
        CartesianToTheta();
        LegPID_Set(0,7,0.1f,1.5f,222,0.1f);
        LegPID_Set(2,7,0.1f,1.5f,222,0.1f);
        LegSpeedLimit(0,3000);
        LegSpeedLimit(2,3000);
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);

        /***********************************************************/
        //前腿逐渐进入水平态（核心，与IMU反馈的pitch角配合执行跳跃（时间打断并记录，退出可以延续前后时间）），后腿迅速卧倒
        while(imu_wait_lock)//通过函数进行判断，当返回为0时，该else if不满足，故必然会执行下一个elseif，故在其中上锁即可。
        {
            timedelay++;
            if(timedelay == 40) //非阻塞延时，约5ms一次。
            {
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******后腿迅速卧倒态（必要，否则会卡住）******/
                LegSpeedLimit(1,SpeedMode_EXTREME);
                LegSpeedLimit(3,SpeedMode_EXTREME);
                TargetAngle1 = 0;TargetAngle2 = 0;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            /***********前腿慢慢水平************/
            LegPID_Set(0,7.5f,0.1f,1.1f,200,0.1f);
            LegPID_Set(2,7.5f,0.1f,1.1f,200,0.1f);
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
            //一直到角度合适，然后切换到下一个步态。
            if(SeconTime==0 && IMU_EulerAngle.EulerAngle[Pitch]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< -(takeoff_inclination-imu_angle_half_range) )
            {
                SeconTime=2;
            }
            else if(SeconTime==2 && IMU_EulerAngle.EulerAngle[Pitch]> -90 && IMU_EulerAngle.EulerAngle[Pitch]< -88 )//当pitch角度出现系统误差时，可能角度范围达不到这里，因此无法前翻，必须先在imu.c接收中断中修正系统误差。
            {
                SeconTime=1;
            }
            else if(SeconTime==1 && IMU_EulerAngle.EulerAngle[Pitch]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< -(takeoff_inclination-imu_angle_half_range) )
            {
                imu_wait_lock=0;
                //避免过快翻转导致后退不复位，这里强制再复一下位。
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******后腿迅速卧倒态（必要，否则会卡住）******/
                LegSpeedLimit(1,SpeedMode_EXTREME);
                LegSpeedLimit(3,SpeedMode_EXTREME);
                TargetAngle1 = 0;TargetAngle2 = 0;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            //IWDG_Feed();//喂狗
            osDelay(5);
        }

        /***********************************************************/
        //前腿跳跃
        //速度拉满
        LegSpeedLimit(0,SpeedMode_JUMPPEDAL);
        LegSpeedLimit(2,SpeedMode_JUMPPEDAL);
        //使用高刚度和低阻尼执行跳转
        ChangeAllGainOfPID(8,0.1f,5.5f,255,0.1f);//PD不要动了，主要是调节I值。
        y=-0.15f;
        x=jump_extension;//跳跃伸腿
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(strech_time);
        /***********************************************************/

        //落地前一瞬间
        //倒立站起
        x=0;
        y=-LegStandLenth;
        CartesianToTheta();
        //使用低刚度和大量的阻尼来处理下降
        ChangeAllGainOfPID(8.0f,0.1f,0.6f,100,0.1f);
        SetCoupledThetaPositionAll();
        //跳出函数
        gpstate = STOP;//回到停止态
        ChangeGainOfPID(8,0.1f,70,0.22f);//恢复正常PD
    }
    //前空翻跳备份用
    if(mode==200)
    {
        uint8_t SeconTime=0;//判断是否是第二次到达
        uint32_t timedelay=0;
        //IMU到达特定角度范围跳跃的控制变量
        uint8_t imu_wait_lock = 1;
        float takeoff_inclination = 80;//核心参数
        float imu_angle_half_range = 1;//可以满足使用
        /*跳跃过程的时间把控（以实测为主设置何时的时间，保证运动过程分段的合理性）*/
        const uint16_t prep_time = 500;            //准备时间，即收缩退准备起跳的时间 [s]
        const uint16_t backleg_jump_time = 200;    //前腿跳的持续时间                [s]
        const uint16_t frontleg_jump_time = 170;   //前腿跳的持续时间                [s]

        /*跳跃的姿态把控*/
        const float stance_height = LegLenthMin + 1.0f;//跳跃之前腿的高度  [cm]，理论上应等于LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax + 0.2f;//最大是LegLenthExtremeMax
        const float backleg_jump_angle  =  81.0f;//理论上前腿后移主要由该角度不合适引起
        const float frontleg_squat_angle = 60.0f;
        const float delta_angle = 10;

        /***********************************************************/
        /*下蹲准备起跳，持续时间为prep_time*/
        ChangeGainOfPID(8,0.1,140,0.22);
        AllLegsSpeedLimit(SpeedMode_FAST);
        //后腿下蹲
        x = -stance_height*cos(backleg_jump_angle*PI/180);
        y =  stance_height*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        //前腿“倾斜”下蹲
        x = -stance_height*cos(frontleg_squat_angle*PI/180);
        y =  stance_height*sin(frontleg_squat_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(prep_time);//跳跃前进行短暂延时

        /***********************************************************/
        /*后腿起跳，前腿立刻变化一段（这种情况下，若前腿变化合适，理论上转动的支点不再是足端，而应当看成是髋关节），持续时间为launch_time*/
        //速度拉满
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
        //使用高刚度和低阻尼执行跳转
        LegPID_Set(1,8,0.1f,5.5f,255,0.1f);
        LegPID_Set(3,8,0.1f,5.5f,255,0.1f);
        /*********后腿起跳**********/
        x = -jump_extension*cos(backleg_jump_angle*PI/180);
        y =  jump_extension*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        /*********前腿立刻再向前一点**********/
        x =  -stance_height*cos((frontleg_squat_angle+delta_angle)*PI/180);
        y =   stance_height*sin((frontleg_squat_angle+delta_angle)*PI/180);
        CartesianToTheta();
        LegPID_Set(0,7,0.1f,1.5f,222,0.1f);
        LegPID_Set(2,7,0.1f,1.5f,222,0.1f);
        LegSpeedLimit(0,3000);
        LegSpeedLimit(2,3000);
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);

        /***********************************************************/
        //前腿逐渐进入水平态（核心，与IMU反馈的pitch角配合执行跳跃），后腿迅速卧倒
        while(imu_wait_lock)
        {
            timedelay++;
            if(timedelay == backleg_jump_time/5) //非阻塞延时，约5ms一次。
            {
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******后腿迅速卧倒态（必要，否则会卡住）******/
                LegSpeedLimit(1,SpeedMode_EXTREME);
                LegSpeedLimit(3,SpeedMode_EXTREME);
                TargetAngle1 = 0;TargetAngle2 = 0;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            /***********前腿慢慢水平************/
            LegPID_Set(0,7.5f,0.1f,1.1f,200,0.1f);
            LegPID_Set(2,7.5f,0.1f,1.1f,200,0.1f);
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
            //一直到角度合适，然后切换到下一个步态。
            if(SeconTime==0 && IMU_EulerAngle.EulerAngle[Pitch]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< -(takeoff_inclination-imu_angle_half_range) )
            {
                SeconTime=2;
            }
            else if(SeconTime==2 && IMU_EulerAngle.EulerAngle[Pitch]> -90 && IMU_EulerAngle.EulerAngle[Pitch]< -88 )//当pitch角度出现系统误差时，可能角度范围达不到这里，因此无法前翻，必须先在imu.c接收中断中修正系统误差。
            {
                SeconTime=1;
            }
            else if(SeconTime==1 && IMU_EulerAngle.EulerAngle[Pitch]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< -(takeoff_inclination-imu_angle_half_range) )
            {
                imu_wait_lock=0;
                //避免过快翻转导致后退不复位，这里强制再复一下位。
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******后腿迅速卧倒态（必要，否则会卡住）******/
                LegSpeedLimit(1,SpeedMode_EXTREME);
                LegSpeedLimit(3,SpeedMode_EXTREME);
                TargetAngle1 = 0;TargetAngle2 = 0;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            osDelay(5);//每次延时5ms，保证其它任务正常执行的同时，也不会有因为延时过高而错过IMU角度，同时也方便进行计数计时。
        }

        /***********************************************************/
        //前腿跳跃
        LegSpeedLimit(0,SpeedMode_JUMPPEDAL);//速度拉满
        LegSpeedLimit(2,SpeedMode_JUMPPEDAL);//速度拉满
        //使用高刚度和低阻尼执行跳转
        ChangeAllGainOfPID(8,0.1f,5.5f,255,0.1f);
        y=-0.15f;
        x=jump_extension;
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(frontleg_jump_time);

        /***********************************************************/
        //前腿收平
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        //使用刚度小，阻尼大的增益
        ChangeAllGainOfPID(8.0f,0.1f,0.25f,100,0.1f);
        TargetAngle2=-180-(-32.9);
        TargetAngle1=0+(154.3);
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(100);
        /***********************************************************/

        //最后阶段
        x=0;
        y=-LegSquatLenth;
        CartesianToTheta();
        //使用低刚度和大量的阻尼来处理下降
        ChangeAllGainOfPID(8.0f,0.1f,0.25f,100,0.1f);
        SetCoupledThetaPositionAll();
        gpstate = STOP;//回到停止态
        ChangeGainOfPID(8,0.1f,70,0.22f);//恢复正常PD
    }
}
