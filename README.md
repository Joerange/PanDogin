# PanDogin
# 开发文档：
#### 目前开发成员：王成、乔蕴之
##### 2023.12.9
    目前根据飞哥的代码重写了部分代码，目前宇树电机因为缺少usb转485暂时无法测试，优化了代码架构，
    并且添加了一些注释增加了代码的可读性，并且更改了部分pid.c的函数使其更加契合于并联狗的结构，
    但是代码因为是从老狗移植，其中不少部分依旧冗余，且因为老狗的频繁使用，imu并没有进行测试，
    目前进度是先将竞速赛完赛，优先完善步态，根据新的结构优化正弦曲线特征量来适配新狗，下一次提交的改进点：
    1.完成对宇树电机的闭环控制。
    2.优化代码的封装与结构，更改结构体的定义与使用，使四个腿可以调整单独的正弦曲线特征量。
    3.完成蓝牙通信的测试（NRF由石磊去完成）
    4.编写初步的姿态封装函数
##### 2023.12.10
    完成单独腿轨迹参数的更改，但是在RM困难重重没法完成电机的测试。
##### 2024.1.22
    很久没有写文档，假期从开始到现在一共工作了八天，在机械哥将狗调出来以后进行了调试，现在已经完成了所有的调试，
    包括陀螺仪，直行以及转弯步态，但是原地踏步还是有点小问题，即使给了0步长也会向前走，观察了步态生成曲线应当没有问题，
    在解决踏步以及差速转弯后继续完成跳跃的调试。
##### 2024.1.22 20：07
    目前提交了完成跳跃测试的提交，现在目前问题在于踏步的不稳定，当然完成了简单的跳跃，但是没有拿出实际的障碍进行测试，
    但是目前假期的目标应当已经完成了大半，剩下的大概五天时间在完善步态的同时，搬出往年的障碍进行初步调试，当然调试的不会很多，
    毕竟障碍材料以及机械狗腿用的玻纤板弯曲问题之后才会解决。调试出的数据没法直接使用。
##### 2024.1.23
    写完了初步的视觉传输任务队列的代码，但是没有进行测试，朱老师没来得等到下学期才可以测试。
# 问题文档
##### 2024.1.22 20：07
###### 在这里写一下这七八天遇到比较头疼的bug，
###### 1.在测试陀螺仪等串口收发的工具时，G4的板子会出现串口在自检的时候会进入打开的空闲中断进行自检，当时的程序大概如下方所示：

    if((tmp_flag != RESET) && BlueTeeth_flag == 1)//idle标志被置位
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);//清除标志位
        HAL_UART_DMAStop(&huart2); //

        temp = huart2.Instance->ISR;  //清除状态寄存器SR,读取SR寄存器可以实现清除SR寄存器的功能
        temp = huart2.Instance->RDR; //读取数据寄存器中的数据
        temp = hdma_usart2_rx.Instance->CNDTR;// 获取DMA中未传输的数据个数，NDTR寄存器分析见下面
        rx_len =  REMOTE_REC_LEN - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
        RemoteCtrl(rx_len);	//中断中的数据处理去改变相应控制变量的值
    }
    HAL_UART_Receive_DMA(&huart2,(uint8_t *)&REMOTE_RX_BUF,REMOTE_REC_LEN);//使能串口5 DMA接受

    可见清除标志位以及关闭DMA传输我都放到了DMA接收检测被触发的判断中而没有放在外面，外面只有一个开启中断，
    这样导致G4在自检的时候无法完成就会将程序卡死一直进行串口的自检程序（Debug真的很重要，感谢飞哥）

###### 解决方法如下：
    
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);//清除标志位
        HAL_UART_DMAStop(&huart2); //
    if((tmp_flag != RESET) && BlueTeeth_flag == 1)//idle标志被置位
    {
        temp = huart2.Instance->ISR;  //清除状态寄存器SR,读取SR寄存器可以实现清除SR寄存器的功能
        temp = huart2.Instance->RDR; //读取数据寄存器中的数据
        temp = hdma_usart2_rx.Instance->CNDTR;// 获取DMA中未传输的数据个数，NDTR寄存器分析见下面
        rx_len =  REMOTE_REC_LEN - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
        RemoteCtrl(rx_len);	//中断中的数据处理去改变相应控制变量的值
    }
    HAL_UART_Receive_DMA(&huart2,(uint8_t *)&REMOTE_RX_BUF,REMOTE_REC_LEN);//使能串口5 DMA接受
###### 这个问题主赛道电控同样遇到，G4串口与F4串口接收有比较大的不同，大家都可以作参考。
###### 2.在进行步态调试的过程中，会出现向前与向后只往一个方向行走的情况。
    目前来说我们也没有特别好的解决方法，就是调整步态，降低狗身，适当减低调整正幅值与负幅值，可以说没有
    完全解决，因为我们的Walk步态目前也有这样的小问题，不知道更换足端材料会不会好一点。
###### 3.在狗进行踱步的时候，一个目标值角度会相反，但是程序没有问题
    机械哥在修狗后自己问题好了，我还得删掉改了的程序，而且步态也比之前好很多。
##### 对于调试狗的时候，有时候机械稳定性会省很多事，而且机械一定程度上比电控重要
###### 4.在进行宇树电机PID位置环计算的时候，遇到了计算的误差出现非数（即nan）的问题
    初步波哥给出的建议是运算过慢，可能是没有打开fpu导致（高精度浮点运算单元），但是验证打开后也没有解决，
    最后我加入了对nan数的检测来规避nan的出现，当nan出现的时候，我们会选取之前的数值，按理来说这样的逻辑会导致电机
    一直转，目前出现电机一直转的问题偶尔也会出现，算一个不算解决的解决方法吧：
###### 解决方法如下：
###### 2024.1.22
     float Now_Point,Now_Error,d_Error;
    Now_Point = ((float )feedbackpos*2*pi)/(6.33f*32768);
    Now_Error=pid->Setpoint-Now_Point;

    if(isnan(Now_Error) == 1) Now_Error = pid->Last_error;

    pid->SumError+=Now_Error;//这部分进行了累加，从而导致积分饱和现象。

    if(isnan(pid->SumError) == 1) pid->SumError -= Now_Error;

    //积分限幅（而增量式PID不需要积分限幅）积分限幅有两种思路，一种是限制sum（相对限制），
    另一种是限制I*sum（绝对限制），后者我认为更加合理，但考虑到新老代码的兼容性，仍然采用前者。
    if(pid->SumError     >  pid->SumError_limit) pid->SumError= pid->SumError_limit;
    else if(pid->SumError< -pid->SumError_limit) pid->SumError=-pid->SumError_limit;

    d_Error = Now_Error - pid->Last_error;//误差之差，表示微分。
    pid->Last_error=Now_Error;

    pid->Out_put = pid->P * Now_Error +
                   pid->I * pid->SumError +
                   pid->D * d_Error;

    if(isnan(pid->Out_put) == 1) pid->Out_put = pid->Last_Out_put;

    //限幅输出
    if(pid->Out_put     > pid->Output_limit) pid->Out_put= pid->Output_limit;
    else if(pid->Out_put<-pid->Output_limit) pid->Out_put=-pid->Output_limit;

    pid->Last_Out_put = pid->Out_put;
###### 2024.1.23
    乔蕴之解决了bug，原因是Attitude_slove.h中对于最大腿长的定义过大，导致解算的时候超出了定义的限制，将
    其调小即可，目前最大腿长可达到120。
    
