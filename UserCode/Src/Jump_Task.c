//
// Created by Joerange on 2024/1/18.
//
#include "main.h"
#include "Jump_Task.h"

uint8_t jump_flag = 0;

void ExecuteJump(uint8_t JumpType,float JumpAngle)
{
    if (JumpType == Standard_Jump)//��׼�����Ծ���������涼�нϺõ���Ӧ�ԣ���߸߶Ⱥ�Զ�ȣ�
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 800;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time = 200;    //��չ�ȵĳ���ʱ��                  [s]  0.2
        const uint16_t fall_time = 200;      //�ڿ��з����ʱ��                 [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time = 1000;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_flylegheight = LegStandLenth; //����ʱ�ȳ���   [cm]��������Ӧ����LegLenthMax 28��LegStandLenth 18.0f֮�䣬��һ�����е�ֵ��
        const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(6.0f, 0, 0, 0);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(JumpAngle, stance_height, prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);//�ٶ�����
        ChangeGainOfPID(20.0f, 0, 0, 0);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(JumpAngle, jump_extension, launch_time);
        /*
        �߸նȵ�ʵ�֣�
        pos_kp�ܴ�
                �����������һ���ܴ��ֵ���ٶȻ�����ֵ���ԽǶȲΪ������ٶ�Ŀ��ֵ��3508�ٶ�����8900���ǶȲ��С10�ȼ��㣬��pos_kp��������Ϊ890��
        sp_kp�ܴ�
                �����ڵ�ǰ�ٶ���Ŀ���ٶȲ���Ǻܴ��ʱ��Ҳ������ܴ�ĵ��������ο��ٶȲ��ϴ��ʱ�򡣴Ӷ�����ʹ������ٶȱ��ֽϴ�
                C620�����������ֵ16384����Ӧ20A������ֵ��������������С��ͨ������Ҫ���ٶ�ֵ��
                ���ǵĿ��ƹ�������Ҫ���ٶȴ�8900���ٱ�Ϊ0���ٶ�ֵΪ250ʱ�Ѿ������ˣ�����Ϊ��׼������sp_kpΪ8�Ѿ������ˣ�����ٶȻ���PID�ǱȽ����ʵģ�һ��8�����¡�
        ��ˣ��ܵĿ���Ч����������ʽ������Speed=Current=angle*pos_kp����ˣ�������ϣ�����ٽ�Ŀ��Ƕ�10��ǰ�����ʼ�ձ�������ٶȣ���ô�����ǿ�������ʽ���㣺
        pos_kp = CurrentMAX/angle_Thresh = 8900/10 = 890��10���Ѿ��ǱȽ�С�ˣ�angle_ThreshԽС��Խ���׳������Ӷ����Σ������˲�����С��10�ȡ�
        �߸նȱ���Ϊ��ת����С�ĽǶȶ��ܷ�����������ת�������
        */
        //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(8.0f, 0, 0, 0);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(-25, jump_flylegheight, fall_time);
        /*
        �͸նȣ�
            ����������ʽ��pos_kp*sp_kp = CurrentMAX/angle_Thresh���͸ն���ζ������Ҫѡ��һ����΢��һ���angle_Thresh���Ӷ����ǿ���С�Ƕ��ڱȽ����װڶ����������һ���Ƕ����������ˡ�
            ��������ѡ��30�ȣ����У�
            pos_kp = CurrentMAX/angle_Thresh = 8900/30 �� 300���ʣ�Ϊ�˿�������pos_kpΪ300��
            �͸նȱ���Ϊ����һ���Ƕȷ�Χ�ڱȽ�����ת�������Ƕ�Խ��Խ�������ﵽ�ٽ���ֵ�ǶȺ��Ѽ���ת�������ҳ���ʱ��Խ��Խ��������Ϊ��Ĭ�ϵ�I����
        */
        //������׼��վ����
        ChangeGainOfPID(8.0f, 0, 0, 0);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(-70, jump_landlegheight, strech_time);
        //���վ���ˣ�ִ�����
        StandUp_Posture();
        gpstate = 1;
    }
}