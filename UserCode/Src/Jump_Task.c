//
// Created by Joerange on 2024/1/18.
//
#include "main.h"
#include "Jump_Task.h"

extern float times;
extern uint8_t reverse_move_flag;

void ExecuteJump(uint8_t JumpType,float JumpAngle)
{
    if (JumpType == Standard_Jump)//��׼�����Ծ���������涼�нϺõ���Ӧ�ԣ���߸߶Ⱥ�Զ�ȣ�
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 1000;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time = 300;    //��չ�ȵĳ���ʱ��                  [s]  0.2
        const uint16_t fall_time = 200;      //�ڿ��з����ʱ��                 [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time = 650;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_flylegheight = LegStandLenth; //����ʱ�ȳ���   [cm]��������Ӧ����LegLenthMax 28��LegStandLenth 18.0f֮�䣬��һ�����е�ֵ��
        const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(6.0f, 0.5f, 0, 0);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(JumpAngle + 13, stance_height, prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(SpeedMode_VERYEX);//�ٶ�����
        ChangeGainOfPID(35.0f,0.23f,0, 0);//ʹ�øն�С������������0
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
        ChangeGainOfPID(8.0f, 0.3f, 0, 0);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(-25, jump_flylegheight, fall_time);
        /*
        �͸նȣ�
            ����������ʽ��pos_kp*sp_kp = CurrentMAX/angle_Thresh���͸ն���ζ������Ҫѡ��һ����΢��һ���angle_Thresh���Ӷ����ǿ���С�Ƕ��ڱȽ����װڶ����������һ���Ƕ����������ˡ�
            ��������ѡ��30�ȣ����У�
            pos_kp = CurrentMAX/angle_Thresh = 8900/30 �� 300���ʣ�Ϊ�˿�������pos_kpΪ300��
            �͸նȱ���Ϊ����һ���Ƕȷ�Χ�ڱȽ�����ת�������Ƕ�Խ��Խ�������ﵽ�ٽ���ֵ�ǶȺ��Ѽ���ת�������ҳ���ʱ��Խ��Խ��������Ϊ��Ĭ�ϵ�I����
        */
        //������׼��վ����
        ChangeGainOfPID(8.0f, 0.18f, 0, 0);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(-80, jump_landlegheight, strech_time);
        //���վ���ˣ�ִ��
        gpstate = 1;
    }
    else if(JumpType == High_Jump)//��ԭ�������ߣ��κε��涼�У�
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 300;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time = 200;    //��չ�ȵĳ���ʱ��                  [s]  0.2
        const uint16_t fall_time = 100;      //�ڿ��з����ʱ��                 [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time = 150;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_flylegheight = LegStandLenth + 4; //����ʱ�ȳ���   [cm]��������Ӧ����LegLenthMax 28��LegStandLenth 18.0f֮�䣬��һ�����е�ֵ��
        const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(8.0f,0.0f,0,0);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(JumpAngle,stance_height,prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(30.0f);//�ٶ�����
        ChangeGainOfPID(30.0f,0,0,0);// ʹ�ø߸նȺ͵�����ִ����ת
        SetPolarPositionAll_Delay(JumpAngle,jump_extension,launch_time);
        //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
        AllLegsSpeedLimit(SpeedMode_VERYFAST);
        ChangeGainOfPID(5.0f,5.0f,0,0);//ʹ�õ͸նȺʹ����������������½�
        SetPolarPositionAll_Delay(-55,jump_flylegheight,fall_time);
        //������׼��վ����
        ChangeGainOfPID(6,0,0.0f,0.0f);//ʹ�õ͸նȺʹ����������������½�
        SetPolarPositionAll_Delay(-88,jump_landlegheight,strech_time);
        //���վ���ˣ�ִ�����
        gpstate = HALT;
    }
    else if(JumpType == Far_Jump)//������Զ��Ҫ�����Ħ���ϴ�
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 300;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time = 200;    //��չ�ȵĳ���ʱ��                  [s]  0.2
        const uint16_t fall_time = 150;      //�ڿ��з����ʱ��                 [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time = 250;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_flylegheight = LegStandLenth-2; //����ʱ�ȳ���   [cm]��������Ӧ����LegLenthMax 28��LegStandLenth 18.0f֮�䣬��һ�����е�ֵ��
        const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,0.0f,0.0f);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(JumpAngle,stance_height,prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(30);//�ٶ�����
        ChangeGainOfPID(30.0f,0.1f,0.0f,0.0f);
        SetPolarPositionAll_Delay(JumpAngle,jump_extension,launch_time);
        //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(9  ,2.1f,0.0f,0.0f);//ʹ�õ͸նȺʹ����������������½�
        SetPolarPositionAll_Delay(-55,jump_flylegheight,fall_time);
        //������׼��վ����
        ChangeGainOfPID(9,2.1f,0.0f,0.0f);//ʹ�õ͸նȺʹ����������������½�
        SetPolarPositionAll_Delay(-80,jump_landlegheight,strech_time);
        //���վ���ˣ�ִ�����
        gpstate = HALT_IMU;
    }
    else if(JumpType == Leap_Jump)//�ǳ����޵�����ͬʱ����
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 400;  //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time=160;  //��չ�ȵĳ���ʱ��                 [s]  0.2
        const uint16_t fall_time = 400;  //�ڿ��з����ʱ��                 [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time=500;  //��ز�����֧�ŵ�ʱ��             [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;//��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthExtremeMax;//��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_landlegheight = LegStandLenth;//���ʱ�ȳ��� [cm]��������Ӧ����LegStandLenth 18.0f
        const float jump_angle = JumpAngle;
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(6.0f,0.2f,0.26f,25,0.22f);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//�ٶ�����
        ChangeAllGainOfPID(8,0.1f,3.0f,200,0.1f);// ʹ�ø߸նȺ͵�����ִ����ת
        SetPolarPositionFB_Delay(Leg_Front,jump_angle,jump_extension,0);
        SetPolarPositionFB_Delay(Leg_Back,jump_angle,jump_extension,launch_time);
        //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(5,0.1f,0.8f,30,2.0f);//Ҫ����зǳ��õ���Ӧ�ٶȺ͵����ٶ�
        SetPolarPositionFB_Delay(Leg_Front,10,LegSquatLenth,0);
        SetPolarPositionFB_Delay(Leg_Back,-65,LegLenthMin,fall_time/2);
        SetPolarPositionFB_Delay(Leg_Front,-70,LegStandLenth,fall_time/2);
        //������׼��վ����
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        FBLegsPID_Set(Leg_Front,6,0.1f,0.08f,35,1.1f);
        FBLegsPID_Set(Leg_Back,6,0.1f,0.1f,130,1.1f);
        SetPolarPositionFB_Delay(Leg_Front,-70,jump_landlegheight,0);
        SetCartesianPositionFB_Delay(Leg_Back,0.2f,LegSquatLenth,strech_time);
        //���վ���ˣ�ִ�����
        gpstate = HALT;
    }
}
//����ר�÷ֶ���Ծ������������������⣩
void StairJump(uint8_t stage)
{
    if(stage == 0)//��������¿���һ����������̨��
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 300;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time =200;    //��չ�ȵĳ���ʱ��                  [s]  0.2
        const uint16_t fall_time =250;      //�ڿ��з����ʱ��                  [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time =300;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_flylegheight = LegStandLenth+5; //����ʱ�ȳ���   [cm]��������Ӧ����LegLenthMax 28��LegStandLenth 18.0f֮�䣬��һ�����е�ֵ��
        const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        const float jump_angle = 68;// ��ǰ��Ծ�ĽǶ�[��]
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,70,0.22f);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//�ٶ�����
        ChangeAllGainOfPID(7,0.1f,2.5f,200,0.1f);//ʹ�ø߸նȺ͵�����ִ����ת
        SetPolarPositionAll_Delay(jump_angle,jump_extension,launch_time);
        //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8,0.1f,70.0f,0);//ʹ�õ͸նȺʹ����������������½�
        SetPolarPositionAll_Delay(-22,jump_flylegheight,fall_time);//ǰ��
        //������׼��վ����
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(5,0.1f,0.7f,25,2.0f);//վ��̬PID
        SetPolarPositionFB_Delay(Leg_Back,-80,LegSquatLenth,0);
        SetPolarPositionFB_Delay(Leg_Front,-70,jump_landlegheight,strech_time);
        //���վ���ˣ�ִ�����
        gpstate = HALT;
    }
    if(stage == 1)//��������¿��Ժ����䵽���ݶ���
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 300;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time =200;    //��չ�ȵĳ���ʱ��                  [s]  0.2
        const uint16_t fall_time =350;      //�ڿ��з����ʱ��                  [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time =300;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_flylegheight = LegStandLenth+7.2f; //����ʱ�ȳ���   [cm]��������Ӧ����LegLenthMax 28��LegStandLenth 18.0f֮�䣬��һ�����е�ֵ��
        const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        const float jump_angle = 55;// ��ǰ��Ծ�ĽǶ�[��]
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,70,0.22f);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//�ٶ�����
        ChangeAllGainOfPID(7,0.1f,2.5f,200,0.1f);//ʹ�ø߸նȺ͵�����ִ����ת
        SetPolarPositionAll_Delay(jump_angle,jump_extension,launch_time);
        //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8,0.1f,70.0f,0);//ʹ�õ͸նȺʹ����������������½�
        SetPolarPositionAll_Delay(-22,jump_flylegheight,fall_time);//ǰ��
        //������׼��վ����
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(5,0.1f,0.7f,25,2.0f);//վ��̬PID
        SetPolarPositionFB_Delay(Leg_Back,80,LegSquatLenth,0);
        SetPolarPositionFB_Delay(Leg_Front,-70,jump_landlegheight,strech_time);
        //���վ���ˣ�ִ�����
        gpstate = HALT;
    }
    else if(stage == 2)//����������Ժ����䵽���һ��̨�ס�
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 300;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time =200;    //��չ�ȵĳ���ʱ��                  [s]  0.2
        const uint16_t fall_time =450;      //�ڿ��з����ʱ��                  [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time =300;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_flylegheight = LegStandLenth; //����ʱ�ȳ���   [cm]��������Ӧ����LegLenthMax 28��LegStandLenth 18.0f֮�䣬��һ�����е�ֵ��
        const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        const float jump_angle = 85;// ��ǰ��Ծ�ĽǶ�[��] �ñ�����������ǰ��Ծ�ĽǶȣ��Ӷ�����ǰ���ľ��롣�ǶȻ�׼�ǵ��棬�����Ǵ�ֱ���棬���ԽСԽԶ��������С��40�ȡ�
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,70,0.22f);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//�ٶ�����
        ChangeAllGainOfPID(7,0.1f,0.2f,190,0.1f);//ʹ�ø߸նȺ͵�����ִ����ת
        SetPolarPositionAll_Delay(jump_angle,jump_extension,launch_time);
        //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8,0.1f,70.0f,0);//ʹ�õ͸նȺʹ����������������½�
        SetPolarPositionFB_Delay(Leg_Back,80,LegSquatLenth,0);
        SetPolarPositionFB_Delay(Leg_Front,-25,jump_flylegheight,fall_time);//ǰ��
        //������׼��վ����
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(6,0.1f,60.0f,0.3f);//ʹ�õ͸նȺʹ����������������½�
        SetPolarPositionFB_Delay(Leg_Back,80,LegSquatLenth,0);
        SetPolarPositionFB_Delay(Leg_Front,-70,jump_landlegheight,strech_time);
        //���վ���ˣ�ִ�����
        gpstate = HALT;
    }
    else if(stage == 3)
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 300;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time =200;    //��չ�ȵĳ���ʱ��                  [s]  0.2
        const uint16_t fall_time =500;      //�ڿ��з����ʱ��                  [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time =300;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_flylegheight = LegStandLenth+5; //����ʱ�ȳ���   [cm]��������Ӧ����LegLenthMax 28��LegStandLenth 18.0f֮�䣬��һ�����е�ֵ��
        const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        const float jump_angle = 60;// ��ǰ��Ծ�ĽǶ�[��] �ñ�����������ǰ��Ծ�ĽǶȣ��Ӷ�����ǰ���ľ��롣�ǶȻ�׼�ǵ��棬�����Ǵ�ֱ���棬���ԽСԽԶ��������С��40�ȡ�
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,70,0.22f);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//�ٶ�����
        ChangeAllGainOfPID(7.38f,0.1f,1.5f,222.5f,0.1f);// ʹ�ø߸նȺ͵�����ִ����ת
        SetPolarPositionAll_Delay(jump_angle,jump_extension,launch_time);
        //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8,0.1f,70.0f,0);//ʹ�õ͸նȺʹ����������������½�
        SetPolarPositionFB_Delay(Leg_Back,65,LegLenthMin,0);
        SetPolarPositionFB_Delay(Leg_Front,-25,jump_flylegheight,fall_time);//ǰ��
        //������׼��վ����
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(6,0.1f,35.0f,0.3f);//ʹ�õ͸նȺʹ����������������½�
        SetPolarPositionAll_Delay(-70,jump_landlegheight,strech_time);
        //���վ���ˣ�ִ�����
        gpstate = HALT;
    }
}
//���ΰ�ר�÷ֶ���Ծ������������������⣩
void SeesawJump(uint8_t stage)
{
    if(stage == 0)
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 400;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time=160;    //��չ�ȵĳ���ʱ��                  [s]  0.2
        const uint16_t fall_time = 200;      //�ڿ��з����ʱ��                 [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time=500;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthExtremeMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        const float jump_angle = 60;
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(6.0f,0.2f,0.26f,25,0.22f);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//�ٶ�����
        ChangeAllGainOfPID(8,0.1f,3.0f,200,0.1f);// ʹ�ø߸նȺ͵�����ִ����ת
        SetPolarPositionFB_Delay(Leg_Front,jump_angle,jump_extension,0);
        SetPolarPositionFB_Delay(Leg_Back,jump_angle,jump_extension,launch_time);
        //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(5,0.1f,0.8f,30,2.0f);//Ҫ����зǳ��õ���Ӧ�ٶȺ͵����ٶ�
        TargetAngle1=71.5f;
        TargetAngle2=180.0 - 59.1;
        SetCoupledThetaPositionAll();
        osDelay(fall_time);
        //������׼��վ����
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        FBLegsPID_Set(Leg_Front,6,0.1f,0.08f,35,1.1f);
        FBLegsPID_Set(Leg_Back,6,0.1f,0.1f,130,1.1f);
        SetPolarPositionAll_Delay(70,jump_landlegheight,strech_time);
        //���վ���ˣ�ִ�����
        gpstate = HALT;
    }
    else if(stage == 1)
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 400;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time=160;    //��չ�ȵĳ���ʱ��                  [s]  0.2
        const uint16_t fall_time = 200;      //�ڿ��з����ʱ��                 [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time=500;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthExtremeMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        const float jump_angle = 55;
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(6.0f,0.2f,0.26f,25,0.22f);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//�ٶ�����
        ChangeAllGainOfPID(8,0.1f,3.0f,200,0.1f);// ʹ�ø߸նȺ͵�����ִ����ת
        SetPolarPositionFB_Delay(Leg_Front,jump_angle,jump_extension,0);
        SetPolarPositionFB_Delay(Leg_Back,jump_angle,jump_extension,launch_time);
        //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(5,0.1f,0.8f,30,2.0f);//Ҫ����зǳ��õ���Ӧ�ٶȺ͵����ٶ�
        TargetAngle1=71.5f;
        TargetAngle2=180-59.1;
        SetCoupledThetaPositionAll();
        osDelay(fall_time);
        //������׼��վ����
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        FBLegsPID_Set(Leg_Front,6,0.1f,0.08f,35,1.1f);
        FBLegsPID_Set(Leg_Back,6,0.1f,0.1f,130,1.1f);
        SetPolarPositionAll_Delay(70,jump_landlegheight,strech_time);
        //���վ���ˣ�ִ�����
        gpstate = HALT;
    }
    else
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 300;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
        const uint16_t launch_time =200;    //��չ�ȵĳ���ʱ��                  [s]  0.2
        const uint16_t fall_time =250;      //�ڿ��з����ʱ��                  [s]  0.25�����ʱ��������õ�С�㣩
        const uint16_t strech_time =300;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
        const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
        const float jump_extension = LegLenthMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
        const float jump_flylegheight = LegStandLenth; //����ʱ�ȳ���   [cm]��������Ӧ����LegLenthMax 28��LegStandLenth 18.0f֮�䣬��һ�����е�ֵ��
        const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        const float jump_angle = 85;// ��ǰ��Ծ�ĽǶ�[��] �ñ�����������ǰ��Ծ�ĽǶȣ��Ӷ�����ǰ���ľ��롣�ǶȻ�׼�ǵ��棬�����Ǵ�ֱ���棬���ԽСԽԶ��������С��40�ȡ�
        //�¶ף�׼������������ʱ��Ϊprep_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8.0f,0.1f,70,0.22f);//ʹ�øն�С������������
        SetPolarPositionAll_Delay(jump_angle,stance_height,prep_time);
        //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//�ٶ�����
        ChangeAllGainOfPID(7.38f,0.1f,0.4f,222.5f,0.1f);// ʹ�ø߸նȺ͵�����ִ����ת
        SetPolarPositionAll_Delay(jump_angle,jump_extension,launch_time);
        //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(8,0.1f,70.0f,0);//ʹ�õ͸նȺʹ����������������½�
        SetPolarPositionAll_Delay(-20,jump_flylegheight,fall_time);//ǰ��
        //������׼��վ����
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeGainOfPID(6,0.1f,60.0f,0.3f);//ʹ�õ͸նȺʹ����������������½���PID�����ٶ�2ms��6,0.1,60��PID�������кܺõĻ�����������
        SetPolarPositionAll_Delay(-70,jump_landlegheight,strech_time);
        //���վ���ˣ�ִ�����
        gpstate = HALT;
    }
}
//˫ľ��ר����Ծ�������˫ľ�ţ�Ȼ����߱�΢��IMU���ɿ���ͨ����Ҳ���Գ����Ƚ�����Ծͨ����
void Bridge_Jump(uint8_t stage)
{
    switch(stage)
    {
        case 0:
        {
            /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
            const uint16_t prep_time = 300;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
            const uint16_t launch_time=200;    //��չ�ȵĳ���ʱ��                  [s]  0.2
            const uint16_t fall_time = 130;      //�ڿ��з����ʱ��                 [s]  0.25�����ʱ��������õ�С�㣩
            const uint16_t strech_time=250;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
            /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
            const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
            const float jump_extension = LegLenthMax-1.0f; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
            const float jump_flylegheight = LegStandLenth-2; //����ʱ�ȳ���   [cm]��������Ӧ����LegLenthMax 28��LegStandLenth 18.0f֮�䣬��һ�����е�ֵ��
            const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
            const float jump_angle = 75;
            //�¶ף�׼������������ʱ��Ϊprep_time
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            ChangeGainOfPID(8.0f,0.1f,0.0f,0);//ʹ�øն�С������������
            SetPolarPositionAll_Delay(jump_angle + IMU_EulerAngle.EulerAngle[Pitch],stance_height,prep_time);
            //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
            AllLegsSpeedLimit(30);//�ٶ�����
            ChangeGainOfPID(30.0f,0.1f,0.0f,0);// ʹ�ø߸նȺ͵�����ִ����ת
            SetPolarPositionFB_Delay(Leg_Front,jump_angle + IMU_EulerAngle.EulerAngle[Pitch],jump_extension,0);
            SetPolarPositionFB_Delay(Leg_Back,jump_angle + IMU_EulerAngle.EulerAngle[Pitch],jump_extension,launch_time);
            //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            ChangeGainOfPID(8,0.1f,0.0f,0);//ʹ�õ͸նȺʹ����������������½�
            SetPolarPositionAll_Delay(-45,jump_flylegheight,fall_time);
            //������׼��վ����
            AllLegsSpeedLimit(SpeedMode_FAST);
            ChangeGainOfPID(8,0.1f,0.0f,0);//ʹ�õ͸նȺʹ����������������½�
            FBLegsPID_Set(Leg_Front,6,0.1f,0.08f,35,1.1f);
            FBLegsPID_Set(Leg_Back,6,0.1f,0.1f,130,1.1f);
            SetPolarPositionAll_Delay(-62,jump_landlegheight,strech_time);
            //���վ���ˣ�ִ�����
            gpstate = HALT_IMU;
            break;
        }
        case 1:
        {
            /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
            const uint16_t prep_time = 300;       //׼��ʱ�䣬��������׼��������ʱ��  [s]  0.4
            const uint16_t launch_time=200;    //��չ�ȵĳ���ʱ��                  [s]  0.2
            const uint16_t fall_time = 300;      //�ڿ��з����ʱ��                 [s]  0.25�����ʱ��������õ�С�㣩
            const uint16_t strech_time=500;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
            /*��Ծ����̬�ѿأ�����ʱ���ɰ�0.1�����������мӼ��������磨LegSquatLenth-0.4����*/
            const float stance_height = LegLenthMin;  //��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f�����������Ծʱ����ʹ��LegLenthMin 10.7f
            const float jump_extension = LegLenthMax; //��չ��Ծ��������ȳ���      [cm]��������Ӧ����LegLenthMax 28
            const float jump_flylegheight = LegStandLenth-2; //����ʱ�ȳ���   [cm]��������Ӧ����LegLenthMax 28��LegStandLenth 18.0f֮�䣬��һ�����е�ֵ��
            const float jump_landlegheight = LegStandLenth; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
            const float jump_angle = 56;
            //�¶ף�׼������������ʱ��Ϊprep_time
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            ChangeAllGainOfPID(6.0f,0.2f,0.26f,25,0.22f);//ʹ�øն�С������������
            SetPolarPositionFB_Delay(Leg_Back,jump_angle,stance_height,0);
            SetPolarPositionFB_Delay(Leg_Front,jump_angle+6,stance_height,prep_time);
            //�ߺ���ɣ����ģ�������ʱ��Ϊlaunch_time
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//�ٶ�����
            ChangeAllGainOfPID(6,0.1f,2.0f,200,0.1f);// ʹ�ø߸նȺ͵�����ִ����ת
            SetPolarPositionFB_Delay(Leg_Front,jump_angle,jump_extension+1.0f,0);
            SetPolarPositionFB_Delay(Leg_Back,jump_angle,jump_extension,launch_time);
            //������̣�Ҳ��������̣��е���̬�����ģ�������ʱ��Ϊfall_time
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            ChangeGainOfPID(5,0.1f,35.0f,0);//ʹ�õ͸նȺʹ����������������½�
            SetPolarPositionAll_Delay(-25,jump_flylegheight,fall_time);
            //���������
            AllLegsSpeedLimit(SpeedMode_FAST);
            FBLegsPID_Set(Leg_Back,5,0.1f,0.20f,20,2.0f);
            FBLegsPID_Set(Leg_Back,5,0.1f,0.20f,20,2.0f);
            SetPolarPositionAll_Delay(-70,jump_landlegheight,strech_time);
            //վ����
            ChangeAllGainOfPID(5,0.1f,0.55f,25,2.0f);//վ��̬PID
            AllLegsSpeedLimit(SpeedMode_VERYSLOW);
            TargetAngle1=0;TargetAngle2=180;
            SetCoupledThetaPositionAll();
            osDelay(600);
            //���վ���ˣ�ִ�����
            gpstate = HALT;
            break;
        }

    }
}
/***
//ǰ�շ���Ծ�������������������ǰ�ȡ�
//����Ծ�в�ͬ��ģʽ����������ǰ�շ�������������˫��ֱ�����ȣ���ԾЧ�������ɹ������������������أ���
//ִ�и���Ծʱ�����ñ�����ʩ���������������ʧ��
***/
void FrontFlipJump(uint8_t mode)
{
    //ģʽ0��ǰ�շ������ɹ�����������
    if(mode==0)
    {
        uint8_t SeconTime=0;//�ж��Ƿ��ǵڶ��ε���
        uint32_t timedelay=0;//��������ʱ������Ծ��IMU��ϸ�����
        //IMU�����ض��Ƕȷ�Χ��Ծ�Ŀ��Ʊ���
        uint8_t imu_wait_lock = 1;//IMU����
        float imu_fullscale_correction = 3;//��Ϊ���������̣�90�ȣ���ʵ��������֮�
        float takeoff_inclination = 67-imu_fullscale_correction;//���Ĳ��� ��inclination������
        float imu_angle_half_range = 1.5f;//��������ʹ��
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 850;            //׼��ʱ�䣬��������׼��������ʱ�� [ms]
        const uint16_t backleg_jump_time  = 150;   //�������ĳ���ʱ��				   [ms]
        const uint16_t frontleg_jump_time = 180;   //ǰ�����ĳ���ʱ��                [ms]
        /*��Ծ����̬�ѿ�*/
        const float stance_height = LegLenthMin+1.0f;//��Ծ֮ǰ�ȵĸ߶�  [cm]
        const float jump_extension = LegLenthExtremeMax;//�����LegLenthExtremeMax
        const float backleg_jump_angle  =  81.0f;//������ǰ�Ⱥ�����Ҫ�ɸýǶȲ���������
        const float frontleg_squat_angle = 60.0f;
        const float delta_angle = 10;
        /***********************************************************/
        /*�¶�׼������������ʱ��Ϊprep_time*/
        ChangeAllGainOfPID(5,0.1f,0.7f,25,2.0f);//վ��̬PID//ChangeAllGainOfPID(8,0.1,0.26,200,0.22);
        AllLegsSpeedLimit(SpeedMode_SLOW);
        //�����¶�
        SetPolarPositionFB_Delay(Leg_Back,backleg_jump_angle,stance_height,0);
        //ǰ�ȡ���б���¶�
        SetPolarPositionFB_Delay(Leg_Front,frontleg_squat_angle,stance_height,prep_time);
        /***********************************************************/
        /*����������ǰ�����̱仯һ�Σ���������£���ǰ�ȱ仯���ʣ�������ת����֧�㲻������ˣ���Ӧ���������Źؽڣ�������ʱ��Ϊlaunch_time*/
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);//�ٶ�����
        LegPID_Set(1,8,0.1f,5.5f,255,0.1f);LegPID_Set(3,8,0.1f,5.5f,255,0.1f);//ʹ�ø߸նȺ͵�����ִ����ת
        /*********��������**********/
        SetPolarPositionFB_Delay(Leg_Back,backleg_jump_angle,jump_extension,0);
        /*********ǰ����������ǰһ��**********/
        LegPID_Set(0,5,0.1f,0.7f,25,2.0f);//ChangeAllGainOfPID(5,0.1,0.7,25,2.0);//վ��̬PID
        LegPID_Set(2,5,0.1f,0.7f,25,2.0f);
        LegSpeedLimit(0,4000);
        LegSpeedLimit(2,4000);
        SetPolarPositionFB_Delay(Leg_Front,frontleg_squat_angle+delta_angle,stance_height,0);
        /***********************************************************/
        //ǰ���𽥽���ˮƽ̬�����ģ���IMU������pitch�����ִ����Ծ��������Ѹ���Ե�
        while(imu_wait_lock)
        {
            /***********���ȵȴ�������ʱ��************/
            timedelay++;
            if(timedelay == backleg_jump_time/5) //��������ʱ��Լ5msһ�Ρ�
            {
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******������������̬����Ҫ������Ῠס��******/
                LegSpeedLimit(1,SpeedMode_SLOW + 150);
                LegSpeedLimit(3,SpeedMode_SLOW + 150);
                //���ȷ���ת
                TargetAngle1=-180;
                TargetAngle2=360;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            /***********ǰ������ˮƽ************/
            LegPID_Set(0,7.5f,0.1f,0.8f,180,0.1f);
            LegPID_Set(2,7.5f,0.1f,0.8f,180,0.1f);
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            SetCartesianPositionFB_Delay(Leg_Front,LegSquatLenth,-0.15f,0);
            //һֱ���ǶȺ��ʣ�Ȼ���л�����һ����̬��
            if(SeconTime==0 && IMU_EulerAngle.EulerAngle[Pitch]> (takeoff_inclination-imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< (takeoff_inclination+imu_angle_half_range) )
            {
                SeconTime=2;
            }
            else if(SeconTime==2 && IMU_EulerAngle.EulerAngle[Pitch]> (88-imu_fullscale_correction) && IMU_EulerAngle.EulerAngle[Pitch]< (90-imu_fullscale_correction) )//��pitch�Ƕȳ���ϵͳ���ʱ�����ܽǶȷ�Χ�ﲻ���������޷�ǰ������������imu.c�����ж�������ϵͳ��
            {
                SeconTime=1;
            }
            else if(SeconTime==1 && IMU_EulerAngle.EulerAngle[Pitch]> (takeoff_inclination-imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< (takeoff_inclination+imu_angle_half_range) )
            {
                imu_wait_lock=0;
                //������췭ת���º���û���ü��Ե���������ǿ���ٽ���һ���Ե���
                FBLegsPID_Set(Leg_Back,6,0.1f,0.04f,35,0.1f);
                FBLegsSpeedLimit(Leg_Back,SpeedMode_SLOW + 150);
                //���ȷ���ת
                TargetAngle1=-180;TargetAngle2=360;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            osDelay(5);//ÿ����ʱ5ms����֤������������ִ�е�ͬʱ��Ҳ��������Ϊ��ʱ���߶����IMU�Ƕȣ�ͬʱҲ������м�����ʱ��
        }
        /***********************************************************/
        //ǰ����Ծ
        LegSpeedLimit(0,SpeedMode_JUMPPEDAL);//�ٶ�����
        LegSpeedLimit(2,SpeedMode_JUMPPEDAL);//�ٶ�����
        ChangeAllGainOfPID(8,0.1f,5.5f,255,0.1f);//ʹ�ø߸նȺ͵�����ִ����ת
        SetCartesianPositionFB_Delay(Leg_Front,jump_extension,-0.15f,frontleg_jump_time);
        /***********************************************************/
        //ǰ����վ��
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        ChangeAllGainOfPID(8.0f,0.1f,0.25f,100,0.1f);
//		ChangeAllGainOfPID(5,0.1,0.7,25,2.0);//վ��̬PID
        TargetAngle1=0;TargetAngle2=180;
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(300);
        /***********************************************************/
        //���׶Σ�ǰ�����Ⱦ���Ϊ�Ľų���̬����ת���ķ����෴��ֹ��е��λ��
        ChangeAllGainOfPID(8.0f,0.1f,0.25f,100,0.1f);//ʹ�õ͸նȺʹ����������������½�
        //���ȷ��������ת��
        ReverseMoveOpen();
        SetCartesianPositionFB_Delay(Leg_Back,0,-LegStandLenth,0);
        ReverseMoveClose();
        //ǰ����������ǰ��ת��
        SetCartesianPositionFB_Delay(Leg_Front,0,-LegStandLenth,0);
        gpstate = STOP;//�ص�ֹ̬ͣ
    }
        //ģʽ1�����в�����
    else if(mode == 1)
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const float prep_time = 0.4f;       //׼��ʱ�䣬��������׼��������ʱ��   [s]  0.4
        const float launch_time = 0.12f;     //��չ�ȵĳ���ʱ��                  [s]  0.2
        const float fall_time = 0.3f;       //�ڿ��з����ʱ��                  [s]  0.25�����ʱ��������õ�С�㣩
        const float strech_time = 0.17f;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        const float shrink_time = 0.21f;
        /*��Ծ����̬�ѿ�*/
        const float stance_height = LegLenthMin+1.0f;//��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax;//�����LegLenthExtremeMax
        const float jump_landlegheight = LegLenthMin+3.0f; //���ʱ�ȳ���  [cm]��������Ӧ����LegStandLenth 18.0f
        const float jump_angle = 88;// ��ǰ��Ծ�ĽǶ�[��] �ñ�����������ǰ��Ծ�ĽǶȣ��Ӷ�����ǰ���ľ��롣�ǶȻ�׼�ǵ��棬�����Ǵ�ֱ���棬���ԽСԽԶ��������С��40�ȡ�
        /*��������*/
        static uint32_t firt_execute=0;//ÿ��ִ�����ڵ�������־
        if(firt_execute == 0 )
        {
            /*ǿ�п�������Ϊ���*/
            times=0;
        }
        firt_execute=1;
        float tt=times*5/1000;
        /*��Ծ����*/
        //�¶�׼������������ʱ��Ϊprep_time
        if (tt < prep_time)
        {
            ChangeGainOfPID(8,0.1f,140,0.22f);//�ָ�����PD
            AllLegsSpeedLimit(SpeedMode_FAST);
            x = -stance_height * cos(jump_angle * PI/180);//Ҫת��Ϊ�����ٴ������Ǻ�������
            y =  stance_height * sin(jump_angle * PI/180);
            CartesianToTheta();
            SetCoupledThetaPositionAll();
        }
            //��������������ʱ��Ϊlaunch_time
        else if (tt >= prep_time && tt < (prep_time + launch_time))
        {
            //�ٶ�����
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //ʹ�ø߸նȺ͵�����ִ����ת
            ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD��Ҫ���ˣ���Ҫ�ǵ���Iֵ��
            /*********��������**********/
            x = -jump_extension * cos(jump_angle * PI/180);
            y =  jump_extension * sin(jump_angle * PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
        }
            //ǰ���𽥽��뵹��̬������վ��
        else if (tt >= (prep_time + launch_time) && tt < (prep_time + launch_time + fall_time))
        {
            AllLegsSpeedLimit(SpeedMode_FAST);
            ChangeAllGainOfPID(8.0f,0.1f,2.5f,200,0.1f);
            /******���ȣ�����վ��̬******/
            TargetAngle2=-180;
            TargetAngle1=0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /*******ǰ���𽥵���*********/
            x=0;
            y=-LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //ǰ����Ծ
        else if (tt >= (prep_time + launch_time + fall_time) && tt < (prep_time + launch_time + fall_time + strech_time))
        {
            /*********ǰ����Ծ********/
            //�ٶ�����
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //ʹ�ø߸նȺ͵�����ִ����ת
            ChangeAllGainOfPID(8,0.1f,6.0f,222,0.1f);//PD��Ҫ���ˣ���Ҫ�ǵ���Iֵ��
            y=-0.15f;
            x=jump_extension+0.5f;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //վ��
        else if (tt >= (prep_time + launch_time + fall_time + strech_time) && tt < (prep_time + launch_time + fall_time + strech_time + shrink_time))
        {
            x = jump_landlegheight * cos(75 * PI/180);
            y = jump_landlegheight * sin(75 * PI/180);
            CartesianToTheta();
            //ʹ�õ͸նȺʹ����������������½�
            ChangeGainOfPID(6,0.1f,35.0f,0.3f);
            SetCoupledThetaPositionAll();
        }
            //���ǰһ˲��
        else
        {
            firt_execute=0;//�ع麯���ĳ�ʼ̬
            gpstate = HALT;//�ص�վ��̬
        }
    }
        //ģʽ2��˫��ֱ����
    else if(mode == 2)
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const float prep_time = 0.4f;        //׼��ʱ�䣬��������׼��������ʱ��   [s]  0.4
        const float launch_time = 0.12f;     //��չ�ȵĳ���ʱ��                  [s]  0.2
        const float fall_time = 0.25f;        //�ڿ��з����ʱ��                  [s]  0.25�����ʱ��������õ�С�㣩
        const float strech_time = 0.17f;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        const float shrink_time = 0.3f;
        /*��Ծ����̬�ѿ�*/
        const float stance_height = LegLenthMin+1.0f;//��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax;//�����LegLenthExtremeMax
        const float jump_angle = 88;// ��ǰ��Ծ�ĽǶ�[��] �ñ�����������ǰ��Ծ�ĽǶȣ��Ӷ�����ǰ���ľ��롣�ǶȻ�׼�ǵ��棬�����Ǵ�ֱ���棬���ԽСԽԶ��������С��40�ȡ�
        /*��������*/
        static uint32_t firt_execute=0;//ÿ��ִ�����ڵ�������־
        if(firt_execute == 0 )
        {
            /*ǿ�п�������Ϊ���*/
            times=0;
        }
        firt_execute=1;
        float tt=times*5/1000;
        /*��Ծ����*/
        //�¶�׼������������ʱ��Ϊprep_time
        if (tt < prep_time)
        {
            ChangeGainOfPID(8,0.1f,140,0.22f);//�ָ�����PD
            AllLegsSpeedLimit(SpeedMode_FAST);
            x = -stance_height * cos(jump_angle * PI/180);//Ҫת��Ϊ�����ٴ������Ǻ�������
            y =  stance_height * sin(jump_angle * PI/180);
            CartesianToTheta();
            SetCoupledThetaPositionAll();
        }
            //��������������ʱ��Ϊlaunch_time�����ģ�Ҫ��ǰ�Ƚ���ˮƽ̬��ʱ����Ϻã�
        else if (tt >= prep_time && tt < (prep_time + launch_time))
        {
            //�ٶ�����
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //ʹ�ø߸նȺ͵�����ִ����ת
            ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD��Ҫ���ˣ���Ҫ�ǵ���Iֵ��
            /*********��������**********/
            x = -jump_extension * cos(jump_angle * PI/180);
            y =  jump_extension * sin(jump_angle * PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
        }
            //ǰ�Ƚ���ˮƽ̬�����ģ�����ʱҪ����Ծ����Ϻã��������Ե�
        else if (tt >= (prep_time + launch_time) && tt < (prep_time + launch_time + fall_time))
        {
            AllLegsSpeedLimit(3500);
            ChangeAllGainOfPID(8.0f,0.1f,4.5f,210,0.2f);
            /******�����Ե�̬����Ҫ������Ῠס��******/
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /************ǰ��ˮƽ************/
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //ǰ����Ծ
        else if (tt >= (prep_time + launch_time + fall_time) && tt < (prep_time + launch_time + fall_time + strech_time))
        {
            //�ٶ�����
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //ʹ�ø߸նȺ͵�����ִ����ת
            ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD��Ҫ���ˣ���Ҫ�ǵ���Iֵ��
            y=-0.15f;
            x=jump_extension;//��Ծ����
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //���ȿ����Ե�
        else if (tt >= (prep_time + launch_time + fall_time + strech_time) && tt < (prep_time + launch_time + fall_time + strech_time + shrink_time))
        {
            /******���ȼ��������Ե�̬����Ҫ������Ῠס��******/
            ChangeAllGainOfPID(7,0.1f,1.1f,150,0.1f);
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
        }
            //���ǰһ˲��
        else
        {
            //����վ��
            x=0;
            y=-LegStandLenth;
            CartesianToTheta();
            //ʹ�õ͸նȺʹ����������������½�
            ChangeAllGainOfPID(8.0f,0.1f,0.6f,100,0.1f);
            SetCoupledThetaPositionAll();
            //��������
            firt_execute=0;//�ع麯���ĳ�ʼ̬
            gpstate = STOP;//�ص�ֹ̬ͣ
            ChangeGainOfPID(8,0.1f,70,0.22f);//�ָ�����PD
        }
    }
        //ģʽ3��������ǰ��
    else if(mode == 3)
    {
        uint8_t SeconTime=0;//�ж��Ƿ��ǵڶ��ε���
        uint32_t timedelay=0;//��������ʱ������Ծ��IMU��ϸ�����
        //IMU�����ض��Ƕȷ�Χ��Ծ�Ŀ��Ʊ���
        uint8_t imu_wait_lock = 1;//IMU����
        float imu_fullscale_correction = 5;//�����̱仯ʱ��������
        float takeoff_inclination = 80-imu_fullscale_correction;//���Ĳ���
        float imu_angle_half_range = 1;//��������ʹ��
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 500;            //׼��ʱ�䣬��������׼��������ʱ�� [s]
        const uint16_t backleg_jump_time  = 150;    //�������ĳ���ʱ��

        /*��Ծ����̬�ѿ�*/
        const float stance_height = LegLenthMin+1.0f;//��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax-1;//�����LegLenthExtremeMax
        const float backleg_jump_angle  =  81.0f;//������ǰ�Ⱥ�����Ҫ�ɸýǶȲ���������
        const float frontleg_squat_angle = 60.0f;
        const float delta_angle = 10;

        /***********************************************************/
        /*�¶�׼������������ʱ��Ϊprep_time*/
        ChangeGainOfPID(8,0.1f,140,0.22f);
        AllLegsSpeedLimit(SpeedMode_SLOW);
        //�����¶�
        x = -stance_height*cos(backleg_jump_angle*PI/180);
        y =  stance_height*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        //ǰ�ȡ���б���¶�
        x = -stance_height*cos(frontleg_squat_angle*PI/180);
        y =  stance_height*sin(frontleg_squat_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(prep_time);//��Ծǰ���ж�����ʱ

        /***********************************************************/
        /*����������ǰ�����̱仯һ�Σ���������£���ǰ�ȱ仯���ʣ�������ת����֧�㲻������ˣ���Ӧ���������Źؽڣ�������ʱ��Ϊlaunch_time*/
        //�ٶ�����
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
        //ʹ�ø߸նȺ͵�����ִ����ת
        LegPID_Set(1,8,0.1f,5.5f,255,0.1f);
        LegPID_Set(3,8,0.1f,5.5f,255,0.1f);
        /*********��������**********/
        x = -jump_extension*cos(backleg_jump_angle*PI/180);
        y =  jump_extension*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        /*********ǰ����������ǰһ��**********/
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
        //ǰ���𽥽���ˮƽ̬�����ģ���IMU������pitch�����ִ����Ծ��������Ѹ���Ե�
        while(imu_wait_lock)
        {
            timedelay++;
            if(timedelay == backleg_jump_time/5) //��������ʱ��Լ5msһ�Ρ�
            {
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******������������̬����Ҫ������Ῠס��******/
                LegSpeedLimit(1,SpeedMode_SLOW + 150);
                LegSpeedLimit(3,SpeedMode_SLOW + 150);
                //���ȷ���ת
                TargetAngle1=-180;
                TargetAngle2=360;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            /***********ǰ������ˮƽ************/
            LegPID_Set(0,7.5f,0.1f,1.1f,200,0.1f);
            LegPID_Set(2,7.5f,0.1f,1.1f,200,0.1f);
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
            //һֱ���ǶȺ��ʣ�Ȼ���л�����һ����̬��
            if(SeconTime==0 && IMU_EulerAngle.EulerAngle[Pitch]> (takeoff_inclination-imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< (takeoff_inclination+imu_angle_half_range) )
            {
                SeconTime=2;
            }
            else if(SeconTime==2 && IMU_EulerAngle.EulerAngle[Pitch]> (88-imu_fullscale_correction) && IMU_EulerAngle.EulerAngle[Pitch]< (90-imu_fullscale_correction) )//��pitch�Ƕȳ���ϵͳ���ʱ�����ܽǶȷ�Χ�ﲻ���������޷�ǰ������������imu.c�����ж�������ϵͳ��
            {
                SeconTime=1;
            }
            else if(SeconTime==1 && IMU_EulerAngle.EulerAngle[Pitch]> (takeoff_inclination-imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< (takeoff_inclination+imu_angle_half_range) )
            {
                imu_wait_lock=0;
                //������췭ת���º���û���ü��Ե���������ǿ���ٽ���һ���Ե���
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******������������̬����Ҫ������Ῠס��******/
                LegSpeedLimit(1,SpeedMode_SLOW + 150);
                LegSpeedLimit(3,SpeedMode_SLOW + 150);
                //���ȷ���ת
                TargetAngle1=-180;
                TargetAngle2=360;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            osDelay(5);//ÿ����ʱ5ms����֤������������ִ�е�ͬʱ��Ҳ��������Ϊ��ʱ���߶����IMU�Ƕȣ�ͬʱҲ������м�����ʱ��
        }
        //���׶Σ�ǰ�����Ⱦ���Ϊ�Ľų���̬����ת���ķ����෴��ֹ��е��λ��
        x=0;
        y=-LegStandLenth;
        //ʹ�õ͸նȺʹ����������������½�
        ChangeAllGainOfPID(8.0f,0.1f,0.25f,40,0.1f);
        //���ȷ��������ת��
        reverse_move_flag=1;//���������
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        //ǰ����������ǰ��ת��
        reverse_move_flag=0;//�ָ�����ʵ��дҲû�¶���
        x=0;
        y=-LegSquatLenth;
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        //��ʱһ�������ǰ������
        osDelay(200);
        //ǰ��Ҳվ������
        x=0;
        y=-LegStandLenth;
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        gpstate = STOP;//�ص�ֹ̬ͣ
        ChangeGainOfPID(8,0.1f,70,0.22f);//�ָ�����PD
    }
        //ģʽ4��˫���ֱ��
    else if(mode == 4)
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const float prep_time = 0.4f;        //׼��ʱ�䣬��������׼��������ʱ��   [s]  0.4
        const float launch_time = 0.12f;     //��չ�ȵĳ���ʱ��                  [s]  0.2
        const float fall_time = 0.25f;        //�ڿ��з����ʱ��                 [s]  0.25�����ʱ��������õ�С�㣩
        const float strech_time = 0.17f;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        const float shrink_time = 0.3f;
        /*��Ծ����̬�ѿ�*/
        const float stance_height = LegLenthMin+1.0f;//��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax;//�����LegLenthExtremeMax
        /*��������*/
        static uint32_t firt_execute=0;//ÿ��ִ�����ڵ�������־
        if(firt_execute == 0 )
        {
            /*ǿ�п�������Ϊ���*/
            times=0;
        }
        firt_execute=1;
        float tt=times*5/1000;
        /*��Ծ����*/
        //�¶�׼������������ʱ��Ϊprep_time
        if (tt < prep_time)
        {
            ChangeGainOfPID(8,0.1,140,0.22);
            AllLegsSpeedLimit(SpeedMode_FAST);
            //���ȴ�ֱ�¶�
            x = -stance_height*cos(88*PI/180);
            y =  stance_height*sin(88*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            //ǰ�ȡ���б���¶�
            x =  -stance_height*cos(75*PI/180);
            y =  stance_height*sin(75*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //����������ǰ��Ѹ�ٻص���ֱ̬������ʱ��Ϊlaunch_time�����ģ�Ҫ��ǰ�Ƚ���ˮƽ̬��ʱ����Ϻã�
        else if (tt >= prep_time && tt < (prep_time + launch_time))
        {
            //�ٶ�����
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //ʹ�ø߸նȺ͵�����ִ����ת
            ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD��Ҫ���ˣ���Ҫ�ǵ���Iֵ
            /*********��������**********/
            x = -jump_extension*cos(66*PI/180);
            y =  jump_extension*sin(66*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /*********ǰ����������ǰһ��**********/
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
            //ǰ���𽥽���ˮƽ̬�����ģ�����ʱҪ����Ծ����Ϻã��������Ե�
        else if (tt >= (prep_time + launch_time) && tt < (prep_time + launch_time + fall_time))
        {
            ChangeAllGainOfPID(8.0f,0.1f,1.5f,200,0.2f);
            /******����Ѹ���Ե�̬����Ҫ������Ῠס��******/
            LegSpeedLimit(1,SpeedMode_EXTREME);
            LegSpeedLimit(3,SpeedMode_EXTREME);
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /***********ǰ������ˮƽ************/
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //ǰ����Ծ
        else if (tt >= (prep_time + launch_time + fall_time) && tt < (prep_time + launch_time + fall_time + strech_time))
        {
//				//�ٶ�����
//				AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
//				//ʹ�ø߸նȺ͵�����ִ����ת
//				ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD��Ҫ���ˣ���Ҫ�ǵ���Iֵ��
//				y=-0.15f;
//				x=jump_extension;//��Ծ����
//				CartesianToTheta();
//				SetCoupledThetaPosition(0);
//				SetCoupledThetaPosition(2);
        }
            //���ȿ����Ե�
        else if (tt >= (prep_time + launch_time + fall_time + strech_time) && tt < (prep_time + launch_time + fall_time + strech_time + shrink_time))
        {
            /******���ȼ��������Ե�̬����Ҫ������Ῠס��******/
            ChangeAllGainOfPID(7,0.1f,1.1f,150,0.1f);
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
        }
            //���ǰһ˲��
        else
        {
            //����վ��
            x=0;
            y=-LegStandLenth;
            CartesianToTheta();
            //ʹ�õ͸նȺʹ����������������½�
            ChangeAllGainOfPID(8.0f,0.1f,0.6f,100,0.1f);
            SetCoupledThetaPositionAll();
            //��������
            firt_execute=0;//�ع麯���ĳ�ʼ̬
            gpstate = STOP;//�ص�ֹ̬ͣ
            ChangeGainOfPID(8,0.1f,70,0.22f);//�ָ�����PD
        }
    }
        //ģʽ5��վ���ٵ�����������ǰ������
    else if(mode == 5)
    {
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const float prep_time = 0.4f;        //׼��ʱ�䣬��������׼��������ʱ��   [s]  0.4
        const float launch_time = 0.12f;     //��չ�ȵĳ���ʱ��                  [s]  0.2
        const float fall_time = 0.25f;        //�ڿ��з����ʱ��                 [s]  0.25�����ʱ��������õ�С�㣩
        const float strech_time = 0.17f;  //��ز�����֧�ŵ�ʱ��              [s]  0.3�����ʱ�������ͻ����̽���վ��̬�ˣ�
        const float shrink_time = 0.3f;
        /*��Ծ����̬�ѿ�*/
        const float stance_height = LegLenthMin+1.0f;//��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax;//�����LegLenthExtremeMax
        /*��������*/
        static uint32_t firt_execute=0;//ÿ��ִ�����ڵ�������־
        if(firt_execute == 0 )
        {
            /*ǿ�п�������Ϊ���*/
            times=0;
        }
        firt_execute=1;
        float tt=times*5/1000;
        /*��Ծ����*/
        //�¶�׼������������ʱ��Ϊprep_time
        if (tt < prep_time)
        {
            ChangeGainOfPID(8,0.1f,140,0.22f);
            AllLegsSpeedLimit(SpeedMode_FAST);
            //���ȴ�ֱ�¶�
            x = -stance_height*cos(88*PI/180);
            y =  stance_height*sin(88*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            //ǰ�ȡ���б���¶�
            x =  -stance_height*cos(75*PI/180);
            y =  stance_height*sin(75*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //����������ǰ��Ѹ�ٻص���ֱ̬������ʱ��Ϊlaunch_time�����ģ�Ҫ��ǰ�Ƚ���ˮƽ̬��ʱ����Ϻã�
        else if (tt >= prep_time && tt < (prep_time + launch_time))
        {
            //�ٶ�����
            AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
            //ʹ�ø߸նȺ͵�����ִ����ת
            ChangeAllGainOfPID(8,0.1f,5.5f,222,0.1f);//PD��Ҫ���ˣ���Ҫ�ǵ���Iֵ
            /*********��������**********/
            x = -jump_extension*cos(75*PI/180);
            y =  jump_extension*sin(75*PI/180);
            CartesianToTheta();
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /*********ǰ����������ǰһ��**********/
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
            //ǰ���𽥽���ˮƽ̬�����ģ�����ʱҪ����Ծ����Ϻã��������Ե�
        else if (tt >= (prep_time + launch_time) && tt < (prep_time + launch_time + fall_time))
        {
            ChangeAllGainOfPID(8.0f,0.1f,1.5f,200,0.2f);
            /******����Ѹ���Ե�̬����Ҫ������Ῠס��******/
            LegSpeedLimit(1,SpeedMode_EXTREME);
            LegSpeedLimit(3,SpeedMode_EXTREME);
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
            /***********ǰ������ˮƽ************/
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
        }
            //ǰ����Ծ
        else if (tt >= (prep_time + launch_time + fall_time) && tt < (prep_time + launch_time + fall_time + strech_time))
        {
            //ʲô������
        }
            //���ȿ����Ե�
        else if (tt >= (prep_time + launch_time + fall_time + strech_time) && tt < (prep_time + launch_time + fall_time + strech_time + shrink_time))
        {
            /******���ȼ��������Ե�̬����Ҫ������Ῠס��******/
            ChangeAllGainOfPID(7,0.1f,1.1f,150,0.1f);
            AllLegsSpeedLimit(SpeedMode_EXTREME);
            TargetAngle1 = 0;TargetAngle2 = 0;
            SetCoupledThetaPosition(1);
            SetCoupledThetaPosition(3);
        }
            //���ǰһ˲��
        else
        {
            //����վ��
            x=0;
            y=-LegStandLenth;
            CartesianToTheta();
            //ʹ�õ͸նȺʹ����������������½�
            ChangeAllGainOfPID(8.0f,0.1f,0.6f,100,0.1f);
            SetCoupledThetaPositionAll();
            //��������
            firt_execute=0;//�ع麯���ĳ�ʼ̬
            gpstate = STOP;//�ص�ֹ̬ͣ
            ChangeGainOfPID(8,0.1f,70,0.22f);//�ָ�����PD
        }
    }
        //ģʽ7���ȶ���90���Իָ�
    else if(mode == 7)
    {
        uint8_t SeconTime=0;//�ж��Ƿ��ǵڶ��ε���
        uint32_t timedelay=0;
        //IMU�����ض��Ƕȷ�Χ��Ծ�Ŀ��Ʊ���
        uint8_t imu_wait_lock = 1;
        float takeoff_inclination = 80;
        float imu_angle_half_range = 1;
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 500;        //׼��ʱ�䣬��������׼��������ʱ�� [s]
        const uint16_t strech_time = 210;  //ǰ�����ĳ���ʱ��                 [s]
        /*��Ծ����̬�ѿ�*/
        const float stance_height = LegLenthMin+1.0f;//��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax+0.4f;//�����LegLenthExtremeMax
        const float backleg_jump_angle  =  80;//������ǰ�Ⱥ�����Ҫ�ɸýǶȲ���������
        const float frontleg_squat_angle = 70;
        const float delta_angle = 10;

        /***********************************************************/
        /*�¶�׼������������ʱ��Ϊprep_time*/
        ChangeGainOfPID(8,0.1f,140,0.22f);
        AllLegsSpeedLimit(SpeedMode_FAST);
        //�����¶�
        x = -stance_height*cos(backleg_jump_angle*PI/180);
        y =  stance_height*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        //ǰ�ȡ���б���¶�
        x = -stance_height*cos(frontleg_squat_angle*PI/180);
        y =  stance_height*sin(frontleg_squat_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(prep_time);//��Ծǰ���ж�����ʱ

        /***********************************************************/
        /*����������ǰ�����̱仯һ�Σ���������£���ǰ�ȱ仯���ʣ�������ת����֧�㲻������ˣ���Ӧ���������Źؽڣ�������ʱ��Ϊlaunch_time*/
        //�ٶ�����
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
        //ʹ�ø߸նȺ͵�����ִ����ת
        LegPID_Set(1,8,0.1f,5.5f,255,0.1f);
        LegPID_Set(3,8,0.1f,5.5f,255,0.1f);
        /*********��������**********/
        x = -jump_extension*cos(backleg_jump_angle*PI/180);
        y =  jump_extension*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        /*********ǰ����������ǰһ��**********/
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
        //ǰ���𽥽���ˮƽ̬�����ģ���IMU������pitch�����ִ����Ծ��ʱ���ϲ���¼���˳���������ǰ��ʱ�䣩��������Ѹ���Ե�
        while(imu_wait_lock)//ͨ�����������жϣ�������Ϊ0ʱ����else if�����㣬�ʱ�Ȼ��ִ����һ��elseif�����������������ɡ�
        {
            timedelay++;
            if(timedelay == 40) //��������ʱ��Լ5msһ�Ρ�
            {
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******����Ѹ���Ե�̬����Ҫ������Ῠס��******/
                LegSpeedLimit(1,SpeedMode_EXTREME);
                LegSpeedLimit(3,SpeedMode_EXTREME);
                TargetAngle1 = 0;TargetAngle2 = 0;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            /***********ǰ������ˮƽ************/
            LegPID_Set(0,7.5f,0.1f,1.1f,200,0.1f);
            LegPID_Set(2,7.5f,0.1f,1.1f,200,0.1f);
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
            //һֱ���ǶȺ��ʣ�Ȼ���л�����һ����̬��
            if(SeconTime==0 && IMU_EulerAngle.EulerAngle[Pitch]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< -(takeoff_inclination-imu_angle_half_range) )
            {
                SeconTime=2;
            }
            else if(SeconTime==2 && IMU_EulerAngle.EulerAngle[Pitch]> -90 && IMU_EulerAngle.EulerAngle[Pitch]< -88 )//��pitch�Ƕȳ���ϵͳ���ʱ�����ܽǶȷ�Χ�ﲻ���������޷�ǰ������������imu.c�����ж�������ϵͳ��
            {
                SeconTime=1;
            }
            else if(SeconTime==1 && IMU_EulerAngle.EulerAngle[Pitch]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< -(takeoff_inclination-imu_angle_half_range) )
            {
                imu_wait_lock=0;
                //������췭ת���º��˲���λ������ǿ���ٸ�һ��λ��
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******����Ѹ���Ե�̬����Ҫ������Ῠס��******/
                LegSpeedLimit(1,SpeedMode_EXTREME);
                LegSpeedLimit(3,SpeedMode_EXTREME);
                TargetAngle1 = 0;TargetAngle2 = 0;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            //IWDG_Feed();//ι��
            osDelay(5);
        }

        /***********************************************************/
        //ǰ����Ծ
        //�ٶ�����
        LegSpeedLimit(0,SpeedMode_JUMPPEDAL);
        LegSpeedLimit(2,SpeedMode_JUMPPEDAL);
        //ʹ�ø߸նȺ͵�����ִ����ת
        ChangeAllGainOfPID(8,0.1f,5.5f,255,0.1f);//PD��Ҫ���ˣ���Ҫ�ǵ���Iֵ��
        y=-0.15f;
        x=jump_extension;//��Ծ����
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(strech_time);
        /***********************************************************/

        //���ǰһ˲��
        //����վ��
        x=0;
        y=-LegStandLenth;
        CartesianToTheta();
        //ʹ�õ͸նȺʹ����������������½�
        ChangeAllGainOfPID(8.0f,0.1f,0.6f,100,0.1f);
        SetCoupledThetaPositionAll();
        //��������
        gpstate = STOP;//�ص�ֹ̬ͣ
        ChangeGainOfPID(8,0.1f,70,0.22f);//�ָ�����PD
    }
    //ǰ�շ���������
    if(mode==200)
    {
        uint8_t SeconTime=0;//�ж��Ƿ��ǵڶ��ε���
        uint32_t timedelay=0;
        //IMU�����ض��Ƕȷ�Χ��Ծ�Ŀ��Ʊ���
        uint8_t imu_wait_lock = 1;
        float takeoff_inclination = 80;//���Ĳ���
        float imu_angle_half_range = 1;//��������ʹ��
        /*��Ծ���̵�ʱ��ѿأ���ʵ��Ϊ�����ú�ʱ��ʱ�䣬��֤�˶����̷ֶεĺ����ԣ�*/
        const uint16_t prep_time = 500;            //׼��ʱ�䣬��������׼��������ʱ�� [s]
        const uint16_t backleg_jump_time = 200;    //ǰ�����ĳ���ʱ��                [s]
        const uint16_t frontleg_jump_time = 170;   //ǰ�����ĳ���ʱ��                [s]

        /*��Ծ����̬�ѿ�*/
        const float stance_height = LegLenthMin + 1.0f;//��Ծ֮ǰ�ȵĸ߶�  [cm]��������Ӧ����LegSquatLenth 11.2f
        const float jump_extension = LegLenthExtremeMax + 0.2f;//�����LegLenthExtremeMax
        const float backleg_jump_angle  =  81.0f;//������ǰ�Ⱥ�����Ҫ�ɸýǶȲ���������
        const float frontleg_squat_angle = 60.0f;
        const float delta_angle = 10;

        /***********************************************************/
        /*�¶�׼������������ʱ��Ϊprep_time*/
        ChangeGainOfPID(8,0.1,140,0.22);
        AllLegsSpeedLimit(SpeedMode_FAST);
        //�����¶�
        x = -stance_height*cos(backleg_jump_angle*PI/180);
        y =  stance_height*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        //ǰ�ȡ���б���¶�
        x = -stance_height*cos(frontleg_squat_angle*PI/180);
        y =  stance_height*sin(frontleg_squat_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(prep_time);//��Ծǰ���ж�����ʱ

        /***********************************************************/
        /*����������ǰ�����̱仯һ�Σ���������£���ǰ�ȱ仯���ʣ�������ת����֧�㲻������ˣ���Ӧ���������Źؽڣ�������ʱ��Ϊlaunch_time*/
        //�ٶ�����
        AllLegsSpeedLimit(SpeedMode_JUMPPEDAL);
        //ʹ�ø߸նȺ͵�����ִ����ת
        LegPID_Set(1,8,0.1f,5.5f,255,0.1f);
        LegPID_Set(3,8,0.1f,5.5f,255,0.1f);
        /*********��������**********/
        x = -jump_extension*cos(backleg_jump_angle*PI/180);
        y =  jump_extension*sin(backleg_jump_angle*PI/180);
        CartesianToTheta();
        SetCoupledThetaPosition(1);
        SetCoupledThetaPosition(3);
        /*********ǰ����������ǰһ��**********/
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
        //ǰ���𽥽���ˮƽ̬�����ģ���IMU������pitch�����ִ����Ծ��������Ѹ���Ե�
        while(imu_wait_lock)
        {
            timedelay++;
            if(timedelay == backleg_jump_time/5) //��������ʱ��Լ5msһ�Ρ�
            {
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******����Ѹ���Ե�̬����Ҫ������Ῠס��******/
                LegSpeedLimit(1,SpeedMode_EXTREME);
                LegSpeedLimit(3,SpeedMode_EXTREME);
                TargetAngle1 = 0;TargetAngle2 = 0;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            /***********ǰ������ˮƽ************/
            LegPID_Set(0,7.5f,0.1f,1.1f,200,0.1f);
            LegPID_Set(2,7.5f,0.1f,1.1f,200,0.1f);
            LegSpeedLimit(0,1000);
            LegSpeedLimit(2,1000);
            y=-0.15f;
            x=LegSquatLenth;
            CartesianToTheta();
            SetCoupledThetaPosition(0);
            SetCoupledThetaPosition(2);
            //һֱ���ǶȺ��ʣ�Ȼ���л�����һ����̬��
            if(SeconTime==0 && IMU_EulerAngle.EulerAngle[Pitch]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< -(takeoff_inclination-imu_angle_half_range) )
            {
                SeconTime=2;
            }
            else if(SeconTime==2 && IMU_EulerAngle.EulerAngle[Pitch]> -90 && IMU_EulerAngle.EulerAngle[Pitch]< -88 )//��pitch�Ƕȳ���ϵͳ���ʱ�����ܽǶȷ�Χ�ﲻ���������޷�ǰ������������imu.c�����ж�������ϵͳ��
            {
                SeconTime=1;
            }
            else if(SeconTime==1 && IMU_EulerAngle.EulerAngle[Pitch]> -(takeoff_inclination+imu_angle_half_range) && IMU_EulerAngle.EulerAngle[Pitch]< -(takeoff_inclination-imu_angle_half_range) )
            {
                imu_wait_lock=0;
                //������췭ת���º��˲���λ������ǿ���ٸ�һ��λ��
                LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
                LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
                /******����Ѹ���Ե�̬����Ҫ������Ῠס��******/
                LegSpeedLimit(1,SpeedMode_EXTREME);
                LegSpeedLimit(3,SpeedMode_EXTREME);
                TargetAngle1 = 0;TargetAngle2 = 0;
                SetCoupledThetaPosition(1);
                SetCoupledThetaPosition(3);
            }
            osDelay(5);//ÿ����ʱ5ms����֤������������ִ�е�ͬʱ��Ҳ��������Ϊ��ʱ���߶����IMU�Ƕȣ�ͬʱҲ������м�����ʱ��
        }

        /***********************************************************/
        //ǰ����Ծ
        LegSpeedLimit(0,SpeedMode_JUMPPEDAL);//�ٶ�����
        LegSpeedLimit(2,SpeedMode_JUMPPEDAL);//�ٶ�����
        //ʹ�ø߸նȺ͵�����ִ����ת
        ChangeAllGainOfPID(8,0.1f,5.5f,255,0.1f);
        y=-0.15f;
        x=jump_extension;
        CartesianToTheta();
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(frontleg_jump_time);

        /***********************************************************/
        //ǰ����ƽ
        AllLegsSpeedLimit(SpeedMode_EXTREME);
        //ʹ�øն�С������������
        ChangeAllGainOfPID(8.0f,0.1f,0.25f,100,0.1f);
        TargetAngle2=-180-(-32.9);
        TargetAngle1=0+(154.3);
        SetCoupledThetaPosition(0);
        SetCoupledThetaPosition(2);
        osDelay(100);
        /***********************************************************/

        //���׶�
        x=0;
        y=-LegSquatLenth;
        CartesianToTheta();
        //ʹ�õ͸նȺʹ����������������½�
        ChangeAllGainOfPID(8.0f,0.1f,0.25f,100,0.1f);
        SetCoupledThetaPositionAll();
        gpstate = STOP;//�ص�ֹ̬ͣ
        ChangeGainOfPID(8,0.1f,70,0.22f);//�ָ�����PD
    }
}
