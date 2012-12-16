#include "LightSensor.h"
#include "GyroSensor.h"
#include "SonarSensor.h"
#include "TouchSensor.h"
#include "BlueTooth.h"
#include "Motor.h"
#include "Tail.h"
#include "Input.h"
#include "UI.h"
#include "PID.h"
#include "balanceControl.h"
#include "Control.h"
#include "SlopeDetection.h"
#include "ColorDetection.h"
#include "ObstacleDetection.h"
#include "Detection.h"
#include "Section.h"

extern "C"{
	#include "kernel.h"
	#include "kernel_id.h"
	#include "ecrobot_interface.h"
};

void ecrobot_device_initialize(){
	// センサ、モータなどの各デバイスの初期化関数を
	// ここで実装することができます
	// ⇒　光センサ赤色LEDをONにする
	ecrobot_set_light_sensor_active(NXT_PORT_S1);
	// ⇒　超音波センサ(I2C通信)を初期化
	ecrobot_init_sonar_sensor(NXT_PORT_S3);
	
	if(ecrobot_get_bt_status() == BT_NO_INIT){
		ecrobot_set_bt_device_name("ET337");
	}
	
	ecrobot_init_bt_slave("unagipai");
}
	
void ecrobot_device_terminate(){
	// センサ、モータなどの各デバイスの終了関数を
	// ここで実装することができます。
	// ⇒　光センサ赤色LEDをOFFにする
	ecrobot_set_light_sensor_inactive(NXT_PORT_S1);
	// ⇒　超音波センサ(I2C通信)を終了
	ecrobot_term_sonar_sensor(NXT_PORT_S3 );
	
	ecrobot_term_bt_connection();
}
	
	// 1msec周期割り込み(ISRカテゴリ2)から起動されるユーザー用フック関数
void user_1ms_isr_type2(void){
	// 今回は何も行わない
}

extern "C" TASK(TaskMain){
	LightSensor *lightSensor = new LightSensor(NXT_PORT_S1);
	GyroSensor *gyroSensor = new GyroSensor(NXT_PORT_S2);
	SonarSensor *sonarSensor = new SonarSensor(NXT_PORT_S3);
	TouchSensor *touchSensor = new TouchSensor(NXT_PORT_S4);
	Motor *rightMotor = new Motor(NXT_PORT_A);
	Tail *tail = new Tail(NXT_PORT_B);
	Motor *leftMotor = new Motor(NXT_PORT_C);
	BlueTooth *blueTooth = new BlueTooth();
	UI *ui = new UI(*touchSensor, *blueTooth, *lightSensor, *tail);
	Input *input = new Input(*gyroSensor, *lightSensor, *sonarSensor);
	PID *pid = new PID();
	BalanceControl *balanceControl = new BalanceControl();
	Control *control = new Control(*pid, *balanceControl, *leftMotor, *rightMotor, *tail);
	SlopeDetection *slopeDetection = new SlopeDetection();
	ColorDetection *colorDetection = new ColorDetection();
	ObstacleDetection *obstacleDetection = new ObstacleDetection();
	
	int course = ui->courseSelect();
		
	if(course == 1){
		// 区間遷移条件の設定
		//                                        gyro_threshold,    g, light_threshold,    l,    s, count
		Detection *basicDetection = new Detection(             0,   -1,               0,   -1,   -1,   500, *slopeDetection, *colorDetection, *obstacleDetection);
		// 区間パラメータ設定
		//                           forward,forward_var, turn,tail,tail_var,gyro_offset,pid_flag, pid_target,   KP,balance_flag,balance_init
		Section *basic = new Section(    100,          0,    0,  90,       0,          0,    TRUE,        600, -0.7,       FALSE,       FALSE, *input, *control, *basicDetection);
		
		ui->waitStart(100);
		
		basic->basicRun();
		
	}
	else{
		while(1){
			
		}
	}
}
