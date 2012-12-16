#include "Control.h"

Control::Control(){

}

Control::Control(PID _pid, BalanceControl _balanceControl, Motor _leftMotor, Motor _rightMotor, Tail _tail){
	pid = _pid;
	balanceControl = _balanceControl;
	leftMotor = _leftMotor;
	rightMotor = _rightMotor;
	tail = _tail;
	safety = 0;
}

// 各制御と出力を行う
void Control::bonusRun(float forward, float turn, float tail_angle, 
	float gyro_offset, BOOL pid_flag, U16 pid_target, F32 KP, BOOL balance_flag, float gyro, U16 light){
		
		static float Turn;
	
		// しっぽを回転させる
		tail.control(tail_angle);
		
		// pid_flagがTRUEのとき、PID制御で旋回速度を計算する
		if(pid_flag){
			Turn = pid.calculation(light, pid_target, KP);
		}
		else{
			Turn = turn;
		}
		
		// balance_flagがTRUEのとき、倒立振子制御でモータ出力値を計算する
		if(balance_flag){
			balanceControl.calculation((char)forward, (char)Turn, gyro, gyro_offset, leftMotor.getAngle(), rightMotor.getAngle());
			// モータを回転させる
			leftMotor.rotate(balanceControl.getLeft());
			rightMotor.rotate(balanceControl.getRight());
		}
		else{
			// モータを回転させる
			leftMotor.rotate(forward + Turn);
			rightMotor.rotate(forward - Turn);
		}

		if(light > 670){
			if(++safety >= 1250)
				while(1){
					leftMotor.rotate(0);
					rightMotor.rotate(0);
				}
		}
		else{
			safety = 0;
		}
		
		/* ロギング */
		ecrobot_bt_data_logger(0, 0);
}

void Control::basicRun(float forward, float turn, float tail_angle, 
	float gyro_offset, BOOL pid_flag, U16 pid_target, F32 KP, BOOL balance_flag, float gyro, U16 light){

		static float Forward, Turn;

		// しっぽを回転させる
		tail.control(tail_angle);
		
		// pid_flagがTRUEのとき、PID制御で旋回速度を計算する
		if(pid_flag){
			Turn = pid.calculation(light, pid_target, KP);
		}
		else{
			Turn = turn;
		}
		
		if(Turn <= -70 || Turn >= 70){
			Forward = forward - 30;
		}
		else{
			Forward = forward;
		}
		
		// balance_flagがTRUEのとき、倒立振子制御でモータ出力値を計算する
		if(balance_flag){
			balanceControl.calculation((char)Forward, (char)Turn, gyro, gyro_offset, leftMotor.getAngle(), rightMotor.getAngle());
			// モータを回転させる
			leftMotor.rotate(balanceControl.getLeft());
			rightMotor.rotate(balanceControl.getRight());
		}
		else{
			// モータを回転させる
			leftMotor.rotate(Forward + Turn);
			rightMotor.rotate(Forward - Turn);
		}

		if(light > 670){
			if(++safety >= 1250)
				while(1){
					leftMotor.rotate(0);
					rightMotor.rotate(0);
				}
		}
		else{
			safety = 0;
		}
		
		/* ロギング */
		ecrobot_bt_data_logger(0, 0);
}

// 倒立振子制御とモータ回転角度の初期化
void Control::resetBalance(){
	balanceControl.init();
	leftMotor.resetAngle();
	rightMotor.resetAngle();
}
