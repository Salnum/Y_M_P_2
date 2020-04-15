#include "iodefine.h"
#include "define_YMP.h"
#include "control.h"
#include "run.h"
#include "encoder.h"
#include "sensor.h"
#include "interrupt.h"
#include "sci.h"
#include "drive.h"
#include "parameter.h"
#include "log.h"

char run_state;
char turn_state;

/*フラグ--------------------------------------*/
//速度制御有効フラグ
char speed_control_flg = 0;
//壁制御有効フラグ
char wall_control_flg = 0;

/*速度, 加速度, 距離-------------------------*/
//目標速度, 加速度, 目標角速度, 角加速度, スラローム時旋回半径
float target_speed = 0.0;
float accel = 0.0;
float target_omega = 0.0;
float alpha = 0.0;
//1msごとの追従目標速度, 距離, 角速度, 角度
volatile float static tar_vel = 0.0;
volatile float static tar_dis = 0.0;
volatile float static tar_omega = 0.0;
volatile float static tar_angle = 0.0;
//1ms前の実際の左右速度
static float previous_vel_r = 0.0;
static float previous_vel_l = 0.0;
//実際の左右速度, 中心速度, 角速度
float current_vel_r = 0.0;
float current_vel_l = 0.0;
volatile float current_vel_ave = 0.0;
float current_omega = 0.0;
//実際の左右距離, 平均, 角度
volatile float current_dis_r = 0.0;
volatile float current_dis_l = 0.0;
volatile float current_dis_ave = 0.0;
volatile float current_angle = 0.0;

/*制御計算用-------------------------*/
//横壁センサ値
short sensor_lf, sensor_rf;
//偏差
short error;

/*モータ制御用-------------------------*/
//右モータの出力電圧, 左モータの出力電圧
volatile static float V_r = 0.0;
volatile static float V_l = 0.0;
//バッテリー電圧
static float V_bat = 0.0;
//右モータDuty, 左モータDuty
static short duty_r = 0;
static short duty_l = 0;

//台形加減速
//距離, 最高速, 終端速, 加速度, 壁制御有無( ON:1, OFF:0 )
void straight(float _length, float _top_speed, float _end_speed, float _accel, char _wall_control){
	
	//加速に必要な距離, 減速に必要な距離
	float accel_length, brake_length;
	//( accel_length + brake_length ) > length だった場合の最高速
	float top_speed2;
	//初速度
	float start_speed;
	
	//目標速度, 目標角速度, 加速度, 角加速度設定
	target_speed = _top_speed;
	target_omega = 0;
	accel = _accel;
	alpha = 0;
	
	//モーターON, 速度制御ON, 壁制御ON/OFF設定
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = _wall_control;
	
	//初速度計測
	start_speed = current_vel_ave;
	//v1^2 - v0^2 = 2ax より機体加速, 減速可能距離を算出
	accel_length = ( ( _top_speed + start_speed ) * ( _top_speed - start_speed ) ) / ( 2.0 * accel );
	brake_length = ( ( _top_speed + _end_speed ) * ( _top_speed - _end_speed ) ) / ( 2.0 * accel );
	//加速可能距離+減速可能距離が走行距離より長かったら減速開始地点を変更する
	if( _length < ( accel_length + brake_length ) ){
		//top_speed^2 - start_speed^2 = 2.0 * acc * x1(加速距離)
		//end_speed^2 - top_speed^2 = 2.0 * -acc * x2(減速距離)
		//(x1 + x2) = length
		//より, 最高速top_speedは
		top_speed2 = ( ( 2.0 * accel * _length ) + ( start_speed * start_speed ) + ( _end_speed * _end_speed ) ) / 2.0;
		//これを2番目の減速距離の式に代入すれば
		brake_length = ( top_speed2 - ( _end_speed * _end_speed ) ) / ( 2.0 * accel );
	}
	
	//減速開始まで待つ
	//現在距離が目標距離-減速可能距離より手前の間等速走行
	while( tar_dis < ( _length - brake_length ) );
	
	//減速開始
	target_speed = _end_speed;
	//終端速度が0だったら最低速度設ける(浮動小数点数の精度を気遣う)
	if( _end_speed > -0.0009 && _end_speed < 0.0009 ){
		target_speed = MIN_VEL;
	}
	
	//目標距離に到達するまで待つ
	while( tar_dis < _length );
	
	//停止処理
	if( _end_speed > -0.0009 && _end_speed < 0.0009 ){
		//スピード制御OFF
		speed_control_flg = 0;
		direction_r_mot(MOT_BRAKE);
		direction_l_mot(MOT_BRAKE);
		duty_l = 0;
		duty_r = 0;
		//ステータスリセット
		reset_run_status();
	}
	
	current_dis_r = 0;
	current_dis_l = 0;
	current_dis_ave = 0;
}

//超新地旋回
//目標角度, 最高角速度, 終端角速度, 角加速度
void turn( float _angle, float _top_omega, float _end_omega, float _alpha ){
	
	//目標角度
	float angle;
	//加速区間角度, 減速区間角度
	float accel_angle, brake_angle;
	//( accel_angle + brake_angle ) > angle だった場合の最高角速度
	float top_omega2;
	//初角速度
	float start_omega = 0.0;
	
	//目標重心速度, 目標角速度, 加速度, 角加速度, 目標角度設定
	target_speed = 0;
	target_omega = _top_omega;
	accel = 0;
	alpha = _alpha;
	angle = _angle;
	
	//モーターON, 速度制御ON, 壁制御OFF
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = 0;
	
	//目標角度が負なら
	if( angle < 0 ){
		target_omega = -target_omega;	//角速度反転
		angle = -angle;			//計算のため角度は正に直す
	}
	
	start_omega = current_omega;
	accel_angle = ( ( _top_omega + start_omega ) * ( _top_omega - start_omega ) ) / ( 2.0 * alpha );
	brake_angle = ( ( _top_omega + _end_omega ) * ( _top_omega - _end_omega ) ) / ( 2.0 * alpha );
	if( angle < ( accel_angle + brake_angle ) ){
		//top_omega^2 - start_omega^2 = 2.0 * alpha * x1(加速角度)
		//end_omega^2 - top_omega^2 = 2.0 * -alpha * x2(減速角度)
		//(x1 + x2) = angle
		//より, 最高速top_omega2は
		top_omega2 = ( ( 2.0 * alpha * angle ) + ( start_omega * start_omega ) + ( _end_omega * _end_omega ) ) / 2.0;
		//これを2番目の減速角度の式に代入すれば
		brake_angle = ( top_omega2 - ( _end_omega * _end_omega ) ) / ( 2.0 * alpha );
	}
	
	//目標角度が正の場合
	if( _angle > 0 ){
		
		//減速開始区間まで待つ
		while( tar_angle < ( angle - brake_angle ) );
		//減速開始
		target_omega = _end_omega;
		//終端角速度が0の場合最低角速度設定
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = MIN_OMEGA;
		}
		//目標角度まで待つ
		while( tar_angle < angle );
		
	//目標角度が負の場合
	}else if( _angle < 0 ){
		
		//減速開始区間まで待つ
		while( tar_angle > -( angle - brake_angle ) );
		//減速開始
		target_omega = _end_omega;
		//終端角速度が0の場合最低角速度設定
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = -MIN_OMEGA;
		}
		//目標角度まで待つ
		while( tar_angle > -angle );
		
	}
	
	//停止処理
	//スピード制御OFF
	speed_control_flg = 0;
	direction_r_mot(MOT_BRAKE);
	direction_l_mot(MOT_BRAKE);
	duty_l = 0;
	duty_r = 0;
	//ステータスリセット
	reset_run_status();
	
}

//スラローム( 重心速度維持 )
//目標角度, 最高角速度, 終端角速度, 角加速度
void slalom( float _angle, float _top_omega, float _end_omega, float _alpha ){
	
	//目標角度
	float angle;
	//角加速区間角度, 角減速区間角度
	float accel_angle, brake_angle;
	//( accel_angle + brake_angle ) > angle だった場合の最高角速度
	float top_omega2;
	//初角速度
	float start_omega = 0.0;
	
	tar_angle = 0;
	current_angle = 0;
	
	//最高角速度, 角加速度, 目標角度設定
	//target_speed = current_vel_ave;
	target_omega = _top_omega;
	alpha = _alpha;
	angle = _angle;
	
	//モーターON, 壁制御OFF, 速度制御ON
	MOT_STBY = 1;
	wall_control_flg = 0;
	speed_control_flg = 1;
	
	//目標角度が負なら
	if( angle < 0 ){
		target_omega = -target_omega;	//角速度反転
		angle = -angle;			//計算のため正に直す
	}
	
	//初角速度計測
	start_omega = current_omega;
	//加速区間角度, 減速区間角度算出
	accel_angle = ( ( _top_omega + start_omega ) * ( _top_omega - start_omega ) ) / ( 2.0 * alpha );
	brake_angle = ( ( _top_omega + _end_omega ) * ( _top_omega - _end_omega ) ) / ( 2.0 * alpha );
	//加速 + 減速区間角度が目標角度を超える場合, 減速角度変更
	if( angle < ( accel_angle + brake_angle ) ){
		//top_omega^2 - start_omega^2 = 2.0 * alpha * x1(加速角度)
		//end_omega^2 - top_omega^2 = 2.0 * -alpha * x2(減速角度)
		//(x1 + x2) = angle
		//より最高角速度は
		top_omega2 = ( ( 2.0 * alpha * angle ) + ( start_omega * start_omega ) + ( _end_omega * _end_omega ) ) / 2.0;
		//2番目の減速区間角度の式に代入すれば
		brake_angle = ( top_omega2 - ( _end_omega * _end_omega ) ) / ( 2.0 * alpha );
	}
	
	//目標角度が正の場合
	if( _angle > 0 ){
		
		//減速開始区間まで待つ
		while( current_angle < ( angle - brake_angle ) );
		//減速開始
		target_omega = _end_omega;
		//終端角速度0なら最低角速度を設定
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = MIN_OMEGA;
		}
		//目標角度まで待つ
		while( current_angle < angle );
		
	//目標角度が負の場合
	}else if( _angle < 0 ){
		
		//減速開始区間まで待つ
		while( current_angle > -( angle - brake_angle ) );
		//減速開始
		target_omega = _end_omega;
		//終端角速度0なら最低角速度を設定
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = -MIN_OMEGA;
		}
		//目標角度まで待つ
		while( current_angle > -angle );
		
	}
	
	alpha = 0;
	tar_omega = 0;
	current_omega = 0;
	
	current_dis_r = 0;
	current_dis_l = 0;
	current_dis_ave = 0;
	
}

//速度制御( 1ms割り込み )
void control_speed(void){
	
	//速度制御フラグが0ならreturn
	if( speed_control_flg == 0 ) return;
	
	//加速度は常に正にする
	if( accel < 0 ) accel = -accel;
		
	//加速
	if( tar_vel < target_speed ){	
		tar_vel += accel / 1000.0;		//1s間に1000回呼び出すから/1000
		run_state = 1;//加速状態
		if( tar_vel >= target_speed ){		//目標速度到達
			tar_vel = target_speed;
			run_state = 2;//等速状態
		}
	}
	//減速
	if( tar_vel > target_speed ){
		tar_vel -= accel / 1000.0;		//1s間に1000回呼び出すから/1000
		run_state = 3;//減速状態
		if( tar_vel <= target_speed ){		//目標速度到達
			tar_vel = target_speed;
			run_state = 2;//等速状態
		}
	}
	tar_dis += tar_vel / 1000;			//1ms単位で進んだ距離加算(積分)
	
	//角加速
	if( tar_omega < target_omega ){
		tar_omega += alpha / 1000.0;
		turn_state = 1;//角加速状態
		if( tar_omega >= target_omega ){
			tar_omega = target_omega;
			turn_state = 2;//角等速状態
		}
	}
	//角減速
	if( tar_omega > target_omega ){
		tar_omega -= alpha / 1000.0;
		turn_state = 3;//角減速状態
		if( tar_omega <= target_omega ){
			tar_omega = target_omega;
			turn_state = 2;//角等速状態
		}
	}
	tar_angle += tar_omega / 1000.0;
	
	//1ms前の左右エンコーダーの速度(m/s)格納
	previous_vel_r = current_vel_r;
	previous_vel_l = current_vel_l;
	
	//左右エンコーダーの現在の速度を独立に取得([mm] -> [m]変換も同時に)
	current_vel_r = get_current_enc_velocity(RIGHT_ENCODER) / 1000.0;
	current_vel_l = get_current_enc_velocity(LEFT_ENCODER) / 1000.0;
	current_vel_ave = ( current_vel_r + current_vel_l ) / 2.0;
	
	//角速度[deg/s]算出
	//のちのちジャイロ
	current_omega = ( ( current_vel_r - current_vel_l ) / 0.072 ) * ( 180.0 / PI );
	
	//実際に進んだ距離
	current_dis_r += current_vel_r / 1000.0;	//1s間に1000回足すから/1000
	current_dis_l += current_vel_l / 1000.0;
	current_dis_ave = (current_dis_r + current_dis_l) / 2.0;
	
	//実際の角度
	current_angle += current_omega / 1000.0;
	
	log_save((short)(current_vel_ave*1000.0));
	//log_save((short)(tar_vel*1000.0));
	//log_save((short)(current_omega));
	//log_save((short)(tar_omega));
}

//PID制御( 1ms割り込み )
void pid_speed(void){
	
	static float vel_error_p, pre_vel_error_p;
	static float vel_error_i = 0.0;
	float vel_error_d;
	static float omega_error_p, pre_omega_error_p;
	static float omega_error_i = 0.0;
	float omega_error_d;
	float r_control, l_control;
	
	//速度制御フラグが0ならreturn
	if( speed_control_flg == 0 ) return;
	
	//FF項
	V_r = 0;
	V_l = 0;
	if( run_state == 1 ){		//加速状態
		V_r += ( tar_vel * FF_KV ) + ( accel * FF_KA ) + FF_FRIC;
		V_l += ( tar_vel * FF_KV ) + ( accel * FF_KA ) + FF_FRIC;
	}else if( run_state == 2 ){	//等速状態
		V_r += ( tar_vel * FF_KV ) + ( 0 * FF_KA ) + FF_FRIC;
		V_l += ( tar_vel * FF_KV ) + ( 0 * FF_KA ) + FF_FRIC;
	}else if( run_state == 3 ){	//減速状態
		V_r += ( tar_vel * FF_KV ) + ( -accel * FF_KA ) + FF_FRIC;
		V_l += ( tar_vel * FF_KV ) + ( -accel * FF_KA ) + FF_FRIC;
	}
	if( turn_state == 1 ){		//角加速状態
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( alpha * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( alpha * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
	}else if( turn_state == 2 ){	//角等速状態
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( 0 * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( 0 * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
	}else if( turn_state == 3 ){	//角減速状態
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( -alpha * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( -alpha * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
	}
	
	//速度PI計算
	pre_vel_error_p = vel_error_p;
	vel_error_p = ( tar_vel - current_vel_ave );
	vel_error_i += vel_error_p * DELTA_T;
	vel_error_d = ( vel_error_p - pre_vel_error_p ) * DELTA_T;
	
	//角速度PI計算
	pre_omega_error_p = omega_error_p;
	omega_error_p = ( tar_omega - current_omega );
	omega_error_i += omega_error_p * DELTA_T;
	omega_error_d = ( omega_error_p - pre_omega_error_p ) * DELTA_T;
	
	//FB制御量計算
	r_control = ( VEL_KP * vel_error_p ) + ( VEL_KI * vel_error_i ) + ( VEL_KD * vel_error_d ) + ( ( OMEGA_KP / 100.0 ) * omega_error_p ) + ( ( OMEGA_KI / 100.0 ) * omega_error_i ) + ( ( OMEGA_KD / 100.0 ) * omega_error_d );
	l_control = ( VEL_KP * vel_error_p ) + ( VEL_KI * vel_error_i ) + ( VEL_KD * vel_error_d ) - ( ( OMEGA_KP / 100.0 ) * omega_error_p ) - ( ( OMEGA_KI / 100.0 ) * omega_error_i ) - ( ( OMEGA_KD / 100.0 ) * omega_error_d );
	
	//出力電圧計算
	V_r += r_control;
	V_l += l_control;
	
	//右モータの出力電圧が正の場合, 負の場合
	if(V_r > 0.0){
		direction_r_mot(MOT_FORWARD);	//正転
	}else{
		direction_r_mot(MOT_BACKWARD);	//逆転
		V_r = -V_r;			//電圧を正に直す
	}
	
	//左モータの出力電圧が正の場合, 負の場合
	if(V_l > 0.0){
		direction_l_mot(MOT_FORWARD);	//正転
	}else{
		direction_l_mot(MOT_BACKWARD);	//逆転
		V_l = -V_l;			//電圧を正に直す
	}
	
	//電源電圧取得
	V_bat = get_battery_voltage();
	
	//duty[%]算出
	duty_r = (V_r / V_bat) * 100;
	duty_l = (V_l / V_bat) * 100;
	
	//フェイルセーフ
	if(duty_r >= 80) duty_r = 80;
	if(duty_l >= 80) duty_l = 80;
}

//壁制御( 1ms割り込み )
//*速度制御より前に記述
void control_wall(void){
	short ref_lf, ref_rf;		//機体中心時の横壁センサ基準値
	short l_threshold, r_threshold;	//横壁センサ閾値
	
	if( wall_control_flg == 0 ) return;
	
	//閾値設定
	l_threshold = LEFT_THRESHOLD;
	r_threshold = RIGHT_THRESHOLD;
	
	//基準値設定
	ref_lf = REF_LF;
	ref_rf = REF_RF;
	
	//センサ値格納
	sensor_lf = get_sen_value(LF_SEN);
	sensor_rf = get_sen_value(RF_SEN);
	
	if( (sensor_rf > r_threshold) && (sensor_lf > l_threshold) ){
		//両壁がある場合
		error = (sensor_rf - ref_rf) - (sensor_lf - ref_lf);
	}else if( (sensor_rf <= r_threshold) && (sensor_lf <= l_threshold) ){
		//両壁とも無い場合
		error = 0;
	}else if( sensor_rf > r_threshold ){
		//右壁のみある場合
		error = 2 * (sensor_rf - ref_rf);
	}else{
		//左壁のみある場合
		error = -2 * ((sensor_lf - ref_lf));
	}
	
	tar_omega = error * WALL_KP;
}

//モーター速度変化( 10us割り込み )
void change_motor_speed(void){
	//速度制御フラグが0ならreturn
	if( speed_control_flg == 0 ) return;
	//左右モータにDuty入力, 回りだす
	MOT_DUTY_L = duty_to_count(duty_l);
	MOT_DUTY_R = duty_to_count(duty_r);
}

void reset_run_status(void){
	accel = 0;
	alpha = 0;
	tar_vel = 0;
	tar_dis = 0;
	tar_omega = 0;
	tar_angle = 0;
	previous_vel_r = 0;
	previous_vel_l = 0;
	current_vel_r = 0;
	current_vel_l = 0;
	current_vel_ave = 0;
	current_dis_r = 0;
	current_dis_l = 0;
	current_dis_ave = 0;
	current_omega = 0;
	current_angle = 0;
	V_r = 0;
	V_l = 0;
}

void print_state_test(void){
	while(1){
		sci_printf("tar_omega:%d, current_omega:%d \r\n", (short)tar_omega, (short)current_omega);
		sci_printf("tar_angle:%d, current_angle:%d \r\n", (short)(tar_angle), (short)(current_angle));
	}
}
