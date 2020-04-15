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

/*�t���O--------------------------------------*/
//���x����L���t���O
char speed_control_flg = 0;
//�ǐ���L���t���O
char wall_control_flg = 0;

/*���x, �����x, ����-------------------------*/
//�ڕW���x, �����x, �ڕW�p���x, �p�����x, �X�����[�������񔼌a
float target_speed = 0.0;
float accel = 0.0;
float target_omega = 0.0;
float alpha = 0.0;
//1ms���Ƃ̒Ǐ]�ڕW���x, ����, �p���x, �p�x
volatile float static tar_vel = 0.0;
volatile float static tar_dis = 0.0;
volatile float static tar_omega = 0.0;
volatile float static tar_angle = 0.0;
//1ms�O�̎��ۂ̍��E���x
static float previous_vel_r = 0.0;
static float previous_vel_l = 0.0;
//���ۂ̍��E���x, ���S���x, �p���x
float current_vel_r = 0.0;
float current_vel_l = 0.0;
volatile float current_vel_ave = 0.0;
float current_omega = 0.0;
//���ۂ̍��E����, ����, �p�x
volatile float current_dis_r = 0.0;
volatile float current_dis_l = 0.0;
volatile float current_dis_ave = 0.0;
volatile float current_angle = 0.0;

/*����v�Z�p-------------------------*/
//���ǃZ���T�l
short sensor_lf, sensor_rf;
//�΍�
short error;

/*���[�^����p-------------------------*/
//�E���[�^�̏o�͓d��, �����[�^�̏o�͓d��
volatile static float V_r = 0.0;
volatile static float V_l = 0.0;
//�o�b�e���[�d��
static float V_bat = 0.0;
//�E���[�^Duty, �����[�^Duty
static short duty_r = 0;
static short duty_l = 0;

//��`������
//����, �ō���, �I�[��, �����x, �ǐ���L��( ON:1, OFF:0 )
void straight(float _length, float _top_speed, float _end_speed, float _accel, char _wall_control){
	
	//�����ɕK�v�ȋ���, �����ɕK�v�ȋ���
	float accel_length, brake_length;
	//( accel_length + brake_length ) > length �������ꍇ�̍ō���
	float top_speed2;
	//�����x
	float start_speed;
	
	//�ڕW���x, �ڕW�p���x, �����x, �p�����x�ݒ�
	target_speed = _top_speed;
	target_omega = 0;
	accel = _accel;
	alpha = 0;
	
	//���[�^�[ON, ���x����ON, �ǐ���ON/OFF�ݒ�
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = _wall_control;
	
	//�����x�v��
	start_speed = current_vel_ave;
	//v1^2 - v0^2 = 2ax ���@�̉���, �����\�������Z�o
	accel_length = ( ( _top_speed + start_speed ) * ( _top_speed - start_speed ) ) / ( 2.0 * accel );
	brake_length = ( ( _top_speed + _end_speed ) * ( _top_speed - _end_speed ) ) / ( 2.0 * accel );
	//�����\����+�����\���������s������蒷�������猸���J�n�n�_��ύX����
	if( _length < ( accel_length + brake_length ) ){
		//top_speed^2 - start_speed^2 = 2.0 * acc * x1(��������)
		//end_speed^2 - top_speed^2 = 2.0 * -acc * x2(��������)
		//(x1 + x2) = length
		//���, �ō���top_speed��
		top_speed2 = ( ( 2.0 * accel * _length ) + ( start_speed * start_speed ) + ( _end_speed * _end_speed ) ) / 2.0;
		//�����2�Ԗڂ̌��������̎��ɑ�������
		brake_length = ( top_speed2 - ( _end_speed * _end_speed ) ) / ( 2.0 * accel );
	}
	
	//�����J�n�܂ő҂�
	//���݋������ڕW����-�����\��������O�̊ԓ������s
	while( tar_dis < ( _length - brake_length ) );
	
	//�����J�n
	target_speed = _end_speed;
	//�I�[���x��0��������Œᑬ�x�݂���(���������_���̐��x���C����)
	if( _end_speed > -0.0009 && _end_speed < 0.0009 ){
		target_speed = MIN_VEL;
	}
	
	//�ڕW�����ɓ��B����܂ő҂�
	while( tar_dis < _length );
	
	//��~����
	if( _end_speed > -0.0009 && _end_speed < 0.0009 ){
		//�X�s�[�h����OFF
		speed_control_flg = 0;
		direction_r_mot(MOT_BRAKE);
		direction_l_mot(MOT_BRAKE);
		duty_l = 0;
		duty_r = 0;
		//�X�e�[�^�X���Z�b�g
		reset_run_status();
	}
	
	current_dis_r = 0;
	current_dis_l = 0;
	current_dis_ave = 0;
}

//���V�n����
//�ڕW�p�x, �ō��p���x, �I�[�p���x, �p�����x
void turn( float _angle, float _top_omega, float _end_omega, float _alpha ){
	
	//�ڕW�p�x
	float angle;
	//������Ԋp�x, ������Ԋp�x
	float accel_angle, brake_angle;
	//( accel_angle + brake_angle ) > angle �������ꍇ�̍ō��p���x
	float top_omega2;
	//���p���x
	float start_omega = 0.0;
	
	//�ڕW�d�S���x, �ڕW�p���x, �����x, �p�����x, �ڕW�p�x�ݒ�
	target_speed = 0;
	target_omega = _top_omega;
	accel = 0;
	alpha = _alpha;
	angle = _angle;
	
	//���[�^�[ON, ���x����ON, �ǐ���OFF
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = 0;
	
	//�ڕW�p�x�����Ȃ�
	if( angle < 0 ){
		target_omega = -target_omega;	//�p���x���]
		angle = -angle;			//�v�Z�̂��ߊp�x�͐��ɒ���
	}
	
	start_omega = current_omega;
	accel_angle = ( ( _top_omega + start_omega ) * ( _top_omega - start_omega ) ) / ( 2.0 * alpha );
	brake_angle = ( ( _top_omega + _end_omega ) * ( _top_omega - _end_omega ) ) / ( 2.0 * alpha );
	if( angle < ( accel_angle + brake_angle ) ){
		//top_omega^2 - start_omega^2 = 2.0 * alpha * x1(�����p�x)
		//end_omega^2 - top_omega^2 = 2.0 * -alpha * x2(�����p�x)
		//(x1 + x2) = angle
		//���, �ō���top_omega2��
		top_omega2 = ( ( 2.0 * alpha * angle ) + ( start_omega * start_omega ) + ( _end_omega * _end_omega ) ) / 2.0;
		//�����2�Ԗڂ̌����p�x�̎��ɑ�������
		brake_angle = ( top_omega2 - ( _end_omega * _end_omega ) ) / ( 2.0 * alpha );
	}
	
	//�ڕW�p�x�����̏ꍇ
	if( _angle > 0 ){
		
		//�����J�n��Ԃ܂ő҂�
		while( tar_angle < ( angle - brake_angle ) );
		//�����J�n
		target_omega = _end_omega;
		//�I�[�p���x��0�̏ꍇ�Œ�p���x�ݒ�
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = MIN_OMEGA;
		}
		//�ڕW�p�x�܂ő҂�
		while( tar_angle < angle );
		
	//�ڕW�p�x�����̏ꍇ
	}else if( _angle < 0 ){
		
		//�����J�n��Ԃ܂ő҂�
		while( tar_angle > -( angle - brake_angle ) );
		//�����J�n
		target_omega = _end_omega;
		//�I�[�p���x��0�̏ꍇ�Œ�p���x�ݒ�
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = -MIN_OMEGA;
		}
		//�ڕW�p�x�܂ő҂�
		while( tar_angle > -angle );
		
	}
	
	//��~����
	//�X�s�[�h����OFF
	speed_control_flg = 0;
	direction_r_mot(MOT_BRAKE);
	direction_l_mot(MOT_BRAKE);
	duty_l = 0;
	duty_r = 0;
	//�X�e�[�^�X���Z�b�g
	reset_run_status();
	
}

//�X�����[��( �d�S���x�ێ� )
//�ڕW�p�x, �ō��p���x, �I�[�p���x, �p�����x
void slalom( float _angle, float _top_omega, float _end_omega, float _alpha ){
	
	//�ڕW�p�x
	float angle;
	//�p������Ԋp�x, �p������Ԋp�x
	float accel_angle, brake_angle;
	//( accel_angle + brake_angle ) > angle �������ꍇ�̍ō��p���x
	float top_omega2;
	//���p���x
	float start_omega = 0.0;
	
	tar_angle = 0;
	current_angle = 0;
	
	//�ō��p���x, �p�����x, �ڕW�p�x�ݒ�
	//target_speed = current_vel_ave;
	target_omega = _top_omega;
	alpha = _alpha;
	angle = _angle;
	
	//���[�^�[ON, �ǐ���OFF, ���x����ON
	MOT_STBY = 1;
	wall_control_flg = 0;
	speed_control_flg = 1;
	
	//�ڕW�p�x�����Ȃ�
	if( angle < 0 ){
		target_omega = -target_omega;	//�p���x���]
		angle = -angle;			//�v�Z�̂��ߐ��ɒ���
	}
	
	//���p���x�v��
	start_omega = current_omega;
	//������Ԋp�x, ������Ԋp�x�Z�o
	accel_angle = ( ( _top_omega + start_omega ) * ( _top_omega - start_omega ) ) / ( 2.0 * alpha );
	brake_angle = ( ( _top_omega + _end_omega ) * ( _top_omega - _end_omega ) ) / ( 2.0 * alpha );
	//���� + ������Ԋp�x���ڕW�p�x�𒴂���ꍇ, �����p�x�ύX
	if( angle < ( accel_angle + brake_angle ) ){
		//top_omega^2 - start_omega^2 = 2.0 * alpha * x1(�����p�x)
		//end_omega^2 - top_omega^2 = 2.0 * -alpha * x2(�����p�x)
		//(x1 + x2) = angle
		//���ō��p���x��
		top_omega2 = ( ( 2.0 * alpha * angle ) + ( start_omega * start_omega ) + ( _end_omega * _end_omega ) ) / 2.0;
		//2�Ԗڂ̌�����Ԋp�x�̎��ɑ�������
		brake_angle = ( top_omega2 - ( _end_omega * _end_omega ) ) / ( 2.0 * alpha );
	}
	
	//�ڕW�p�x�����̏ꍇ
	if( _angle > 0 ){
		
		//�����J�n��Ԃ܂ő҂�
		while( current_angle < ( angle - brake_angle ) );
		//�����J�n
		target_omega = _end_omega;
		//�I�[�p���x0�Ȃ�Œ�p���x��ݒ�
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = MIN_OMEGA;
		}
		//�ڕW�p�x�܂ő҂�
		while( current_angle < angle );
		
	//�ڕW�p�x�����̏ꍇ
	}else if( _angle < 0 ){
		
		//�����J�n��Ԃ܂ő҂�
		while( current_angle > -( angle - brake_angle ) );
		//�����J�n
		target_omega = _end_omega;
		//�I�[�p���x0�Ȃ�Œ�p���x��ݒ�
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = -MIN_OMEGA;
		}
		//�ڕW�p�x�܂ő҂�
		while( current_angle > -angle );
		
	}
	
	alpha = 0;
	tar_omega = 0;
	current_omega = 0;
	
	current_dis_r = 0;
	current_dis_l = 0;
	current_dis_ave = 0;
	
}

//���x����( 1ms���荞�� )
void control_speed(void){
	
	//���x����t���O��0�Ȃ�return
	if( speed_control_flg == 0 ) return;
	
	//�����x�͏�ɐ��ɂ���
	if( accel < 0 ) accel = -accel;
		
	//����
	if( tar_vel < target_speed ){	
		tar_vel += accel / 1000.0;		//1s�Ԃ�1000��Ăяo������/1000
		run_state = 1;//�������
		if( tar_vel >= target_speed ){		//�ڕW���x���B
			tar_vel = target_speed;
			run_state = 2;//�������
		}
	}
	//����
	if( tar_vel > target_speed ){
		tar_vel -= accel / 1000.0;		//1s�Ԃ�1000��Ăяo������/1000
		run_state = 3;//�������
		if( tar_vel <= target_speed ){		//�ڕW���x���B
			tar_vel = target_speed;
			run_state = 2;//�������
		}
	}
	tar_dis += tar_vel / 1000;			//1ms�P�ʂŐi�񂾋������Z(�ϕ�)
	
	//�p����
	if( tar_omega < target_omega ){
		tar_omega += alpha / 1000.0;
		turn_state = 1;//�p�������
		if( tar_omega >= target_omega ){
			tar_omega = target_omega;
			turn_state = 2;//�p�������
		}
	}
	//�p����
	if( tar_omega > target_omega ){
		tar_omega -= alpha / 1000.0;
		turn_state = 3;//�p�������
		if( tar_omega <= target_omega ){
			tar_omega = target_omega;
			turn_state = 2;//�p�������
		}
	}
	tar_angle += tar_omega / 1000.0;
	
	//1ms�O�̍��E�G���R�[�_�[�̑��x(m/s)�i�[
	previous_vel_r = current_vel_r;
	previous_vel_l = current_vel_l;
	
	//���E�G���R�[�_�[�̌��݂̑��x��Ɨ��Ɏ擾([mm] -> [m]�ϊ���������)
	current_vel_r = get_current_enc_velocity(RIGHT_ENCODER) / 1000.0;
	current_vel_l = get_current_enc_velocity(LEFT_ENCODER) / 1000.0;
	current_vel_ave = ( current_vel_r + current_vel_l ) / 2.0;
	
	//�p���x[deg/s]�Z�o
	//�̂��̂��W���C��
	current_omega = ( ( current_vel_r - current_vel_l ) / 0.072 ) * ( 180.0 / PI );
	
	//���ۂɐi�񂾋���
	current_dis_r += current_vel_r / 1000.0;	//1s�Ԃ�1000�񑫂�����/1000
	current_dis_l += current_vel_l / 1000.0;
	current_dis_ave = (current_dis_r + current_dis_l) / 2.0;
	
	//���ۂ̊p�x
	current_angle += current_omega / 1000.0;
	
	log_save((short)(current_vel_ave*1000.0));
	//log_save((short)(tar_vel*1000.0));
	//log_save((short)(current_omega));
	//log_save((short)(tar_omega));
}

//PID����( 1ms���荞�� )
void pid_speed(void){
	
	static float vel_error_p, pre_vel_error_p;
	static float vel_error_i = 0.0;
	float vel_error_d;
	static float omega_error_p, pre_omega_error_p;
	static float omega_error_i = 0.0;
	float omega_error_d;
	float r_control, l_control;
	
	//���x����t���O��0�Ȃ�return
	if( speed_control_flg == 0 ) return;
	
	//FF��
	V_r = 0;
	V_l = 0;
	if( run_state == 1 ){		//�������
		V_r += ( tar_vel * FF_KV ) + ( accel * FF_KA ) + FF_FRIC;
		V_l += ( tar_vel * FF_KV ) + ( accel * FF_KA ) + FF_FRIC;
	}else if( run_state == 2 ){	//�������
		V_r += ( tar_vel * FF_KV ) + ( 0 * FF_KA ) + FF_FRIC;
		V_l += ( tar_vel * FF_KV ) + ( 0 * FF_KA ) + FF_FRIC;
	}else if( run_state == 3 ){	//�������
		V_r += ( tar_vel * FF_KV ) + ( -accel * FF_KA ) + FF_FRIC;
		V_l += ( tar_vel * FF_KV ) + ( -accel * FF_KA ) + FF_FRIC;
	}
	if( turn_state == 1 ){		//�p�������
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( alpha * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( alpha * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
	}else if( turn_state == 2 ){	//�p�������
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( 0 * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( 0 * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
	}else if( turn_state == 3 ){	//�p�������
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( -alpha * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( -alpha * ( PI / 180.0 ) * FF_KALPHA ) + FF_FRIC;
	}
	
	//���xPI�v�Z
	pre_vel_error_p = vel_error_p;
	vel_error_p = ( tar_vel - current_vel_ave );
	vel_error_i += vel_error_p * DELTA_T;
	vel_error_d = ( vel_error_p - pre_vel_error_p ) * DELTA_T;
	
	//�p���xPI�v�Z
	pre_omega_error_p = omega_error_p;
	omega_error_p = ( tar_omega - current_omega );
	omega_error_i += omega_error_p * DELTA_T;
	omega_error_d = ( omega_error_p - pre_omega_error_p ) * DELTA_T;
	
	//FB����ʌv�Z
	r_control = ( VEL_KP * vel_error_p ) + ( VEL_KI * vel_error_i ) + ( VEL_KD * vel_error_d ) + ( ( OMEGA_KP / 100.0 ) * omega_error_p ) + ( ( OMEGA_KI / 100.0 ) * omega_error_i ) + ( ( OMEGA_KD / 100.0 ) * omega_error_d );
	l_control = ( VEL_KP * vel_error_p ) + ( VEL_KI * vel_error_i ) + ( VEL_KD * vel_error_d ) - ( ( OMEGA_KP / 100.0 ) * omega_error_p ) - ( ( OMEGA_KI / 100.0 ) * omega_error_i ) - ( ( OMEGA_KD / 100.0 ) * omega_error_d );
	
	//�o�͓d���v�Z
	V_r += r_control;
	V_l += l_control;
	
	//�E���[�^�̏o�͓d�������̏ꍇ, ���̏ꍇ
	if(V_r > 0.0){
		direction_r_mot(MOT_FORWARD);	//���]
	}else{
		direction_r_mot(MOT_BACKWARD);	//�t�]
		V_r = -V_r;			//�d���𐳂ɒ���
	}
	
	//�����[�^�̏o�͓d�������̏ꍇ, ���̏ꍇ
	if(V_l > 0.0){
		direction_l_mot(MOT_FORWARD);	//���]
	}else{
		direction_l_mot(MOT_BACKWARD);	//�t�]
		V_l = -V_l;			//�d���𐳂ɒ���
	}
	
	//�d���d���擾
	V_bat = get_battery_voltage();
	
	//duty[%]�Z�o
	duty_r = (V_r / V_bat) * 100;
	duty_l = (V_l / V_bat) * 100;
	
	//�t�F�C���Z�[�t
	if(duty_r >= 80) duty_r = 80;
	if(duty_l >= 80) duty_l = 80;
}

//�ǐ���( 1ms���荞�� )
//*���x������O�ɋL�q
void control_wall(void){
	short ref_lf, ref_rf;		//�@�̒��S���̉��ǃZ���T��l
	short l_threshold, r_threshold;	//���ǃZ���T臒l
	
	if( wall_control_flg == 0 ) return;
	
	//臒l�ݒ�
	l_threshold = LEFT_THRESHOLD;
	r_threshold = RIGHT_THRESHOLD;
	
	//��l�ݒ�
	ref_lf = REF_LF;
	ref_rf = REF_RF;
	
	//�Z���T�l�i�[
	sensor_lf = get_sen_value(LF_SEN);
	sensor_rf = get_sen_value(RF_SEN);
	
	if( (sensor_rf > r_threshold) && (sensor_lf > l_threshold) ){
		//���ǂ�����ꍇ
		error = (sensor_rf - ref_rf) - (sensor_lf - ref_lf);
	}else if( (sensor_rf <= r_threshold) && (sensor_lf <= l_threshold) ){
		//���ǂƂ������ꍇ
		error = 0;
	}else if( sensor_rf > r_threshold ){
		//�E�ǂ݂̂���ꍇ
		error = 2 * (sensor_rf - ref_rf);
	}else{
		//���ǂ݂̂���ꍇ
		error = -2 * ((sensor_lf - ref_lf));
	}
	
	tar_omega = error * WALL_KP;
}

//���[�^�[���x�ω�( 10us���荞�� )
void change_motor_speed(void){
	//���x����t���O��0�Ȃ�return
	if( speed_control_flg == 0 ) return;
	//���E���[�^��Duty����, ��肾��
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
