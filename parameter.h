#ifndef __PARAMETER_HEADER__
#define __PARAMETER_HEADER__

//�G���R�[�_1�J�E���g������ɐi�ދ���[mm/count] �����1ms���Ƃ̃G���R�[�_�̃J�E���g���̕ω�[count/ms]�Ɋ|�����[m/s]���o��
#define	DISTANCE_PER_ENC_COUNT		TIRE_DIAMETER * PI * TIRE_ROTATION_PER_MOTOR_ROTATION * ENC_COUNT_PER_TIRE_ROTATION

//�㎮�̍\���v�f
#define	TIRE_DIAMETER	28.2	//�^�C�����a[mm]	//25.7
#define	PI	3.141593	//�~����
#define	TIRE_ROTATION_PER_MOTOR_ROTATION	4/15	//���[�^1��]������̃^�C����]��
#define	ENC_COUNT_PER_TIRE_ROTATION	1 / 1024

//�@�̌ŗL�萔
#define MOUSE_WEIGHT	100	//�@�̏d��[g]
#define	TREAD	0.130//�g���b�h��[m]
#define	REDUCTION_RATIO	3.75	//������
//SCR13-2005���[�^�[�̒萔
#define	TORQUE_CONSTANT		4.49	//�g���N�萔Kt[mN*m/A]
#define BACK_EMF_CONSTANT	0.4702	//�t�N�d�͒萔KE[mV/rpm]
#define TERMINAL_RESISTANCE	5.41	//�[�q�Ԓ�R[��]

////�悭�g������(m)
#define	HALF_SECTION	0.09	//�����[m]
#define	SECTION		0.18	//����[m]

//������������p�����[�^--------------------------------------
#define	MAX_VEL	3.0		//�ō����x[m/s]
#define	MIN_VEL	0.05		//�Œᑬ�x[m/s]
#define	MIN_OMEGA	100.0	//�Œ�p���x[deg/s]
#define	SEARCH_SPEED	0.2	//�T�����x[m/s]
#define	SEARCH_ACCEL	0.5	//�T�������x[m/s^2]
#define	FAST_SPEED	1.0	//�ŒZ���x[m/s]
#define	FAST_ACCEL	2.0	//�ŒZ�����x[m/s^2]
#define	TURN_OMEGA	300.0	//���V�n����p���x[deg/s]
#define	TURN_ALPHA	3000.0	//���V�n����p���x[deg/s^2]

#define	TURN_SPEED	0.5	//��]���x[m/s]
#define	TURN_ACCEL	1.0	//��]�����x[m/s^2]
#define	SLALOM_TURN_ACCEL	1.0	//�X�����[�������x[m/s^2]

#define	REF_LF	1023	//�����ǃZ���T��l
#define	REF_RF	793	//�E���ǃZ���T��l
#define	LEFT_THRESHOLD	700	//���ǃZ���T臒l
#define	RIGHT_THRESHOLD	700	//�E�ǃZ���T臒l
#define	FRONT_THRESHOLD	400	//�O�ǃZ���T臒l

//PID��������(1ms)
#define DELTA_T	0.001
//�t�B�[�h�t�H���[�h����p�萔
//FF_KV = ( Ke * 60 * n ) / ( 2 * PI * r )
//FF_KA = ( r * m * R ) / ( 2 * Kt *n ) 
//FF_KOMEGA = ( Ke * 60 * n * ( TREAD / 2 ) ) / ( 2 * PI * r )
//FF_KALPHA = ( r * m * R * ( TREAD / 2 ) ) / ( 2 * Kt * n )
//FF_FRIC = ( R / Kt ) * f	(f�͖��C)
#define	FF_KV		1.310334	//( 0.0004702 * 60.0 * 3.75 ) / ( 2.0 * PI * 0.01285 )
#define	FF_KA		0.40//0.2064395	//( 0.01285 * 0.100 * 5.41 ) / ( 2.0 * 0.00449 * 3.75 )
#define	FF_KOMEGA	0.0471720	//( 0.0004702 * 60.0 * 3.75 * 0.036 ) / ( 2.0 * PI * 0.01285 )
#define	FF_KALPHA	0.0125//0.0074318	//( 0.01285 * 0.100 * 5.41 * 0.036 ) / ( 2.0 * 0.00449 * 3.75 )
#define	FF_FRIC		0.38
//PID�Q�C��
//���x
#define	VEL_KP		6.0
#define	VEL_KI		0
#define	VEL_KD		0
//�p���x
#define	OMEGA_KP	0.2
#define	OMEGA_KI	0
#define	OMEGA_KD	0
//�ǐ���
#define	WALL_KP		0.1

#endif