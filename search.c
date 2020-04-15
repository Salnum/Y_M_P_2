#include "parameter.h"
#include "sci.h"
#include "search.h"
#include "run.h"
#include "interrupt.h"
#include "function_test.h"
#include "sensor.h"
#include "maze.h"
#include "log.h"

//����@�ɂ��T�����s���֐�
void left_hand(void){
	
	//(0,0)����������
	init_wall();
	init_stepMap();
	m_dir = 0;	//�ŏ��͖k����
	
	straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
	
	while(1){
		
		update_coordinate();	//���W�X�V
		set_wall(x_coordinate, y_coordinate, m_dir);	//�ǔ���, �i�[
		if( goal_judge() == 1 )	break;		//�S�[����������
		
		if( get_sen_value(LF_SEN) < LEFT_THRESHOLD ){	//���ǖ���
			
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.060, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(--m_dir < 0)	m_dir = 3;
			
		}else if( ( get_sen_value(LS_SEN) + get_sen_value(RS_SEN) ) / 2 < FRONT_THRESHOLD ){	//�O�ǖ���
		
			straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			
		}else if( get_sen_value(RF_SEN) < RIGHT_THRESHOLD ){	//�E�ǖ���
		
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.060, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(++m_dir > 3)	m_dir = 0;
			
		}else{	//U�^�[��
		
			straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
			turn(180.0, 700, 0, 7000);
			if(--m_dir < 0)	m_dir = 3;
			if(--m_dir < 0)	m_dir = 3;
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			
		}
	}
	//�I���s��
	straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
	turn(180.0, 700, 0, 7000);
}

//�����@�ɂ��T�����s���֐�
void adachi_method(void){
	
	char nextdir;
	//(0,0)����������
	init_wall();		//�Ǐ�񏉊���
	init_stepMap();		//�����}�b�v������
	update_stepMap();	//�����}�b�v�쐬
	m_dir = 0;		//�ŏ��͖k����
	
	straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
	
	while(1){
		update_coordinate();	//���W�X�V
		set_wall(x_coordinate, y_coordinate, m_dir);	//�ǔ���, �i�[
		
		//���ݍ��W�̑S�Ă̕ǂ����m�Ƃ���
		add_knownWall(x_coordinate, y_coordinate, NORTH);
		add_knownWall(x_coordinate, y_coordinate, EAST);
		add_knownWall(x_coordinate, y_coordinate, SOUTH);
		add_knownWall(x_coordinate, y_coordinate, WEST);
		
		init_stepMap();		//�����}�b�v������
		update_stepMap();	//�����}�b�v�쐬
		if( goal_judge() == 1 )	break;		//�S�[����������
		//�i�s�������f
		nextdir = adachi_judge_nextdir();
		//���i
		if( nextdir == 0 ){
			straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
		}
		//�E��
		if( nextdir == 1 ){
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.060, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(++m_dir > 3)	m_dir = 0;
		}
		//U�^�[��
		if( nextdir == 2 ){
			straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
			turn(180.0, 700, 0, 7000);
			if(--m_dir < 0)	m_dir = 3;
			if(--m_dir < 0)	m_dir = 3;
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
		}
		//����
		if( nextdir == 3 ){
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.060, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(--m_dir < 0)	m_dir = 3;
		}
	}
	//�I���s��
	straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
	turn(180.0, 700, 0, 7000);
	
	generate_adachi_shortestRoute();	//�ŒZ�o�H�p�X����
}

//���������p�X�ɉ����čŒZ���s���s���֐�
void run_shortestRoute(void){
	short i=0;
	while(1){
		if( path[i] <= 15 ){		//���i
			straight(SECTION*path[i], FAST_SPEED, 0, FAST_ACCEL, 1);
		}else if( path[i] == 20 ){	//�E��
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
		}else if( path[i] == 30 ){	//����
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
		}
		if( i == path_size )	break;	//�p�X�S�����s���I������烋�[�v�E�o
		i++;
	}
	turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
}