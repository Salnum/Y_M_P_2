#include "parameter.h"
#include "sci.h"
#include "maze.h"
#include "run.h"
#include "sensor.h"
#include "interrupt.h"
#include "log.h"

#define MAZE_SIZE	4
#define GOAL_X_COODINATE	3
#define GOAL_Y_COODINATE	3

//�O���[�o���ϐ�--------------
char x_coordinate = 0;
char y_coordinate = 0;
char m_step = 1;
signed char m_dir = 0;
unsigned char path[256] = {0};
unsigned char path_size = 0;
//-------------------------

//���������Ɛ��������̕�
unsigned short wallH[MAZE_SIZE+1] = {0};
unsigned short wallV[MAZE_SIZE+1] = {0};
//���������Ɛ��������̊��m�̕�
unsigned char knownWallH[MAZE_SIZE+1] = {0};
unsigned char knownWallV[MAZE_SIZE+1] = {0};
//�����}�b�v
unsigned short stepMap[MAZE_SIZE][MAZE_SIZE];

//�Ǐ��̏������֐�
void init_wall(void){
	//�O��
	wallV[0] = 0xffff;
	wallV[MAZE_SIZE] = 0xffff;
	wallH[0] = 0xffff;
	wallH[MAZE_SIZE] = 0xffff;
	//�X�^�[�g���
	wallV[1] = 0x0001;
	//�X�^�[�g�������m��
	add_knownWall(0, 0, NORTH);
	add_knownWall(0, 0, EAST);
	add_knownWall(0, 0, SOUTH);
	add_knownWall(0, 0, WEST);
}

//�����}�b�v�������֐�
void init_stepMap(void){
	short i, j;
	//�S���, �����l��300
	for(i=0;i<MAZE_SIZE;i++){
		for(j=0;j<MAZE_SIZE;j++){
			stepMap[i][j] = 300;
		}
	}
	//�S�[�����W��0�ɐݒ�
	stepMap[GOAL_X_COODINATE][GOAL_Y_COODINATE] = 0;
}

//�����}�b�v�W�J�֐�
void update_stepMap(void){
	unsigned short temp_stepMap_val;
	char update_flg = 1;
	short i, j;
	//��ԏ��߂̓W�J���̓S�[�����W
	temp_stepMap_val = stepMap[GOAL_X_COODINATE][GOAL_Y_COODINATE];
	//�W�J�����C���N�������g���Ă���, �W�J�s�\(update_flg�������Ȃ�)�ɂȂ�܂Ń��[�v
	while(update_flg == 1){
		update_flg = 0;
		for(i=0;i<MAZE_SIZE;i++){
			for(j=0;j<MAZE_SIZE;j++){
				if(stepMap[i][j] == temp_stepMap_val){
					//�W�J����
					//�W�J���̎��͂�, �ǂ��Ȃ�, ����MAX�l(���W�J)�̍��W�ɓW�J�ł���
					if( judge_wall(i, j, NORTH) == 0 ){
						if( stepMap[i][j+1] == 300 ){
							stepMap[i][j+1] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
					if( judge_wall(i, j, EAST) == 0 ){
						if( stepMap[i+1][j] == 300 ){
							stepMap[i+1][j] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
					if( judge_wall(i, j, SOUTH) == 0 ){
						if( stepMap[i][j-1] == 300 ){
							stepMap[i][j-1] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
					if( judge_wall(i, j, WEST) == 0 ){
						if( stepMap[i-1][j] == 300 ){
							stepMap[i-1][j] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
				}
			}
		}
		temp_stepMap_val++;	//�W�J���X�V
	}
}

//�����}�b�v�����, ���ݍ��W���猩�čŏ������̕�����Ԃ��֐�
char adachi_judge_nextdir(void){
	unsigned short minimum;
	char nextdir = 0;
	
	minimum = stepMap[x_coordinate][y_coordinate];	//�f�t�H���g�̍ŏ��͌��ݍ��W
	//������r(�ǂȂ����m�F���Ă���)	�D��x: �k>��>��>��
	//�k
	if( judge_wall(x_coordinate, y_coordinate, NORTH) == 0 ){
		if( minimum > stepMap[x_coordinate][y_coordinate+1] ){
			minimum = stepMap[x_coordinate][y_coordinate+1];
			nextdir = 0;
		}
	}
	//��
	if( judge_wall(x_coordinate, y_coordinate, EAST) == 0 ){
		if( minimum > stepMap[x_coordinate+1][y_coordinate] ){
			minimum = stepMap[x_coordinate+1][y_coordinate];
			nextdir = 1;
		}
	}
	//��
	if( judge_wall(x_coordinate, y_coordinate, SOUTH) == 0 ){
		if( minimum > stepMap[x_coordinate][y_coordinate-1] ){
			minimum = stepMap[x_coordinate][y_coordinate-1];
			nextdir = 2;
		}
	}
	//��
	if( judge_wall(x_coordinate, y_coordinate, WEST) == 0 ){
		if( minimum > stepMap[x_coordinate-1][y_coordinate] ){
			minimum = stepMap[x_coordinate-1][y_coordinate];
			nextdir = 3;
		}
	}
	//���茋��
	nextdir += 4;
	nextdir -= m_dir;
	if( nextdir >= 4 ){
		nextdir -= 4;
	}
	return nextdir;
}

//�����}�b�v����ɍŒZ�o�H�̃p�X�𐶐�����֐�
void generate_adachi_shortestRoute(void){
	static signed char previousdir, nextdir;
	short i, j;
	
	//���m�ǂ͂�����̂Ƃ��ĕǏ��X�V
	for(i=0;i<MAZE_SIZE;i++){
		for(j=0;j<MAZE_SIZE;j++){
			if( judge_knownWall(i, j, NORTH) == 0 )	add_wall(i, j, NORTH);
			if( judge_knownWall(i, j, EAST) == 0 )	add_wall(i, j, EAST);
			if( judge_knownWall(i, j, SOUTH) == 0 )	add_wall(i, j, SOUTH);
			if( judge_knownWall(i, j, WEST) == 0 )	add_wall(i, j, WEST);
		}
	}
	
	init_stepMap();		//�����}�b�v������
	update_stepMap();	//�����}�b�v�쐬
	
	//���W�ƌ���������
	x_coordinate = 0;
	y_coordinate = 0;
	m_dir = 0;
	previousdir = 0;
	nextdir = 0;
	//�S�[���܂Ńp�X�������[�v
	i = 0;
	while( !( x_coordinate == GOAL_X_COODINATE && y_coordinate == GOAL_Y_COODINATE ) ){
		previousdir = nextdir;				//�ЂƂO�̌����ۑ�
		nextdir = adachi_judge_nextdir();		//���ݍ��W���猩�čŏ��̕����ƂȂ������m��
		if( nextdir == 0 ){
			if( previousdir != nextdir )	i++;	//�ЂƂO�ƍ���̌����������łȂ���΃p�X�v�f��i�߂�
			path[i] ++;				//���i�p�X
			update_coordinate();			//���W�X�V
		}else if( nextdir == 1 ){
			i++;					//���i�łȂ��̂Ńp�X�i�߂�
			path[i] = 20;				//�E�܃p�X
			if(++m_dir > 3)	m_dir = 0;		//���݌����E��
		}else if( nextdir == 3 ){
			i++;					//���i�łȂ��̂Ńp�X�i�߂�
			path[i] = 30;				//���܃p�X
			if(--m_dir < 0)	m_dir = 3;		//���݌�������
		}
	}
	path_size = i;
}

//���W�ƕ��ʂ�^�����炻�̏�̕ǂ̔z��ɕǂ�����֐�
void add_wall(char _x_coordinate, char _y_coordinate, char _dir){
	unsigned short one = 0x80;	//MSB��1
	//one��K�v�Ȃ����E�V�t�g���������̂ƑΉ�����z���OR����邱�Ƃŕǂ�ǉ�
	//�O�ǂƃX�^�[�g���͍X�V����return
	if(_dir == NORTH){
		if(_y_coordinate == MAZE_SIZE - 1)	return;
		wallH[_y_coordinate + 1] |= ( one >> _x_coordinate );
	}else if(_dir == WEST){
		if(_x_coordinate == 0)	return;
		else if(_x_coordinate == 1 && _y_coordinate == 0)	return;	//����̓X�^�[�g���̓���
		//y��0�̂Ƃ��̓V�t�g�������ɂȂ�̂ŗ�O����
		if(_y_coordinate == 0)	wallV[_x_coordinate] |= ( one >> ( MAZE_SIZE - 1 ) );
		else			wallV[_x_coordinate] |= ( one >> ( _y_coordinate - 1 ) );
	}else if(_dir == SOUTH){
		if(_y_coordinate == 0)	return;
		wallH[_y_coordinate] |= ( one >> _x_coordinate );
	}else if(_dir == EAST){
		if(_x_coordinate == MAZE_SIZE - 1)	return;
		else if(_x_coordinate == 0 && _y_coordinate == 0)	return;
		//y��0�̂Ƃ��̓V�t�g�������ɂȂ�̂ŗ�O����
		if(_y_coordinate == 0)	wallV[_x_coordinate + 1] |= ( one >> ( MAZE_SIZE - 1 ) );
		else			wallV[_x_coordinate + 1] |= ( one >> ( _y_coordinate - 1 ) );
	}
}

//������W�Ƌ@�̌����ɂ����č��ƑO�ƉE�Z���T�̒l��臒l�𒴂��Ă����炻�̕����̕ǂɕǂ�����֐�
void set_wall(_x_coordinate, _y_coordinate, _m_dir){
	signed char temp_dir = _m_dir;
	if( get_sen_value(LF_SEN) > LEFT_THRESHOLD ){
		if(--temp_dir < 0)	temp_dir = 3;
		add_wall(_x_coordinate, _y_coordinate, temp_dir);
		temp_dir = _m_dir;	//���̂��߂ɒ���
	}
	if( ( get_sen_value(LS_SEN) + get_sen_value(RS_SEN) ) / 2 > FRONT_THRESHOLD ){
		add_wall(_x_coordinate, _y_coordinate, temp_dir);
	}
	if( get_sen_value(RF_SEN) > RIGHT_THRESHOLD ){
		if(++temp_dir > 3)	temp_dir = 0;
		add_wall(_x_coordinate, _y_coordinate, temp_dir);
		temp_dir = _m_dir;	//���̂��߂ɒ���
	}
}

//���W�ƕ��ʂ�^�����炻�̏�ɕǂ����邩�ǂ������肷��֐�
//���� : 1, �Ȃ� : 0
char judge_wall(_x_coordinate, _y_coordinate, _dir){
	unsigned short checker = 0x80;	//MSB��1
	
	if(_dir == NORTH){
		if(_y_coordinate == MAZE_SIZE - 1)	return 1;
		else if( ( wallH[_y_coordinate + 1] & ( checker >> _x_coordinate ) ) == 0x00 ){
			return 0;
		}
		else	return 1;
	}else if(_dir == WEST){
		if(_x_coordinate == 0)	return 1;
		else if(_x_coordinate == 1 && _y_coordinate == 0)	return 1;
		//y��0�̂Ƃ��̓V�t�g�������ɂȂ�̂ŗ�O����
		if(_y_coordinate == 0){
			if( ( wallV[_x_coordinate] & ( checker >> ( MAZE_SIZE - 1 ) ) ) == 0x00 ){
				return 0;
			}
			else	return 1;
		}
		if( ( wallV[_x_coordinate] & ( checker >> ( _y_coordinate - 1 ) ) ) == 0x00 ){
			return 0;
		}
		else return 1;
	}else if(_dir == SOUTH){
		if(_y_coordinate == 0)	return 1;
		else if( ( wallH[_y_coordinate] & ( checker >> _x_coordinate ) ) == 0x00 ){
			return 0;
		}
		else return 1;
	}else if(_dir == EAST){
		if(_x_coordinate == MAZE_SIZE - 1)	return 1;
		else if(_x_coordinate == 0 && _y_coordinate == 0)	return 1;
		//y��0�̂Ƃ��̓V�t�g�������ɂȂ�̂ŗ�O����
		if(_y_coordinate == 0){
			if( ( wallV[_x_coordinate + 1] & ( checker >> ( MAZE_SIZE - 1 ) ) ) == 0x00 ){
			return 0;
			}
			else return 1;
		}
		if( ( wallV[_x_coordinate + 1] & ( checker >> ( _y_coordinate - 1 ) ) ) == 0x00 ){
			return 0;
		}
		else return 1;
	}
}

//���W�ƕ��ʂ�^�����炻�̏�̊��m�ǂ�ۑ�����֐�
void add_knownWall(char _x_coordinate, char _y_coordinate, char _dir){
	unsigned short one = 0x80;	//MSB��1
	//one��K�v�Ȃ����E�V�t�g���������̂ƑΉ�����z���OR����邱�ƂŊ��m�ǂ�ǉ�
	if(_dir == NORTH){
		knownWallH[_y_coordinate + 1] |= ( one >> _x_coordinate );
	}else if(_dir == WEST){
		//y��0�̂Ƃ��̓V�t�g�������ɂȂ�̂ŗ�O����
		if(_y_coordinate == 0)	knownWallV[_x_coordinate] |= ( one >> ( MAZE_SIZE - 1 ) );
		else			knownWallV[_x_coordinate] |= ( one >> ( _y_coordinate - 1 ) );
	}else if(_dir == SOUTH){
		knownWallH[_y_coordinate] |= ( one >> _x_coordinate );
	}else if(_dir == EAST){
		//y��0�̂Ƃ��̓V�t�g�������ɂȂ�̂ŗ�O����
		if(_y_coordinate == 0)	knownWallV[_x_coordinate + 1] |= ( one >> ( MAZE_SIZE - 1 ) );
		else			knownWallV[_x_coordinate + 1] |= ( one >> ( _y_coordinate - 1 ) );
	}
}

//���W�ƕ��ʂ�^�����炻�̏�̕ǂ����m���ǂ������肷��֐�
//���m : 1, ���m : 0
char judge_knownWall(char _x_coordinate, char _y_coordinate, char _dir){
	unsigned short checker = 0x80;	//MSB��1
	if(_dir == NORTH){
		if( ( knownWallH[_y_coordinate + 1] & ( checker >> _x_coordinate ) ) == 0x00 ){
			return 0;
		}
		else	return 1;
	}else if(_dir == WEST){
		//y��0�̂Ƃ��̓V�t�g�������ɂȂ�̂ŗ�O����
		if(_y_coordinate == 0){
			if( ( knownWallV[_x_coordinate] & ( checker >> ( MAZE_SIZE - 1 ) ) ) == 0x00 ){
				return 0;
			}
			else return 1;
		}
		if( ( knownWallV[_x_coordinate] & ( checker >> ( _y_coordinate - 1 ) ) ) == 0x00 ){
			return 0;
		}
		else return 1;
	}else if(_dir == SOUTH){
		if( ( knownWallH[_y_coordinate] & ( checker >> _x_coordinate ) ) == 0x00 ){
			return 0;
		}
		else return 1;
	}else if(_dir == EAST){
		//y��0�̂Ƃ��̓V�t�g�������ɂȂ�̂ŗ�O����
		if(_y_coordinate == 0){
			if( ( knownWallV[_x_coordinate + 1] & ( checker >> ( MAZE_SIZE - 1 ) ) ) == 0x00 ){
				return 0;
			}
			else return 1;
		}
		if( ( knownWallV[_x_coordinate + 1] & ( checker >> ( _y_coordinate - 1 ) ) ) == 0x00 ){
			return 0;
		}
		else return 1;
	}
}

//�Ǐ��f���o���֐�
void print_wall(void){
	short i, j, k;
	
	for(j=MAZE_SIZE-1;j>=0;j--){
		//����1��
		for(i=0;i<MAZE_SIZE;i++){
			sci_printf("+");
			if(judge_wall(i, j, NORTH) == 1)	sci_printf("---");
			else	sci_printf("   ");
		}
		sci_printf("+");
		sci_printf("\r\n");
		//�c��1��
		if(judge_wall(0, j, WEST) == 1)	sci_printf("|");
		else	sci_printf(" ");
		for(i=0;i<MAZE_SIZE;i++){
			//�}�X�̒�----------------------------------------------------------
			
			if(stepMap[i][j] < 10)	sci_printf(" %d ", stepMap[i][j]);
			else if(stepMap[i][j] < 100)	sci_printf("%d ", stepMap[i][j]);
			else if(stepMap[i][j] < 1000)	sci_printf("%d", stepMap[i][j]);
			
			//sci_printf("   ");
			//----------------------------------------------------------------
			if(judge_wall(i, j, EAST) == 1)	sci_printf("|");
			else	sci_printf(" ");
		}
		sci_printf("\r\n");
	}
	//�O�Ǖ�
	for(i=0;i<MAZE_SIZE;i++){
		sci_printf("+");
		if(judge_wall(i, 0, SOUTH) == 1)	sci_printf("---");
		else	sci_printf("   ");
	}
	sci_printf("+");
}

//�ŒZ�o�H�p�X�f���o���֐�
void print_shortestRoute(void){
	short i=0;
	while( path[i] != 0 ){
		sci_printf("%d \r\n", path[i]);
		i++;
	}
}

//���W�X�V
void update_coordinate(void){
	if(m_dir == 0)		y_coordinate++;
	else if(m_dir == 1)	x_coordinate++;
	else if(m_dir == 2)	y_coordinate--;
	else if(m_dir == 3)	x_coordinate--;
}

//���ݍ��W���S�[�����ǂ������肷��֐�
char goal_judge(void){
	short i=0;
	//�S�[�����W�Ȃ�1��Ԃ�
	if(x_coordinate == GOAL_X_COODINATE && y_coordinate == GOAL_Y_COODINATE)	return 1;
	else	return 0;
}