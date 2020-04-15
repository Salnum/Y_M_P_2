#include "parameter.h"
#include "sci.h"
#include "search.h"
#include "run.h"
#include "interrupt.h"
#include "function_test.h"
#include "sensor.h"
#include "maze.h"
#include "log.h"

//左手法による探索を行う関数
void left_hand(void){
	
	//(0,0)区画内初期化
	init_wall();
	init_stepMap();
	m_dir = 0;	//最初は北向き
	
	straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
	
	while(1){
		
		update_coordinate();	//座標更新
		set_wall(x_coordinate, y_coordinate, m_dir);	//壁判定, 格納
		if( goal_judge() == 1 )	break;		//ゴール到着判定
		
		if( get_sen_value(LF_SEN) < LEFT_THRESHOLD ){	//左壁無し
			
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.060, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(--m_dir < 0)	m_dir = 3;
			
		}else if( ( get_sen_value(LS_SEN) + get_sen_value(RS_SEN) ) / 2 < FRONT_THRESHOLD ){	//前壁無し
		
			straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			
		}else if( get_sen_value(RF_SEN) < RIGHT_THRESHOLD ){	//右壁無し
		
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.060, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(++m_dir > 3)	m_dir = 0;
			
		}else{	//Uターン
		
			straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
			turn(180.0, 700, 0, 7000);
			if(--m_dir < 0)	m_dir = 3;
			if(--m_dir < 0)	m_dir = 3;
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			
		}
	}
	//終了行動
	straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
	turn(180.0, 700, 0, 7000);
}

//足立法による探索を行う関数
void adachi_method(void){
	
	char nextdir;
	//(0,0)区画内初期化
	init_wall();		//壁情報初期化
	init_stepMap();		//歩数マップ初期化
	update_stepMap();	//歩数マップ作成
	m_dir = 0;		//最初は北向き
	
	straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
	
	while(1){
		update_coordinate();	//座標更新
		set_wall(x_coordinate, y_coordinate, m_dir);	//壁判定, 格納
		
		//現在座標の全ての壁を既知とする
		add_knownWall(x_coordinate, y_coordinate, NORTH);
		add_knownWall(x_coordinate, y_coordinate, EAST);
		add_knownWall(x_coordinate, y_coordinate, SOUTH);
		add_knownWall(x_coordinate, y_coordinate, WEST);
		
		init_stepMap();		//歩数マップ初期化
		update_stepMap();	//歩数マップ作成
		if( goal_judge() == 1 )	break;		//ゴール到着判定
		//進行方向判断
		nextdir = adachi_judge_nextdir();
		//直進
		if( nextdir == 0 ){
			straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
		}
		//右折
		if( nextdir == 1 ){
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.060, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(++m_dir > 3)	m_dir = 0;
		}
		//Uターン
		if( nextdir == 2 ){
			straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
			turn(180.0, 700, 0, 7000);
			if(--m_dir < 0)	m_dir = 3;
			if(--m_dir < 0)	m_dir = 3;
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
		}
		//左折
		if( nextdir == 3 ){
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.060, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(--m_dir < 0)	m_dir = 3;
		}
	}
	//終了行動
	straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
	turn(180.0, 700, 0, 7000);
	
	generate_adachi_shortestRoute();	//最短経路パス生成
}

//生成したパスに沿って最短走行を行う関数
void run_shortestRoute(void){
	short i=0;
	while(1){
		if( path[i] <= 15 ){		//直進
			straight(SECTION*path[i], FAST_SPEED, 0, FAST_ACCEL, 1);
		}else if( path[i] == 20 ){	//右折
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
		}else if( path[i] == 30 ){	//左折
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
		}
		if( i == path_size )	break;	//パス全部実行し終わったらループ脱出
		i++;
	}
	turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
}