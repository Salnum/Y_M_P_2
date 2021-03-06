#include "define_YMP.h"
#include "iodefine.h"
#include "interrupt.h"
#include "sensor.h"
#include "encoder.h"
#include "run.h"
#include "control.h"

static unsigned short mtu3_cnt = 0;
static unsigned int timer_ms = 0;
static unsigned int timer_sec = 0;
volatile static unsigned int my_timer_ms = 0;

//102us割り込み( *8つのスロットに分割することで1msの塊を作っている )
void mtu3_tgra(){
	mtu3_cnt ++;
	switch(mtu3_cnt){
		case 1:
			//左前センサ
			S12AD.ADANSA.WORD = 0x01;			//AN000を選択
			S12AD.ADCSR.BIT.ADST = 1;			//AD変換開始
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADSTが0になるまで待つ
			set_sen_value(LF_SEN, LED_OFF, S12AD.ADDR0);	//センサ値格納
			LF_SEN_LED = SEN_LED_ON;			//LED点灯
		break;
		
		case 2:
			//左横センサ
			S12AD.ADANSA.WORD = 0x02;			//AN001を選択
			S12AD.ADCSR.BIT.ADST = 1;			//AD変換開始
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADSTが0になるまで待つ
			set_sen_value(LS_SEN, LED_OFF, S12AD.ADDR1);	//センサ値格納
			LS_SEN_LED = SEN_LED_ON;			//LED点灯
		break;
		
		case 3:
			//右横センサ
			S12AD.ADANSA.WORD = 0x04;			//AN002を選択
			S12AD.ADCSR.BIT.ADST = 1;			//AD変換開始
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADSTが0になるまで待つ
			set_sen_value(RS_SEN, LED_OFF, S12AD.ADDR2);	//センサ値格納
			RS_SEN_LED = SEN_LED_ON;			//LED点灯
		break;
		
		case 4:
			//右前センサ
			S12AD.ADANSA.WORD = 0x08;			//AN003を選択
			S12AD.ADCSR.BIT.ADST = 1;			//AD変換開始
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADSTが0になるまで待つ
			set_sen_value(RF_SEN, LED_OFF, S12AD.ADDR3);	//センサ値格納
			RF_SEN_LED = SEN_LED_ON;			//LED点灯
		break;
		
		case 5:
			calc_enc_value();	//エンコーダ変化量計算
		break;
		
		case 6:
			control_speed();	//速度制御
			
		break;
		
		case 7:
			
		break;
		
		case 8:	
			
		break;
		
		default:
		break;
	}

}

//125us割り込み( *8つのスロットに分割することで1msの塊を作っている )
void mtu3_tgrb(){
	switch(mtu3_cnt){
		case 1:
			//左前センサ
			S12AD.ADANSA.WORD = 0x01;			//AN000を選択
			S12AD.ADCSR.BIT.ADST = 1;			//AD変換開始
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADSTが0になるまで待つ
			set_sen_value(LF_SEN, LED_ON, S12AD.ADDR0);	//センサ値格納
			LF_SEN_LED = SEN_LED_OFF;			//LED消灯
			calc_sen_value(LF_SEN);				//センサ値計算
		break;
		
		case 2:
			//左横センサ
			S12AD.ADANSA.WORD = 0x02;			//AN001を選択
			S12AD.ADCSR.BIT.ADST = 1;			//AD変換開始
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADSTが0になるまで待つ
			set_sen_value(LS_SEN, LED_ON, S12AD.ADDR1);	//センサ値格納
			LS_SEN_LED = SEN_LED_OFF;			//LED消灯
			calc_sen_value(LS_SEN);				//センサ値計算
		break;
		
		case 3:
			//右横センサ
			S12AD.ADANSA.WORD = 0x04;			//AN002を選択
			S12AD.ADCSR.BIT.ADST = 1;			//AD変換開始
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADSTが0になるまで待つ
			set_sen_value(RS_SEN, LED_ON, S12AD.ADDR2);	//センサ値格納
			RS_SEN_LED = SEN_LED_OFF;			//LED点灯
			calc_sen_value(RS_SEN);				//センサ値計算
		break;
		
		case 4:
			//右前センサ
			S12AD.ADANSA.WORD = 0x08;			//AN003を選択
			S12AD.ADCSR.BIT.ADST = 1;			//AD変換開始
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADSTが0になるまで待つ
			set_sen_value(RF_SEN, LED_ON, S12AD.ADDR3);	//センサ値格納
			RF_SEN_LED = SEN_LED_OFF;			//LED点灯
			calc_sen_value(RF_SEN);				//センサ値計算
			
			//電源電圧監視
			S12AD.ADANSA.WORD = 0x10;			//AN004を選択
			S12AD.ADCSR.BIT.ADST = 1;			//AD変換開始
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADSTが0になるまで待つ
			set_battery_value(S12AD.ADDR4);			//センサ値格納
		break;
		
		case 5:
			control_wall();		//壁制御
		break;
		
		case 6:
			
			pid_speed();		//PIDでDuty比計算
		break;
		
		case 7:
			
		break;
		
		case 8:
			increment_timer_ms();
			increment_my_timer_ms();
			mtu3_cnt = 0;
			
		break;
		
		default:
		break;
	}


}

//10us割り込み( PWMと同じ周期 )
void mtu4_tgrb(){
	
	change_motor_speed();	//モーター速度変化
	
}

void increment_timer_ms(){
	timer_ms ++;
	if(timer_ms >= 1000){
		increment_timer_sec();
		timer_ms = 0;
	}
	
}

void increment_my_timer_ms(){
	my_timer_ms ++;
}

void increment_timer_sec(){
	timer_sec++;
}

unsigned int get_time(char type){
	if(type == TYPE_MS){
		return timer_ms;
	}
	else if(type == TYPE_SEC){
		return timer_sec;
	}
}

void wait_sec(unsigned int sec){
	unsigned int temp_sec = get_time(TYPE_SEC);
	while(1){
		if((temp_sec + sec) - get_time(TYPE_SEC) == 0) break;
	}
}

//大抵のms待機はできるはず
void wait_ms(unsigned int ms){
	my_timer_ms = 0;
	while(my_timer_ms <= ms);
}