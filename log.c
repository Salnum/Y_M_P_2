#include "parameter.h"
#include "log.h"
#include "sci.h"
#include "encoder.h"
#include "interrupt.h"
#include "run.h"

#define LOG_SIZE	1000

float run_log[LOG_SIZE] = {0};
unsigned short log_cnt = 0;

void log_save(short value){
	if(log_cnt >= LOG_SIZE-1)	return;
	run_log[log_cnt] = value;
	log_cnt ++;
}

void print_run_log(void){
	short i;
	for(i=0;i<LOG_SIZE;i++){
		sci_printf("%d \r\n", (short)(run_log[i]));
	}
}