/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "typedefine.h"

/*-----------------------------------*/
#include "iodefine.h"
#include "parameter.h"
#include "define_YMP.h"
#include "init_rx220.h"
#include "function_test.h"
#include "sci.h"
#include "sensor.h"
#include "encoder.h"
#include "interrupt.h"
#include "run.h"
#include "control.h"
#include "search.h"
#include "log.h"
/*-----------------------------------*/

#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif

void main(void)
{

	init_rx220();
	wait_sec(3);
	
	straight(SECTION, 0.3, 0.3, 1.0, 0);
	//slalom(90.0, 500, 0, 2000);
	//straight(SECTION, 0.3, 0, 1.0, 0);
	//straight(SECTION, 0.5, 0, 2.0, 0);
	//turn(-180.0, 700, 0, 7000);
	
	//straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 0);
	
	/*
	slalom(-90.0, 300, 0, 3000);
	*/
	//straight(SECTION, 0.5, 0, 2.0, 0);
	//slalom(90.0, TURN_OMEGA, 0, TURN_ALPHA);
	
	wait_sec(15);
	
	print_run_log();
	
	/*
	adachi_method();
	wait_sec(10);
	run_shortestRoute();
	*/
	
	/*
	while(1){
		adc_test_all();
	}
	*/
	
	while(1);
}

#ifdef __cplusplus
void abort(void)
{

}
#endif
