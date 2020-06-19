#include "../lib/car_traker.h"

//------------------------------------------------------------------------------
//							MAIN APPLICATION	
//------------------------------------------------------------------------------

/**
 * initialises the resources, creates the tasks and manages the shutdown
 */
int main() {

	init();

	//create and launch all the functionalities of the application
	create_tasks();

	//inserting a character closes the app 
	getchar();
	close_app();

    return 0;
}