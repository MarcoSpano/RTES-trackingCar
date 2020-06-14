#include "../lib/car_traker.h"

int main()
{
	init();

	//launch all the functionalities of the application
	create_tasks();

	//inserting a character closes the app 
	getchar();
	close_app();

    return 0;
}