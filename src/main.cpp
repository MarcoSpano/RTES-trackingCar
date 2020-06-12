#include "../lib/carTraker.h"

using namespace std;
using namespace cv;

int main()
{
	//launch all the functionalities of the application
	create_tasks();

	//inserting a character closes the app 
	getchar();
	close_app();

    return 0;
}