#include "filtro_particulas_samcl.h"

int main (int argc, char** argv)
{
	ros::init( argc, argv, "filtro_particulas_samcl_node" );

	ros::NodeHandle n;

	Filtro_Particulas_Samcl fpsamcl(n);

	fpsamcl.spin();

	return 0;

}
