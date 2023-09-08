// -------------------------------------------------------------------------------------------------------------------
//  Copy form  Decawave Ltd, Dublin, Ireland.
//-------------------------------------------------------------------------------------------------------------------
#include "imu_uwb/fusion/imu_uwb_fusion_flow.hpp"

namespace IMU_UWB {

int InitPosition::double_equals(double a, double b)
{
	static const double ZERO = 1e-9;
	return fabs(a - b) < ZERO;
}

double InitPosition::distance_sqr(struct point_t* a, struct point_t* b)
{
	return (a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y);
}

double InitPosition::distance(struct point_t* a, struct point_t* b)
{
	return sqrt(distance_sqr(a, b));
}

int InitPosition::insect(struct circle_t circles[], struct point_t points[])
{
	double d, a, b, c, p, q, r;
	double cos_value[2], sin_value[2];
	if (double_equals(circles[0].center.x, circles[1].center.x)
		&& double_equals(circles[0].center.y, circles[1].center.y)
		&& double_equals(circles[0].r, circles[1].r))
	{
		return -1;
	}

	d = distance(&circles[0].center, &circles[1].center);
	if (d > circles[0].r + circles[1].r
		|| d < fabs(circles[0].r - circles[1].r))
	{
		return 0;
	}

	a = 2.0 * circles[0].r * (circles[0].center.x - circles[1].center.x);
	b = 2.0 * circles[0].r * (circles[0].center.y - circles[1].center.y);
	c = circles[1].r * circles[1].r - circles[0].r * circles[0].r
		- distance_sqr(&circles[0].center, &circles[1].center);
	p = a * a + b * b;
	q = -2.0 * a * c;
	if (double_equals(d, circles[0].r + circles[1].r)
		|| double_equals(d, fabs(circles[0].r - circles[1].r)))
	{
		cos_value[0] = -q / p / 2.0;
		sin_value[0] = sqrt(1 - cos_value[0] * cos_value[0]);

		points[0].x = circles[0].r * cos_value[0] + circles[0].center.x;
		points[0].y = circles[0].r * sin_value[0] + circles[0].center.y;

		if (!double_equals(distance_sqr(&points[0], &circles[1].center),
			circles[1].r * circles[1].r))
		{
			points[0].y = circles[0].center.y - circles[0].r * sin_value[0];
		}
		return 1;
	}

	r = c * c - b * b;
	cos_value[0] = (sqrt(q * q - 4.0 * p * r) - q) / p / 2.0;
	cos_value[1] = (-sqrt(q * q - 4.0 * p * r) - q) / p / 2.0;
	sin_value[0] = sqrt(1 - cos_value[0] * cos_value[0]);
	sin_value[1] = sqrt(1 - cos_value[1] * cos_value[1]);

	points[0].x = circles[0].r * cos_value[0] + circles[0].center.x;
	points[1].x = circles[0].r * cos_value[1] + circles[0].center.x;
	points[0].y = circles[0].r * sin_value[0] + circles[0].center.y;
	points[1].y = circles[0].r * sin_value[1] + circles[0].center.y;

	if (!double_equals(distance_sqr(&points[0], &circles[1].center),
		circles[1].r * circles[1].r))
	{
		points[0].y = circles[0].center.y - circles[0].r * sin_value[0];
	}
	if (!double_equals(distance_sqr(&points[1], &circles[1].center),
		circles[1].r * circles[1].r))
	{
		points[1].y = circles[0].center.y - circles[0].r * sin_value[1];
	}
	if (double_equals(points[0].y, points[1].y)
		&& double_equals(points[0].x, points[1].x))
	{
		if (points[0].y > 0)
		{
			points[1].y = -points[1].y;
		}
		else
		{
			points[0].y = -points[0].y;
		}
	}
	return 2;
}


void InitPosition::Cross_Point(struct circle_t circles[], struct point_t Location[])
{
	int cross_num = 5;
	struct point_t cross_points[2];
	cross_num = insect(circles, cross_points);// 0 1

	if (cross_num == 2)
	{
		double points_AC_0 = distance(&cross_points[0], &circles[2].center);
		double points_AC_1 = distance(&cross_points[1], &circles[2].center);

		if (abs((int)(points_AC_0 - circles[2].r) )< abs((int)(points_AC_1 - circles[2].r)))//cross_point[0]
		{
			Location[0].x = cross_points[0].x;
			Location[0].y = cross_points[0].y;
		}
		else
		{
			Location[0].x = cross_points[1].x;
			Location[0].y = cross_points[1].y;
		}

	}
	else if (cross_num == 1 || cross_num == 0)
	{
		Location[0].x = cross_points[0].x;
		Location[0].y = cross_points[0].y;
	}
}

double InitPosition::norm(struct point p) // get the norm of a vector 求向量的范数
{
	return pow(pow(p.x, 2) + pow(p.y, 2), 0.5);
}

Eigen::Vector3d InitPosition::trilateration(struct point point1, struct point point2, struct point point3, double r1, double r2, double r3) {
	struct point resultPose;
	//unit vector in a direction from point1 to point 2  从点1到点2方向上的单位向量
	double p2p1Distance = pow(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2), 0.5);
	struct point ex = { (point2.x - point1.x) / p2p1Distance, (point2.y - point1.y) / p2p1Distance };
	struct point aux = { point3.x - point1.x,point3.y - point1.y };
	//signed magnitude of the x component  x分量的符号大小
	double i = ex.x * aux.x + ex.y * aux.y;
	//the unit vector in the y direction.  y方向的单位向量。T
	struct point aux2 = { point3.x - point1.x - i * ex.x, point3.y - point1.y - i * ex.y };
	struct point ey = { aux2.x / norm(aux2), aux2.y / norm(aux2) };
	//the signed magnitude of the y component  y分量的符号大小
	double j = ey.x * aux.x + ey.y * aux.y;
	//coordinates  协调
	double x = (pow(r1, 2) - pow(r2, 2) + pow(p2p1Distance, 2)) / (2 * p2p1Distance);
	double y = (pow(r1, 2) - pow(r3, 2) + pow(i, 2) + pow(j, 2)) / (2 * j) - i * x / j;
	//result coordinates   结果坐标
	double finalX = point1.x + x * ex.x + y * ey.x;
	double finalY = point1.y + x * ex.y + y * ey.y;
	resultPose.x = finalX;
	resultPose.y = finalY;

	//std::cout<<"x ="<<resultPose.x<<"y ="<<resultPose.y;//打印未知点坐标
    Eigen::Vector3d tag_position;
	tag_position<<resultPose.x,resultPose.y,0;

	return tag_position;

}
Eigen::Vector3d InitPosition::InitUwbTagPosition(double dis_0, double dis_1, double dis_2,int uwb_id_0, int uwb_id_1, int uwb_id_2)
{
    struct point points[3];
    //基站0，1，2
    std::cout<<"uwb_id_0 "<<uwb_id_0<<std::endl;
    std::cout<<"uwb_id_1 "<<uwb_id_1<<std::endl;
    std::cout<<"uwb_id_2 "<<uwb_id_2<<std::endl;

    points[0].x= uwb_stations_position[uwb_id_0](0);
    points[0].y =uwb_stations_position[uwb_id_0](1);

    points[1].x= uwb_stations_position[uwb_id_1](0);
    points[1].y =uwb_stations_position[uwb_id_1](1);

    points[2].x= uwb_stations_position[uwb_id_2](0);
    points[2].y =uwb_stations_position[uwb_id_2](1);
    //std::cout<< "uwb station position 0 x "<<points[0].x<<std::endl;
    //std::cout<< "uwb station position 0 y "<<points[0].y<<std::endl;
    //std::cout<< "uwb station position 1 x "<<points[1].x<<std::endl;
    //std::cout<< "uwb station position 1 y "<<points[1].y<<std::endl;
    //std::cout<< "uwb station position 2 x "<<points[2].x<<std::endl;
    //std::cout<< "uwb station position 2 y "<<points[2].y<<std::endl;
    //std::cout<<"dis_0 "<<dis_0<<std::endl;
    //std::cout<<"dis_1 "<<dis_1<<std::endl;
    //std::cout<<"dis_2 "<<dis_2<<std::endl;

    init_position=trilateration(points[0], points[1], points[2], dis_0, dis_1, dis_2);
    return init_position;

}
void InitPosition::ReadUwbPosion(){
    std::string pg_path = data_dir_path_ + "/uwb_settled_postions.txt";
        std::FILE* pg_ifile = std::fopen(pg_path.c_str(),"r");
        if (pg_ifile == NULL) {
            std::cout << "Error ! Open [ " << pg_path << " ] failed ..." << std::endl;
            return;
        }
        int i;
        double px, py, pz;
        Eigen::Vector3d temp_position;
        while(std::fscanf(pg_ifile,"%d %lf %lf %lf",&i,&px,&py,&pz)!=EOF){
            //std::cout << "i " << i <<std::endl;
            temp_position<< px,py,pz;
            //std::cout<<"uwb stations position "<<temp_position.transpose()<<std::endl;
            uwb_stations_position.push_back(temp_position);
        }
        std::fclose(pg_ifile);
        std::cout<<"successful read uwb station position"<<std::endl;

}


}