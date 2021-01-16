#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <linux/input-event-codes.h>
#include <vector>

#include <ros/time.h>
#include "ros/package.h"
#include "math.h"
#include "myData.h"
#include "myFunction.h"
#include "origarm_ros/Sensor.h"
#include "origarm_ros/States.h"
#include "origarm_ros/keynumber.h"
#include "ros/ros.h"
#include <time.h>
#include <cstdlib>
#include <stdio.h>

using namespace Eigen;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::MatrixXf;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using Eigen::VectorXf;

using namespace std;

// #define GET_VARIABLE_NAME(Variable) (#Variable)

static float rad2deg = 180.0f / M_PI;
static float deg2rad = M_PI / 180.0f;

// given data read from file
ifstream inFile;
string GivenDataFilePath = ros::package::getPath("origarm_ros") + "/data/GridData/";
string GivenDataFileName = "Quadrant1.txt";
string name = "";

// saving data into file
ofstream outFile;
string SaveDataFilePath = ros::package::getPath("origarm_ros") + "/data/GridData/";
string SaveDataFileName = "";

// given data 2d vector[N][13], N = max(dataNo)
static vector< vector<float> > givenData;

// given PosGrid & OrtGrid, both expressions are necessary
static vector<float> PosCarGrid_x{-65.6539,   -7.8542,   49.9456,  107.7453,  165.5451,  223.3448};
static vector<float> PosCarGrid_y{-72.6837,  -15.4107,   41.8623,   99.1354,  156.4084,  213.6814};
static vector<float> PosCarGrid_z{182.2847,  230.6516,  279.0186,  327.3855,  375.7524};
static vector<float> OrtSphGrid_omega{0, 0.7879, 1.5758, 2.3637, 3.1516};
static vector<float> OrtSphGrid_phi  {0, 0.7879, 1.5758, 2.3637, 3.1516};
static vector<float> OrtSphGrid_theta{0, 1.5733, 3.1466, 4.7199, 6.2932};

static vector<vector<float>> Grid_group{    
    {-65.6539,   -7.8542,   49.9456,  107.7453,  165.5451,  223.3448},
    {-72.6837,  -15.4107,   41.8623,   99.1354,  156.4084,  213.6814},
    {182.2847,  230.6516,  279.0186,  327.3855,  375.7524},
    {      0,  0.7879, 1.5758, 2.3637, 3.1516},
    {      0, 0.7879, 1.5758, 2.3637,  3.1516},
    {      0, 1.5733, 3.1466, 4.7199,  6.2932}};

int Grid_size[6] = {5, 5, 4, 4, 4, 4};

// given workspace {{xmin, ..., thetamin}, {xmax,...,thetamax}}
// static float workspace[2][6] = 
//     {{-0.3457, -0.3457, 0.0744, 1.0955,      0,      0},
//      { 0.3457,  0.3457, 0.4784, 2.0461, 3.0000, 6.0000}};

static float workspace[2][6] = 
    {{-65.6539, -72.6837, 182.2847,      0,      0,      0},
     {223.3448, 213.6814, 375.7524, 3.0000, 3.0000, 6.0000}};

// DataGridTable 4d vector DataGridTable[i][j][k] = dataNo1, dataNo2, ...
static vector< vector< vector< vector<int> > > > PosDataGridTable;
static vector< vector< vector< vector<int> > > > OrtDataGridTable;

// given target position & orientation
Eigen::Vector3f    target_pos(0.000000, 0.000000, 330.000);
Eigen::Quaternionf target_quat(1.000000, 0.000000, 0.000000, 0.000000); // initilization as :(qw,qx,qy,qz)
Eigen::Vector3f    target_OrtSphCoord;
Eigen::Vector3f    target_OrtCarCoord;
vector<int> target_gridno;
vector<int> solution_candidate;
int solution_selected;
float ep = 0.5;
float eo = 0.5;
int flag_sol = 1; // flag_sol == 1: solution found, == 0: not found

const int ms = 1000;  //1ms
int ts[] = {10*ms, 3000*ms};
int datanumber = 0;
int flag_start = 0;
int repeat = 3;
int cyclenumber = 0;

// finally selected ABL based on grids and given data
Eigen::Vector3f    selected_pos;
Eigen::Quaternionf selected_quat;
float alpha_s[2] = {0, 0};
float beta_s[2]  = {0, 0};
float length_s[2]= {length0*3, length0*3};

float alpha_prev[2] = {0, 0};
float beta_prev[2]  = {0, 0};
float length_prev[2]= {length0*3, length0*3};

// distance between selected & target pose
float position_distance;
float orientation_distance;
float weightedsum_distance;

//keyboard callback
void keyCallback(const origarm_ros::keynumber &key)
{
	if (key.keycodePressed == KEY_B) // break and reset all command pressure to 0
	{		
		printf("KEY_B pressed!\r\n");
		flag_start = 0;
	}
	else if (key.keycodePressed == KEY_J) // same as saving button
	{
		printf("KEY_J pressed!\r\n");
		flag_start = 1;
	}	
}

//getcurrent time for evaluating computational efficiency
time_t curr_time;
time_t diff_time;
ros::Time BeginTime;
static time_t getCurrentTime()
{
	time_t rawtime;
	struct tm *timeinfo;
    time_t curr_time;

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	struct timeval time_now
	{
	};
	gettimeofday(&time_now, nullptr);
	time_t msecs_time = time_now.tv_usec;
	time_t msec = (msecs_time / 1000) % 1000;

    curr_time = msecs_time;
    return curr_time;
} 

std::string getTimeString()
{
	time_t rawtime;
	struct tm *timeinfo;
	char buffer[100];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	struct timeval time_now
	{
	};
	gettimeofday(&time_now, nullptr);
	time_t msecs_time = time_now.tv_usec;
	time_t msec = (msecs_time / 1000) % 1000;

	strftime(buffer, 100, "%G_%h_%d_%H_%M_%S", timeinfo);
	std::string ret1 = buffer;
	std::string ret = ret1 + "_" + std::to_string(msec);
	return ret;
}

std::string getTimeNsecString()
{
	ros::Time curtime = ros::Time::now();
	ros::Duration pasttime = curtime - BeginTime;
	int64_t pasttimems = pasttime.toNSec() / 1000000;
	std::string ret = std::to_string(pasttimems);
	return ret;
}

//Coord conversion: quat2OrtCarCoord
static Eigen::Vector3f quat2OrtCarCoord(Eigen::Quaternionf& quat)
{
    Eigen::Vector3f OrtCartesianCoord;
    Eigen::Matrix3f rotm(quat);
    Eigen::AngleAxisf axang(rotm);

    float angle = axang.angle();
    Eigen::Vector3f axis = axang.axis();

    float new_angle;
    Eigen::Vector3f new_axis;

    if (axis(1) < 0 && angle < 0)
    {
        new_angle = -1 * angle;
        new_axis(0) = -1* axis(0); 
        new_axis(1) = -1* axis(1);
        new_axis(2) = -1* axis(2); 
    }
    else if (axis(1) < 0 && angle > 0)
    {
        new_angle = 2*M_PI - angle;
        new_axis(0) = -1* axis(0); 
        new_axis(1) = -1* axis(1);
        new_axis(2) = -1* axis(2); 
    }
    else if (axis(1) > 0 && angle < 0)
    {
        new_angle = 2*M_PI + angle;
        new_axis(0) = axis(0); 
        new_axis(1) = axis(1);
        new_axis(2) = axis(2); 
    }
    else
    {
        new_angle = angle;
        new_axis(0) = axis(0); 
        new_axis(1) = axis(1);
        new_axis(2) = axis(2);
    }
    
    OrtCartesianCoord << (new_angle/(2*M_PI))*new_axis(0),
                      (new_angle/(2*M_PI))*new_axis(1),
                      (new_angle/(2*M_PI))*new_axis(2);

    return OrtCartesianCoord;                      

}

//Coord conversion: OrtCarCoord2quat
static Eigen::Quaternionf OrtCarCoord2quat(Eigen::Vector3f& OrtCarCoord)
{
    Eigen::Quaternionf quat;
    
    float theta = OrtCarCoord.norm()*2*M_PI;
    if (abs(theta) < 1e-6)
    {
        quat.w() = 1;
        quat.x() = 0;
        quat.y() = 0;
        quat.z() = 0;
    }
    else
    {
        Eigen::AngleAxisf axang(theta, OrtCarCoord);
        Eigen::Matrix3f   rotm;

        rotm = axang;
        quat = rotm;
        // quat.w() = cos(theta/2);
        // quat.x() = sin(theta/2) * OrtCarCoord.x();
        // quat.y() = sin(theta/2) * OrtCarCoord.y();
        // quat.z() = sin(theta/2) * OrtCarCoord.z();
    }
    
    return quat;
}

//Coord conversion: Cartesian2Sphere, return [omega; phi; normalized theta]
static Eigen::Vector3f Cartesian2Sphere(Eigen::Vector3f& CartesianCoord)
{
    Eigen::Vector3f SphereCoord;
    float magnitude = CartesianCoord.norm();
    float omega, phi;

    if (abs(magnitude) < 1e-6)
    {
        omega = 0;
        phi = 0;
        magnitude = 0;
    }
    else
    {
        omega = acos(CartesianCoord(2)/magnitude); //acos return value in interval [0,pi] radians
        float somega = sin(omega);

        if (abs(omega) < 1e-10)
        {
            phi = 0;
        }
        else
        {
            phi = acos(CartesianCoord(0)/(magnitude*somega));
        }        
    }
    
    SphereCoord << omega,
                   phi,
                   magnitude;

    return SphereCoord;
}

//Coord conversion: Sphere2Cartesian, return [x, y, z]
static Eigen::Vector3f Sphere2Cartesian(Eigen::Vector3f& SphereCoord)
{
    Eigen::Vector3f CartesianCoord;
    float omega = SphereCoord.x();
    float phi = SphereCoord.y();
    float norm_theta = SphereCoord.z();
    CartesianCoord.x() = norm_theta*sin(omega)*cos(phi);
    CartesianCoord.y() = norm_theta*sin(omega)*sin(phi);
    CartesianCoord.z() = norm_theta*cos(omega);

    return CartesianCoord;
}

//read given data from File and save into a vector
static void readFromDataFile()
{
	inFile.open(GivenDataFilePath + GivenDataFileName, ios::in);

    float group_readin[13];
    vector<float> group;
    vector<float> group_latest;

	if (!inFile)
	{
		printf("%s\n", "unable to open the file.");
		exit(1);
	}
	else
	{
		while (true)
		{
			inFile >> group_readin[0] >> group_readin[1] >> group_readin[2] 
                   >> group_readin[3] >> group_readin[4] >> group_readin[5] 
                   >> group_readin[6] >> group_readin[7] >> group_readin[8] 
                   >> group_readin[9] >> group_readin[10] >> group_readin[11] >> group_readin[12];

			if ( inFile.eof() )	
			{
				break;
			}	

            for (int i = 0; i < 13; i++)
            {
                group.push_back(group_readin[i]);
            }                      

			if (group.size() < 13)
			{
				printf("%s\n", "Wrong with input file!");
                exit(1);
			}
			else
			{
				group_latest.assign(group.end()-13, group.end());				
				givenData.push_back(group_latest);
			}	
		}		
	}

	inFile.close();	
}

//determine which grid the point located into (in terms of 1 D) 
static int GridNo(float target, vector<float> & gridarray)
{
    int gridNo = -1;
    int arraysize = gridarray.size();

    // printf("target: %f\r\n", target);
    if (arraysize <= 1)
    {
        printf("%s\n", "Wrong definition of input grids.");
		exit(1);
    }
    else
    {
        for (int i = 0; i < arraysize - 1; i++)
        {
            if (target >= gridarray[i] && target < gridarray[i+1])
            {
                gridNo = i;
                break;
            }
        }       
    }

    if (gridNo == -1)
    {
        printf("target: %f\r\n", target);
        printf("%s\n", "target out of workspace.");
        exit(1);
    } 

    return gridNo;       
}

//determine which grid the point located into (in terms of 6 D)
static vector<int> GridNoArray(Eigen::Vector3f& pos_t, Eigen::Vector3f& ort_t, vector<vector<float>> & gridarray)
{
    vector<int> gridNoArray;
    vector<float> target{pos_t.x(), pos_t.y(), pos_t.z(), ort_t.x(), ort_t.y(), ort_t.z()};

    for (int i = 0; i < target.size(); i++)
    {        
        int no = GridNo(target[i], gridarray[i]);
        gridNoArray.push_back(no);       
    }

    return gridNoArray;
}

//generate DataGridTable
static void generateDataGridTable()
{
    int rowsize = givenData.size();
    int px_No[rowsize], py_No[rowsize], pz_No[rowsize], ox_No[rowsize], oy_No[rowsize], oz_No[rowsize];

    for (int dataNo = 0; dataNo < rowsize; dataNo ++)
    {
        px_No[dataNo] = GridNo(givenData[dataNo][6], PosCarGrid_x);    
        py_No[dataNo] = GridNo(givenData[dataNo][7], PosCarGrid_y); 
        pz_No[dataNo] = GridNo(givenData[dataNo][8], PosCarGrid_z); 

        // Coord conversion
        Eigen::Vector3f OrtCartesianCoord, OrtSphereCoord;
        Eigen::Quaternionf quat(givenData[dataNo][12], givenData[dataNo][9], givenData[dataNo][10], givenData[dataNo][11]);
        OrtCartesianCoord = quat2OrtCarCoord(quat);
        OrtSphereCoord = Cartesian2Sphere(OrtCartesianCoord);

        ox_No[dataNo] = GridNo(OrtSphereCoord(0), OrtSphGrid_omega); 
        oy_No[dataNo] = GridNo(OrtSphereCoord(1), OrtSphGrid_phi); 
        oz_No[dataNo] = GridNo(OrtSphereCoord(2), OrtSphGrid_theta); 

        // if (px_No[dataNo] == -1 || py_No[dataNo] == -1 || pz_No[dataNo] == -1 || ox_No[dataNo] == -1 || oy_No[dataNo] == -1 || oz_No[dataNo] == -1)  
        // {
        //     printf("wrong gridNo: dataNo: %d\r\n", dataNo);           
        // }     

        for (int dimension_x = 0; dimension_x < Grid_size[0]; dimension_x ++)
        {
            vector<vector<vector<int>>> gridNo_x; 
            PosDataGridTable.push_back(gridNo_x);
            for (int dimension_y = 0; dimension_y < Grid_size[1]; dimension_y ++)
            {
                vector<vector<int>> gridNo_y;
                PosDataGridTable[dimension_x].push_back(gridNo_y);
                for (int dimension_z = 0; dimension_z < Grid_size[2]; dimension_z ++)
                {
                    vector<int> gridNo_z;
                    PosDataGridTable[dimension_x][dimension_y].push_back(gridNo_z);
                    if (dimension_x == px_No[dataNo] && dimension_y == py_No[dataNo] && dimension_z == pz_No[dataNo])
                    {
                        PosDataGridTable[dimension_x][dimension_y][dimension_z].push_back(dataNo);
                    }                                       
                }
            }
        }

        for (int dimension_ox = 0; dimension_ox < Grid_size[3]; dimension_ox ++)
        {
            vector<vector<vector<int>>> gridNo_ox; 
            OrtDataGridTable.push_back(gridNo_ox);
            for (int dimension_oy = 0; dimension_oy < Grid_size[4]; dimension_oy ++)
            {
                vector<vector<int>> gridNo_oy;
                OrtDataGridTable[dimension_ox].push_back(gridNo_oy);
                for (int dimension_oz = 0; dimension_oz < Grid_size[5]; dimension_oz ++)
                {
                    vector<int> gridNo_oz;
                    OrtDataGridTable[dimension_ox][dimension_oy].push_back(gridNo_oz);
                    if (dimension_ox == ox_No[dataNo] && dimension_oy == oy_No[dataNo] && dimension_oz == oz_No[dataNo])
                    {
                        OrtDataGridTable[dimension_ox][dimension_oy][dimension_oz].push_back(dataNo);
                    }                   
                }
            }
        }
    }  
}

//randomly generate target pose, case 0:selecting from givendata, case1: close to givendata, case2: randomly pick in workspace
static void generatetarget(int casen, int rand_no[7], float distp, float disto)
{
    if (casen == 0)
    {
        int n = rand_no[0];
        target_pos.x() = givenData[n][6];
        target_pos.y() = givenData[n][7];
        target_pos.z() = givenData[n][8];
        target_quat.w() = givenData[n][12];
        target_quat.x() = givenData[n][9];
        target_quat.y() = givenData[n][10];
        target_quat.z() = givenData[n][11];
    }
    else if (casen == 1)
    {
        int n = rand_no[0];

        float p1 = (rand_no[1]/1000)*2*M_PI;
        float p2 = (rand_no[2]/1000)*2*M_PI;
        float p3 = (rand_no[3]/1000)*distp;
        float o1 = (rand_no[4]/1000)*2*M_PI;
        float o2 = (rand_no[5]/1000)*2*M_PI;
        float o3 = (rand_no[6]/1000)*disto;

        Eigen::Vector3f pos_temp;
        Eigen::Vector3f ort_temp;
        Eigen::Vector3f ortCarCoord_t;
        Eigen::Vector3f ortCarCoord_temp;
        Eigen::Quaternionf ortquat_t(givenData[n][12], givenData[n][9], givenData[n][10], givenData[n][11]);

        pos_temp.x() = p3*sin(p1)*cos(p2);
        pos_temp.y() = p3*sin(p1)*sin(p2);
        pos_temp.z() = p3*cos(p1);

        ort_temp.x() = o3*sin(o1)*cos(o2);
        ort_temp.y() = o3*sin(o1)*sin(o2);
        ort_temp.z() = o3*cos(o1);

        // printf("orientation_dist: %.8f, ort_temp magnitude: %.8f\r\n", o3, ort_temp.norm());

        target_pos.x() = givenData[n][6] + pos_temp.x();
        target_pos.y() = givenData[n][7] + pos_temp.y();
        target_pos.z() = givenData[n][8] + pos_temp.z();

        ortCarCoord_temp = quat2OrtCarCoord(ortquat_t);
        ortCarCoord_t.x() = ortCarCoord_temp.x() + ort_temp.x();
        ortCarCoord_t.y() = ortCarCoord_temp.y() + ort_temp.y();
        ortCarCoord_t.z() = ortCarCoord_temp.z() + ort_temp.z();

        target_quat = OrtCarCoord2quat(ortCarCoord_t);
    }
    else if (casen == 2)
    {
        float px = workspace[0][0] + (rand_no[1]/1000) *(workspace[1][0]-workspace[0][0]);
        float py = workspace[0][1] + (rand_no[2]/1000) *(workspace[1][1]-workspace[0][1]);
        float pz = workspace[0][2] + (rand_no[3]/1000) *(workspace[1][2]-workspace[0][2]);
        float o_omega = workspace[0][3] + (rand_no[4]/1000) *(workspace[1][3]-workspace[0][3]);
        float o_phi   = workspace[0][4] + (rand_no[5]/1000) *(workspace[1][4]-workspace[0][4]);
        float o_theta = workspace[0][5] + (rand_no[6]/1000) *(workspace[1][5]-workspace[0][5]);

        target_pos.x() = px;
        target_pos.y() = py;
        target_pos.z() = pz;

        Eigen::Vector3f ortSphCoord_t;
        Eigen::Vector3f ortCarCoord_t;
        ortSphCoord_t.x() = o_omega;
        ortSphCoord_t.y() = o_phi;
        ortSphCoord_t.z() = o_phi;

        ortCarCoord_t = Sphere2Cartesian(ortSphCoord_t);
        target_quat = OrtCarCoord2quat(ortCarCoord_t);
    }
}

//generate target from specified groups
static void getTarget()
{
    // xy line
    // vector<vector<float>> pose{
    //     { -4.9128,    2.8308,   23.7181,   31.2469,   38.6679,   45.3325,   49.4207,   54.2969,   61.2286,   75.5831,  61.2286,   54.2969,  49.4207,   45.3325,   38.6679,   31.2469,   23.7181,    2.8308,   -4.9128},
    //     { -7.4833,    7.0717,   34.7918,   44.4003,   55.7579,   63.3405,   72.7515,   81.4504,   97.3609,  122.9970,  97.3609,   81.4504,  72.7515,   63.3405,   55.7579,   44.4003,   34.7918,    7.0717,   -7.4833},
    //     {286.3539,  286.3228,  288.8755,  287.4387,  284.4454,  284.9149,  283.6367,  282.8943,  287.7892,  289.4274, 287.7892,  282.8943, 283.6367,  284.9149,  284.4454,  287.4387,  288.8755,  286.3228,  286.3539},
    //     { 0.9981,    0.9963,    0.9981,    0.9837,    0.9784,    0.9975,    0.9569,    0.9965,    0.9590,    0.9548,    0.9590,    0.9965,   0.9569,    0.9975,    0.9784,    0.9837,    0.9981,    0.9963,    0.9981},
    //     { 0.0513,   -0.0828,   -0.0411,   -0.1507,   -0.1267,    0.0624,   -0.2554,   -0.0701,   -0.2031,   -0.2567,   -0.2031,   -0.0701,  -0.2554,    0.0624,   -0.1267,   -0.1507,   -0.0411,   -0.0828,    0.0513},
    //     {-0.0341,    0.0246,    0.0457,    0.0975,   -0.1603,   -0.0042,    0.1380,    0.0421,    0.1948,    0.1496,    0.1948,    0.0421,   0.1380,   -0.0042,   -0.1603,    0.0975,    0.0457,    0.0246,   -0.0341},
    //     { 0.0085,   -0.0024,    0.0096,    0.0114,    0.0303,    0.0341,   -0.0028,   -0.0152,   -0.0349,   -0.0091,   -0.0349,   -0.0152,  -0.0028,    0.0341,    0.0303,    0.0114,    0.0096,   -0.0024,    0.0085}
    // };

    // vector<vector<float>> pose{
    //     {     -5,         3,        24,        31,        39,        45,        49,        54,        61,        76,      61,        54,        49,        45,        39,        31,        24,         3,        -5},
    //     {     -7,         7,        35,        44,        56,        63,        73,        81,        97,       123,      97,        81,        73,        63,        56,        44,        35,         7,        -7},
    //     {    287,       287,       287,       287,       287,       287,       287,       287,       287,       287,     287,       287,       287,       287,       287,       287,       287,       287,       287},
    //     { 0.9981,    0.9963,    0.9981,    0.9837,    0.9784,    0.9975,    0.9569,    0.9965,    0.9590,    0.9548,  0.9590,    0.9965,    0.9569,    0.9975,    0.9784,    0.9837,    0.9981,    0.9963,    0.9981},
    //     { 0.0513,   -0.0828,   -0.0411,   -0.1507,   -0.1267,    0.0624,   -0.2554,   -0.0701,   -0.2031,   -0.2567, -0.2031,   -0.0701,   -0.2554,    0.0624,   -0.1267,   -0.1507,   -0.0411,   -0.0828,    0.0513},
    //     {-0.0341,    0.0246,    0.0457,    0.0975,   -0.1603,   -0.0042,    0.1380,    0.0421,    0.1948,    0.1496,  0.1948,    0.0421,    0.1380,   -0.0042,   -0.1603,    0.0975,    0.0457,    0.0246,   -0.0341},
    //     { 0.0085,   -0.0024,    0.0096,    0.0114,    0.0303,    0.0341,   -0.0028,   -0.0152,   -0.0349,   -0.0091, -0.0349,   -0.0152,   -0.0028,    0.0341,    0.0303,    0.0114,    0.0096,   -0.0024,    0.0085}
    // };

    // xz line
    // vector<vector<float>> pose{
    //     {-22.5556,  -10.7705,   -4.0489,    3.3324,    8.5535,   10.9776,   28.1908,   35.9824,   44.4595,   54.9354,  44.4595,   35.9824,   28.1908,   10.9776,    8.5535,    3.3324,   -4.0489,  -10.7705,  -22.5556},
    //     { 69.5237,   70.3328,   66.8795,   66.2239,   73.6699,   71.8197,   67.9891,   71.5107,   70.4459,   67.0506,  70.4459,   71.5107,   67.9891,   71.8197,   73.6699,   66.2239,   66.8795,   70.3328,   69.5237},
    //     {300.2442,  301.5906,  300.8895,  301.5647,  300.8822,  301.0794,  301.4203,  300.6809,  300.8366,  301.0689, 300.8366,  300.6809,  301.4203,  301.0794,  300.8822,  301.5647,  300.8895,  301.5906,  300.2442},
    //     { 0.9859,    0.9428,    0.9949,    0.9958,    0.9741,    0.9814,    0.9945,    0.9989,    0.9723,    0.9766,    0.9723,    0.9989,    0.9945,    0.9814,    0.9741,    0.9958,    0.9949,    0.9428,    0.9859},
    //     { 0.0546,   -0.1278,   -0.1005,    0.0878,   -0.1427,   -0.1640,   -0.1033,    0.0379,   -0.1501,   -0.1481,   -0.1501,    0.0379,   -0.1033,   -0.1640,   -0.1427,    0.0878,   -0.1005,   -0.1278,    0.0546},
    //     {-0.1494,   -0.3033,   -0.0078,    0.0134,   -0.1737,    0.0969,   -0.0053,   -0.0268,    0.1778,   -0.1492,    0.1778,   -0.0268,   -0.0053,    0.0969,   -0.1737,    0.0134,   -0.0078,   -0.3033,   -0.1494},
    //     { 0.0513,    0.0535,   -0.0061,   -0.0229,    0.0224,   -0.0257,   -0.0166,   -0.0090,   -0.0206,    0.0450,   -0.0206,   -0.0090,   -0.0166,   -0.0257,    0.0224,   -0.0229,   -0.0061,    0.0535,    0.0513}
    // };

    // vector<vector<float>> pose{
    //     {    -23,       -11,        -4,         3,         9,        11,        28,        36,        44,        55,      44,        36,        28,        11,         9,         3,        -4,       -11,       -23},
    //     {     70,        70,        70,        70,        70,        70,        70,        70,        70,        70,      70,        70,        70,        70,        70,        70,        70,        70,        70},
    //     {    300,       302,       301,       302,       301,       301,       301,       301,       301,       301,     301,       301,       301,       301,       301,       302,       301,       302,       300},
    //     { 0.9859,    0.9428,    0.9949,    0.9958,    0.9741,    0.9814,    0.9945,    0.9989,    0.9723,    0.9766,  0.9723,    0.9989,    0.9945,    0.9814,    0.9741,    0.9958,    0.9949,    0.9428,    0.9859},
    //     { 0.0546,   -0.1278,   -0.1005,    0.0878,   -0.1427,   -0.1640,   -0.1033,    0.0379,   -0.1501,   -0.1481, -0.1501,    0.0379,   -0.1033,   -0.1640,   -0.1427,    0.0878,   -0.1005,   -0.1278,    0.0546},
    //     {-0.1494,   -0.3033,   -0.0078,    0.0134,   -0.1737,    0.0969,   -0.0053,   -0.0268,    0.1778,   -0.1492,  0.1778,   -0.0268,   -0.0053,    0.0969,   -0.1737,    0.0134,   -0.0078,   -0.3033,   -0.1494},
    //     { 0.0513,    0.0535,   -0.0061,   -0.0229,    0.0224,   -0.0257,   -0.0166,   -0.0090,   -0.0206,    0.0450, -0.0206,   -0.0090,   -0.0166,   -0.0257,    0.0224,   -0.0229,   -0.0061,    0.0535,    0.0513}
    // };

    // yz line
    // vector<vector<float>> pose{
    //     { 67.7210,   70.5729,   66.0869,   73.0463,   72.2866,   69.0400,   65.0088,   68.5409,   69.6764,   67.6263,   70.4315,  67.6263,   69.6764,   68.5409,   65.0088,   69.0400,   72.2866,   73.0463,   66.0869,   70.5729,   67.7210},
    //     {-21.0424,  -15.4553,    2.1864,   16.3502,   22.3324,   31.7347,   41.9415,   59.8508,   82.2219,  127.3857,  151.4255, 127.3857,   82.2219,   59.8508,   41.9415,   31.7347,   22.3324,   16.3502,    2.1864,  -15.4553,  -21.0424},
    //     {285.3869,  286.7497,  288.4001,  289.1978,  288.2873,  288.5113,  288.2698,  289.5599,  290.4155,  288.7492,  286.9740, 288.7492,  290.4155,  289.5599,  288.2698,  288.5113,  288.2873,  289.1978,  288.4001,  286.7497,  285.3869},
    //     { 0.9845,    0.9757,    0.9822,    0.9796,    0.9772,    0.9774,    0.9589,    0.9414,    0.9307,    0.9599,    0.9372,    0.9599,    0.9307,    0.9414,    0.9589,    0.9774,    0.9772,    0.9796,    0.9822,    0.9757,    0.9845},
    //     { 0.0376,   -0.0166,    0.0960,   -0.0553,   -0.0839,   -0.1240,   -0.0971,   -0.2204,   -0.2628,   -0.2530,   -0.3171,   -0.2530,   -0.2628,   -0.2204,   -0.0971,   -0.1240,   -0.0839,   -0.0553,    0.0960,   -0.0166,    0.0376},
    //     { 0.1708,    0.2181,    0.1542,    0.1924,    0.1947,    0.1711,    0.2651,    0.2553,    0.2544,    0.1209,    0.1452,    0.1209,    0.2544,    0.2553,    0.2651,    0.1711,    0.1947,    0.1924,    0.1542,    0.2181,    0.1708},
    //     {-0.0128,   -0.0084,   -0.0478,   -0.0155,   -0.0124,   -0.0086,   -0.0260,    0.0077,    0.0005,   -0.0007,    0.0016,   -0.0007,    0.0005,    0.0077,   -0.0260,   -0.0086,   -0.0124,   -0.0155,   -0.0478,   -0.0084,   -0.0128}
    // };

    vector<vector<float>> pose{
        {     70,        70,        70,        70,        70,        70,        70,        70,        70,        70,        70,      70,        70,        70,        70,        70,        70,        70,        70,        70,        70},
        {    -21,     -15.5,         2,        16,        22,        32,        42,        60,        82,       127,       151,     127,        82,        60,        42,        32,        22,        16,         2,     -15.5,       -21},
        {    285,       287,       288,       289,       288,       289,       288,       290,       290,       289,       287,     289,       290,       290,       288,       289,       288,       289,       288,       287,       285},
        { 0.9845,    0.9757,    0.9822,    0.9796,    0.9772,    0.9774,    0.9589,    0.9414,    0.9307,    0.9599,    0.9372,  0.9599,    0.9307,    0.9414,    0.9589,    0.9774,    0.9772,    0.9796,    0.9822,    0.9757,    0.9845},
        { 0.0376,   -0.0166,    0.0960,   -0.0553,   -0.0839,   -0.1240,   -0.0971,   -0.2204,   -0.2628,   -0.2530,   -0.3171, -0.2530,   -0.2628,   -0.2204,   -0.0971,   -0.1240,   -0.0839,   -0.0553,    0.0960,   -0.0166,    0.0376},
        { 0.1708,    0.2181,    0.1542,    0.1924,    0.1947,    0.1711,    0.2651,    0.2553,    0.2544,    0.1209,    0.1452,  0.1209,    0.2544,    0.2553,    0.2651,    0.1711,    0.1947,    0.1924,    0.1542,    0.2181,    0.1708},
        {-0.0128,   -0.0084,   -0.0478,   -0.0155,   -0.0124,   -0.0086,   -0.0260,    0.0077,    0.0005,   -0.0007,    0.0016, -0.0007,    0.0005,    0.0077,   -0.0260,   -0.0086,   -0.0124,   -0.0155,   -0.0478,   -0.0084,   -0.0128}
    };
             
    // given data
    if (flag_start)
    {
        if (datanumber < pose[0].size() && cyclenumber < repeat)   
        {
            target_pos.x() = pose[0][datanumber];
            target_pos.y() = pose[1][datanumber];
            target_pos.z() = pose[2][datanumber];
            target_quat.w() = pose[3][datanumber];
            target_quat.x() = pose[4][datanumber];
            target_quat.y() = pose[5][datanumber];
            target_quat.z() = pose[6][datanumber];
            usleep(ts[1]);
            datanumber = datanumber + 1;
        }     
        else if (cyclenumber < repeat)
        {
            cyclenumber = cyclenumber + 1;
            datanumber = 0;
        }
        else
        {
            flag_start = 0;
        }        
    }
    else
    {
        target_pos.x() = 0;
        target_pos.y() = 0;
        target_pos.z() = 330.0;
        target_quat.w() = 1;
        target_quat.x() = 0;
        target_quat.y() = 0;
        target_quat.z() = 0;
        usleep(ts[0]);
    }
   
}

//print Vector3df
static void print3dVector(string& title, Eigen::Vector3f& vect)
{
    cout<< title <<" "<<"x: "<< vect.x() <<" y: "<< vect.y()<<" z: "<<vect.z()<<endl;   
}

//print 1d int vector
static void printOnedVector(string& title, vector<int> & v_int)
{    
    for (int i = 0; i < v_int.size(); i++)
    {        
        cout<< title <<"["<<i<<"]: "<<v_int[i]<<endl;
    }
}

//print 2d int vector
static void printTwodVector(string& title, vector<vector<int>> & v_int)
{
    for (int i = 0; i < v_int.size(); i++)
    {
        for (int j = 0; j < v_int[i].size(); j++)
        {            
            cout<< title <<"["<<i<<"]"<<"["<<j<<"]: "<<v_int[i][j]<<endl;
        }        
    }
}

//print results
static void ResultsDisplay()
{
    if (flag_sol)
    {
        printf("computational time(us): %lu \r\n", diff_time);    
        printf("target   :  x: %.4f,  y: %.4f,  z: %.4f, qw: %.4f, qx: %.4f, qy: %.4f, qz: %.4f\r\n", target_pos.x(), target_pos.y(), target_pos.z(),target_quat.w(), target_quat.x(), target_quat.y(), target_quat.z());
        printf("selected :  x: %.4f,  y: %.4f,  z: %.4f, qw: %.4f, qx: %.4f, qy: %.4f, qz: %.4f\r\n", selected_pos.x(), selected_pos.y(), selected_pos.z(),selected_quat.w(), selected_quat.x(), selected_quat.y(), selected_quat.z());
        printf("command  : a1: %.4f, b1: %.4f, l1: %.4f, a2: %.4f, b2: %.4f, l2: %.4f\r\n", alpha_s[0], beta_s[0], length_s[0], alpha_s[1], beta_s[1], length_s[1]);
        printf("dataNo   : %d,       distp: %.4f,     disto: %.4f,      distsum: %.4f\r\n", solution_selected, position_distance, orientation_distance, weightedsum_distance);
    }
    else
    {
        alpha_s[0]   = alpha_prev[0];
        alpha_s[1]   = alpha_prev[1];
        beta_s[0]    = beta_prev[0];
        beta_s[1]    = beta_prev[1];
        length_s[0]  = length_prev[0];
        length_s[1]  = length_prev[1];

        printf("computational time(us): %lu \r\n", diff_time); 
        printf("target   :  x: %.4f,  y: %.4f,  z: %.4f, qw: %.4f, qx: %.4f, qy: %.4f, qz: %.4f\r\n", target_pos.x(), target_pos.y(), target_pos.z(),target_quat.w(), target_quat.x(), target_quat.y(), target_quat.z()); 
        printf("No solution found!\r\n");  
    }   
}

//search neighbour grid for each dimension, e.g. grino: 5, new_gridno[]: 4,6
static vector<int> SearchNeighbourGrid(int gridno, vector<float> & gridarray)
{      
    vector<int> new_gridno; 
    new_gridno.push_back(gridno);

    int forward  = gridno - 1;
    int backward = gridno + 1;

    if (forward >= 0 && forward < gridarray.size()-1)
    {
        new_gridno.push_back(forward);            
    }

    if (backward > 0 && backward < gridarray.size()-1)
    {
        new_gridno.push_back(backward);
    }        
         
    return new_gridno;
}

static vector<vector<int>> SearchMoreGrids(vector<vector<int>> & gridno, vector<vector<float>> & gridarray)
{    
    vector<vector<int>> new_gridno;
    vector<int> temp_gridno;
     
    for (int i = 0; i < gridno.size(); i ++)
    {
        vector<int> gridno_temp;
        for (int j = 0; j < gridno[i].size(); j++)
        {
            
            temp_gridno = SearchNeighbourGrid(gridno[i][j], gridarray[i]); 
            for (int k = 0; k < temp_gridno.size(); k++)
            {
                gridno_temp.push_back(temp_gridno[k]);
                // printf("i: %d, j: %d, temp_gridno: %d\r\n", i, j, temp_gridno[k]);
            }                           
        } 
        new_gridno.push_back(gridno_temp);                    
    }

    // remove repeated elements
    for (int i = 0; i < new_gridno.size(); i++)
    {       
        for (int j = 0; j < new_gridno[i].size(); j++)
        {   
            int repeat = new_gridno[i][j];
            for (int k = j+1; k < new_gridno[i].size(); k++)
            {
                if (repeat == new_gridno[i][k])
                {
                    new_gridno[i].erase(new_gridno[i].begin()+k);
                }
            }           
        }
    }

    return new_gridno;
}

//search for solutions, gridnoarray: [pxn,...,ozn], solution[0]: posNo; solution[1]: ortNo, get solution(dataNo) based on gridno
static vector<int> SearchSolution_once(vector<vector<int>> & gridnoarray, vector<vector<vector<vector<int>>>> & posgridarray, vector<vector<vector<vector<int>>>> & ortgridarray) 
{
    vector<int> solution{-1};
    vector<int> solution_pos{-1};
    vector<int> solution_ort{-1};
    int n1 = 0;
    int n2 = 0;
    int n3 = 0;
    int n4 = 0;
    int n5 = 0;
    int n6 = 0;

    for (int i = 0; i < gridnoarray[0].size(); i++)
    {
        for (int j = 0; j < gridnoarray[1].size(); j++)
        {
            for (int k = 0; k < gridnoarray[2].size(); k++)
            {
                n1 = gridnoarray[0][i];
                n2 = gridnoarray[1][j];
                n3 = gridnoarray[2][k];
                if (posgridarray[n1][n2][n3].size() >= 1)
                {
                    for (int p = 0; p < posgridarray[n1][n2][n3].size(); p++)
                    {
                        solution_pos.push_back(posgridarray[n1][n2][n3][p]); 
                        // printf("pos: %d\r\n", posgridarray[n1][n2][n3][p]);                                   
                    }                                
                }
            }
        }
    }
   
    for (int r = 0; r < gridnoarray[3].size(); r++)
    {
        for (int s = 0; s < gridnoarray[4].size(); s++)
        {
            for (int t = 0; t < gridnoarray[5].size(); t++)
            {               
                n4 = gridnoarray[3][r];
                n5 = gridnoarray[4][s];
                n6 = gridnoarray[5][t];
                                                                                                    
                if (ortgridarray[n4][n5][n6].size() >= 1)
                {
                    for (int q = 0; q < ortgridarray[n4][n5][n6].size(); q++)
                    {
                        solution_ort.push_back(ortgridarray[n4][n5][n6][q]);                                    
                    }                               
                }                                                                                                                                        
            }
        }
    } 

    for (int u = 0; u < solution_pos.size(); u++)
    {
        for (int v = 0; v < solution_ort.size(); v++)
        {           
            if (solution_pos[u] == solution_ort[v] && solution_pos[u] != -1)
            {                
                solution.push_back(solution_pos[u]);
            }            
        }
    }
   
    // name = "solution";
    // printOnedVector(name, solution);
    return solution;
}

static vector<int> SearchSolution_candidate(vector<int> & gridno, vector<vector<vector<vector<int>>>> & posgridarray, vector<vector<vector<vector<int>>>> & ortgridarray, vector<vector<float>> & gridarray)
{
    vector<int> solution{-1};
    vector<int> solution_pos{-1};
    vector<int> solution_ort{-1};
    vector<int> newgridno;

    int pxn = gridno[0];
    int pyn = gridno[1];
    int pzn = gridno[2];
    int oxn = gridno[3];
    int oyn = gridno[4];
    int ozn = gridno[5];

    int posgrid_size = posgridarray[pxn][pyn][pzn].size();
    int ortgrid_size = ortgridarray[oxn][oyn][ozn].size();

    if (posgrid_size >= 1)
    {
        for (int r = 0; r < posgrid_size; r++)
        {
            solution_pos.push_back(posgridarray[pxn][pyn][pzn][r]);
        }       
    }

    if (ortgrid_size >= 1)
    {
        for (int r = 0; r < ortgrid_size; r++)
        {
            solution_ort.push_back(ortgridarray[oxn][oyn][ozn][r]);
        }        
    }

    for (int i = 0; i < solution_pos.size(); i++)
    {
        for (int j = 0; j < solution_ort.size(); j++)
        {
            if (solution_pos[i] == solution_ort[j] && solution_pos[i] != -1)
            {
                solution.push_back(solution_pos[i]);
            }            
        }
    }

    // name = "gridno";
    // printOnedVector(name, gridno);
    // printf("size of solution: %lu\r\n", solution.size());

    if (solution.size() <= 1)
    {
        vector<vector<int>> newgridnoarray;
        for (int i = 0; i < gridno.size(); i++)
        {
            newgridno = SearchNeighbourGrid(gridno[i], gridarray[i]);
            newgridnoarray.push_back(newgridno);
        }        

        // name = "newgridnoarray";
        // printTwodVector(name, newgridnoarray);

        solution = SearchSolution_once(newgridnoarray, posgridarray, ortgridarray);

        // name = "solution";
        // printOnedVector(name, solution);

        int n1 = 0;
        int n2 = 0;
        int n3 = 0;
        int n4 = 0;
        int n5 = 0;
        int n6 = 0;
                           
        printf("size of solution: %lu\r\n", solution.size());

        while (solution.size() <= 1 && n1 < gridarray[0].size()-1 && n2 < gridarray[1].size()-1 && n3 < gridarray[2].size()-1 
                                    && n4 < gridarray[3].size()-1 && n5 < gridarray[4].size()-1 && n6 < gridarray[5].size()-1)
        {
            newgridnoarray = SearchMoreGrids(newgridnoarray, gridarray);
                     
            name = "updated newgridnoarray";           
            printTwodVector(name, newgridnoarray); 

            solution = SearchSolution_once(newgridnoarray, posgridarray, ortgridarray);
            n1 = newgridnoarray[0].size();
            n2 = newgridnoarray[1].size();
            n3 = newgridnoarray[2].size();
            n4 = newgridnoarray[3].size();
            n5 = newgridnoarray[4].size();
            n6 = newgridnoarray[5].size(); 

            printf("n1: %d, n2: %d, n3: %d, n4: %d, n5: %d, n6: %d", n1, n2, n3, n4, n5, n6);                    
        }
    }   

    if (solution.size() <= 1)
    {
        flag_sol = 0;
    }
    else
    {
        solution.erase(solution.begin());
    }
     
    return solution;
}

//select the closest point by comparing distance with target, if there is any solution
static int SelectSolution(Eigen::Vector3f& pos_t, Eigen::Vector3f& ort_t, vector<int> & solution_candidate, float ep, float eo, vector<vector<float>> & data)
{    
    int selected_solution;
    vector<float> sum_distance;
    vector<float> pos_distance;
    vector<float> ort_distance;
    int dataNo_candidate;
    int distminNo;
    
    for (int i = 0; i < solution_candidate.size(); i++)
    {       
        dataNo_candidate = solution_candidate[i];        
        Eigen::Vector3f canditate_position(data[dataNo_candidate][6], data[dataNo_candidate][7], data[dataNo_candidate][8]);
        Eigen::Vector3f diff_posCarCoord;

        diff_posCarCoord = pos_t - canditate_position;
        float pos_dist = diff_posCarCoord.norm();

        Eigen::Quaternionf candidate_quat(data[dataNo_candidate][12], data[dataNo_candidate][9], data[dataNo_candidate][10], data[dataNo_candidate][11]);
        Eigen::Vector3f candidate_ortCarCoord;
        Eigen::Vector3f diff_ortCarCoord;

        candidate_ortCarCoord = quat2OrtCarCoord(candidate_quat);
        diff_ortCarCoord = ort_t - candidate_ortCarCoord;
        float ort_dist = diff_ortCarCoord.norm();

        pos_distance.push_back(pos_dist);
        ort_distance.push_back(ort_dist);

        float sum_dist = ep*pos_dist + eo*ort_dist;
                
        sum_distance.push_back(sum_dist);     
    }
  
    // comparing weighted sum of pos_distance & ort_distance
    // for (int i = 0; i < sum_distance.size(); i++)
    // {
    //     printf("sum_distance[%d]: %f\r\n", i, sum_distance[i]);
    // }
    
    distminNo = 0;
    for (int i = 0; i < sum_distance.size(); i++)
    {       
        if (sum_distance[i] < sum_distance[distminNo])
        {
            distminNo = i;
        }               
    }

    position_distance = pos_distance[distminNo];
    orientation_distance = ort_distance[distminNo];
    weightedsum_distance = sum_distance[distminNo];

    // printf("distminNo: %d\r\n",distminNo);

    selected_solution = solution_candidate[distminNo];
    selected_pos.x() = data[selected_solution][6];
    selected_pos.y() = data[selected_solution][7];
    selected_pos.z() = data[selected_solution][8];
    selected_quat.w() = data[selected_solution][12];
    selected_quat.x() = data[selected_solution][9];
    selected_quat.y() = data[selected_solution][10];
    selected_quat.z() = data[selected_solution][11];

    alpha_s[0]   = data[selected_solution][0];
    alpha_s[1]   = data[selected_solution][3];
    beta_s[0]    = data[selected_solution][1];
    beta_s[1]    = data[selected_solution][4];
    length_s[0]  = data[selected_solution][2];
    length_s[1]  = data[selected_solution][5];

    return selected_solution;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Kinematics_grids");
    ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz
		
	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);
    ros::Subscriber key_sub_ = nh.subscribe("key_number", 1, keyCallback);	
	origarm_ros::Command_ABL Command_ABL_demo;
   
    SaveDataFileName = "kinematicData_" + getTimeString() + ".txt";
    outFile.open(SaveDataFilePath + SaveDataFileName, ios::app);
    BeginTime = ros::Time::now();
    if (!outFile)
    {
        printf("Unable to open file for saving data!");
        exit(1);
    }
            
    readFromDataFile(); 

    generateDataGridTable();

    // for (int i = 0; i < PosDataGridTable.size(); i++)
    // {
    //     for (int j = 0; j < PosDataGridTable[i].size(); j++)
    //     {
    //         for (int k = 0; k < PosDataGridTable[i][j].size(); k++)
    //         {
    //             for (int p = 0; p < PosDataGridTable[i][j][k].size(); p++)
    //             {
    //                 printf("PosDataGridTable[%d][%d][%d][%d]: %d\r\n", i, j, k, p, PosDataGridTable[i][j][k][p]);
    //             }
    //         }
    //     }
    // }
                 
    while (ros::ok())
	{
        // int size_givendata = givenData.size(); 
        // int randn_array[7];

        // randn_array[0] = rand() % size_givendata;
        // for (int i = 1; i < 7; i++)
        // {
        //     randn_array[i] = rand() % 1000;           
        // }
              
        // int caseNo = 0;
        // float distp = 5;
        // float disto = 5;
        // generatetarget(caseNo, randn_array, distp, disto); 

        // given target pos & ort
         getTarget();
         printf("flag_start: %d, datanumber: %d, cyclenumber: %d\r\n",flag_start, datanumber, cyclenumber);
        // printf("target_pos: %f, %f, %f\r\n",target_pos.x(), target_pos.y(), target_pos.z());
       
        curr_time = getCurrentTime();
        target_OrtCarCoord = quat2OrtCarCoord(target_quat); 
        time_t dt1 = getCurrentTime()-curr_time; 

        curr_time = getCurrentTime();     
        target_OrtSphCoord = Cartesian2Sphere(target_OrtCarCoord);
        time_t dt2 = getCurrentTime()-curr_time;

        curr_time = getCurrentTime();       
        target_gridno = GridNoArray(target_pos, target_OrtSphCoord, Grid_group); 
        time_t dt3 = getCurrentTime()-curr_time; 

        curr_time = getCurrentTime(); 
        solution_candidate = SearchSolution_candidate(target_gridno, PosDataGridTable, OrtDataGridTable, Grid_group);        

        time_t dt4 = getCurrentTime()-curr_time; 

        curr_time = getCurrentTime();
        if (flag_sol)
        {
            solution_selected  = SelectSolution(target_pos, target_OrtCarCoord, solution_candidate, ep, eo, givenData);            
        }
        time_t dt5 = getCurrentTime()-curr_time;
        diff_time = dt1 + dt2 + dt3 + dt4 + dt5;

        // printf("seperated computational time (us): %lu, %lu, %lu, %lu, %lu\r\n", dt1, dt2, dt3, dt4, dt5);
        ResultsDisplay();

        if (flag_start)
        {
            std::string curtime = getTimeNsecString();            
            outFile << curtime <<" "<< solution_candidate.size() <<" "<< dt1 <<" "<< dt2 <<" "<< dt3 <<" "<< dt4 <<" "<< dt5 <<" "<< diff_time << endl;
            outFile << curtime <<" "<< target_pos.x() << " " << target_pos.y() << " " << target_pos.z() <<" "<< target_quat.w() <<" "<< target_quat.x() <<" "<<target_quat.y()<<" "<<target_quat.z()<<endl; 
            outFile << curtime <<" "<< selected_pos.x() << " " << selected_pos.y() << " " << selected_pos.z() <<" "<< selected_quat.w() <<" "<< selected_quat.x() <<" "<<selected_quat.y()<<" "<<selected_quat.z()<<endl;  
        }
                             
        for (int i = 0; i < 3; i++)
        {
            Command_ABL_demo.segment[i].A = alpha_s[0]/3;
            Command_ABL_demo.segment[i].B = beta_s[0];
            Command_ABL_demo.segment[i].L = length_s[0]/3;
        }
        for (int i = 3; i < 6; i++)
        {
            Command_ABL_demo.segment[i].A = alpha_s[1]/3;
            Command_ABL_demo.segment[i].B = beta_s[1];
            Command_ABL_demo.segment[i].L = length_s[1]/3;
        }
        for (int i = 6; i < SEGNUM; i++)
        {
            Command_ABL_demo.segment[i].A = 0;
            Command_ABL_demo.segment[i].B = 0;
            Command_ABL_demo.segment[i].L = length0;
        }

        pub1.publish(Command_ABL_demo);	

        for (int i = 0; i < 2; i++)
        {
            alpha_prev[i] = alpha_s[i];
            beta_prev[i] = beta_s[i];
            length_prev[i] = length_s[i];
        }
        
        ros::spinOnce(); //necessary for subscribe
		r.sleep();
	}

	outFile.close();	
	return 0;
}
