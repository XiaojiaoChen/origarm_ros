#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <linux/input-event-codes.h>
#include <vector>

#include "ros/package.h"
#include "math.h"
#include "myData.h"
#include "myFunction.h"
#include "origarm_ros/Sensor.h"
#include "origarm_ros/States.h"
#include "origarm_ros/keynumber.h"
#include "ros/ros.h"
#include <time.h>

using namespace Eigen;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::MatrixXf;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using Eigen::VectorXf;

using namespace std;
static float rad2deg = 180.0f / M_PI;
static float deg2rad = M_PI / 180.0f;

ifstream inFile;
string GivenDataFilePath = ros::package::getPath("origarm_ros") + "/data/GridData/";
string GivenDataFileName = "data2.txt";

// given data 2d vector[N][13], N = max(dataNo)
static vector< vector<float> > givenData;

// given PosGrid & OrtGrid, both expressions are necessary
static vector<float> PosCarGrid_x{-0.3557, -0.1779,      0, 0.1779, 0.3557};
static vector<float> PosCarGrid_y{-0.3557, -0.1779,      0, 0.1779, 0.3557};
static vector<float> PosCarGrid_z{ 0.0644,  0.1704, 0.2764, 0.3824, 0.4884};
static vector<float> OrtSphGrid_omega{0, 0.7879, 1.5758, 2.3637, 3.1516};
static vector<float> OrtSphGrid_phi  {0, 0.7879, 1.5758, 2.3637, 3.1516};
static vector<float> OrtSphGrid_theta{0, 1.5733, 3.1466, 4.7199, 6.2932};

static vector<vector<float>> Grid_group{    
    {-0.3557, -0.1779,      0, 0.1779, 0.3557},
    {-0.3557, -0.1779,      0, 0.1779, 0.3557},
    { 0.0644,  0.1704, 0.2764, 0.3824, 0.4884},
    {      0,  0.7879, 1.5758, 2.3637, 3.1516},
    {      0, 0.7879, 1.5758, 2.3637,  3.1516},
    {      0, 1.5733, 3.1466, 4.7199,  6.2932}};

int Grid_size[6] = {4, 4, 4, 4, 4, 4};

// DataGridTable 4d vector DataGridTable[i][j][k] = dataNo1, dataNo2, ...
static vector< vector< vector< vector<int> > > > PosDataGridTable;
static vector< vector< vector< vector<int> > > > OrtDataGridTable;

// given target position & orientation
// 0.000000 0.000000 0.090000 0.400000 1.000000 0.090000 0.0096 0.0149 0.1776 -0.167174 0.107341 0.000000 0.980067
Eigen::Vector3f    target_pos(0.0096, 0.0149, 0.1776);
Eigen::Quaternionf target_quat(0.980067, -0.167174, 0.107341, 0.000000); // initilization as :(qw,qx,qy,qz)
Eigen::Vector3f    target_OrtSphCoord;
Eigen::Vector3f    target_OrtCarCoord;
vector<int> target_gridno;
vector<int> solution_candidate;
int solution_selected;
float ep = 0.5;
float eo = 0.5;

// finally selected ABL based on grids and given data
Eigen::Vector3f    selected_pos;
Eigen::Quaternionf selected_quat;
float alpha_s[2] = {0, 0};
float beta_s[2]  = {0, 0};
float length_s[2]= {length0, length0};

// distance between selected & target pose
float position_distance;
float orientation_distance;
float weightedsum_distance;

//Coord conversion: quat2OrtCarCoord
static Eigen::Vector3f quat2OrtCarCoord(Eigen::Quaternionf quat)
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

//Coord conversion: Cartesian2Sphere, return in [omega; phi; normalized theta]
static Eigen::Vector3f Cartesian2Sphere(Eigen::Vector3f CartesianCoord)
{
    Eigen::Vector3f SphereCoord;
    float magnitude = CartesianCoord.norm();
    float omega, phi;

    if (abs(magnitude) < 1e-10)
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

//randomly target pose generation
static void generatetarget(int rand_no)
{   
    target_pos.x() = givenData[rand_no][6];
    target_pos.y() = givenData[rand_no][7];
    target_pos.z() = givenData[rand_no][8];
    target_quat.w() = givenData[rand_no][12];
    target_quat.x() = givenData[rand_no][9];
    target_quat.y() = givenData[rand_no][10];
    target_quat.z() = givenData[rand_no][11];
}
    
//print DataGridTable
static void printDataGridTable(vector<vector<vector<vector<int>>>> & gridtable)
{
    for (int i = 0; i < gridtable.size(); i++)
    {
        for (int j = 0; j < gridtable[i].size(); j++)
        {
            for (int k = 0; k < gridtable[i][j].size(); k++)
            {
                for (int r = 0; r < gridtable[i][j][k].size(); r++)
                {
                    printf("gridtable[%d][%d][%d][%d]: %d\r\n", i, j, k, r, gridtable[i][j][k][r]);
                }               
            }
        }
    }
}

//print results
static void ResultsDisplay()
{
    printf("target   :  x: %.3f,  y: %.3f,  z: %.3f, qw: %.3f, qx: %.3f, qy: %.3f, qz: %.3f\r\n", target_pos.x(), target_pos.y(), target_pos.z(),target_quat.w(), target_quat.x(), target_quat.y(), target_quat.z());
    printf("selected :  x: %.3f,  y: %.3f,  z: %.3f, qw: %.3f, qx: %.3f, qy: %.3f, qz: %.3f\r\n", selected_pos.x(), selected_pos.y(), selected_pos.z(),selected_quat.w(), selected_quat.x(), selected_quat.y(), selected_quat.z());
    printf("command  : a1: %.3f, b1: %.3f, l1: %.3f, a2: %.3f, b2: %.3f, l2: %.3f\r\n", alpha_s[0], beta_s[0], length_s[0], alpha_s[1], beta_s[1], length_s[1]);
    printf("dataNo   : %d,       distp: %.3f,     disto: %.3f,      distsum: %.3f\r\n", solution_selected, position_distance, orientation_distance, weightedsum_distance);
}

//search neighbour grid for each dimension, grino: pxn...ozn, gridarray should be GridGroup, new_gridno[0][i], px dimension, i^{th} gridno
static vector<vector<int>> SearchNeighbourGrid(vector<int> & gridno, vector< vector<float> > & gridarray)
{    
    int size_gridno = gridno.size(); //should be 6
    vector<vector<int>> new_gridno;
    vector<int> temp_gridno;

    new_gridno.push_back(gridno);
    
    for (int i = 0; i < size_gridno; i ++)
    {
        int forward  = gridno[i] - 1;
        int backward = gridno[i] + 1;

        if (forward >= 0 && forward < gridarray[i].size())
        {
            temp_gridno.push_back(forward);            
        }

        if (backward > 0 && backward <= gridarray[i].size())
        {
            temp_gridno.push_back(backward);
        }        
    }

    new_gridno.push_back(temp_gridno);
    
    return new_gridno;
}

//search for solutions, gridnoarray: [pxn,...,ozn], solution[0]: posNo; solution[1]: ortNo, get solution(dataNo) based on gridno, keep searching until find solution/ search all grids
static vector<int> SearchSolution(vector<int> & gridnoarray, vector<vector<vector<vector<int>>>> & posgridtable, vector<vector<vector<vector<int>>>> & ortgridtable, vector<vector<float>> & gridarray)
{
    vector<int> solution;
    vector<int> solution_pos{-1};
    vector<int> solution_ort{-1};

    if (posgridtable[gridnoarray[0]][gridnoarray[1]][gridnoarray[2]].size() >= 1)
    {
        for (int r = 0; r < posgridtable[gridnoarray[0]][gridnoarray[1]][gridnoarray[2]].size(); r++)
        solution_pos.push_back(posgridtable[gridnoarray[0]][gridnoarray[1]][gridnoarray[2]][r]);
    }

    if (ortgridtable[gridnoarray[3]][gridnoarray[4]][gridnoarray[5]].size() >= 1)
    {
        for (int r = 0; r < ortgridtable[gridnoarray[3]][gridnoarray[4]][gridnoarray[5]].size(); r++)
        solution_ort.push_back(ortgridtable[gridnoarray[3]][gridnoarray[4]][gridnoarray[5]][r]);
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

    vector<vector<int>> newgridnoarray;
    int n1 = 0;
    int n2 = 0;
    int n3 = 0;
    int n4 = 0;
    int n5 = 0;
    int n6 = 0;

    while (solution.size() < 1 && n1 < gridarray[0].size() && n2 < gridarray[1].size() && n3 < gridarray[2].size() 
                               && n4 < gridarray[3].size() && n5 < gridarray[4].size() && n6 < gridarray[5].size())
    {
        newgridnoarray = SearchNeighbourGrid(gridnoarray, gridarray); 
       
        for (int i = 0; i < newgridnoarray[0].size(); i++)
        {
            for (int j = 0; j < newgridnoarray[1].size(); j++)
            {
                for (int k = 0; k < newgridnoarray[2].size(); k++)
                {
                    for (int r = 0; r < newgridnoarray[3].size(); r++)
                    {
                        for (int s = 0; s < newgridnoarray[4].size(); s++)
                        {
                            for (int t = 0; t < newgridnoarray[5].size(); t++)
                            {
                                n1 = newgridnoarray[0][i];
                                n2 = newgridnoarray[1][j];
                                n3 = newgridnoarray[2][k];
                                n4 = newgridnoarray[3][r];
                                n5 = newgridnoarray[4][s];
                                n6 = newgridnoarray[5][t];

                                if (posgridtable[n1][n2][n3].size() >= 1)
                                {
                                    for (int p = 0; p < posgridtable[n1][n2][n3].size(); p++)
                                    solution_pos.push_back(posgridtable[n1][n2][n3][p]);
                                }

                                if (ortgridtable[n4][n5][n6].size() >= 1)
                                {
                                    for (int q = 0; q < ortgridtable[n4][n5][n6].size(); q++)
                                    solution_ort.push_back(ortgridtable[n4][n5][n6][q]);
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
                            }
                        }
                    }
                }
            }               
        }  
    } 

    if (solution.size() < 1)
    {
        printf("target is out of range!");
        exit(1);
    } 
       
    return solution;
}

//select the closest point by comparing distance with target,
static int SelectSolution(Eigen::Vector3f& pos_t, Eigen::Vector3f& ort_t, vector<int> & solution_candidate, float ep, float eo, vector<vector<float>> & data)
{    
    int selected_solution;
    vector<float> sum_distance;
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

        float sum_dist = ep*pos_dist + eo*ort_dist;
                
        sum_distance.push_back(sum_dist);     
    }
  
    //comparing weighted sum of pos_distance & ort_distance
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

    // printf("distminNo: %d\r\n",distminNo);

    selected_solution = solution_candidate[distminNo];
    selected_pos.x() = givenData[selected_solution][6];
    selected_pos.y() = givenData[selected_solution][7];
    selected_pos.z() = givenData[selected_solution][8];
    selected_quat.w() = givenData[selected_solution][12];
    selected_quat.x() = givenData[selected_solution][9];
    selected_quat.y() = givenData[selected_solution][10];
    selected_quat.z() = givenData[selected_solution][11];

    alpha_s[0]   = givenData[selected_solution][0];
    alpha_s[1]   = givenData[selected_solution][3];
    beta_s[0]    = givenData[selected_solution][1];
    beta_s[1]    = givenData[selected_solution][4];
    length_s[0]  = givenData[selected_solution][2];
    length_s[1]  = givenData[selected_solution][5];

    position_distance = (pos_t - selected_pos).norm();
    orientation_distance = (ort_t - quat2OrtCarCoord(selected_quat)).norm();
    weightedsum_distance = ep*position_distance + eo*orientation_distance;

    return selected_solution;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Kinematics_grids");
    ros::NodeHandle nh;	
	ros::Rate r(100);     //Hz
		
	ros::Publisher  pub1 = nh.advertise<origarm_ros::Command_ABL>("Cmd_ABL_joy", 100);	
	origarm_ros::Command_ABL Command_ABL_demo;

    readFromDataFile(); 
    generateDataGridTable();

    // generatetarget(number);
    // target_OrtCarCoord = quat2OrtCarCoord(target_quat);
    // target_OrtSphCoord = Cartesian2Sphere(target_OrtCarCoord); 
    // target_gridno = GridNoArray(target_pos, target_OrtSphCoord, Grid_group);
    // solution_candidate = SearchSolution(target_gridno, PosDataGridTable, OrtDataGridTable, Grid_group); 
    // solution_selected  = SelectSolution(target_pos, target_OrtCarCoord, solution_candidate, ep, eo, givenData);
    // ResultsDisplay();

    while (ros::ok())
	{
        int size_givendata = givenData.size(); 
        int number = rand() % size_givendata;
        generatetarget(number);
        
        target_OrtCarCoord = quat2OrtCarCoord(target_quat);
        target_OrtSphCoord = Cartesian2Sphere(target_OrtCarCoord); 
    
        target_gridno = GridNoArray(target_pos, target_OrtSphCoord, Grid_group);
        solution_candidate = SearchSolution(target_gridno, PosDataGridTable, OrtDataGridTable, Grid_group);    
        solution_selected  = SelectSolution(target_pos, target_OrtCarCoord, solution_candidate, ep, eo, givenData);

        ResultsDisplay();  

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
		r.sleep();
	}
		
	return 0;
}
