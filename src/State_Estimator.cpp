#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <linux/input-event-codes.h>

#include "ros/package.h"
#include "math.h"
#include "myData.h"
#include "myFunction.h"
#include "origarm_ros/Sensor.h"
#include "origarm_ros/States.h"
#include "origarm_ros/keynumber.h"
#include "ros/ros.h"
#include <time.h>

#define DEBUG_DISPLAY 0
#define DUMMY_QUAT_USED 0

using namespace Eigen;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::MatrixXf;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using Eigen::VectorXf;

using namespace std;
static float rad2deg = 180.0f / M_PI;

static float alpha[SEGNUM];
static float beta[SEGNUM];
static float length[SEGNUM];

static float alphaDeg[SEGNUM];
static float betaDeg[SEGNUM];
static float lengthMM[SEGNUM];

// local definition of pressure, distance, quaternion
static int16_t pressureActuator[SEGNUM][ACTNUM];
static int16_t lengthActuator[SEGNUM][ACTNUM];

static Quaternionf QuattInIMU[SEGNUM][ACTNUM];
static Quaternionf Quat0InIMU[SEGNUM][ACTNUM];

static Matrix3f Rtrans[SEGNUM][ACTNUM];
static Matrix3f R0InIMU[SEGNUM][ACTNUM], RtInIMU[SEGNUM][ACTNUM];
static Matrix3f R0InArm[SEGNUM][ACTNUM], RtInArm[SEGNUM][ACTNUM];
static Matrix3f dRInArm[SEGNUM][ACTNUM];
static Matrix3f ddRInArm[9][3][SEGNUM];

static float alphaCandidates[9][3][SEGNUM];
static float betaCandidates[9][3][SEGNUM];
static float alphaFromActuators[SEGNUM];
static float betaFromActuators[SEGNUM];

static Quaternionf quaternionPlates[SEGMENTNUM];
static Matrix3f R0PlateInArm[SEGNUM], RtPlateInArm[SEGNUM], dRPlateInArm[SEGNUM];
static float alphaFromPlates[SEGNUM];
static float betaFromPlates[SEGNUM];

/*Flag to control how the alpha is calculated*/
static const int calculate_alpha_using_pre_cur_flag = 1;
static const int calculate_alpha_using_cur_cur_flag = 0;
static const int calculate_alpha_using_cur_nex_flag = 1;

/*Path to save the IMU data at zero position*/

static string quat0DefaultFileName = "default_data_imu0.txt";
static string quattDummyFileName = "imu_move_segment0.txt";
static string quatSaveFileName = "data_imu0.txt";
static string quatReadFileName = "data_imu0.txt";
static string IMUDataPath = "";

/*Indication of whether the IMU is working well*/
static int goodIMU[SEGNUM][ACTNUM] = {
    {0, 1, 0, 0, 1, 1},
    {0, 0, 1, 0, 1, 0},
    {1, 1, 1, 0, 1, 1},
    {1, 0, 0, 1, 0, 0},
    {1, 1, 0, 1, 0, 1},
    {0, 1, 0, 0, 1, 0}};

static void InitFrames();

std::string getTimeString()
{

    time_t rawtime;
    struct tm *timeinfo;
    char buffer[100];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, 100, "%G_%h_%d_%H_%M_%S", timeinfo);
    std::string ret = buffer;
    return ret;
}

/**
 * @brief Save IMU data to file
 * 
 * @param filePath The path where the data is to be stored
 */
static void saveQuatToFile(Quaternionf (&qua)[SEGNUM][ACTNUM], string filePath)
{
    ofstream data;
    data.open(filePath, ios::trunc); // ios::app
    // write imu data into yaml file/imu_data.txt
    cout << "Saving current IMU data to" + filePath << endl;
    for (int i = 0; i < SEGNUM; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {
            data << qua[i][j].w() << " " << qua[i][j].x() << " " << qua[i][j].y()
                 << " " << qua[i][j].z() << endl;
            cout << qua[i][j].w() << " " << qua[i][j].x() << " " << qua[i][j].y()
                 << " " << qua[i][j].z() << endl;
        }
    }
    data.close();
    cout << "IMU data Saved" << endl;
}

/**
 * @brief read IMU data from file
 * 
 * @param filePath The filepath where the original data is stored
 */
static void readQuatFromFile(string filePath, Quaternionf (&qua)[SEGNUM][ACTNUM])
{
    ifstream inFile;
    inFile.open(filePath, ios::in);
    if (inFile.fail())
    {
        inFile.close();
        cout << "unable to open the IMU file in " << filePath;
        filePath = IMUDataPath + quat0DefaultFileName;
        cout << ". The IMU default values will be used in " << filePath << endl;
        inFile.open(filePath, ios::in);
    }

    cout << "Reading IMU data from" + filePath << endl;
    if (!inFile.eof())
    {
        for (int p = 0; p < SEGNUM; p++)
        {
            for (int q = 0; q < ACTNUM; q++)
            {
                inFile >> qua[p][q].w() >> qua[p][q].x() >> qua[p][q].y() >> qua[p][q].z();
                cout << qua[p][q].w() << " " << qua[p][q].x() << " " << qua[p][q].y()
                     << " " << qua[p][q].z() << endl;
            }
        }
    }
    cout << "IMU data Read completed" << endl;
    inFile.close();
}

/*set DEBUG_DISPLAY=1 to print all the matrices*/
static void displayAllMatrix()
{

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {
            cout << "Rtrans Seg " << i << " Act " << j << endl;
            dispMatrix(Rtrans[i][j]);
        }
    }

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {

            cout << "R0InIMU Seg " << i << " Act " << j << endl;
            cout << Quat0InIMU[i][j].w() << " " << Quat0InIMU[i][j].x() << " " << Quat0InIMU[i][j].y() << " " << Quat0InIMU[i][j].z() << endl;
            dispMatrix(R0InIMU[i][j]);
        }
    }

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {
            cout << "RtInIMU Seg " << i << " Act " << j << endl;
            dispMatrix(RtInIMU[i][j]);
        }
    }

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {
            cout << "R0InArm Seg " << i << " Act " << j << endl;
            dispMatrix(R0InArm[i][j]);
        }
    }

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {
            cout << "RtInArm Seg " << i << " Act " << j << endl;
            dispMatrix(RtInArm[i][j]);
        }
    }

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {
            cout << "dRInArm Seg " << i << " Act " << j << endl;
            dispMatrix(dRInArm[i][j]);
        }
    }

    for (int i = 0; i < 6; i++)
    {
        cout << "alphaFromActuator Seg " << i << endl;
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 9; k++)
            {

                cout << alphaCandidates[k][j][i] << " ";
            }
            cout << endl;
        }
    }
}

class State_Estimator
{
public:
    State_Estimator()
    {
        key_sub_ = n_.subscribe("key_number", 1, &State_Estimator::keyCallback, this);
        sub_ = n_.subscribe("Sensor", 300, &State_Estimator::callback, this);
        pub_ = n_.advertise<origarm_ros::States>("States", 300);
    }

    void keyCallback(const origarm_ros::keynumber &key)
    {
        if (key.keycodePressed == KEY_C) // 'C' pressed
        {
            printf(" KEY_C pressed!\r\n");
            string path = IMUDataPath + quatSaveFileName;
            saveQuatToFile(QuattInIMU, path);
            InitFrames();
        }
        else if (key.keycodePressed == KEY_R) // 'r' pressed
        {
            printf(" KEY_R pressed! Saving IMU data...\r\n");
            std::string path = IMUDataPath + "Rec_IMU_" + getTimeString() + ".txt";
            saveQuatToFile(QuattInIMU, path);
        }
    }

    void callback(const origarm_ros::Sensor &Sensor_)
    {
        for (int i = 0; i < SEGNUM; i++)
        {
            for (int j = 0; j < ACTNUM; j++)
            {
                pressureActuator[i][j] = Sensor_.sensor_segment[i].sensor_actuator[j].pressure;
                lengthActuator[i][j] = Sensor_.sensor_segment[i].sensor_actuator[j].distance;
                QuattInIMU[i][j].w() = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.w; // double check
                QuattInIMU[i][j].x() = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.x;
                QuattInIMU[i][j].y() = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.y;
                QuattInIMU[i][j].z() = Sensor_.sensor_segment[i].sensor_actuator[j].pose.orientation.z;
                RtInIMU[i][j] = QuattInIMU[i][j];
            }
        }
    }

    void pub()
    {
        // real ABL calculated from sensor
        for (int i = 0; i < SEGNUM; i++)
        {
            states_.ABL.segment[i].A = alpha[i];
            states_.ABL.segment[i].B = beta[i];
            states_.ABL.segment[i].L = length[i];
        }

        // real pose calculated from sensor
        // states_.pose.position.x = ;
        // states_.pose.position.y = ;
        // states_.pose.position.z = ;
        // states_.pose.orientation.w = ;
        // states_.pose.orientation.x = ;
        // states_.pose.orientation.y = ;
        // states_.pose.orientation.z = ;

        pub_.publish(states_);
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Subscriber key_sub_;

    origarm_ros::States states_;
};

/**/
/**
 * @brief Choose good actuators. More complex rules could be added
 * 
 * @param p the segment number
 * @param q the actautor number on the segment
 * @return 0: Bad actuators
 *         1: Good actuators
 */
static int goodActuator(int p, int q)
{
    int ret = 0;
    if (p >= 0 && p < SEGNUM && q >= 0 && q < ACTNUM)
        if (goodIMU[p][q])
            ret = 1;
    return ret;
}

/**
 * @brief Init all frames
 * 1. init the IMU body frame at time 0 in the ARM Base, based on the physical location of the actuators in the arm
 * 2. Read the IMU body frame at time 0 in the IMU Base
 * 3. Calculate the transformation matrix of IMU Base in the ARM Base
 * 4. define the plate pose at time 0 in the Arm base, which are all identities.
 */
void InitFrames()
{
    /*set the IMU body frame at time 0 in the ARM Base*/

    for (int i = 0; i < SEGNUM; i++)
    {
        R0InArm[i][0] = myRotz(-M_PI / 2);
        R0InArm[i][2] = myRotz(-M_PI / 2 + 2 * M_PI / 3);
        R0InArm[i][4] = myRotz(-M_PI / 2 + 4 * M_PI / 3);

        R0InArm[i][1] = myRotz(-M_PI / 6) * myRoty(M_PI);
        R0InArm[i][3] = myRotz(-M_PI / 6 + 2 * M_PI / 3) * myRoty(M_PI);
        R0InArm[i][5] = myRotz(-M_PI / 6 + 4 * M_PI / 3) * myRoty(M_PI);
    }

    /*Read the IMU body frame at time 0 in the IMU Base*/
    string path = IMUDataPath + quatReadFileName;
    readQuatFromFile(path, Quat0InIMU);
    for (int i = 0; i < SEGNUM; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {
            R0InIMU[i][j] = Quat0InIMU[i][j];
        }
    }

#if DUMMY_QUAT_USED == 1
    /*****************Debug Only.  Uncomment in real application***********************/
    /*Read dummy IMU body frame at time t in the IMU Base*/
    readQuatFromFile(IMUDataPath + quattDummyFileName, QuattInIMU);
    for (int i = 0; i < SEGNUM; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {
            RtInIMU[i][j] = QuattInIMU[i][j];
        }
    }
#endif

    /*Calculate the transformation matrix of IMU Base in the ARM Base*/
    for (int i = 0; i < SEGNUM; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {
            Rtrans[i][j] = R0InArm[i][j] * R0InIMU[i][j].transpose();
        }
    }

    /*define the plate pose at time 0 in the Arm base*/
    for (int i = 0; i < SEGNUM; i++)
    {
        R0PlateInArm[i] = Matrix3f::Identity();
    }
}

/*get the IMU body frame at time t in the arm base*/
void getRtInArm()
{
    for (int i = 0; i < SEGNUM; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {
            RtInArm[i][j] = Rtrans[i][j] * RtInIMU[i][j];
        }
    }
}

/**
 * @brief get the IMU body frame change at time t relative to time 0,  in the Arm Base
 * 
 */
void getdRtInArm()
{
    for (int i = 0; i < SEGNUM; i++)
    {
        for (int j = 0; j < ACTNUM; j++)
        {
            dRInArm[i][j] = RtInArm[i][j] * R0InArm[i][j].transpose();
        }
    }
}

/**
 * @brief get ddRtInArm using different actautor pairs. 
 * There are 3*9 pairs for each segment, that is
 * 1. Using Act(1 3 5) in previous segment and Act(2 4 6) in current segment
 * 2. Using Act(0 2 4) and Act(1 3 5) in current segment
 * 3. Using Act(0 2 4) in current segment and Act(0 2 4) in next segment
 * Only Elegible and good Actuators are considered during the process.
 */
void getddRtInArm()
{
    int p1 = 0; //segment number of the first chosen actuator
    int p2 = 0; //segment number of the second chosen actuator
    int k = 0;  //0~8, representing all the 9 possbile actuator combinations in a given flag

    /*For every segment*/
    for (int i = 0; i < SEGNUM; i++)
    {

        /*Using Act(1 3 5) in previous segment and Act(2 4 6) in current segment*/
        if (calculate_alpha_using_pre_cur_flag)
        {
            k = 0;
            p1 = i - 1;
            p2 = i;
            for (int q1 = 1; q1 < ACTNUM; q1 += 2)
            {
                for (int q2 = 1; q2 < ACTNUM; q2 += 2)
                {
                    /*only Elegible and good Actuator*/
                    if (goodActuator(p1, q1) && goodActuator(p2, q2))
                        ddRInArm[k++][0][i] = dRInArm[p1][q1].transpose() * dRInArm[p2][q2];
                }
            }
        }

        /*Using Act(0 2 4) and Act(1 3 5) in current segment*/
        if (calculate_alpha_using_cur_cur_flag)
        {
            k = 0;
            p1 = i;
            p2 = i;
            for (int q1 = 0; q1 < ACTNUM; q1 += 2)
            {
                for (int q2 = 1; q2 < ACTNUM; q2 += 2)
                {
                    /*only Elegible and good Actuator*/
                    if (goodActuator(p1, q1) && goodActuator(p2, q2))
                        ddRInArm[k++][1][i] = dRInArm[p1][q1].transpose() * dRInArm[p2][q2];
                }
            }
        }
        /*Using Act(0 2 4) in current segment and Act(0 2 4) in next segment*/
        if (calculate_alpha_using_cur_nex_flag)
        {
            k = 0;
            p1 = i;
            p2 = i + 1;
            for (int q1 = 0; q1 < ACTNUM; q1 += 2)
            {
                for (int q2 = 0; q2 < ACTNUM; q2 += 2)
                {
                    /*only Elegible and good Actuator*/
                    if (goodActuator(p1, q1) && goodActuator(p2, q2))
                        ddRInArm[k++][2][i] = dRInArm[p1][q1].transpose() * dRInArm[p2][q2];
                }
            }
        }
    }
}

/**
 * @brief Get the Alpha From R object
 * 
 * @param R 
 * @return float 
 */
float getAlphaFromR(Matrix3f &R)
{
    float val = -1000;
    float alphaCandi = 0;
    if (!R.isZero())
    {
        val = CONSTRAIN(R(2, 2), -1, 1);
        alphaCandi = acos(val);
    }
    else
    {
        alphaCandi = -1000;
    }
    return alphaCandi;
}
/**
 * @brief Get the Beta From a rotation matrix R
 * 
 * @param R The rotation matrix
 * @return float beta
 */
float getBetaFromR(Matrix3f &R)
{
    float alphaCandi = 0;
    float betaCandi = 0;
    alphaCandi = getAlphaFromR(R);
    if (alphaCandi > 0.05f || alphaCandi < -0.05f)
        betaCandi = atan2(R(1, 2) / sin(alphaCandi), R(0, 2) / sin(alphaCandi));
    else
    {
        betaCandi = 0;
    }
    return betaCandi;
}
/**
 * @brief Get the Alpha Beta From Actuators. 
 * For every segment, there are 3*9 = 27 candidate transformation matrix ddR for calculating alpha and beta. 
 * The valid values are chosen and averaged to get the final value
 * 
 */
void getAlphaBetaFromActuators()
{
    float sumAlpha = 0;
    int countsAlpha = 0;
    float sumBeta = 0;
    float countsBeta = 0;
    int average_flag[3] = {calculate_alpha_using_pre_cur_flag, calculate_alpha_using_cur_cur_flag, calculate_alpha_using_cur_nex_flag};

    for (int i = 0; i < SEGNUM; i++)
    {
        sumAlpha = 0;
        countsAlpha = 0;
        sumBeta = 0;
        countsAlpha = 0;
        for (int j = 0; j < 3; j++)
        {
            if (average_flag[j])
            {
                for (int k = 0; k < 9; k++)
                {
                    alphaCandidates[k][j][i] = getAlphaFromR(ddRInArm[k][j][i]);
                    betaCandidates[k][j][i] = getBetaFromR(ddRInArm[k][j][i]);
                    /*unuseful data are set to be -1000*/
                    if (alphaCandidates[k][j][i] > -10)
                    {
                        sumAlpha += alphaCandidates[k][j][i];
                        countsAlpha++;
                        sumBeta += betaCandidates[k][j][i];
                        countsBeta++;
                    }
                }
            }
        }
        alphaFromActuators[i] = sumAlpha / countsAlpha;
        betaFromActuators[i] = sumBeta / countsBeta;
    }
}

/**
 * @brief Average quaternions in an array Quats[], starting from index, with a total number of startnum
 * 
 * @param Quats The quaternion array to be averaged
 * @param startnum The starting index
 * @param num Number of total quaternion to be averaged
 * @return Quaternionf Averaged quaternion
 */
Quaternionf averageQuaternions(Quaternionf *Quats, int startnum, int num)
{
    Quaternionf avQuat;
    Eigen::MatrixXf Qori(4, num);
    Eigen::Matrix4f Qmat;
    Eigen::Vector4f QmatEigenValue;
    Eigen::Matrix4f QmatEigenVector;
    SelfAdjointEigenSolver<Matrix4f> es(Qmat);
    float coeff = 1.0f / num;
    for (int i = 0; i < num; i++)
    {
        Qori(0, i) = Quats[startnum + i].w() * coeff;
        Qori(1, i) = Quats[startnum + i].x() * coeff;
        Qori(2, i) = Quats[startnum + i].y() * coeff;
        Qori(3, i) = Quats[startnum + i].z() * coeff;
    }
    Qmat = Qori * Qori.transpose();
    QmatEigenValue = es.eigenvalues();
    QmatEigenVector = es.eigenvectors();
    avQuat.w() = QmatEigenVector(0, 3);
    avQuat.x() = QmatEigenVector(1, 3);
    avQuat.y() = QmatEigenVector(2, 3);
    avQuat.z() = QmatEigenVector(3, 3);
    return avQuat;
}

/**
 * @brief Get the Plates rotation matrix R in the Arm base
 * The plates quaternions are determined by averaging the quaternions of the actuators on it. Physically these actuator share a common quaternion
 * The choice of Actuators can choose the three actuators on current segment or the next segment.
 */
void getPlatesR()
{
    int use_cur_seg = 1;
    int use_nex_seg = 1;
    Quaternionf quaternionCandi[6];

    int startnum = 0;
    int num = 0;
    for (int i = 0; i < SEGNUM; i++)
    {

        /*The last plate could only use current segment actuators*/
        if (use_cur_seg == 1 || i == SEGNUM - 1)
        {
            quaternionCandi[0] = dRInArm[i][1];
            quaternionCandi[1] = dRInArm[i][3];
            quaternionCandi[2] = dRInArm[i][5];
            startnum = 0;
            num = 3;
        }
        if (use_nex_seg == 1 && (i != SEGNUM - 1))
        {
            quaternionCandi[3] = dRInArm[i + 1][0];
            quaternionCandi[4] = dRInArm[i + 1][2];
            quaternionCandi[5] = dRInArm[i + 1][4];
            startnum += 3;
            num += 3;
        }
        quaternionPlates[i] = averageQuaternions(quaternionCandi, startnum, num);
        RtPlateInArm[i] = quaternionPlates[i];
    }
}

/**
 * @brief Get the Alpha Beta From the relative rotation matrix between plates
 * 
 */
void getAlphaBetaFromPlates()
{
    Matrix3f dRplate;
    for (int i = 0; i < SEGNUM; i++)
    {
        if (i == 0)
            dRplate = RtPlateInArm[i];
        else
            dRplate = RtPlateInArm[i - 1].transpose() * RtPlateInArm[i];
        alphaFromPlates[i] = getAlphaFromR(dRplate);
        betaFromPlates[i] = getBetaFromR(dRplate);
    }
}

/**
 * @brief Get the Alpha of all the segments
 * 
 */
static void getAlpha(float (&alphaCandi)[SEGNUM])
{
    for (int i = 0; i < SEGNUM; i++)
    {
        alpha[i] = alphaCandi[i];
        alphaDeg[i] = alpha[i] * rad2deg;
    }
}

/**
 * @brief Get the Beta af all the segments
 * 
 */
static void getBeta(float (&betaCandi)[SEGNUM])
{
    for (int i = 0; i < SEGNUM; i++)
    {
        beta[i] = betaCandi[i];
        betaDeg[i] = betaCandi[i] * rad2deg;
    }
}

/**
 * @brief Get the Length of all the segments
 * 
 */
static void getLength()
{
    float lengthSum = 0;
    for (int i = 0; i < SEGNUM; i++)
    {
        lengthSum = 0;
        for (int j = 0; j < ACTNUM; j++)
        {
            lengthSum += lengthActuator[i][j];
        }
        lengthMM[i] = lengthSum / ACTNUM;
        length[i] = lengthMM[i] / 1000.0f;
    }
}

/**
 * @brief Estmate the alpha, beta, length from IMU data for all the segments
 * 
 */
void estimateABL()
{
    /*Calculation of useful matrices*/
    getRtInArm();
    getdRtInArm();
    getddRtInArm();

    /*Two methods to obtain alpha and beta*/
    getAlphaBetaFromActuators();
    getAlphaBetaFromPlates();

    /*Choose a result for alpha and beta*/
    getAlpha(alphaFromActuators);
    getBeta(betaFromActuators);

    /*estimate length*/
    getLength();
}

int main(int argc, char **argv)
{



    ros::init(argc, argv, "State_Estimator_Node");

    IMUDataPath = ros::package::getPath("origarm_ros") + "/predefined_param/";
    cout << "IMUDataPath:" << IMUDataPath << endl;

    InitFrames();

    State_Estimator stateEstimator;

    ros::AsyncSpinner s(2);

    s.start();

    ros::Rate loop_rate(100);

    ROS_INFO("Ready for State_Estimator_Node");



    int printFre = 0;
    while (ros::ok())
    {

        estimateABL();

        stateEstimator.pub();

        if (printFre++ > 2)
        {
            printf("A[0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d, [5]:%d  | B[0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d, [5]:%d  |  L[0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d, [5]:%d\n",
                   (int)(alphaDeg[0]),
                   (int)(alphaDeg[1]),
                   (int)(alphaDeg[2]),
                   (int)(alphaDeg[3]),
                   (int)(alphaDeg[4]),
                   (int)(alphaDeg[5]),
                   (int)(betaDeg[0]),
                   (int)(betaDeg[1]),
                   (int)(betaDeg[2]),
                   (int)(betaDeg[3]),
                   (int)(betaDeg[4]),
                   (int)(betaDeg[5]),
                   (int)(lengthMM[0]),
                   (int)(lengthMM[1]),
                   (int)(lengthMM[2]),
                   (int)(lengthMM[3]),
                   (int)(lengthMM[4]),
                   (int)(lengthMM[5]));
            printFre = 0;
        }

#if DEBUG_DISPLAY == 1
        displayAllMatrix();
#endif
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
