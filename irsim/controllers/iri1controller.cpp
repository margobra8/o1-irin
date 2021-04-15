/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <cstdio>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "encodersensor.h"
#include "bluebatterysensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"

/******************************************************************************/
/******************************************************************************/

extern gsl_rng *rng;
extern long int rngSeed;

const int mapGridX = 30;
const int mapGridY = 30;
double mapLengthX = 6.0;
double mapLengthY = 6.0;
int robotStartGridX = 2;
int robotStartGridY = 27;

int maxPacketsHigh = 5;
int maxPacketsLow = 5;
int maxPacketCapacity = 3;

vector<dVector2> dropPositionsHigh = {{-2.0, -1.5}, {-1.0, -1.5}, {-1.5, 1.55}, {1.25, 1.95}, {2.5, 1.45}};
vector<dVector2> dropPositionsLow = {{-0.5, -2.5}, {-1.35, -1.5}, {-2.25, 2.5}, {2, -0.5}, {2.75, -2.75}};
dVector2 startingPosition = {-2.5, -2.5};

vector<dVector2> dropPositionsHighQueue = dropPositionsHigh;
vector<dVector2> dropPositionsLowQueue = dropPositionsLow;

vector<dVector2> dropQueue;
//vector<dVector2> dropDelivered(1);

const int n = mapGridX; // horizontal size of the map
const int m = mapGridY; // vertical size size of the map
static int map[n][m];
static int onlineMap[n][m];
static int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
static int open_nodes_map[n][m];   // map of open (not-yet-tried) nodes
static int dir_map[n][m];          // map of directions
const int dir = 8;                 // number of possible directions to go at any position
//if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dx[dir] = {1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir] = {0, 1, 1, 1, 0, -1, -1, -1};

#define ERROR_DIRECTION 0.05
#define ERROR_POSITION 0.02
/******************************************************************************/
/******************************************************************************/

using namespace std;
/******************************************************************************/
/******************************************************************************/

#define BEHAVIORS 7

#define AVOID_PRIORITY 0
#define GO_HOME_PRIORITY 1
#define RELOAD_PRIORITY 2
#define DELIVERHIGH_PRIORITY 3
#define DELIVERLOW_PRIORITY 4
#define NAVIGATE_PRIORITY 5
#define GO_GOAL_PRIORITY 6

/* Threshold to avoid obstacles */
#define PROXIMITY_THRESHOLD 0.7
/* Threshold to define the battery discharged */
#define BATTERY_THRESHOLD 0.5
/* Threshold to reduce the speed of the robot */
#define NAVIGATE_LIGHT_THRESHOLD 0.9
/* Threshold to determine proximity to light */
#define LIGHT_PROXIMITY_THRESHOLD 0.5
#define BLUE_LIGHT_PROXIMITY_THRESHOLD 0.85

#define SPEED 800

#define NO_OBSTACLE 0
#define OBSTACLE 1
#define START 2
#define PATH 3
#define END 4
#define NEST 5
#define PREY 6
/******************************************************************************/
/******************************************************************************/

class node
{
  // current position
  int xPos;
  int yPos;
  // total distance already travelled to reach the node
  int level;
  // priority=level+remaining distance estimate
  int priority; // smaller: higher priority

public:
  node(int xp, int yp, int d, int p)
  {
    xPos = xp;
    yPos = yp;
    level = d;
    priority = p;
  }

  int getxPos() const { return xPos; }
  int getyPos() const { return yPos; }
  int getLevel() const { return level; }
  int getPriority() const { return priority; }

  void updatePriority(const int &xDest, const int &yDest)
  {
    priority = level + estimate(xDest, yDest) * 10; //A*
  }

  // give better priority to going strait instead of diagonally
  void nextLevel(const int &i) // i: direction
  {
    level += (dir == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
  }

  // Estimation function for the remaining distance to the goal.
  const int &estimate(const int &xDest, const int &yDest) const
  {
    static int xd, yd, d;
    xd = xDest - xPos;
    yd = yDest - yPos;

    // Euclidian Distance
    d = static_cast<int>(sqrt(xd * xd + yd * yd));

    // Manhattan distance
    //d=abs(xd)+abs(yd);

    // Chebyshev distance
    //d=max(abs(xd), abs(yd));

    return (d);
  }
};

/******************************************************************************/
/******************************************************************************/

// Determine priority (in the priority queue)
bool operator<(const node &a, const node &b)
{
  return a.getPriority() > b.getPriority();
}

/******************************************************************************/
/******************************************************************************/
CIri1Controller::CIri1Controller(const char *pch_name, CEpuck *pc_epuck, int n_write_to_file) : CController(pch_name, pc_epuck)

{
  m_nWriteToFile = n_write_to_file;

  /* Set epuck */
  m_pcEpuck = pc_epuck;
  /* Set Wheels */
  m_acWheels = (CWheelsActuator *)m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
  /* Set Prox Sensor */
  m_seProx = (CEpuckProximitySensor *)m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
  /* Set light Sensor */
  m_seLight = (CRealLightSensor *)m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
  /* Set blue light Sensor */
  m_seBlueLight = (CRealBlueLightSensor *)m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
  /* Set red light Sensor */
  m_seRedLight = (CRealRedLightSensor *)m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
  /* Set contact Sensor */
  m_seContact = (CContactSensor *)m_pcEpuck->GetSensor(SENSOR_CONTACT);
  /* Set ground Sensor */
  m_seGround = (CGroundSensor *)m_pcEpuck->GetSensor(SENSOR_GROUND);
  /* Set ground memory Sensor */
  m_seGroundMemory = (CGroundMemorySensor *)m_pcEpuck->GetSensor(SENSOR_GROUND_MEMORY);
  /* Set battery Sensor */
  m_seBattery = (CBatterySensor *)m_pcEpuck->GetSensor(SENSOR_BATTERY);
  /* Set blue battery Sensor */
  m_seBlueBattery = (CBlueBatterySensor *)m_pcEpuck->GetSensor(SENSOR_BLUE_BATTERY);
  /* Set encoder Sensor */
  m_seEncoder = (CEncoderSensor *)m_pcEpuck->GetSensor(SENSOR_ENCODER);
  m_seEncoder->InitEncoderSensor(m_pcEpuck);

  /* Initialize Motor Variables */
  m_fLeftSpeed = 0.0;
  m_fRightSpeed = 0.0;

  /* Initialize Inhibitors */
  fDeliverToGoalInhibitor = 1.0;
  fDeliverHighToDeliverLowInhibitor = 1.0;
  fLoadInhibitor = 1.0;
  fFinalInhibitor = 1.0;

  string mapString = "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
                     "%############################%"
                     "%############################%"
                     "%#######%%%%%%%%#############%"
                     "%#######%######%#############%"
                     "%############################%"
                     "%############################%"
                     "%#################%%%%%%%%###%"
                     "%#################%##########%"
                     "%#################%##########%"
                     "%############################%"
                     "%############################%"
                     "%###%%%%%%%%#################%"
                     "%###%%%%%%%%#################%"
                     "%###%%%%%%%%#################%"
                     "%#############%%%%%%%%%%%%###%"
                     "%###########%%%##############%"
                     "%###########%################%"
                     "%###########%%%%%%%##########%"
                     "%############################%"
                     "%############################%"
                     "%############################%"
                     "%############################%"
                     "%################%%%%%%%%%###%"
                     "%#########%##################%"
                     "%#####%###%##################%"
                     "%#####%%%%%#######%%%%%%%####%"
                     "%############################%"
                     "%############################%"
                     "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";

  m_cHeightMap = new char[mapString.length() + 1];
  m_cHeightMap = mapString.c_str();

  for (int y = 0; y < m; y++)
    for (int x = 0; x < n; x++)
      switch (*(m_cHeightMap + x + (mapGridY - 1 - y) * mapGridX))
      {
      case '%':
        map[x][y] = OBSTACLE;
        break;
      default:
        map[x][y] = NO_OBSTACLE;
        break;
      }

  /* Initialize Activation Table */
  m_fActivationTable = new double *[BEHAVIORS];
  for (int i = 0; i < BEHAVIORS; i++)
  {
    m_fActivationTable[i] = new double[3];
  }

  /* Odometry */
  m_nState = 0;
  m_nPathPlanningStops = 0;
  m_fOrientation = 0.0;

  m_rPosition.x = startingPosition.x;
  m_rPosition.y = startingPosition.y;
  m_vPosition.x = 0.0;
  m_vPosition.y = 0.0;
  /* Set Actual Position to robot Start Grid */
  m_nRobotActualGridX = robotStartGridX;
  m_nRobotActualGridY = robotStartGridY;

  fXmov = mapLengthX / ((double)mapGridX);
  fYmov = mapLengthY / ((double)mapGridY);

  /* DEBUG */
  PrintMap(&map[0][0]);
  /* DEBUG */

  m_nPacketsDelivered = new int[2];
  m_nPacketsDelivered[0] = 0;
  m_nPacketsDelivered[1] = 0;
  m_nPacketsLoaded = maxPacketCapacity;

  /* Initialize PAthPlanning Flag*/
  m_nPathPlanningDone = 0;
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
  for (int i = 0; i < BEHAVIORS; i++)
  {
    delete[] m_fActivationTable;
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
  /* Move time to global variable, so it can be used by the bahaviors to write to files*/
  m_fTime = f_time;

  /* Execute the levels of competence */
  ExecuteBehaviors();

  /* Execute Coordinator */
  Coordinator();

  /* Set Speed to wheels */
  m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);

  if (m_nWriteToFile)
  {
    /* INIT: WRITE TO FILES */
    /* Write robot position and orientation */
    FILE *filePosition = fopen("outputFiles/robotPosition", "a");
    fprintf(filePosition, "%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_pcEpuck->GetPosition().x, m_pcEpuck->GetPosition().y, m_pcEpuck->GetRotation());
    fclose(filePosition);

    /* Write robot wheels speed */
    FILE *fileWheels = fopen("outputFiles/robotWheels", "a");
    fprintf(fileWheels, "%2.4f %2.4f %2.4f \n", m_fTime, m_fLeftSpeed, m_fRightSpeed);
    fclose(fileWheels);
    /* END WRITE TO FILES */
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ExecuteBehaviors(void)
{
  for (int i = 0; i < BEHAVIORS; i++)
  {
    m_fActivationTable[i][2] = 0.0;
  }

  /* Release Inhibitors */
  fDeliverToGoalInhibitor = 1.0;
  fDeliverHighToDeliverLowInhibitor = 1.0;
  fLoadInhibitor = 1.0;
  fFinalInhibitor = 1.0;

  /* Set Leds to BLACK */
  m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLACK);

  /* Execute Behaviors */
  ObstacleAvoidance(AVOID_PRIORITY);
  GoHome(GO_HOME_PRIORITY);
  GoLoad(RELOAD_PRIORITY);

  ComputeActualCell(GO_GOAL_PRIORITY);
  PathPlanning(GO_GOAL_PRIORITY);

  DeliverHigh(DELIVERHIGH_PRIORITY);
  DeliverLow(DELIVERLOW_PRIORITY);

  GoGoal(GO_GOAL_PRIORITY);

  Navigate(NAVIGATE_PRIORITY);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Coordinator(void)
{
  /* Create counter for behaviors */
  int nBehavior;
  /* Create angle of movement */
  double fAngle = 0.0;
  /* Create vector of movement */
  dVector2 vAngle;
  vAngle.x = 0.0;
  vAngle.y = 0.0;

  /* For every Behavior */
  for (nBehavior = 0; nBehavior < BEHAVIORS; nBehavior++)
  {
    /* If behavior is active */
    if (m_fActivationTable[nBehavior][2] == 1.0)
    {
      /* DEBUG */
      printf("Behavior %d: %2f\n", nBehavior, m_fActivationTable[nBehavior][0]);
      /* DEBUG */
      /* INIT: WRITE TO FILES */
      vAngle.x += m_fActivationTable[nBehavior][1] * cos(m_fActivationTable[nBehavior][0]);
      vAngle.y += m_fActivationTable[nBehavior][1] * sin(m_fActivationTable[nBehavior][0]);
    }
  }
  for (int i = 0; i < BEHAVIORS; i++)
  {
    /* Write coordinator ouputs */
    if (m_fActivationTable[i][2] == 1.0)
    {
      FILE *fileOutput = fopen("outputFiles/behaviorOutput", "a");
      fprintf(fileOutput, "%2.4f %d\n", m_fTime, i);
      fclose(fileOutput);
      break;
      /* END WRITE TO FILES */
    }
  }

  /* Calc angle of movement */
  fAngle = atan2(vAngle.y, vAngle.x);
  /* DEBUG */
  printf("fAngle: %2f\n", fAngle);
  printf("\n");
  /* DEBUG */

  if (fAngle > 0)
  {
    m_fLeftSpeed = SPEED * (1 - fmin(fAngle, ERROR_DIRECTION) / ERROR_DIRECTION);
    m_fRightSpeed = SPEED;
  }
  else
  {
    m_fLeftSpeed = SPEED;
    m_fRightSpeed = SPEED * (1 - fmin(-fAngle, ERROR_DIRECTION) / ERROR_DIRECTION);
  }

  if (m_nWriteToFile)
  {
    /* INIT: WRITE TO FILES */
    /* Write coordinator ouputs */
    FILE *fileOutput = fopen("outputFiles/coordinatorOutput", "a");
    fprintf(fileOutput, "%2.4f %d %2.4f %2.4f \n", m_fTime, nBehavior, m_fLeftSpeed, m_fRightSpeed);
    fclose(fileOutput);
    /* END WRITE TO FILES */
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ObstacleAvoidance(unsigned int un_priority)
{
  /* Leer Sensores de Proximidad */
  double *prox = m_seProx->GetSensorReading(m_pcEpuck);

  double fMaxProx = 0.0;
  const double *proxDirections = m_seProx->GetSensorDirections();

  dVector2 vRepelent;
  vRepelent.x = 0.0;
  vRepelent.y = 0.0;

  /* Calc vector Sum */
  for (int i = 0; i < m_seProx->GetNumberOfInputs(); i++)
  {
    vRepelent.x += prox[i] * cos(proxDirections[i]);
    vRepelent.y += prox[i] * sin(proxDirections[i]);

    if (prox[i] > fMaxProx)
      fMaxProx = prox[i];
  }

  /* Calc pointing angle */
  float fRepelent = atan2(vRepelent.y, vRepelent.x);
  /* Create repelent angle */
  fRepelent -= M_PI;
  /* Normalize angle */
  while (fRepelent > M_PI)
    fRepelent -= 2 * M_PI;
  while (fRepelent < -M_PI)
    fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = 0.2 / (1 + exp(-fMaxProx));

  /* If above a threshold */
  if (fMaxProx > PROXIMITY_THRESHOLD)
  {
    /* Set Leds to GREEN */
    m_pcEpuck->SetAllColoredLeds(LED_COLOR_GREEN);
    /* Mark Behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;
  }

  if (m_nWriteToFile)
  {
    /* INIT WRITE TO FILE */
    /* Write level of competence ouputs */
    FILE *fileOutput = fopen("outputFiles/avoidOutput", "a");
    fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, prox[0], prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7], fMaxProx, fRepelent);
    fprintf(fileOutput, "%2.4f %2.4f %2.4f\n", m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
    fclose(fileOutput);
    /* END WRITE TO FILE */
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoHome(unsigned int un_priority)
{
  /* Leer Sensores de Luz */
  double *blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);

  double fMaxLight = 0.0;
  const double *lightDirections = m_seBlueLight->GetSensorDirections();

  /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
  dVector2 vRepelent;
  vRepelent.x = 0.0;
  vRepelent.y = 0.0;

  /* Calc vector Sum */
  for (int i = 0; i < m_seBlueLight->GetNumberOfInputs(); i++)
  {
    vRepelent.x += blueLight[i] * cos(lightDirections[i]);
    vRepelent.y += blueLight[i] * sin(lightDirections[i]);

    if (blueLight[i] > fMaxLight)
      fMaxLight = blueLight[i];
  }

  /* Calc pointing angle */
  float fRepelent = atan2(vRepelent.y, vRepelent.x);

  /* Normalize angle */
  while (fRepelent > M_PI)
    fRepelent -= 2 * M_PI;
  while (fRepelent < -M_PI)
    fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = 0.1 / (1 + exp(-fMaxLight));

  if (m_nPacketsDelivered[0] == maxPacketsHigh && m_nPacketsDelivered[1] == maxPacketsLow)
  {
    /* Enable Final Inhibitor */
    fFinalInhibitor = 0.0;
    m_fActivationTable[un_priority][2] = 1.0;
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Navigate(unsigned int un_priority)
{
  /* Direction Angle 0.0 and always active. We set its vector intensity to 0.5 if used */
  m_fActivationTable[un_priority][0] = 0.0;
  m_fActivationTable[un_priority][1] = 0.1;
  m_fActivationTable[un_priority][2] = 1.0;

  if (m_nWriteToFile)
  {
    /* INIT: WRITE TO FILES */
    /* Write level of competence ouputs */
    FILE *fileOutput = fopen("outputFiles/navigateOutput", "a");
    fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
    fclose(fileOutput);
    /* END WRITE TO FILES */
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoLoad(unsigned int un_priority)
{
  /* Leer Battery Sensores */
  double *blueBattery = m_seBlueBattery->GetSensorReading(m_pcEpuck);

  /* Leer Sensores de Luz */
  double *blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);

  double fMaxLight = 0.0;
  const double *lightDirections = m_seBlueLight->GetSensorDirections();

  /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
  dVector2 vRepelent;
  vRepelent.x = 0.0;
  vRepelent.y = 0.0;

  /* Calc vector Sum */
  for (int i = 0; i < m_seBlueLight->GetNumberOfInputs(); i++)
  {
    vRepelent.x += blueLight[i] * cos(lightDirections[i]);
    vRepelent.y += blueLight[i] * sin(lightDirections[i]);

    if (blueLight[i] > fMaxLight)
      fMaxLight = blueLight[i];
  }

  /* Calc pointing angle */
  float fRepelent = atan2(vRepelent.y, vRepelent.x);

  /* Normalize angle */
  while (fRepelent > M_PI)
    fRepelent -= 2 * M_PI;
  while (fRepelent < -M_PI)
    fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = 0.1 / (1 + exp(-fMaxLight));
  printf("Blue Battery: %f, Current Packets in the van: %d\n", blueBattery[0], m_nPacketsLoaded);

  /* If battery below a BATTERY_THRESHOLD or there are no packets left in the trunk */
  if (blueBattery[0] < BATTERY_THRESHOLD || m_nPacketsLoaded <= 0)
  {
    /* Inhibit all other behaviors dependent of battery or having at least a packet in the trunk*/
    fLoadInhibitor = 0.0;
    /* Set Leds to BLUE */
    m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);

    /* Mark behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;
  }

  if (m_nWriteToFile)
  {
    /* INIT WRITE TO FILE */
    FILE *fileOutput = fopen("outputFiles/loadOutput", "a");
    fprintf(fileOutput, "%2.4f %d %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, m_nPacketsLoaded, blueBattery[0], blueLight[0], blueLight[1], blueLight[2], blueLight[3], blueLight[4], blueLight[5], blueLight[6], blueLight[7]);
    fprintf(fileOutput, "%2.4f %2.4f %2.4f\n", m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
    fclose(fileOutput);
    /* END WRITE TO FILE */
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::DeliverHigh(unsigned int un_priority)
{
  //MODIFIED
  //nota3: apagar luz foragehigh cuando esté en el umbral, en ese mismo punto también cambiar el
  //dropPositionsHigh y aumentar los packetsdelivered y pararse (inhibe goGoal) y quitarlo de la variable maletero
  //de nuestro robot
  // nota4: inhibir ForageLow hasta que no se hayan entregado todo los paquetes high (hasta entonces en la activation table no se pondría active)
  //IDEM EN EL LOW
  /* Leer Sensores de Luz */
  double *light = m_seLight->GetSensorReading(m_pcEpuck);

  double fMaxLight = 0.0;
  const double *lightDirections = m_seLight->GetSensorDirections();

  /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
  dVector2 vRepelent;
  vRepelent.x = 0.0;
  vRepelent.y = 0.0;

  unsigned int un_noInputs = m_seLight->GetNumberOfInputs();

  /* Calc vector Sum */
  for (int i = 0; i < un_noInputs; i++)
  {
    vRepelent.x += light[i] * cos(lightDirections[i]);
    vRepelent.y += light[i] * sin(lightDirections[i]);

    if (light[i] > fMaxLight)
      fMaxLight = light[i];
  }

  /* Calc pointing angle */
  float fRepelent = atan2(vRepelent.y, vRepelent.x);

  /* Normalize angle */
  while (fRepelent > M_PI)
    fRepelent -= 2 * M_PI;
  while (fRepelent < -M_PI)
    fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = 0.1 / (1 + exp(-fMaxLight));

  double fLightIntensity;

  for (int i = 0; i < un_noInputs; i++)
  {
    fLightIntensity += light[i];
  }

  fLightIntensity /= 2;
  printf("fLightIntensity Yellow: %f\n", fLightIntensity);
  printf("DeliveredHigh = %d, DeliveredLow = %d\n", m_nPacketsDelivered[0], m_nPacketsDelivered[1]);

  if (m_nPacketsDelivered[0] < maxPacketsHigh)
  {
    /* Enable DeliverHighToDeliverLowInhibitor */
    fDeliverHighToDeliverLowInhibitor = 0.0;
  }
  int packetsAlreadyDelivered = m_nPacketsDelivered[0];
  /*Primero comprobar si faltan de los de high, incrementar el numero de paquetes,y después comprobar si el
  que se acaba de coger es el último de los de alta prioridad para que sepa que lo siguiente es ir a por los
  de baja e ir a por los de baja*/
  if ((fLightIntensity > 0.001) && (m_nPacketsDelivered[0] < maxPacketsHigh) && (fLoadInhibitor * fFinalInhibitor == 1.0))
  {
    /* Set Leds to YELLOW */
    m_pcEpuck->SetAllColoredLeds(LED_COLOR_YELLOW);
    /* Enable Deliver to Goal inhibitor*/
    fDeliverToGoalInhibitor = 0.0;
    /* Mark Behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;

    if (fLightIntensity >= LIGHT_PROXIMITY_THRESHOLD)
    {
      /* Another High Priority Packet is delivered */
      m_nPacketsDelivered[0]++;
      dropPositionsHighQueue.erase(dropPositionsHighQueue.begin());

      /* We have one less packet in our trunk */
      m_nPacketsLoaded--;

      /* We turn off the light representing the delivery point */
      m_seLight->SwitchNearestLight(0);
    }
  }

  if (packetsAlreadyDelivered == maxPacketsHigh)
  {
    m_fActivationTable[un_priority][2] = 0.0;
  }

  if (m_nWriteToFile)
  {
    /* INIT WRITE TO FILE */
    FILE *fileOutput = fopen("outputFiles/deliverHighOutput", "a");
    fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, fLoadInhibitor, m_nPacketsDelivered[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
    fprintf(fileOutput, "%2.4f %2.4f %2.4f\n", m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
    fclose(fileOutput);
    /* END WRITE TO FILE */
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::DeliverLow(unsigned int un_priority)
{
  //MODIFIED

  /* Leer Sensores de Luz Roja */
  double *redLight = m_seRedLight->GetSensorReading(m_pcEpuck);

  double fMaxLight = 0.0;
  const double *redLightDirections = m_seRedLight->GetSensorDirections();

  /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
  dVector2 vRepelent;
  vRepelent.x = 0.0;
  vRepelent.y = 0.0;

  /* Calc vector Sum */
  unsigned int un_noInputs = m_seRedLight->GetNumberOfInputs();

  for (int i = 0; i < un_noInputs; i++)
  {
    vRepelent.x += redLight[i] * cos(redLightDirections[i]);
    vRepelent.y += redLight[i] * sin(redLightDirections[i]);

    if (redLight[i] > fMaxLight)
      fMaxLight = redLight[i];
  }

  /* Calc pointing angle */
  float fRepelent = atan2(vRepelent.y, vRepelent.x);

  /* Normalize angle */
  while (fRepelent > M_PI)
    fRepelent -= 2 * M_PI;
  while (fRepelent < -M_PI)
    fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = 0.1 / (1 + exp(-fMaxLight));

  double fLightIntensity;

  for (int i = 0; i < un_noInputs; i++)
  {
    fLightIntensity += redLight[i];
  }

  fLightIntensity /= 2;
  printf("fLightIntensity Red: %f\n", fLightIntensity);
  printf("DeliveredHigh = %d, DeliveredLow = %d\n", m_nPacketsDelivered[0], m_nPacketsDelivered[1]);

  int packetsAlreadyDelivered = m_nPacketsDelivered[1];

  if ((fLightIntensity > 0.001) && (m_nPacketsDelivered[1] < maxPacketsLow) && (fLoadInhibitor * fFinalInhibitor * fDeliverHighToDeliverLowInhibitor == 1.0))
  {
    /* Set Leds to RED */
    m_pcEpuck->SetAllColoredLeds(LED_COLOR_RED);
    /* Enable Deliver to Goal inhibitor*/
    fDeliverToGoalInhibitor = 0.0;
    /* Mark Behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;

    if (fLightIntensity >= LIGHT_PROXIMITY_THRESHOLD)
    {
      /* Another High Priority Packet is delivered */
      m_nPacketsDelivered[1]++;
      dropPositionsLowQueue.erase(dropPositionsLowQueue.begin());

      /* We have one less packet in our trunk */
      m_nPacketsLoaded--;
      /* We turn off the light representing the delivery point */
      m_seRedLight->SwitchNearestLight(0);
    }
  }

  if (packetsAlreadyDelivered == maxPacketsLow)
  {
    m_fActivationTable[un_priority][2] = 0.0;
  }

  if (m_nWriteToFile)
  {
    /* INIT WRITE TO FILE */
    FILE *fileOutput = fopen("outputFiles/deliverLowOutput", "a");
    fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, fLoadInhibitor, m_nPacketsDelivered[1], redLight[0], redLight[1], redLight[2], redLight[3], redLight[4], redLight[5], redLight[6], redLight[7]);
    fprintf(fileOutput, "%2.4f %2.4f %2.4f\n", m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
    fclose(fileOutput);
    /* END WRITE TO FILE */
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ComputeActualCell(unsigned int un_priority)
{
  /* Leer Encoder */
  double *encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
  /* Leer sensores de luz azul */
  //double *groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
  double *blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);

  CalcPositionAndOrientation(encoder);

  /* DEBUG */
  //printf("POS: X: %2f, %2f\r", m_vPosition.x, m_vPosition.y );
  /* DEBUG */

  /* Calc increment of position, correlating grid and metrics */
  // double fXmov = mapLengthX / ((double)mapGridX);
  // double fYmov = mapLengthY / ((double)mapGridY);
  fXmov = mapLengthX / ((double)mapGridX);
  fYmov = mapLengthY / ((double)mapGridY);

  /* Compute X grid */
  double tmp = m_vPosition.x;
  tmp += robotStartGridX * fXmov + double(0.5 * fXmov);
  m_nRobotActualGridX = (int)(tmp / fXmov);

  /* Compute Y grid */
  tmp = -m_vPosition.y;
  tmp += robotStartGridY * fYmov + double(0.5 * fYmov);
  m_nRobotActualGridY = (int)(tmp / fYmov);

  /* DEBUG */
  printf("GRID: X: %d, %d\n", m_nRobotActualGridX, m_nRobotActualGridY);
  /* DEBUG */

  double fLightIntensity;
  unsigned int un_noInputs = m_seBlueLight->GetNumberOfInputs();

  for (int i = 0; i < un_noInputs; i++)
  {
    fLightIntensity += blueLight[i];
  }

  fLightIntensity /= 2;
  printf("fLightIntensity Blue: %f\n", fLightIntensity);
  if (fLightIntensity >= BLUE_LIGHT_PROXIMITY_THRESHOLD)
  {
    /* Asumme Path Planning is done */
    m_nPathPlanningDone = 0;
    /* Restart PathPlanning state */
    m_nState = 0;

    /* Reload trunk with new set of packets */
    m_nPacketsLoaded = maxPacketCapacity;

    /* DEBUG */
    PrintMap(&map[0][0]);
    /* DEBUG */
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::PathPlanning(unsigned int un_priority)
{
  if (m_nPathPlanningDone == 0)
  {

    m_nPathPlanningStops = 0;
    m_fActivationTable[un_priority][2] = 1;

    string route = "";

    vector<int> start = {m_nRobotActualGridX, m_nRobotActualGridY};

    vector<vector<int>> dropQueueCells = {start};

    /* Generate queue of next drop points in map (dVector2 type) */
    dropQueue.clear();
    computeDeliveriesQueues(un_priority, dropPositionsHighQueue, dropPositionsLowQueue);

    for (int i = 0; i < dropQueue.size(); i++)
    {
      vector<int> lCell;
      lCell = toMapCell(dropQueue[i]);
      dropQueueCells.push_back(lCell);
    }

    for (int i = 1; i < dropQueueCells.size(); i++)
    {
      /* Obtain optimal paths */
      route.append(pathFind(dropQueueCells[i - 1][0], dropQueueCells[i - 1][1], dropQueueCells[i][0], dropQueueCells[i][1]));
      /* DEBUG */
      printf("STEP: %d - START: %d, %d - END: %d, %d\n", i - 1, dropQueueCells[i - 1][0], dropQueueCells[i - 1][1], dropQueueCells[i][0], dropQueueCells[i][1]);
      /* DEBUG */
    }
    /* DEBUG */
    if (route == "")
      cout << "An empty route generated!" << endl;
    cout << "Route:" << route << endl;
    printf("route Length: %d\n", route.length());

    // /* Obtain Map */
    // for (int y = 0; y < m; y++)
    //   for (int x = 0; x < n; x++)
    //     if (onlineMap[x][y] != NO_OBSTACLE && onlineMap[x][y] != NEST && onlineMap[x][y] != PREY)
    //       map[x][y] = OBSTACLE;

    /* DEBUG */

    /* Obtain number of changing directions */
    for (int i = 1; i < route.length(); i++)
      if (route[i - 1] != route[i])
        m_nPathPlanningStops++;

    /* Add last movement */
    m_nPathPlanningStops++;
    /* DEBUG */
    printf("STOPS: %d\n", m_nPathPlanningStops);
    /* DEBUG */

    /* Define vector of desired positions. One for each changing direction */
    m_vPositionsPlanning = new dVector2[m_nPathPlanningStops];

    /* Calc increment of position, correlating grid and metrics */
    fXmov = mapLengthX / (double)mapGridX;
    fYmov = mapLengthY / (double)mapGridY;

    /* Get actual position */
    dVector2 actualPos;
    //actualPos.x = robotStartGridX * fXmov;
    actualPos.x = m_nRobotActualGridX * fXmov;
    //actualPos.y = robotStartGridY * fYmov;
    actualPos.y = m_nRobotActualGridY * fYmov;

    /* Fill vector of desired positions */
    int stop = 0;
    int counter = 0;
    /* Check the route and obtain the positions*/
    for (int i = 1; i < route.length(); i++)
    {
      /* For every position in route, increment countr */
      counter++;
      /* If a direction changed */
      if ((route[i - 1] != route[i]))
      {
        /* Obtain the direction char */
        char c;
        c = route.at(i - 1);

        /* Calc the new stop according to actual position and increment based on the grid */
        m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov * dx[atoi(&c)];
        m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov * dy[atoi(&c)];

        /* Update position for next stop */
        actualPos.x = m_vPositionsPlanning[stop].x;
        actualPos.y = m_vPositionsPlanning[stop].y;

        /* Increment stop */
        stop++;
        /* reset counter */
        counter = 0;
      }

      /* If we are in the last update, calc last movement */
      if (i == (route.length() - 1))
      {
        /* Increment counter */
        counter++;
        /* Obtain the direction char */
        char c;
        c = route.at(i);

        /* DEBUG */
        //printf("COUNTER: %d, CHAR: %c\n", counter, c);
        /* END DEBUG */

        /* Calc the new stop according to actual position and increment based on the grid */
        m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov * dx[atoi(&c)]; // - robotStartGridX * fXmov;
        m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov * dy[atoi(&c)]; // - robotStartGridY * fYmov;

        /* Update position for next stop */
        actualPos.x = m_vPositionsPlanning[stop].x;
        actualPos.y = m_vPositionsPlanning[stop].y;

        /* Increment stop */
        stop++;
        /* reset counter */
        counter = 0;
      }
    }

    /* DEBUG */
    if (route.length() > 0)
    {
      int j;
      char c;
      int x = m_nRobotActualGridX;
      int y = m_nRobotActualGridY;
      map[x][y] = START;
      for (int i = 0; i < route.length(); i++)
      {
        c = route.at(i);
        j = atoi(&c);
        x = x + dx[j];
        y = y + dy[j];
        map[x][y] = PATH;
      }
      map[x][y] = END;

      //PrintMap(&map[0][0]);
      printf("\n\n");
      PrintMap(&map[0][0]);
    }
    /* END DEBUG */

    /* DEBUG */
    //printf("Start: %2f, %2f\n", m_nRobotActualGridX * fXmov, m_nRobotActualGridY * fXmov);
    //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
    //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
    /* END DEBUG */

    /* Convert to simulator coordinates */
    for (int i = 0; i < m_nPathPlanningStops; i++)
    {
      m_vPositionsPlanning[i].x -= (mapGridX * fXmov) / 2;
      m_vPositionsPlanning[i].y -= (mapGridY * fYmov) / 2;
      m_vPositionsPlanning[i].y = -m_vPositionsPlanning[i].y;
    }
    /* DEBUG */
    //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
    //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
    /* END DEBUG */

    /* Convert to robot coordinates. FAKE!!. */
    /* Notice we are only working with initial orientation = 0.0 */
    for (int i = 0; i < m_nPathPlanningStops; i++)
    {
      /* Traslation */
      m_vPositionsPlanning[i].x -= ((robotStartGridX * fXmov) - (mapGridX * fXmov) / 2);
      m_vPositionsPlanning[i].y += ((robotStartGridY * fXmov) - (mapGridY * fYmov) / 2);
      /* Rotation */
      //double compass = m_pcEpuck->GetRotation();
      //m_vPositionsPlanning[i].x = m_vPositionsPlanning[i].x * cos (compass) - m_vPositionsPlanning[i].y  * sin(compass);
      //m_vPositionsPlanning[i].y = m_vPositionsPlanning[i].x * sin (compass) + m_vPositionsPlanning[i].y  * cos(compass);
    }
    /* DEBUG */
    //for (int i = 0 ; i < m_nPathPlanningStops ; i++)
    //printf("MOV %d: %2f, %2f\n", i, m_vPositionsPlanning[i].x, m_vPositionsPlanning[i].y);
    /* END DEBUG */

    m_nPathPlanningDone = 1;
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoGoal(unsigned int un_priority)
{
  printf("Delivertogoal inhibitor %f\n", fDeliverToGoalInhibitor);
  if ((fFinalInhibitor * fLoadInhibitor * fDeliverToGoalInhibitor * m_nPathPlanningDone == 1))
  {
    /* If something not found at the end of planning, reset plans */
    if (m_nState >= m_nPathPlanningStops)
    {
      printf(" --------------- LOST!!!!!!!! --------------\n");
      m_nPathPlanningDone = 0;
      m_nState = 0;
      return;
    }

    /* DEBUG */
    printf("PlanningX: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].x, m_vPosition.x);
    printf("PlanningY: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].y, m_vPosition.y);
    /* DEBUG */

    double fX = (m_vPositionsPlanning[m_nState].x - m_vPosition.x);
    double fY = (m_vPositionsPlanning[m_nState].y - m_vPosition.y);
    double fGoalDirection = 0;

    /* If on Goal, return 1 */
    if ((fabs(fX) <= ERROR_POSITION) && (fabs(fY) <= ERROR_POSITION))
      m_nState++;

    fGoalDirection = atan2(fY, fX);

    /* Translate fGoalDirection into local coordinates */
    fGoalDirection -= m_fOrientation;
    /* Normalize Direction */
    while (fGoalDirection > M_PI)
      fGoalDirection -= 2 * M_PI;
    while (fGoalDirection < -M_PI)
      fGoalDirection += 2 * M_PI;

    m_fActivationTable[un_priority][0] = fGoalDirection;
    m_fActivationTable[un_priority][1] = 0.1;
    m_fActivationTable[un_priority][2] = 1;
  }
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::CalcPositionAndOrientation(double *f_encoder)
{
  /* DEBUG */
  //printf("Encoder: %2f, %2f\n", f_encoder[0], f_encoder[1]);
  /* DEBUG */

  /* Remake kinematic equations */
  double fIncU = (f_encoder[0] + f_encoder[1]) / 2;
  double fIncTetha = (f_encoder[1] - f_encoder[0]) / CEpuck::WHEELS_DISTANCE;

  /* Substitute arc by chord, take care of 0 division */
  if (fIncTetha != 0.0)
    fIncU = ((f_encoder[0] / fIncTetha) + (CEpuck::WHEELS_DISTANCE / 2)) * 2.0 * sin(fIncTetha / 2.0);

  /* Update new Position */
  double xInc = fIncU * cos(m_fOrientation + fIncTetha / 2);
  double yInc = fIncU * sin(m_fOrientation + fIncTetha / 2);
  m_vPosition.x += xInc;
  m_vPosition.y += yInc;
  m_rPosition.x += xInc;
  m_rPosition.y += yInc;
  // FIXME: added variable that has robot position w/ respect to sim coords

  /* Update new Orientation */
  m_fOrientation += fIncTetha;

  /* Normalize Orientation */
  while (m_fOrientation < 0)
    m_fOrientation += 2 * M_PI;
  while (m_fOrientation > 2 * M_PI)
    m_fOrientation -= 2 * M_PI;
}

/******************************************************************************/
/******************************************************************************/

// A-star algorithm.
// The route returned is a string of direction digits.
string CIri1Controller::pathFind(const int &xStart, const int &yStart,
                                 const int &xFinish, const int &yFinish)
{
  static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
  static int pqi;                    // pq index
  static node *n0;
  static node *m0;
  static int i, j, x, y, xdx, ydy;
  static char c;
  pqi = 0;

  // reset the node maps
  for (y = 0; y < m; y++)
  {
    for (x = 0; x < n; x++)
    {
      closed_nodes_map[x][y] = 0;
      open_nodes_map[x][y] = 0;
    }
  }

  // create the start node and push into list of open nodes
  n0 = new node(xStart, yStart, 0, 0);
  n0->updatePriority(xFinish, yFinish);
  pq[pqi].push(*n0);
  //open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

  // A* search
  while (!pq[pqi].empty())
  {
    // get the current node w/ the highest priority
    // from the list of open nodes
    n0 = new node(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                  pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

    x = n0->getxPos();
    y = n0->getyPos();

    pq[pqi].pop(); // remove the node from the open list
    open_nodes_map[x][y] = 0;
    // mark it on the closed nodes map
    closed_nodes_map[x][y] = 1;

    // quit searching when the goal state is reached
    //if((*n0).estimate(xFinish, yFinish) == 0)
    if (x == xFinish && y == yFinish)
    {
      // generate the path from finish to start
      // by following the directions
      string path = "";
      while (!(x == xStart && y == yStart))
      {
        j = dir_map[x][y];
        c = '0' + (j + dir / 2) % dir;
        path = c + path;
        x += dx[j];
        y += dy[j];
      }

      // garbage collection
      delete n0;
      // empty the leftover nodes
      while (!pq[pqi].empty())
        pq[pqi].pop();
      return path;
    }

    // generate moves (child nodes) in all possible directions
    for (i = 0; i < dir; i++)
    {
      xdx = x + dx[i];
      ydy = y + dy[i];

      if (!(xdx < 0 || xdx > n - 1 || ydy < 0 || ydy > m - 1 || map[xdx][ydy] == 1 || closed_nodes_map[xdx][ydy] == 1))
      {
        // generate a child node
        m0 = new node(xdx, ydy, n0->getLevel(),
                      n0->getPriority());
        m0->nextLevel(i);
        m0->updatePriority(xFinish, yFinish);

        // if it is not in the open list then add into that
        if (open_nodes_map[xdx][ydy] == 0)
        {
          open_nodes_map[xdx][ydy] = m0->getPriority();
          pq[pqi].push(*m0);
          // mark its parent node direction
          dir_map[xdx][ydy] = (i + dir / 2) % dir;
        }
        else if (open_nodes_map[xdx][ydy] > m0->getPriority())
        {
          // update the priority info
          open_nodes_map[xdx][ydy] = m0->getPriority();
          // update the parent direction info
          dir_map[xdx][ydy] = (i + dir / 2) % dir;

          // replace the node
          // by emptying one pq to the other one
          // except the node to be replaced will be ignored
          // and the new node will be pushed in instead
          while (!(pq[pqi].top().getxPos() == xdx &&
                   pq[pqi].top().getyPos() == ydy))
          {
            pq[1 - pqi].push(pq[pqi].top());
            pq[pqi].pop();
          }
          pq[pqi].pop(); // remove the wanted node

          // empty the larger size pq to the smaller one
          if (pq[pqi].size() > pq[1 - pqi].size())
            pqi = 1 - pqi;
          while (!pq[pqi].empty())
          {
            pq[1 - pqi].push(pq[pqi].top());
            pq[pqi].pop();
          }
          pqi = 1 - pqi;
          pq[pqi].push(*m0); // add the better node instead
        }
        else
          delete m0; // garbage collection
      }
    }
    delete n0; // garbage collection
  }
  return ""; // no route found
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::PrintMap(int *print_map)
{

  // /* DEBUG */
  for (int x = 0; x < n; x++)
  {
    for (int y = 0; y < m; y++)
    {
      if (print_map[y * n + x] == 0)
        cout << ".";
      else if (print_map[y * n + x] == 1)
        cout << "O"; //obstacle
      else if (print_map[y * n + x] == 2)
        cout << "S"; //start
      else if (print_map[y * n + x] == 3)
        cout << "R"; //route
      else if (print_map[y * n + x] == 4)
        cout << "F"; //finish
      else if (print_map[y * n + x] == 5)
        cout << "N"; //finish
      else if (print_map[y * n + x] == 6)
        cout << "P"; //finish
    }
    cout << endl;
  }
  // /* DEBUG */
}

/* Computes cartesian distances between a start point and an array of points in the map */
vector<double> CIri1Controller::calcDistHeuristics(dVector2 start, vector<dVector2> points)
{
  int size = points.size();
  vector<double> distances(size);

  for (int i = 0; i < size; i++)
  {
    distances[i] = sqrt((pow((points[i].x - start.x), 2) + pow((points[i].y - start.y), 2)));
  }

  return distances;
}

void CIri1Controller::computeDeliveriesQueues(unsigned int un_priority, vector<dVector2> &graphHigh, vector<dVector2> &graphLow)
{
  // empezar con los amarillos
  // se empieza desde la posición del robot
  /* Los parametros que pasamos a esta funcion son copias de dropPositionsHigh o Low,
  modificados en el Forage a medida que entregamos paquetes de una u otra prioridad*/
  dVector2 current_pos = m_rPosition; // * FIXME: m_vPosition doesn't represent robot current position -> ARREGLADO!!!: changed to m_rPosition

  current_pos = processPendingDrops(graphHigh, current_pos);
  current_pos = processPendingDrops(graphLow, current_pos);
}

dVector2 CIri1Controller::processPendingDrops(vector<dVector2> &graphRef, dVector2 current_pos)
{
  vector<dVector2> graph(graphRef);
  graphRef.clear();

  while (graph.size() > 0)
  {
    double distancia_mas_cercana = 9999;
    int indice_mas_cercano = -1;

    vector<double> distancias_1 = calcDistHeuristics(current_pos, graph);
    for (int i = 0; i < distancias_1.size(); i++)
    {
      if (distancias_1[i] < distancia_mas_cercana)
      {
        distancia_mas_cercana = distancias_1[i];
        indice_mas_cercano = i;
      }
    }
    current_pos = graph[indice_mas_cercano];
    graph.erase(graph.begin() + indice_mas_cercano);
    dropQueue.push_back(current_pos);
    graphRef.push_back(current_pos);
  }
  return current_pos;
}

vector<int> CIri1Controller::toMapCell(dVector2 point)
{
  vector<int> cell;
  fXmov = mapLengthX / ((double)mapGridX);
  fYmov = mapLengthY / ((double)mapGridY);
  /* Compute X grid */
  double tmp = point.x + (mapLengthX / 2);
  cell.push_back((int)(tmp / fXmov));

  /* Compute Y grid */
  tmp = -point.y + (mapLengthY / 2);
  cell.push_back((int)(tmp / fYmov));

  return cell;
}