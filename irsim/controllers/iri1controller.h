#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri1Controller : public CController
{
public:
  CIri1Controller(const char *pch_name, CEpuck *pc_epuck, int n_wrtie_to_file);
  ~CIri1Controller();
  void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
  /* ROBOT */
  CEpuck *m_pcEpuck;

  /* SENSORS */
  CWheelsActuator *m_acWheels;
  CEpuckProximitySensor *m_seProx;
  CRealLightSensor *m_seLight;
  CRealBlueLightSensor *m_seBlueLight;
  CRealRedLightSensor *m_seRedLight;
  CContactSensor *m_seContact;
  CGroundSensor *m_seGround;
  CGroundMemorySensor *m_seGroundMemory;
  CBatterySensor *m_seBattery;
  CBlueBatterySensor *m_seBlueBattery;
  CEncoderSensor *m_seEncoder;

  /* Global Variables */
  double m_fLeftSpeed;
  double m_fRightSpeed;
  double **m_fActivationTable;
  int m_nWriteToFile;
  double m_fTime;
  double fDeliverToGoalInhibitor;
  double fDeliverHighToDeliverLowInhibitor;
  double fFinalInhibitor; //Inhibe todo menos Navigate y Avoid para que vuelva a casa
  double fLoadInhibitor;
  int *m_nPacketsDelivered; // se va guardando el numero de paquetes de cierto tipo que se han ya repartido, [0] los high, [1] los low
  const char *m_cHeightMap;
  int m_nPacketsLoaded;// paquetes que carga el robot en cada momento
  /* Odometry */
  float m_fOrientation;
  dVector2 m_vPosition;
  dVector2 m_rPosition;
  int m_nState;
  dVector2 *m_vPositionsPlanning;
  int m_nPathPlanningStops;
  int m_nRobotActualGridX;
  int m_nRobotActualGridY;

  int m_nForageStatus;

  int m_nNestFound;
  int m_nNestGridX;
  int m_nNestGridY;

  int m_nPreyFound;
  int m_nPreyGridX;
  int m_nPreyGridY;

  int m_nPathPlanningDone;

  double fXmov;
  double fYmov;

  /* Functions */

  void ExecuteBehaviors(void);
  void Coordinator(void);

  void CalcPositionAndOrientation(double *f_encoder);
  string pathFind(const int &xStart, const int &yStart, const int &xFinish, const int &yFinish);
  vector<double> calcDistHeuristics(dVector2 start, vector<dVector2> points);
  vector<int> toMapCell(dVector2 point);
  void computeDeliveriesQueues(unsigned int un_priority, vector<dVector2> &graphHigh, vector<dVector2> &graphLow);
  dVector2 processPendingDrops(vector<dVector2> &graphRef, dVector2 current_pos);

  void PrintMap(int *print_map);
  /* Behaviors */
  void ObstacleAvoidance(unsigned int un_priority);
  void Navigate(unsigned int un_priority);
  void GoLoad(unsigned int un_priority);
  void GoHome(unsigned int un_priority); //modified
  void DeliverHigh(unsigned int un_priority); //modified
  void DeliverLow(unsigned int un_priority);  //modified

  void ComputeActualCell(unsigned int un_priority);
  void PathPlanning(unsigned int un_priority);
  void GoGoal(unsigned int un_priority);
};

#endif
