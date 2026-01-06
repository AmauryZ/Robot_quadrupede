#include <Dynamixel2Arduino.h>
#include <vector>
#define DXL_SERIAL Serial3 //définit le type de bus pour la communication avec les servos
#define DEBUG_SERIAL Serial

//const uint8_t OLD_ID = 15;
//const uint8_t NEW_ID = 1;

const int DXL_DIR_PIN = 22;
const float DXL_PROTOCOL_VERSION = 1.0;
uint32_t BAUDRATE = 1000000;

float ax12_2_deg(uint16_t X) 
{
  float flottant = ((X * 150 / 512) - 150);
  return flottant;
}

uint16_t deg_2_ax12(float X)  //ou int deg2_2_ax12
{
  uint16_t entier = round((((X + 150) * 512) / 150));
  return entier;
}

std::vector<float> Offset{0, 0, -90, 0, 0, 0, 90, 0, -90, 90, 0, -90};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//COMMANDE D'ANGLES AUX MOTEURS
void setcoordR(const std::vector<float>& Q) {
    dxl.setGoalPosition(1,deg_2_ax12(Q[0]+Offset[0]));
    dxl.setGoalPosition(10,deg_2_ax12(Q[0]+Offset[9]));
    dxl.setGoalPosition(2,deg_2_ax12(Q[1]+Offset[1]));
    dxl.setGoalPosition(11,deg_2_ax12(Q[1]+Offset[10]));
    dxl.setGoalPosition(3,deg_2_ax12(-90-Q[1]+Offset[2]));
    dxl.setGoalPosition(12,deg_2_ax12(-90-Q[1]+Offset[11]));
    dxl.setGoalPosition(4,deg_2_ax12(Q[2]+Offset[3]));
    dxl.setGoalPosition(7,deg_2_ax12(Q[2]+Offset[6]));
    dxl.setGoalPosition(5,deg_2_ax12(Q[3]+Offset[4]));
    dxl.setGoalPosition(8,deg_2_ax12(Q[3]+Offset[7]));
    dxl.setGoalPosition(6,deg_2_ax12(-90-Q[3]+Offset[5]));
    dxl.setGoalPosition(9,deg_2_ax12(-90-Q[3]+Offset[8]));
}



//LISTE DE COORDONNEES OPERATIONNELLES à copier/coller depuis un .txt
//std::vector<float> PosIni{0, 90, -90, 0, -90, 0, 90, -90, -90, 90, -90, -90}; //pour mettre le robot debout
std::vector<float> PosIni{0, -90, 0, -90};

std::vector <std::vector <float>> qR1 = {{44.42,-65.56,-90.0,-66.17},
{40.98,-62.12,-86.56,-66.13},
{37.54,-58.68,-83.12,-66.13},
{34.1,-55.25,-79.69,-66.13},
{30.67,-51.81,-76.25,-66.13},
{27.23,-48.37,-72.81,-66.13},
{23.79,-44.93,-69.37,-66.13},
{20.35,-41.49,-65.94,-66.13},
{16.92,-38.06,-62.5,-66.13},
{13.48,-34.62,-59.06,-66.13},
{10.04,-31.56,-55.62,-66.13},
{6.6,-29.84,-52.18,-66.13},
{3.16,-26.4,-48.75,-66.13},
{-0.27,-22.96,-45.31,-66.13},
{-3.71,-19.53,-41.87,-66.13},
{-7.15,-16.09,-38.43,-66.13},
{-10.59,-12.65,-35.0,-66.13},
{-14.02,-9.21,-31.56,-66.13},
{-17.46,-7.67,-28.12,-66.13},
{-20.9,-7.67,-24.68,-66.13},
{-24.34,-7.67,-21.25,-66.13},
{-27.78,-7.67,-17.81,-66.13},
{-31.07,-8.25,-14.37,-66.13},
{-34.51,-11.68,-10.93,-66.13},
{-37.95,-15.12,-7.49,-66.13},
{-41.39,-18.56,-4.06,-66.13},
{-44.82,-22.0,-0.62,-66.13},
{-48.26,-25.44,2.82,-66.13},
{-51.7,-28.87,6.26,-66.13},
{-55.14,-32.31,9.69,-66.13},
{-58.57,-35.75,13.13,-66.13},
{-62.01,-39.19,16.57,-66.13},
{-65.45,-42.62,20.01,-66.13},
{-68.89,-46.06,23.45,-66.13},
{-72.33,-49.5,26.88,-66.13},
{-75.76,-52.94,30.32,-66.13},
{-79.2,-56.37,33.76,-66.13},
{-82.64,-59.81,37.2,-66.13},
{-86.08,-63.25,40.63,-66.13},
{-89.51,-66.17,44.07,-66.13},
{-87.71,-66.13,42.7,-63.84},
{-84.27,-66.13,39.26,-60.4},
{-80.83,-66.13,35.82,-56.96},
{-77.39,-66.13,32.38,-53.53},
{-73.96,-66.13,28.95,-50.09},
{-70.52,-66.13,25.51,-46.65},
{-67.08,-66.13,22.07,-43.21},
{-63.64,-66.13,18.63,-39.78},
{-60.21,-66.13,15.2,-36.34},
{-56.77,-66.13,11.76,-32.9},
{-53.33,-66.13,8.32,-31.56},
{-49.89,-66.13,4.88,-28.12},
{-46.46,-66.13,1.45,-24.68},
{-43.02,-66.13,-1.99,-21.25},
{-39.58,-66.13,-5.43,-17.81},
{-36.14,-66.13,-8.87,-14.37},
{-32.7,-66.13,-12.31,-10.93},
{-29.27,-66.13,-15.74,-7.67},
{-25.83,-66.13,-19.18,-7.67},
{-22.39,-66.13,-22.62,-7.67},
{-18.95,-66.13,-26.06,-7.67},
{-15.52,-66.13,-29.49,-7.67},
{-12.08,-66.13,-32.79,-9.97},
{-8.64,-66.13,-36.23,-13.4},
{-5.2,-66.13,-39.67,-16.84},
{-1.76,-66.13,-43.1,-20.28},
{1.67,-66.13,-46.54,-23.72},
{5.11,-66.13,-49.98,-27.15},
{8.55,-66.13,-53.42,-30.59},
{11.99,-66.13,-56.86,-34.03},
{15.42,-66.13,-60.29,-37.47},
{18.86,-66.13,-63.73,-40.9},
{22.3,-66.13,-67.17,-44.34},
{25.74,-66.13,-70.61,-47.78},
{29.18,-66.13,-74.04,-51.22},
{32.61,-66.13,-77.48,-54.66},
{36.05,-66.13,-80.92,-58.09},
{39.49,-66.13,-84.36,-61.53},
{42.93,-66.13,-87.8,-64.97}
};


void RotationP90() {
  for (const auto& element : qR1)
  {
    setcoordR(element);
    delay(3);
  }
}

void RotationM90(){
  for (int i=0; i<qR1.size(); i++)
    {
      setcoordR(qR1[qR1.size()-i-1]);
      delay(3);
    }
}
/*
void Avancer(){
  for (const auto& element : qR2)
  {
    setcoordT(element);
    delay(3);
  }
}

void Reculer(){
  for (int i=0; i<qR2.size(); i++)
  {
    setcoordT(qR2[qR2.size()-i-1]);
    delay(3);
  }
}
*/


void setup() 
{
  dxl.begin(BAUDRATE); // on initialise la communication avec le moteur par la définition du baudrate
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION); // on déclare le protocole de communication

  for (int i{1}; i<13; i++)
  {
    dxl.torqueOff(i);
    dxl.setOperatingMode(i,OP_POSITION);
    //dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, i, 0); ne contrôle rien
    dxl.torqueOn(i);
    //Offset.push_back(ax12_2_deg(dxl.getPresentPosition(i)));
  }

  for (int i{1}; i<13; i++)
  {
    dxl.setGoalPosition(i,deg_2_ax12(Offset[i-1]));;
  }
  
  delay(2000);
  

//Levage du robot
  setcoordR(PosIni);
  delay(2000);
  /*
  setcoordT(qR2[0]);
  delay(1000);
  */

  
  for (int i{0}; i<3; i++) //pour EMC25
  {
    RotationP90();
  }  
}

void loop() {
  /*
  RotationP90();
   RotationP90();
  for (int i{0}; i<4; i++) //pour EMC25
  {
    Avancer();
  }
  RotationP90();
  */
} 
