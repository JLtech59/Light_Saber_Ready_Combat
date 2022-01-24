//librairies pour le mpu
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

//librairies pour le df player
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

//////////////////////////////////////////////////////////////////////
//Variables utiles pour le MPU
MPU6050 mpu;

//Toutes les variables utiles venant du code exemple
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

//////////////////////////////////////////////////////////////////////


//float pour les accelerations
float acce = 0;
float acceav = acce;

//toutes les variables pour le dfplayer

bool playsound = false;
SoftwareSerial mySoftwareSerial(8, 12); // RX, TX
DFRobotDFPlayerMini myDFPlayer;


//bouton et led rgb
#define BP 5
#define LEDB A2
#define LEDV A1
#define LEDR A0


//bool pour vérifier si le bouton d'allumage est bien relaché après l appui
bool go = 0;
bool use = 0;

//sensibilités des mouvements
int sensicoup = 60;
int sensiswing = 18;
int sensimvt = 8;
//variable à modifier en fonction de sensicoup
int crash = 19;


//variables pour les millis
unsigned long timewait;
unsigned int interval = 200;
static unsigned long timer;


// ================================================================
// ===                       SETUP                             ===
// ================================================================

void setup() {

  //initialistation i2c pour le mpu
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock


  Serial.begin(115200);
  //initialisation du mpu -> code exemple modifié
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }


  ////////////


  //initialisation du dfplayer
  mySoftwareSerial.begin(9600);
  Serial.println("demarrage audio");
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    while (true) {
      delay(0);
    }
  }
  myDFPlayer.EQ(DFPLAYER_EQ_BASS);
  //sur la doc le volume ne peut pas depasser 30 mais en réalité c'est possible. Je ne connais pas les risques par contre
  myDFPlayer.volume(26);

  myDFPlayer.play(20);


  /////////////


  //pour la led

  pinMode(LEDB, OUTPUT);
  pinMode(LEDV, OUTPUT);
  pinMode(LEDR, OUTPUT);


  //Pour le bouton
  pinMode(BP, INPUT);


  Serial.println("saber ready");

}



// =====================================================
// ===                      LOOP                     ===
// =====================================================

void loop() {

  //pour l'allumage
  if (digitalRead(BP) == HIGH) {
    go = 1;

  }
  if (digitalRead(BP) == LOW && go) {
    go = 0;
    use = !use;
    if (use) {
      Serial.println("allumage");

      myDFPlayer.play(1);
      for (int i = 75; i < 256; i++) {
        //analogWrite(LEDR, i);
        analogWrite(LEDB, i);
        delay(5);
      }

      myDFPlayer.volume(33);
      myDFPlayer.loop(3);

    } else {
      myDFPlayer.volume(24);
      myDFPlayer.play(2);
      for (int i = 0; i < 256; i++) {
        //analogWrite(LEDR, 255 - i);
        analogWrite(LEDB, 255 - i);
        delay(3);
      }
      // analogWrite(LEDR, 0);
      analogWrite(LEDB, 0);
      // analogWrite(LEDR, 0);
    }
  }

  if (!use) return;
  //si le sabre est allumé
  if (playsound == true) {
    Serial.println(timewait);
    if (millis() - timewait >= 450) {

      myDFPlayer.loop(3);

      playsound = false;
    }
  }
  action();


}
// ================================================================
// ===                    ACTION DU SABRE                       ===
// ================================================================
void action() {


  //toujours appelé dans le void loop si le sabre est allumé
  //Actualisation de l'accéléromètre,
  //code venant de la librairie
  mpuIntStatus = mpu.getIntStatus();


  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
  }
  else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    /////////////////////////////////////////////////

    //////////////////////////////////////////////////

    //L'acceleration calculée précedemment est actualisée
    acceav = acce;
    //formule du vecteur acceleration ajusté avec les données du mpu
    acce = abs(((((aaReal.x ^ 2) + (aaReal.y ^ 2) + (aaReal.z ^ 2)) ^ 1 / 2) / 100));


    ///////////////////////////////////////

    //////////////////////////////////////

    //Suite de condition pour détecter quelle action faire
    //On regarde si la différence d'accélération est assez grande pour faire une action
    if (acceav >= sensicoup && acceav - acce >= crash) {
      Serial.println("crash");
      //permet de faire un effet lumineux
      analogWrite(LEDR, 255);

      son(3);

      analogWrite(LEDR, 0);
    }
    if (acce > acceav + sensimvt) {
      if (abs(acce - acceav) > sensimvt && abs(acce - acceav) < sensiswing) {
        Serial.println("petit mvt");


        son(1);


      }
      if (abs(acce - acceav) > sensiswing) {
        Serial.println("swing");


        son(2);
      }
    }
  }
}
// ================================================================
// ===                    SONS DU SABRE                         ===
// ================================================================

//Fonction qui permet de jouer les sons
void son(int quelcoup) {
  static unsigned long timer = millis();

  if (millis() - timer >= 300) {
    timer = millis();
    if (quelcoup == 1) {
      int r1;
      //de 5 à 12 les sons sont les coups faibles
      int r = random(4, 11);
      while (r == r1)
      {
        r = random(4, 11);
      }
      r1 = r;
      myDFPlayer.play(r);

    }
    if (quelcoup == 2) {
      int r2;
      //de 12 à 20 les sons sont les swings
      int r = random(11, 19);
      while (r == r2)
      {
        r = random(11, 19);
      }
      r2 = r;
      myDFPlayer.play(r);

    }
    if (quelcoup == 3) {
      int r3;
      //de 20 à 25 les sons sont les crash
      int r = random(19, 24);
      while (r == r3)
      {
        r = random(19, 24);
      }
      r3 = r;
      myDFPlayer.play(r);


    }
    playsound = true;
    timewait = millis();
    Serial.println(playsound);

  }
}
