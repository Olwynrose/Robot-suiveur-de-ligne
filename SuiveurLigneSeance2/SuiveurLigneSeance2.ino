#include <Servo.h>

int delta_t; // loop duration (ms) - temps entre 2 mesures

// sensor calibration
float G0, G1, G2, G3, G4, G5, G6, G7; //gains
float O0, O1, O2, O3, O4, O5, O6, O7; //offsets
float i0, i1, i2, i3, i4, i5, i6, i7; //inputs
float seuil_ligne; // seuil de perte de ligne
int side;

// control coeeficients
float as0, as, ds; //vitesse max moyenne, avergage speed, delta speed
float kp, ki, kd; //proportionnal coefficient, integral coefficient
float si, sat_i;  // sum for the integral correction
float g_ML, g_MR; //gain diff moteur

float n_fg; //nombre de mesures de la fenêtre glissante

Servo servoR, servoL; //servos
int s0R = 90; int s0L = 90; //no rotation values

void setup() {
  Serial.begin(9600);
  as = 0;
  si = 0;
  delta_t = 20; 
  g_ML = 1;
  g_MR = 1;
  n_fg = 20;
  seuil_ligne = 0.4;
  
  pinMode(13, OUTPUT);
  servoR.attach(31);
  servoL.attach(30);
  servoR.write(s0R);
  servoL.write(s0L);

  // control setup 
  as0 = 5; // vitesse moyenne max
  kp = 0.2; // coefficient proportionnel
  ki = 0; // coefficient intégrale
  kd = 0; // coefficient dérivé
    
  calibration();
}

void loop() {  
  float dp; //delta position
  float buf_dp;
  int t_0, t_1;

  t_0 = millis();
  
  cac(analogRead(A0),analogRead(A1),analogRead(A2),analogRead(A3),analogRead(A4),analogRead(A5),analogRead(A6),analogRead(A7));
  //displayInputsAC();

  // décalement de la ligne
  dp = detectLine();

  // 
  ds = (as0 * (kp * dp) );
  //Serial.println(ds);


  if(perteLigne() == 1) {
    if(side == 1) {
    Serial.println("Ligne perdue par la droite");
    }
    else {
    Serial.println("Ligne perdue par la gauche");
    }
  }
  else {
    cSide();
  }

/*
  // ralenti si ligne proche du bord
  float coefralentissement = 0;
  as = max(0, as0*(1 - abs(coefralentissement*dp)));

  */
  /*
  servoR.write(s0R - (int)(g_MR*(as + ds)));
  servoL.write(s0L + (int)(g_ML*(as - ds)));
  */
  // control the timing
  // faire en sorte qu'une boucle prenne exactement delta_t
  t_1 = millis(); 
  
  if ( (t_1-t_0) < delta_t ) {
  delay(delta_t - t_1 + t_0);
  }
}





void calibration() {
  int i;
  int n = 100;
  float m0, m1;
  O0 = 0; O1 = 0; O2 = 0; O3 = 0; O4 = 0; O5 = 0, O6 = 0, O7 = 0;
  G0 = 0; G1 = 0; G2 = 0; G3 = 0; G4 = 0; G5 = 0, G6 = 0, G7 = 0; 

  delay(1000);
  digitalWrite(13, HIGH);
  Serial.println("Place the robot's sensor on a white area. You have 5 seconds.");
  delay(3000);
  digitalWrite(13, LOW);
  Serial.println("Calibration (1 second)");
  for (i = 0; i < n ; i++) {
    // G_i -> M_0
    G0 = G0 + analogRead(A0)/n;
    G1 = G1 + analogRead(A1)/n;
    G2 = G2 + analogRead(A2)/n;
    G3 = G3 + analogRead(A3)/n;
    G4 = G4 + analogRead(A4)/n;
    G5 = G5 + analogRead(A5)/n;
    G6 = G6 + analogRead(A6)/n;
    G7 = G7 + analogRead(A7)/n;
    
    delay(10);
  }
  Serial.println("Calibration done.");
  delay(1000);
  digitalWrite(13, HIGH);
  
  Serial.println("Place the robot's sensor on a black area. You have 5 seconds.");
  delay(3000);
  digitalWrite(13, LOW);
  Serial.println("Calibration (1 second)");
  for (i = 0; i < n ; i++) {
    // O_i -> M_1
    O0 = O0 + analogRead(A0)/n;
    O1 = O1 + analogRead(A1)/n;
    O2 = O2 + analogRead(A2)/n;
    O3 = O3 + analogRead(A3)/n;
    O4 = O4 + analogRead(A4)/n;
    O5 = O5 + analogRead(A5)/n; 
    O6 = O6 + analogRead(A6)/n; 
    O7 = O7 + analogRead(A7)/n; 
    delay(10);
  }
  Serial.print("Calibration done.");

  //displayInputsBC();
  
  delay(1000);
  digitalWrite(13, HIGH);
  delay(2000);
  digitalWrite(13, LOW);
  m0 = G0; 
  m1 = O0;
  G0 = 1 / ( m1 - m0);
  O0 = m0 / (m0 - m1);
  m0 = G1; 
  m1 = O1;
  G1 = 1 / ( m1 - m0);
  O1 = m0 / (m0 - m1);
  m0 = G2; 
  m1 = O2;
  G2 = 1 / ( m1 - m0);
  O2 = m0 / (m0 - m1);
  m0 = G3; 
  m1 = O3;
  G3 = 1 / ( m1 - m0);
  O3 = m0 / (m0 - m1);
  m0 = G4; 
  m1 = O4;
  G4 = 1 / ( m1 - m0);
  O4 = m0 / (m0 - m1);
  m0 = G5; 
  m1 = O5;
  G5 = 1 / ( m1 - m0);
  O5 = m0 / (m0 - m1);
    m0 = G6; 
  m1 = O6;
  G6 = 1 / ( m1 - m0);
  O6 = m0 / (m0 - m1);
    m0 = G7; 
  m1 = O7;
  G7 = 1 / ( m1 - m0);
  O7 = m0 / (m0 - m1);
}






void cac(int ar0, int ar1, int ar2, int ar3, int ar4, int ar5, int ar6, int ar7) {
  //Correction After Calibration
  //Input : 
  //Output : /
  //blabalbalblala
  i0 = max(((float)ar0) * G0 + O0 - 0.1, 0);
  i1 = max(((float)ar1) * G1 + O1 - 0.1, 0);
  i2 = max(((float)ar2) * G2 + O2 - 0.1, 0);
  i3 = max(((float)ar3) * G3 + O3 - 0.1, 0);
  i4 = max(((float)ar4) * G4 + O4 - 0.1, 0);
  i5 = max(((float)ar5) * G5 + O5 - 0.1, 0);
  i6 = max(((float)ar6) * G6 + O6 - 0.1, 0);
  i7 = max(((float)ar7) * G7 + O7 - 0.1, 0);
}









float detectLine() {
  float avrg;
  avrg = (( 0*i0 + 1*i1 + 2*i2 + 3*i3 + 4*i4 + 5*i5 + 6*i6 + 7*i7 ) / (i0+i1+i2+i3+i4+i5+i6+i7+0.00001)) - 3.5;
  Serial.print("Line position : ");
  Serial.println(avrg);
  return avrg;
}








void displayInputsAC() {
  Serial.print(i0);
  Serial.print("     ");
  Serial.print(i1);
  Serial.print("     ");
  Serial.print(i2);
  Serial.print("     ");
  Serial.print(i3);
  Serial.print("     ");
  Serial.print(i4);
  Serial.print("     ");
  Serial.print(i5);
  Serial.print("     ");
  Serial.print(i6);
  Serial.print("     ");
  Serial.println(i7);
}


void displayInputsBC() {
  Serial.println("BLANC");
  Serial.print(G0);
  Serial.print("     ");
  Serial.print(G1);
  Serial.print("     ");
  Serial.print(G2);
  Serial.print("     ");
  Serial.print(G3);
  Serial.print("     ");
  Serial.print(G4);
  Serial.print("     ");
  Serial.print(G5);
  Serial.print("     ");
  Serial.print(G6);
  Serial.print("     ");
  Serial.println(G7);
  
  Serial.println();
  Serial.println("NOIR");
  Serial.print(O0);
  Serial.print("     ");
  Serial.print(O1);
  Serial.print("     ");
  Serial.print(O2);
  Serial.print("     ");
  Serial.print(O3);
  Serial.print("     ");
  Serial.print(O4);
  Serial.print("     ");
  Serial.print(O5);
  Serial.print("     ");
  Serial.print(O6);
  Serial.print("     ");
  Serial.println(O7);
}

int perteLigne()
{
  if((i0+i1+i2+i3+i4+i5+i6+i7)>seuil_ligne) {
    return 0;
  }
  else {
    return 1;
  }
}

void cSide() {
  if (ds>0 && ds<2) {
    side = -1;
  }
  if (ds<0 && ds>-2) {
    side = 1;
  }
}
