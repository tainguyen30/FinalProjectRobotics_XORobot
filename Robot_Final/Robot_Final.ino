#include <AccelStepper.h>
#include <MultiStepper.h>
AccelStepper stepperX(1,12,13,8);
AccelStepper stepperY(1,3,6,8);
AccelStepper stepperZ(1,4,7,8);

float theta1, theta2, steps;
float do_dai, X,Y;
float pi=3.14;
float X_position, Y_position;
float c2,s2,t2,c1,s1,t1;
float l2 = 220; // Khau_2.1
float l3 = 185; // Khau_2.2
float l01 =0;   // Set x lui lai -140 
float l02 = 30;   // Set y lui lai -190 
int a =5;         // tịnh tiến z ban đầu
float do_dai_L = a*850/1.1;

void setup(){
   pinMode(12,OUTPUT);
   pinMode(13,OUTPUT);
   pinMode(3,OUTPUT);
   pinMode(6,OUTPUT);
   pinMode(4,OUTPUT);
   pinMode(7,OUTPUT);
   pinMode(8,OUTPUT);
   digitalWrite(8,LOW);
   Serial.begin(9600);

   // Step_X
   stepperX.setMaxSpeed(1500);//Không biết lun
   stepperX.setSpeed(1500);//Tốc độ quay nhanh chậm
   stepperX.setAcceleration(3000);//Gia tốc (không rõ lắm)toc

   // Step_Y
   stepperY.setMaxSpeed(1500);
   stepperY.setSpeed(1000);
   stepperY.setAcceleration(3000);

   // Step_Z
   stepperZ.setMaxSpeed(1000);
   stepperZ.setSpeed(1500);
   stepperZ.setAcceleration(1000);

   stepperX.setCurrentPosition(0);                    //Set goc ve 0
   stepperY.setCurrentPosition(0);
   stepperZ.setCurrentPosition(0);
 }

void z_tinh_tien(float do_dai_L)
{
   stepperZ.setCurrentPosition(0);
   stepperZ.setMaxSpeed(1000);
   stepperZ.setSpeed(900);
   stepperZ.setAcceleration(1000);
   stepperZ.moveTo(do_dai_L);
     while(stepperZ.distanceToGo() != 0 )
     {
       stepperZ.run();
     }
}

void XO( float Px, float Py)
{
   float A,B;
   A = Px + l01;
   B = Py + l02;
   c2 = (pow(A,2) + pow(B,2) - pow(l2,2) - pow(l3,2))/(2*l2*l3);
   s2 = sqrt(abs(1 - pow(c2,2)));
   t2 = atan2(s2,c2)*180/pi;
   s1 = B*(l3*c2  + l2) - A*(l3*s2);
   c1 = A*(l3*c2 + l2) + B*(l3*s2);
   t1 = atan2(s1,c1)*180/pi;
   Serial.println(t1);
   Serial.println(t2);

   X_position=(t1)*40/6;
   Y_position=t2*40/8;
   stepperX.moveTo(X_position);
   stepperY.moveTo(Y_position);
   while(stepperX.distanceToGo()!=0  || stepperY.distanceToGo() != 0)    // cho chạy với giá trị X ,Y mong muốn
   {
          stepperX.run();
          delayMicroseconds(1000);  
          stepperY.run(); // Lenh chay co gia toc
          delayMicroseconds(1000); 
   }
}

void run_O(float a, float b, int l, int m )
{
  float x1, y1;
  int R=20; // set bán kính của đường tròn
  for (float  t=a*pi/2;t<=b*pi*2 + pi/3;t=t+0.05) 
  {
          x1 = l + R*cos(t);                                          // Phuong trinh duong tron voi vi tri x = xA; y = yA; ban kinh = R
          y1 = m + R*sin(t);

          // Step_X
          stepperX.setMaxSpeed(1500);             // Set toc do toi da
          stepperX.setSpeed(1500);               // Toc do set
          stepperX.setAcceleration(900);        // Set gia toc

          // Step_Y
          stepperY.setMaxSpeed(1500);          // Set toc do toi da
          stepperY.setSpeed(1000);            // Toc do set
          stepperY.setAcceleration(900);      // Set gia toc

          XO(x1,y1);
          delay(100);
          Serial.println(t); 
   }   
}

void run_X(float k, float e)
  {
    float x2,y2,x3,y3;
    int r=15;
          // Step_X
          stepperX.setMaxSpeed(1500);             //Không biết lun
          stepperX.setSpeed(1500);                //Tốc độ quay nhanh chậm
          stepperX.setAcceleration(300);          //Gia tốc (không rõ lắm)

          // Step_Y
        stepperY.setMaxSpeed(1000);
        stepperY.setSpeed(1000);
        stepperY.setAcceleration(3000);

        // Step_Z
        stepperZ.setMaxSpeed(1000);
        stepperZ.setSpeed(900);
        stepperZ.setAcceleration(1000);

   for( float t = 0; t<= 1; t+=0.05)
   {
        float xa1 = k+r ;   float ya1 = e+r ;
        float xb1 = k-r;    float yb1 = e-r ;
        x2 = xa1 + (xb1-xa1)*t;
        y2 = ya1 + (yb1-ya1)*t;    
        XO(x2,y2);
        delay(100);
  }
  
   z_tinh_tien(-do_dai_L);
        float xa2 = k-r ;   float ya2 = e+r ;
        float xb2 = k+r;    float yb2 = e-r ;
        x3 = xa2;
        y3 = ya2;
        XO(x3,y3);
        delay(100);
  
  z_tinh_tien(do_dai_L);
  for( float t = 0; t<= 1; t+=0.05)
  {
       float xa2 = k-r ;    float ya2 = e+r ;
       float xb2 = k+r;     float yb2 = e-r ;   
       x3 = xa2 + (xb2-xa2)*t;
       y3 = ya2 + (yb2-ya2)*t;  
       XO(x3,y3);
       delay(100);    
  }  
  
}

void Set_Home()
{
    stepperX.moveTo(0);
    stepperY.moveTo(0);
    while(stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
    {
        stepperX.run();
        stepperY.run();
    }
}

void loop()
{
   float X00,Y00; /// tọa  độ điểm muốn đánh 
   Serial.println("Nhap toa do x: ");
   while(Serial.available()==0){};
        delay(300);
        X00 = Serial.parseInt();
        delay(100);
        Serial.println(X00);

   Serial.println("Nhap toa do y: ");
   while(Serial.available()==0){};
        delay(300);
        Y00 = Serial.parseInt();
        delay(100);
        Serial.println(Y00);

   Serial.println("Which do you want to play?    X = 1 and O = 0 ");
   while(Serial.available()==0){};
        delay(300);
        int Play = Serial.parseInt();
        delay(100);

    float X_auto = X00*100+50;
    float Y_auto = Y00*100+50;

    if( Play == 0)
    {
      Serial.println(" Ban la O ");
      XO(X_auto,Y_auto);
      z_tinh_tien(do_dai_L);
      run_O(0,5,X_auto,Y_auto);
      z_tinh_tien(-do_dai_L);
      Set_Home();
      delay(100);
    }

    else if( Play == 1)
    {
      Serial.println(" Ban la X ");
      XO(X_auto,Y_auto);
      z_tinh_tien(do_dai_L);
      run_X(X_auto,Y_auto);
      z_tinh_tien(-do_dai_L);
      Set_Home();
      delay(100);
    }
}
    
  
