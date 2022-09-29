/*This code is made by Yudha Putra Arisandy
  an undergraduate student of Mechanical and Biosystem Engineering
  Faculty of Agricultural Technology
  IPB University
  Bogor
  Indonesia
  feel free to contact me : yudhapa17@gmail.com
  Instagram : @ydharsndy || @wangdoson
  2019*/

//Lidar_read
#include <Wire.h>
#include <LIDARLite.h>
#include <Servo.h>
#include <SPI.h>
#include <ServoCds55.h>

LIDARLite myLidarLite;

int tunda = 1; //waktu jeda servo muter per derajat
float a = 0.29; //pertambahan sudut putar servo
int b = 900; //batas threshold u[n], v[n] //filter background -- hanya objek  -- semakin besar maka semakin banyak objek yg dtangkap
int w = 30 ; //semakin kecil semakin dikit titik tengah,//batas threshold x,y //beda titik yg dekat2 akan menjadi satu titik, makin besar makin sidikit pasangan u,v
int t = 40; // batas threshold x_atas, y_atas, semakin kecil -- banyak pasangan atas-bawah-tengah

//Variables for the duration and the distance
double duration;
double distanceL;

Servo myServoV; //Declare servo name
ServoCds55 myServo;
int servoNum = 1;

double  r[1000];// {10, 10, 12, 10, 24, 15, 13, 12, 13, 13, 12, 11, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 150, 150, 150, 150, 150, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70};
double teta[1000];
double u[1000];
double v[1000];
double x[1000];
double y[1000];
double x_atas[50];
double y_atas[50];
double x_bawah[50];
double y_bawah[50];
double x_tengah[50];
double y_tengah[50];
double x_GLCD[20];
double y_GLCD[20];
long m = 0; //r,teta,x,y
long n = 0; //v[n]
long s = 0; //x_atas  ;  y_atas
long d = 0; //
long z = 0;
int q;
int c;
long zxc = 1; //banyaknya looping keseluruhan
//int u[-1] = 0;
//int v[-1] = 0;
int k = 0;
double sigma[100] ;
double sigmaCY[100] ;
double sigmaCX[100] ;
double luas_poligon = 0;
double cx = 0;
double cy = 0;
double cz[100];


double selisih_sudut = 0;

//Baca MPU6050
#include <MPU6050_tockn.h>
#include <Wire.h>
MPU6050 mpu6050(Wire);
double sudut = 0;

//GLCD
#include "U8glib.h"
U8GLIB_ST7920_128X64_4X u8g(18, 16, 17);

int pwm_motorA = 5; //roda kiri
int motorA1 = 3;
int motorA2 = 4;
int motorB1 = 8;  //roda kanan
int motorB2 = 9;
int pwm_motorB = 7;
int pin_servo = 53;



void setup() {
  pinMode(pwm_motorA, OUTPUT);
  pinMode(pwm_motorB, OUTPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  analogWrite(pwm_motorA, 255);
  analogWrite(pwm_motorB, 255);

  Serial3.begin(9600);
  Serial.begin(115200);
  myLidarLite.begin(0, true);
  myLidarLite.configure(0);

  myServo.begin ();
  myServo.Reset(servoNum);
  myServo.setVelocity(50);// set velocity to 100(range:0-300) in Servo mode
  myServoV.attach(pin_servo);
  myServoV.write(90);

  //Setup GLCD
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255, 255, 255);
  }
  u8g.setFont(u8g_font_unifont);
  u8g.firstPage();
  do {
    u8g.drawStr(30, 30, "Scanning");
    //u8g.drawLine(0, 60, 127, 60);
    //u8g.drawLine(64, 64,  64, 0);
  } while (u8g.nextPage());

  //setup mpu6050
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  /*  int aaa = 0;
    do {
      aaa = aaa + 1;
      baca_sudut();
      Serial.println(sudut);
      //u8g.drawStr(0, 22, sudut);
    } while (aaa <= 1000);*/
  //delay(10000000000);
  Serial.println(sudut);
  Serial.println("");
}


//------------------------------------------------------------------
//LOOP
//------------------------------------------------------------------
void loop() {
  reset_array();
  setup_dynmxl();
  lidar_read();
  cal_pt_to_go();
  send_to_nano();
  while (1); //program stuck
  move_to_point();

  zxc++; //menghitung perulangan berapa kali
}

void reset_array() {
  memset(r, 0, sizeof(r));
  memset(teta, 0, sizeof(teta));
  memset(x, 0, sizeof(x));
  memset(y, 0, sizeof(y));
  memset(u, 0, sizeof(u));
  memset(v, 0, sizeof(v));
  memset(x_bawah, 0, sizeof(x_bawah));
  memset(y_bawah, 0, sizeof(y_bawah));
  memset(x_atas, 0, sizeof(x_atas));
  memset(y_atas, 0, sizeof(y_atas));
  memset(x_tengah, 0, sizeof(x_tengah));
  memset(y_tengah, 0, sizeof(y_tengah));
  memset(sigma, 0, sizeof(sigma));
  memset(sigmaCX, 0, sizeof(sigmaCX));
  memset(sigmaCY, 0, sizeof(sigmaCY));
  memset(x_GLCD, 0, sizeof(x_GLCD));
  memset(y_GLCD, 0, sizeof(y_GLCD));
}

void setup_dynmxl() {
  Serial.print("dynamixel gerak");
  for (float i = 40; i <= 60; i++) { //prepare servo agar sampai di 60
    myServo.write(servoNum, i);
    Serial.print(".");
    delay(300);
  }
  Serial.println("");
  delay(1000);
}
void lidar_read() {
  Serial.println("Scanning");
  u8g.firstPage();
  do {
    u8g.drawStr(30, 30, "Scanning");
    //u8g.drawLine(0, 60, 127, 60);
    //u8g.drawLine(64, 64,  64, 0);
  } while (u8g.nextPage());
  m = 0;
  myServo.write(servoNum, 240);
  for (m = 0; m <= 180; m++ ) {
    distanceL = myLidarLite.distance();
    //Serial.print(i); // Sends the current degree into the Serial Port
    //Serial.print(";"); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
    //Serial.println(distance); // Sends the distance value into the Serial Port
    r[m] = distanceL;
    teta[m] = m + 0.29;
    x[m] = r[m] * cos(teta[m] * 3.14 / 180);
    y[m] = r[m] * sin(teta[m] * 3.14 / 180);
    //Serial.print(r[m]);
    //Serial.print(";");
    Serial.print(x[m]);
    Serial.print(";");
    Serial.println(y[m]);
  }
  Serial.println("xxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
  // Serial.println(m);
  //
  //sekarang ada x[0] s.d. x[600] dan y[0] s.d. y[600]
  //SELESAI

}

void cal_pt_to_go() {
  n = 0;
  //bisa cek selisih sebelum dan sesuah
  for (m = 0; m <= 180; m++) {
    if (abs(x[m] - x[m + 1]) <= w && abs(y[m] - y[m + 1]) <= w ) {  //w=60 rekomendasi
      x[m + 1] = x[m];
      y[m + 1] = y[m];
      u[n] = x[m];
      v[n] = y[m];
    }
    else {
      //setiap salah maka n berubah
      //x[m] = 0;
      //y[m] = 0;
      n = n + 1;
    }
    //Serial.print(u[n]);
    //Serial.print(";");
    //Serial.println(v[n]);
  }
  //Serial.println("~~~~~~~~~~~~~~~~~~~");
  //delay(10000000);
  int p = n;
  //delay(10000);
  //skrg punya u[0] s.d. u[p]
  for (n = 1; n <= p; n++) {
    if (u[n] >= b || v[n] >= b || u[n] <= -b ) { //di luar b=800, threshold background
      u[n] = u[n - 1];
      v[n] = v[n - 1];
    }
    if ((u[n] <= 1 && u[n] >= -1) || (v[n] <= 1 && v[n] >= -1)) {
      u[n] = u[n - 1];
      v[n] = v[n - 1];
    }
    //Serial.print(u[n]);
    //Serial.print(";");
    //Serial.println(v[n]);
  }
  Serial.println("-------------------------------------------------------------------------");
  //delay(50000);
  //skrg punya u[n] dan v[n] berupa barisan objek tanpa background tapi ada noise
  //di dalam lingkungan objek

  k = 0;
  s = 0;
  for (n = 0; n <= p; n++) {
    if (u[n] != 0 && k == 0) {
      x_bawah[s] = u[n];
      y_bawah[s] = v[n];
      k = 1;
    }
    if (u[n] != 0 && (abs(u[n + 1] - u[n]) >= t  || abs(v[n + 1] - v[n]) >= t ) ) {
      x_atas[s] = u[n];
      y_atas[s] = v[n];
      s++;
      x_bawah[s] = u[n + 1];
      y_bawah[s] = v[n + 1];
    }
  }
  int j = s;
  //skrg punya x_bawah ; y_bawah dan x_atas ; y_atas , yaitu titik paling bawah dan paling atas pada setiap objek
  //Serial.println(j);
  for (s = 1; s < j; s++) {
    Serial.print(x_bawah[s]);
    Serial.print(";");
    Serial.print(y_bawah[s]);
    Serial.print(";");
    Serial.print(x_atas[s]);
    Serial.print(";");
    Serial.println(y_atas[s]);
    if (y_atas[s] > y_bawah[s]) {
      y_tengah[s] = abs(y_atas[s] - y_bawah[s]) / 2 + y_bawah[s];
    }
    if (y_atas[s] < y_bawah[s]) {
      y_tengah[s] = abs(y_atas[s] - y_bawah[s]) / 2 + y_atas[s];
    }
    x_tengah[s] = abs(x_atas[s] - x_bawah[s]) / 2 + x_atas[s];

    //filter x,y_tengah_[s], hanya ada di sekitaran u,v (p=jumlah pasangan u,v)
    //ada ga sih u,v di sekitaran dia?, kalo ada 1 aja titik u,v deket dia, berarti iya, dia tengah
    for (n = 0; n <= p; n++) {
      if (( ((abs(x_tengah[s] - u[n])) < 200)  && ((abs(y_tengah[s] - v[n])) < 200) )) { //jika ya, maka
        goto b;
      }
    }
    x_tengah[s] = 0;  //jika tidak, maka akan jadi =0
    y_tengah[s] = 0;

b: delayMicroseconds(1);
    Serial.print(x_tengah[s]);
    Serial.print(";");
    Serial.println(y_tengah[s]);
  }

  //memastikan di satu spot (satu grup u,v) hanya ada satu x,y_tengah (x,y yg diperiksa sebanyak j)
  for (s = 1; s < j; s++) {
    for (int v = 1; v < j; v++) {
      if ( (abs(x_tengah[s] - x_tengah[s + v])) < 100 && (abs(y_tengah[s] - y_tengah[s + v])) < 100) { //yg deketan
        x_tengah[s] = 0;
        y_tengah[s] = 0;
        goto zz;
      }
    }
zz: delayMicroseconds(1);
  }

  //kalau ada x,y_tengah==0, maka akan dihilangkan dari array, maka x,y tidak akan 0,0
  for (s = 1; s <= j; s++) {
    if (x_tengah[s] == 0 || y_tengah[s] == 0) {
      x_tengah[s] = x_tengah[s + 1];
      for (int i = s + 1; i <= j; i++) {
        x_tengah[i] = x_tengah[i + 1];
      }
    }
  }
  for (s = 1; s <= j; s++) {
    if (x_tengah[s] == 0 || y_tengah[s] == 0) {
      s = s - 1;
      goto aw;
    }
  }
aw:   q = s ;
  Serial.println("");
  Serial.println("x_tengah, y_tengah");
  for (int s = 1; s <= q; s++) {
    Serial.print(x_tengah[s]);
    Serial.print(";");
    Serial.println(y_tengah[s]);
  }

  /*pengecekan berdasar kemungkinan tiang, jarak antar tiang tetap, kalau di luar itu berarti
    noise/bukan tiang*/

  //delay(10000000);
  //skrg punya x_tengah[s] s.d. y_tengah[s]

  //zxc[1] =1;

  cz[zxc] = cy; //cz=cy sebelumnya

  //Serial.println(q); //5
  sigma[0] = 0;
  sigmaCX[0] = 0;
  sigmaCY[0] = 0;
  int s = 1;
  for (s = 1; s <= q; s++) {
    sigma[s] = x_tengah[s] * y_tengah[s + 1] - x_tengah[s + 1] * y_tengah[s];
    sigma[s] = sigma[s] + sigma[s - 1];
    //Serial.println(sigma[s]);
  }
  luas_poligon = sigma[s - 1] / 2;

  for (int s = 1; s <= q; s++) {
    sigmaCX[s] = (x_tengah[s] + x_tengah[s + 1]) * (x_tengah[s] * y_tengah[s + 1] - x_tengah[s + 1] * y_tengah[s]);
    sigmaCX[s] = sigmaCX[s] + sigmaCX[s - 1];

  }
  cx = sigmaCX[s - 1 ] / (6 * luas_poligon);

  for (int s = 0; s <= q; s++) {
    sigmaCY[s] = (y_tengah[s] + y_tengah[s + 1]) * (x_tengah[s] * y_tengah[s + 1] - x_tengah[s + 1] * y_tengah[s]);
    sigmaCY[s] = sigmaCY[s] + sigmaCY[s - 1];
    //Serial.println(sigmaCY[s]);
  }
  cy =  sigmaCY[s - 1] / (6 * luas_poligon) ;
  cetak_xy_GLCD();

  //kontrol cx,cy agar tidak lebih dari 1000 (melenceng keluar)
  if (cx > 1000) {
    cx = 0;
  }
  if (cy > 1000) {
    cy = 500;
  }
  Serial.println("");
  Serial.println("cx,cy: ");
  Serial.println(cx);
  Serial.println(cy);
  Serial.println(luas_poligon);
  Serial.println("");

  //delay(100000000000000);
}
//sudah ada cx, cy Tujuan pergerakkan

void move_to_point() {
  double r_m = sqrt(cx * cx + cy * cy);  //jarak r (cm), robot ke titik cx,cy
  double teta_m = abs( (asin(cx / r_m)) * 180 / 3.14); //sudut belok ke titik cx,cy dari sumbu y(90 drjt)

  //sudah ada r_m dan teta_m, yaitu jarak dan sudut belok terhadap cx,cy
  //belok dulu hingga selisih teta =0
  Serial.print("r_m= ");
  Serial.print(r_m);
  Serial.print("   teta_m= ");
  Serial.print(teta_m);
  Serial.println("");
  Serial.println("Sudut Tes");

  //int xx = 0;
  /*do {
    baca_sudut();
    Serial.println(sudut);
    //xx = xx + 1;
    } while ( sudut < 200);
    delay(15000);*/
  int h = 0;
  double sudut_smntra;
  do { //muter badan sampe ke arah 'r_m'
    if (h == 0) {
      baca_sudut();
      sudut_smntra = sudut; //sudut sementara
      h = 1;
    }
    if (cx < 0) { //titik central ada di kiri
      move_left();
    }
    else { //titik central ada di kanan
      move_right();
    }
    baca_sudut();
    //Serial.println(sudut);
    selisih_sudut = abs(sudut - sudut_smntra);
  } while (selisih_sudut <= teta_m );
  baca_sudut();
  double  sudut_smntra2 = sudut;
  Serial.print("Sudut_smntra2= ");
  Serial.println(sudut_smntra2);
  stuck();
  Serial.println("Belok Selesai");
  Serial.println("");

  //menghitung sd_blk_lidar pada setiap x,y tengah untuk pengecekan titik sampai pada cx,cy
  double r_n[q + 1];
  double teta_n[q + 1];
  double sd_blk_lidar[q + 1];
  double asd[q + 1];
  for (s = 1; s <= q; s++) {
    r_n[s] = sqrt(((y_tengah[s] - cy) * (y_tengah[s] - cy)) + ((x_tengah[s] - cx) * (x_tengah[s] - cx))); //jarak titik center ke 'point2'
    teta_n[s] = (asin(abs(y_tengah[s] - cy) / r_n[s])) * 180 / 3.14;  //sudut antara titik center thd 'point2'
    if (x_tengah[s] < 0 && cx < 0 && y_tengah[s] > cy) { //titik tiang ada di sumbu negatif, dan di atas cx,cy
      sd_blk_lidar[s] = 150 + (90 - teta_m - teta_n[s]); //cx,cy di kiri
      asd[s] = 1;
    }
    else if ((x_tengah[s] < 0) && (cx > 0) && (y_tengah[s] > cy)) { //titik tiang ada di sumbu negatif, dan di atas cx,cy
      sd_blk_lidar[s] = 150 + (90 + teta_m - teta_n[s]); //cx,cy di kanan
      asd[s] = 2;
    }
    else if ((x_tengah[s] < 0) && (cx < 0) && (y_tengah[s] < cy)) { //titik tiang ada di sumbu negatif, dan di bawah cx,cy
      sd_blk_lidar[s] = 150 + (90 - teta_m + teta_n[s]); //cx,cy di kiri
      asd[s] = 3;
    }
    else if ((x_tengah[s] < 0) && (cx > 0) && (y_tengah[s] < cy)) { //titik tiang ada di sumbu negatif, dan di bawah cx,cy
      sd_blk_lidar[s] = 150 + (90 + teta_m + teta_n[s]); //cx,cy di kanan
      asd[s] = 4;
    }
    else  if ((x_tengah[s] > 0) && (cx < 0) && (y_tengah[s] > cy)) { //titik tiang ada di sumbu positif, dan di atas cx,cy
      sd_blk_lidar[s] = 150 - (90 + teta_m - teta_n[s]); //cx,cy di kiri
      asd[s] = 5;
    }
    else if ((x_tengah[s] > 0) && (cx > 0) && (y_tengah[s] > cy)) { //titik tiang ada di sumbu positif, dan di atas cx,cy
      sd_blk_lidar[s] = 150 - (90 - teta_m - teta_n[s]); //cx,cy di kanan
      asd[s] = 6;
    }
    else if ((x_tengah[s] > 0) && (cx < 0) && (y_tengah[s] < cy)) { //titik tiang ada di sumbu positif, dan di bawah cx,cy
      sd_blk_lidar[s] = 150 - (90 + teta_m + teta_n[s]); //cx,cy di kiri
      asd[s] = 7;
    }
    else if ((x_tengah[s] > 0) && (cx > 0) && (y_tengah[s] < cy)) { //titik tiang ada di sumbu positif, dan di bawah cx,cy
      sd_blk_lidar[s] = 150 - (90 - teta_m + teta_n[s]); //cx,cy di kanan
      asd[s] = 8;
    }
    else if (x_tengah[s] == 0 || y_tengah[s] == 0) {
      sd_blk_lidar[s] = 150;
      asd[s] = 9;
    }
  }
  //sudah ada sd_blk_lidar sejumlah tiang yg terbaca pada x,y tengah, dan jaraknya dari titik cx,cy sebagai r_n[s]


  /*myServo.write(servoNum, sd_blk_lidar);
    delay(1000);
    Serial.print("teta_n = ");
    Serial.print(teta_n);
    Serial.print("  sd Belok Lidar= ");
    Serial.print(sd_blk_lidar );;
    Serial.print("   r_n=  ");
    Serial.println(r_n);*/

  //maju sambil melihat ke semua titik x,y_tengah, mengukur r_n


  //maju sejauh r_m cm  selama distanceL lebih dari r_n
  int xx = 0;
  Serial.println("Sd Belok Lidar: ");
  move_forward();
  myServo.write(servoNum, 150);
  delay(10000);
  do {
    move_forward();
    //Serial.println(distanceL);
    for (s = 1; s <= q; s++) {
      myServo.write(servoNum, sd_blk_lidar[s]);
      //Serial.print(asd[s]);
      //Serial.print("\t");
      //Serial.println(sd_blk_lidar[s]);
      distanceL = myLidarLite.distance();
      if ((distanceL >= r_n[s] - 30) && (distanceL <= r_n[s] + 30)) {
        stuck();
        xx = 1;
        goto xxx;
      }
      move_forward();
      delay(800);
    }
    for (s = q; s >= 1; s--) {
      myServo.write(servoNum, sd_blk_lidar[s]);
      //Serial.print(asd[s]);
      //Serial.print("\t");
      //Serial.println(sd_blk_lidar[s]);
      distanceL = myLidarLite.distance();
      if ((distanceL >= r_n[s] - 30) && (distanceL <= r_n[s] + 30)) {
        stuck();
        xx = 1;
        goto xxx;
      }
      move_forward();
      delay(800);
    }
  } while (xx == 0);

xxx:
  /*  double sudut_smntra2;
    baca_sudut();
    sudut_smntra2 = sudut; //sudut sementara
    double sudut_smntra3;
    sudut_smntra3 = sudut_smntra2 - sudut_smntra1;
    double sudut_smntra4;
    baca_sudut();
    sudut_smntra4 = sudut;
    do { //muter badan lagi sampe ke arah garis x=0; (sejajar sb y)
    if (cx < 0) { //menghadap ke kiri
      move_right();
    }
    else { //menghadap ke kanan
      move_left();
    }
    baca_sudut();
    selisih_sudut = abs(sudut - sudut_smntra4);
    } while (selisih_sudut <= sudut_smntra3 ); */

  baca_sudut();
  double  sudut_smntra1 = sudut;
  Serial.println("");
  Serial.print("Sudut_smntra= ");
  Serial.println(sudut_smntra);
  Serial.println("");
  Serial.print("Sudut_smntra1= ");
  Serial.println(sudut_smntra1);
  do { //muter badan lagi sampe ke arah garis x=0; (sejajar sb y)
    if (cx < 0) { //menghadap ke kiri
      move_right();
    }
    else { //menghadap ke kanan
      move_left();
    }
    baca_sudut();
    selisih_sudut = abs(sudut - sudut_smntra1);
  } while (selisih_sudut <= teta_m);
  Serial.println("");
  Serial.print("Selisih sudut= ");
  Serial.println(selisih_sudut);
  Serial.println("");
  Serial.println(distanceL);
  Serial.println("arrived");
  stuck();
  //delay(1000000);
  //robot sudah maju ke cx,cy
}

void capture_image() {
}

void move_forward() {
  digitalWrite(motorA1, HIGH); //roda kiri
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH); //roda kanan
  digitalWrite(motorB2, LOW);
  //Serial.println("Maju");
}
void mundur() {
  digitalWrite(motorA1, LOW); //roda kiri
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW); //roda kanan
  digitalWrite(motorB2, HIGH);
  //Serial.println("mundur");
}
void move_left() {
  digitalWrite(motorA1, LOW); //roda kiri
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH); //roda kanan
  digitalWrite(motorB2, LOW);
  //Serial.println("Left");
}
void move_right() {
  digitalWrite(motorA1, HIGH); //roda kiri
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); //roda kanan
  digitalWrite(motorB2, HIGH);
  //Serial.println("Right");
}
void stuck() {
  digitalWrite(motorA1, LOW); //roda kiri
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); //roda kanan
  digitalWrite(motorB2, LOW);
  //Serial.println("Stuck");
}

void baca_sudut() {  //outputnya "sudut"
  mpu6050.update();
  sudut = mpu6050.getAngleZ();
}

void cetak_xy_GLCD(void) {
  double x_GLCD[q + 1];
  double y_GLCD[q + 1];
  int x_awal = 64;
  int y_awal = 60;
  c = 1;
  for (c = 1; c <= q ; c++) {  //x,y_tengah mulai dari 1
    x_GLCD[c] = x_tengah[c] / 12.5 + x_awal;
    y_GLCD[c] = -y_tengah[c] / 12.5 + y_awal;
    //Serial.print(x_GLCD[c]);
    // Serial.print("\t");
    //Serial.println(y_GLCD[c]);
  }
  x_GLCD[0] = cx / 12.5 + x_awal;
  y_GLCD[0] = -cy / 12.5 + y_awal;
  u8g.firstPage();
  do {
    u8g.drawLine(0, 60, 127, 60);
    u8g.drawLine(64, 64,  64, 0);
    u8g.drawDisc(x_awal, y_awal, 1);
    u8g.drawDisc(x_GLCD[0], y_GLCD[0], 2);  //cx,cy
    for (c = 0; c <= q; c++) {
      u8g.drawDisc(x_GLCD[c], y_GLCD[c], 1);
    }
    /*u8g.drawDisc(x_GLCD[2], y_GLCD[2], 1);
      u8g.drawDisc(x_GLCD[3], y_GLCD[3], 1);
      u8g.drawDisc(x_GLCD[4], y_GLCD[4], 1);
      u8g.drawDisc(x_GLCD[5], y_GLCD[5], 1);
      u8g.drawDisc(x_GLCD[6], y_GLCD[6], 1);
      u8g.drawDisc(x_GLCD[7], y_GLCD[7], 1);
      u8g.drawDisc(x_GLCD[8], y_GLCD[8], 1);
      u8g.drawDisc(x_GLCD[9], y_GLCD[9], 1);
      u8g.drawDisc(x_GLCD[10], y_GLCD[10], 1);
      u8g.drawDisc(x_GLCD[11], y_GLCD[11], 1);
      u8g.drawDisc(x_GLCD[12], y_GLCD[12], 1);*/
  } while (u8g.nextPage());
  //Serial.println("Cetak GLCD");
}

void send_to_nano() { //save data x_tengah, y_tengah -- untuk pemetaan tanaman sawit
  c = 1;
  for (c = 1; c <= zxc; c++) {
    cz[c] = cz[c] + cz[c - 1] ;
  }
  for (s = 1; s <= q; s++) {
    Serial3.print(x_tengah[s]);
    Serial3.print(";");
    Serial3.println(y_tengah[s] + cz[c - 1]);
  }
  delay(1000);
}
