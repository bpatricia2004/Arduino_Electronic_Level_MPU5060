/*
  Proiect: Sensors System Control using Arduino boards
  Student: Bălășcău Patricia-Andreea
  Descriere: Citire date senzor MPU-6050 si afisare unghiuri Pitch/Roll
  Interfata: LCD I2C, indicator LED de nivel si alarma sonora (buzzer)
*/


#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// configurare pini pentru adaptorul I2C al LCD-ului
const int  en = 2, rw = 1, rs = 0, d4 = 4, d5 = 5, d6 = 6, d7 = 7, bl = 3;

// adresa I2C standard pentru modulul LCD PCF8574 (poate fi 0x27 sau 0x3F)
const int i2c_addr = 0x27;

LiquidCrystal_I2C lcd(i2c_addr, en, rw, rs, d4, d5, d6, d7, bl, POSITIVE);

// nivelurile led urilor (pinii)
int levelLED_neg1  = 9;
int levelLED_neg0  = 10;
int levelLED_level = 11;
int levelLED_pos0  = 12;
int levelLED_pos1  = 13;

// pin pt buzzer
const int buzzerPin = 8;

// variabilele pentru giroscop pt stocarea datelor brute
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

// variabile pentru accelerometru
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

// variabilele finale pentru unghiuri (grade) rezultate dupa filtrare
float angle_pitch, angle_roll;
float angle_pitch_output, angle_roll_output;

// timer pt mentinerea buclei de 4ms (250Hz) si variabila temperaturii
long loop_timer;
int temp;   // MPU-6050 are si un senzor de temperatura intern

// counter pt a nu bloca procesarea cu update-ul LCD-ului (refresh la ~0.4s)
int displaycount = 0;

void setup() {
     // start I2C => arduino devine master pe bus
  Wire.begin();

    // LCD initializare + lumina background
  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // setarea led urilor ca outputs
  pinMode(levelLED_neg1, OUTPUT);
  pinMode(levelLED_neg0, OUTPUT);
  pinMode(levelLED_level, OUTPUT);
  pinMode(levelLED_pos0, OUTPUT);
  pinMode(levelLED_pos1, OUTPUT);

  // buzzer output (pin 8 trimite curent catre buzzer)
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // config senzor (setarea registrelor)
  setup_mpu_6050_registers();

  // calibrare gyro (!sa fie nemiscat)
  gyro_x_cal = gyro_y_cal = gyro_z_cal = 0;
  for (int cal_int = 0; cal_int < 1000; cal_int++) {
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;   //se calculează suma a 1000 de masuratori
    delay(3); // ~250Hz gen asteptare intre citiri
  }
  gyro_x_cal /= 1000;   //calcularea mediei erorii pe fiecare axa
  gyro_y_cal /= 1000;   //rezultatul este "zero-rate offset"
  gyro_z_cal /= 1000;   //aceste valori sunt apoi scazute din toate citirile ulterioare pentru a opri drift-ul

  Serial.begin(115200); //comunicare cu PC-ul pentru debugging

  loop_timer = micros();   //initializare timer pt loop
}

void loop() {
  // se citesc datele curente (14 bytes) de la mpu6050
  read_mpu_6050_data();

  // // se scad offset urile (eroarea calculata la setup) pentru a avea 0 cand sta pe loc
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // calcul unghiuri prin integrarea vitezei unghiulare (se bazeaza pe freq=250Hz)
  // 0.0000611 = constanta rezultata din sensibilitatea giros. (65.5 LSB/deg/s) si esantionarea de 0.004s
  angle_pitch += gyro_x * 0.0000611;  // constanta rezultata din sensibilitatea giros. si esantionarea de 4ms
  angle_roll  += gyro_y * 0.0000611;  

  // compensare Yaw- corectie daca senzorul se roteste si pe verticala in timp ce e inclinat
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  angle_roll  -= angle_pitch * sin(gyro_z * 0.000001066);

  // calcul unghiuri folosind accelerometrul (gravitațională)
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296; //57.296 = convertire Radiani in Grade
  angle_roll_acc  = asin((float)acc_x / acc_total_vector) * -57.296;

  optional accel => calibrare
  angle_pitch_acc -= 0.0;
  angle_roll_acc  -= 0.0;

  if (set_gyro_angles) {
    // FILTRU COMPLEMENTAR: combina ambele date pentru stabilitate 
    // 99.96% din unghiul vechi + giro si doar 0.04% din accelerometru (corectie drift)
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll  = angle_roll  * 0.9996 + angle_roll_acc  * 0.0004;
  } else {
    // prima rulare: initializeaza unghiurile direct cu datele de la accelerometru
    angle_pitch = angle_pitch_acc;
    angle_roll  = angle_roll_acc;
    set_gyro_angles = true;
  }

  // extra => ajuta la eliminarea tremuratului cifrelor pe LCD
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_roll_output  = angle_roll_output  * 0.9 + angle_roll  * 0.1;

  // BUZZER: sound if |pitch(x)| > 45 OR |roll(y)| > 45 
  if (fabs(angle_pitch_output) > 45.0 || fabs(angle_roll_output) > 45.0) {
    tone(buzzerPin, 2000);   // 2kHz makes beep cand depaseste pragul critic
  } else {
    noTone(buzzerPin);      //opreste sunetul sub 45 grade
  }                         //fabs() pt obtinerea valorii absolute (ignora semnul +/-) 

  // Update LCD/LEDs slower - nu are sens sa scriem pe LCD la fiecare 4ms
  displaycount++;

  if (displaycount > 100) {   //intra aici la fiecare ~400ms (100 iterații * 4ms)
    lcd.clear();

    // linia1: afisare pitch(x)
    lcd.setCursor(0, 0);
    lcd.print("P:");
    lcd.print(angle_pitch_output, 1);  //afisare cu o zecimala
    lcd.print((char)223); // simbolul de grad (°)
    lcd.print(" ");

    // linia2: afisare roll(y)
    lcd.setCursor(0, 1);
    lcd.print("R:");
    lcd.print(angle_roll_output, 1);
    lcd.print((char)223);
    lcd.print(" ");

    // functia imparte domeniul Pitch(x) in zone si activeaza LED-ul corespunzator
    updateLEDs(angle_pitch_output);

    displaycount = 0;
  }

  // SINCRONIZARE: esantionarea trb sa fie constanta (4ms) pt calculele giroscopului
  while (micros() - loop_timer < 4000);       //blocheaza executia pana trec exact 4000 µs (4 ms)
  loop_timer = micros();   // reseteaza timerul pt urmatoarea iteratie
}

// Logica LED-urilor - vizualizarea inclinarii ca la o bula de aer
void updateLEDs(float angle_pitch_deg) {
  if (angle_pitch_deg < -2.01) {
    digitalWrite(levelLED_neg1, HIGH);
    digitalWrite(levelLED_neg0, LOW);
    digitalWrite(levelLED_level, LOW);
    digitalWrite(levelLED_pos0, LOW);
    digitalWrite(levelLED_pos1, LOW);
  } else if ((angle_pitch_deg > -2.00) && (angle_pitch_deg < -1.01)) {
    digitalWrite(levelLED_neg1, LOW);
    digitalWrite(levelLED_neg0, HIGH);
    digitalWrite(levelLED_level, LOW);
    digitalWrite(levelLED_pos0, LOW);
    digitalWrite(levelLED_pos1, LOW);
  } else if ((angle_pitch_deg < 1.00) && (angle_pitch_deg > -1.00)) {   // NIVEL PERFECT
    digitalWrite(levelLED_neg1, LOW);
    digitalWrite(levelLED_neg0, LOW);
    digitalWrite(levelLED_level, HIGH);
    digitalWrite(levelLED_pos0, LOW);
    digitalWrite(levelLED_pos1, LOW);
  } else if ((angle_pitch_deg > 1.01) && (angle_pitch_deg < 2.00)) {
    digitalWrite(levelLED_neg1, LOW);
    digitalWrite(levelLED_neg0, LOW);
    digitalWrite(levelLED_level, LOW);
    digitalWrite(levelLED_pos0, HIGH);
    digitalWrite(levelLED_pos1, LOW);
  } else if (angle_pitch_deg > 2.01) {
    digitalWrite(levelLED_neg1, LOW);
    digitalWrite(levelLED_neg0, LOW);
    digitalWrite(levelLED_level, LOW);
    digitalWrite(levelLED_pos0, LOW);
    digitalWrite(levelLED_pos1, HIGH);
  }
}

void setup_mpu_6050_registers() {
  // accesare MPU-6050 prin adresa I2C 0x68
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  //registru de management al alimentarii
  Wire.write(0x00);  //registrul pt a dezactiva modul de repaus (Sleep Mode)
  Wire.endTransmission();  //senzorul poate sa inceapa masuratorile

  // configurarea accelerometrului - interval +/- 8g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);   //registrul pt configurare Accel
  Wire.write(0x10);   //valoarea 0x10 selecteaza intervalul ±8 g (echilibru sensibilitate/limita)
  Wire.endTransmission();

  // configurarea giroscopului - interval +/- 500 deg/s
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);  //registrul pt configurare Gyro
  Wire.write(0x08);  //corespunde intervalului ±500°/s pt miscari lente si medii, cat si pt rezolutie buna
  Wire.endTransmission();
}

void read_mpu_6050_data() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);    //incepe citirea de la primul registru de date (Accel X High)
  Wire.endTransmission();

//achizitii date senzor-> 14 bytes (6 pt Accel, 2 pt Temp, 6 pt Gyro)
  Wire.requestFrom(0x68, 14);   //pt valori brute intr-o singura op, 14 bytes de la 0x3B
  while (Wire.available() < 14);   //asteapta pana sosesc toti octetii

//reconstructia valorilor pe 16 biti din cate 2 bytes de 8 biti (High/Low)
  acc_x  = (Wire.read() << 8) | Wire.read(); //deplasare 8 biti la stanga si unire cu restul (bitwise OR)
  acc_y  = (Wire.read() << 8) | Wire.read();  //primul Wire.read() furnizeaza byte-ul superior, deplasat la stânga cu 8 biți
  acc_z  = (Wire.read() << 8) | Wire.read();   //al doilea Wire.read() furnizeaza byte-ul inferior
  temp   = (Wire.read() << 8) | Wire.read();   //operatorul | uneste cele doua parti intr-un intreg pe 16 biti
  gyro_x = (Wire.read() << 8) | Wire.read();
  gyro_y = (Wire.read() << 8) | Wire.read();
  gyro_z = (Wire.read() << 8) | Wire.read();
}
