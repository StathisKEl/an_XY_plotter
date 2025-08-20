#include <SpeedyStepper.h>

#include <SPI.h>
#include <SD.h>

#include<Wire.h>
#include<ADS1115_WE.h> 


#define I2C_ADDRESS 0x48

#define warning_led 15
#define success_led 16
#define success_buzzer 7

#define EN_PIN_X 6
#define DIR_PIN_X 4
#define STEP_PIN_X 5
#define limit_switch_x 17

#define EN_PIN_Y 3
#define DIR_PIN_Y 14
#define STEP_PIN_Y 2
#define limit_switch_y 18

#define stop_button 19

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

const int width_metal = 230;
const int length_metal = 350;

const int microstep_setting_x = 8;
const int microstep_setting_y = 8;

volatile bool serious_error = false;

void homing_xy();
void get_content_line(unsigned long index_line, unsigned long num_char_line, int* information_line);
void go_point(int coordinate_x, int coordinate_y, float degrees_one_step_x, float degrees_one_step_y, float div_max_values_x, float div_max_values_y);
int write_measurement_coordinates(int coordinates_error);
//Uncomment this for plotting the magnetic flux density in real time.
//Caution: you can not use the SD card write functionality and the Serial Print functionality at the same time.
//void plot_measurement_coordinates();
//Uncomment this for finding the X and Y axis in the physical workspace.
//void config(int coordinate_x, int coordinate_y, float degrees_one_step_x, float degrees_one_step_y);


File myFile;

SpeedyStepper stepper_x;
SpeedyStepper stepper_y;

void setup()
{
  int num_lines_file = 0, general_error = 0, i = 1, k, information_line[3] = {0}, writing_error;
  unsigned long size_file, pos_last_char_line[100], index_line, num_char_line;
  float degrees_one_step_x, degrees_one_step_y, width_metal_steps, length_metal_steps, div_max_values_x, div_max_values_y;
  char char_file;

  //Uncomment this for plotting the magnetic flux density in real time.
  //Caution: you can not use the SD card write functionality and the Serial Print functionality at the same time.
  //Serial.begin(9600);

  Wire.begin();

  attachInterrupt(digitalPinToInterrupt(stop_button), turn_off_motors, FALLING);

  pinMode(success_buzzer, OUTPUT);
  pinMode(warning_led, OUTPUT);
  pinMode(success_led, OUTPUT);

  stepper_x.connectToPins(STEP_PIN_X, DIR_PIN_X);
  stepper_y.connectToPins(STEP_PIN_Y, DIR_PIN_Y);

  pinMode(EN_PIN_X, OUTPUT);
  pinMode(EN_PIN_Y, OUTPUT);
  pinMode(limit_switch_x, INPUT_PULLUP);
  pinMode(limit_switch_y, INPUT_PULLUP);
  pinMode(stop_button, INPUT_PULLUP);

  digitalWrite(EN_PIN_X, LOW);
  digitalWrite(EN_PIN_Y, LOW);

  pos_last_char_line[0] = 0;

  if(!adc.init())
  {
    // Ο ADC δεν συνδέθηκε.
    general_error = 1;
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_6144);
  
  adc.setConvRate(ADS1115_860_SPS);

  
  if (!SD.begin()) 
  {
    // Η κάρτα microSD δεν αρχικοποιήθηκε.
    general_error = 1;
  }
  else
  {
    if (SD.exists("CRD.csv"))
    {
      myFile = SD.open("CRD.csv");
      if (!myFile)
      {
        // Το αρχείο CRD δεν άνοιξε.
        general_error = 1;
      }
      else
      {
        size_file = myFile.size();
        while (myFile.available()>0) 
        {
          char_file = myFile.read();
          if (char_file=='\n')
          {
            num_lines_file += 1;
            pos_last_char_line[i] = myFile.position();
            i += 1;
          }
        }
        myFile.close();
      }
    }
    else
    {
      // Το αρχείο CRD δεν υπάρχει στην κάρτα microSD.
      general_error = 1;
    }
  }
  
  degrees_one_step_x = 1.8/microstep_setting_x;
  degrees_one_step_y = 1.8/microstep_setting_y;
  
  width_metal_steps = (width_metal*0.028125)/(degrees_one_step_x*0.003134667506);
  length_metal_steps = (length_metal*0.028125)/(degrees_one_step_y*0.003134025412);
  div_max_values_x = 12500/width_metal_steps;
  div_max_values_y = 12500/length_metal_steps;

  homing_xy();

  //Uncomment this for finding the X and Y axis in the physical workspace.
  /*delay(1000);

  config(230 ,0, degrees_one_step_x, degrees_one_step_y);
  config(0, 0, degrees_one_step_x, degrees_one_step_y);
  config(0, 350, degrees_one_step_x, degrees_one_step_y);
  config(0, 0, degrees_one_step_x, degrees_one_step_y);*/

  delay(3000);

  if (SD.exists("MSR.txt"))
  {
    SD.remove("MSR.txt");
  }
  if (general_error==0)
  {
    for (k=0;k<num_lines_file+1;k++)
    {
      if (serious_error==true)
      {
        break;
      }
      index_line = pos_last_char_line[k];
      if (k==num_lines_file)
      {
        num_char_line = size_file - pos_last_char_line[k];
      }
      else
      {
        num_char_line = pos_last_char_line[k+1] - pos_last_char_line[k] - 2;
      }
      get_content_line(index_line, num_char_line, information_line);
      if (information_line[2]!=1)
      {
        go_point(information_line[0], information_line[1], degrees_one_step_x, degrees_one_step_y, div_max_values_x, div_max_values_y);
      }
      writing_error = write_measurement_coordinates(information_line[2]);
      //Uncomment this for plotting the magnetic flux density in real time.
      //Caution: you can not use the SD card write functionality and the Serial Print functionality at the same time.
      //plot_measurement_coordinates();
      if (writing_error==1)
      {
        digitalWrite(warning_led, HIGH);
        break;
      }
      delay(3000);
    }
  }
  else
  {
    digitalWrite(warning_led, HIGH);
  }
  
  if (general_error!=1 && writing_error!=1 && serious_error!=true)
  {
    stepper_x.setSpeedInStepsPerSecond(200);
    stepper_x.setAccelerationInStepsPerSecondPerSecond(100);
    stepper_y.setSpeedInStepsPerSecond(200);
    stepper_y.setAccelerationInStepsPerSecondPerSecond(100);

    stepper_x.moveToPositionInSteps(0);
    delay(100);
    stepper_y.moveToPositionInSteps(0);

    delay(1000);

    digitalWrite(EN_PIN_X, HIGH);
    digitalWrite(EN_PIN_Y, HIGH);
    
    delay(1000);

    digitalWrite(success_led, HIGH);
    digitalWrite(success_buzzer, HIGH);
    delay(5000);
    digitalWrite(success_buzzer, LOW);
  }
}

void loop()
{
}

void homing_xy()
{
  bool good_home_x, good_home_y;

  do {
  good_home_x = stepper_x.moveToHomeInSteps(1, 200.0, 76085, limit_switch_x);
  delay(100);
  good_home_y = stepper_y.moveToHomeInSteps(1, 200.0, 113751, limit_switch_y);
  } while (good_home_x==false || good_home_y==false);
}

void get_content_line(unsigned long index_line, unsigned long num_char_line, int* information_line)
{
  String coordinates_string = "", x_coordinate_string = "", y_coordinate_string = "";
  int pos_question_mark, coordinates_error = 0;
  char tpr_char;
  unsigned long j;
  
  myFile = SD.open("CRD.csv");
  if (myFile)
  {
    myFile.seek(index_line);
    for (j=0;j<num_char_line;j++)
    {
      coordinates_string += (char)myFile.read();
    }
    myFile.close();
  }
  pos_question_mark = coordinates_string.indexOf(';');
  if (pos_question_mark!=-1)
  {
    if (pos_question_mark>0 && pos_question_mark<coordinates_string.length()-1)
    {
      for (j=0;j<coordinates_string.length();j++)
      {
        tpr_char = coordinates_string.charAt(j);
        if (tpr_char==';')
        {
          continue;
        }
        if (isDigit(tpr_char))
        {
          if (j<pos_question_mark)
          {
            x_coordinate_string += tpr_char;
          }
          if (j>pos_question_mark)
          {
            y_coordinate_string += tpr_char;
          }
        } 
        else
        {
          // Συντεταγμένες με έναν ή περισσότερους μη αριθμητικούς χαρακτήρες.
          coordinates_error = 1;
          break;
        }
      }
    }
    else
    {
      // Απουσία μιας εκ των δυο συντεταγμένων.
      coordinates_error = 1;
    }
  }
  else
  {
    // Απουσία του χαρακτήρα ;.
    coordinates_error = 1;
  }

  if (coordinates_error!=1 && (x_coordinate_string.toInt()>230 || y_coordinate_string.toInt()>350))
  {
    // Μια εκ των δυο συντεταγμένων ή και οι δυο συντεταγμένες είναι out of range.
    coordinates_error = 1;
  }

  if(coordinates_error!=1)
  {
    *information_line = x_coordinate_string.toInt();
    *(information_line+1) = y_coordinate_string.toInt();
  }

  *(information_line+2) = coordinates_error;
}

void go_point(int coordinate_x, int coordinate_y, float degrees_one_step_x, float degrees_one_step_y, float div_max_values_x, float div_max_values_y)
{
  float x_coordinate_steps, y_coordinate_steps, difference_x, difference_y;
  double speed_per_sec_x, speed_per_sec_y;

  x_coordinate_steps = (-coordinate_x*0.028125)/(degrees_one_step_x*0.003134667506);
  y_coordinate_steps = (-coordinate_y*0.028125)/(degrees_one_step_y*0.003134025412);

  difference_x = x_coordinate_steps - (float)stepper_x.getCurrentPositionInSteps();
  difference_y = y_coordinate_steps - (float)stepper_y.getCurrentPositionInSteps();
  difference_x = abs(difference_x);
  difference_y = abs(difference_y);

  if (difference_x>difference_y)
  {
    speed_per_sec_x = (div_max_values_x*pow(difference_x,2)+div_max_values_y*difference_y*difference_x)/(difference_x+difference_y);
    speed_per_sec_y = (div_max_values_x*pow(difference_x,2)*difference_y+div_max_values_y*pow(difference_y,2)*difference_x)/(difference_x*(difference_x+difference_y));
  }
  else if (difference_x<difference_y)
  {
    speed_per_sec_y = (div_max_values_y*pow(difference_y,2)+div_max_values_x*difference_y*difference_x)/(difference_x+difference_y);
    speed_per_sec_x = (div_max_values_x*pow(difference_x,2)*difference_y+div_max_values_y*pow(difference_y,2)*difference_x)/(difference_y*(difference_x+difference_y));
  }
  else
  {
    speed_per_sec_x = (div_max_values_x*difference_x)/2+(div_max_values_y*difference_y)/2;
    speed_per_sec_y = (div_max_values_x*difference_x)/2+(div_max_values_y*difference_y)/2;
  }

  stepper_x.setSpeedInStepsPerSecond((float)speed_per_sec_x);
  stepper_x.setAccelerationInStepsPerSecondPerSecond((float)speed_per_sec_x/2);
  stepper_y.setSpeedInStepsPerSecond((float)speed_per_sec_y);
  stepper_y.setAccelerationInStepsPerSecondPerSecond((float)speed_per_sec_y/2);

  stepper_x.setupMoveInSteps((long)x_coordinate_steps);
  stepper_y.setupMoveInSteps((long)y_coordinate_steps);

  while((stepper_x.motionComplete()==false) || (stepper_y.motionComplete()==false))
  {
    stepper_x.processMovement();
    stepper_y.processMovement();
  }
}

int write_measurement_coordinates(int coordinates_error)
{
  int writing_error = 0;
  float msr_value;
  char buff[7];
  myFile = SD.open("MSR.txt", FILE_WRITE);
  if (myFile)
  {
    if (coordinates_error!=1)
    {
      adc.setCompareChannels(ADS1115_COMP_0_GND);
      adc.startSingleMeasurement();
      while(adc.isBusy()){}
      msr_value = adc.getResult_mV();
      msr_value = map(msr_value, 500, 4500, -64, 64);
      dtostrf(msr_value, 6, 3, buff);
      buff[6]='\n';
      myFile.write(buff, 7);
    }
    else
    {
      myFile.println('!');
    }
    myFile.close();
  }
  else
  {
    // Αδυναμία πρόσβασης της πλακέτας Arduino στα δεδομένα του αρχείου MSR.
    writing_error = 1;
  }
  return writing_error;
}

//Uncomment this for plotting the magnetic flux density in real time.
//Caution: you can not use the SD card write functionality and the Serial Print functionality at the same time.
/*void plot_measurement_coordinates()
{
  float msr_value;

  adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.startSingleMeasurement();
  while(adc.isBusy()){}
  msr_value = adc.getResult_mV();
  msr_value = map(msr_value, 500, 4500, -64, 64);
  Serial.println(msr_value);
}*/

void turn_off_motors()
{
  digitalWrite(EN_PIN_X, HIGH);
  digitalWrite(EN_PIN_Y, HIGH);

  serious_error = true;
}

//Uncomment this for finding the X and Y axis in the physical workspace.
/*void config(int coordinate_x, int coordinate_y, float degrees_one_step_x, float degrees_one_step_y)
{
  float x_coordinate_steps, y_coordinate_steps;
  
  x_coordinate_steps = (-coordinate_x*0.028125)/(degrees_one_step_x*0.003134667506);
  y_coordinate_steps = (-coordinate_y*0.028125)/(degrees_one_step_y*0.003134025412);

  stepper_x.setSpeedInStepsPerSecond(200);
  stepper_x.setAccelerationInStepsPerSecondPerSecond(100);
  stepper_y.setSpeedInStepsPerSecond(200);
  stepper_y.setAccelerationInStepsPerSecondPerSecond(100);

  stepper_x.setupMoveInSteps((long)x_coordinate_steps);
  stepper_y.setupMoveInSteps((long)y_coordinate_steps);

  while((stepper_x.motionComplete()==false) || (stepper_y.motionComplete()==false))
  {
    stepper_x.processMovement();
    stepper_y.processMovement();
  }
}*/