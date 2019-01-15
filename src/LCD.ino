

int menu=1;
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 32, en = 34, d4 = 36, d5 = 38, d6 = 40, d7 = 42;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
int kicker_strong=10;
void buton_test(){
  if(digitalRead(44)){lcd.setCursor(12,0); lcd.print('1');}
 else{lcd.setCursor(12,0); lcd.print('0');}
  if(digitalRead(24)){lcd.setCursor(9,0); lcd.print('1');}
 else{lcd.setCursor(9,0); lcd.print('0');}
  if(digitalRead(26)){lcd.setCursor(6,0); lcd.print('1');}
 else{lcd.setCursor(6,0); lcd.print('0');}
  if(digitalRead(28)){lcd.setCursor(3,0); lcd.print('1');}
 else{lcd.setCursor(3,0); lcd.print('0');}
  if(digitalRead(30)){lcd.setCursor(0,0); lcd.print('1');}
 else{lcd.setCursor(0,0); lcd.print('0');}}

void co(){
    lcd.clear();
    while(!digitalRead(44)){
    lcd.setCursor(0,0);lcd.print("SET X X X BACK");
    lcd.setCursor(0,1);lcd.print("compass:");
    lcd.print(compass());
     if(digitalRead(30)){
       compass_set_north();
      }
    delay(60);
    lcd.clear();
    }
    lcd.clear();
  }
void ki(){
    int x=kicker_strong;
    lcd.clear();
    while(!digitalRead(44)){
    lcd.setCursor(0,0);lcd.print("- SET + KIK BACK");
    lcd.setCursor(0,1);lcd.print("constant:");
    lcd.print(x);
    if(digitalRead(26)){
      x++;
      delay(500);
      lcd.clear();
      }
    else if(digitalRead(30)){
      x--;
      delay(500);
      lcd.clear();
      }
    else if(digitalRead(28)){
      kicker_strong = x;
      lcd.clear();
      lcd.print("kicker_strong=");
      lcd.print(x);
      delay(100);

      }
    }
    lcd.clear();
    }
void ca(){
lcd.clear();
    while(!digitalRead(44)){
     static int cam_mode = 1;
      pixyViSy.update();
    ball_distance = pixyViSy.getBallDist();
    ball_angle = pixyViSy.getBallAngle();
    if (ball_distance == ~0) {
      lcd.clear();
       lcd.setCursor(0,0);lcd.print("ANG DIS X X BACK");

       lcd.setCursor(0,1);lcd.print("NO BALL DETECTED");

       delay(200);
    }
    else{
       lcd.clear();
       lcd.setCursor(0,0);lcd.print("ANG DIS X X BACK");
       if(cam_mode==1){
       lcd.setCursor(0,1);lcd.print("ANGLE:");
       lcd.print(ball_angle); }
       else if(cam_mode==2){
              lcd.setCursor(0,1);lcd.print("DISTANCE:");
       lcd.print(ball_distance);
       }

       delay(200);
      }
      if(digitalRead(30)){
        cam_mode=1;
        }
      if(digitalRead(28)){
        cam_mode=2;
        }
    }
    lcd.clear();}
void st(){
    lcd.clear();
    while(!digitalRead(44)&&!digitalRead(30)){
    lcd.setCursor(0,0);lcd.print("START X X X BACK");
    lcd.setCursor(0,1);lcd.print("READY FOR PLAY");
    }
    if(digitalRead(30)){menu=0;}
    lcd.clear();
      lcd.setCursor(3,0);lcd.print("PLAYING...");
  lcd.setCursor(5,1);lcd.write("DANCZI");
  }
void li(){
  lcd.clear();
  while(!digitalRead(44)){
     lcd.setCursor(0,1);
lcd.print(digitalRead(LINE_FRONT_PIN));
lcd.print("  ");
lcd.print(digitalRead(LINE_BACK_PIN));
lcd.print("  ");
lcd.print(digitalRead(LINE_LEFT_PIN_1));
lcd.print(digitalRead(LINE_LEFT_PIN_2));
lcd.print("  ");
lcd.print(digitalRead(LINE_RIGHT_PIN_1));
lcd.println(digitalRead(LINE_RIGHT_PIN_2));

      }
  }
void mo(){
  int mode_motor;
      lcd.clear();

    while(!digitalRead(44)){
      lcd.setCursor(0,0);lcd.print("A B C D BACK");

         if(digitalRead(30)){
          while(digitalRead(30)){
          for(int i=-255;i<255&&digitalRead(30);i++){   //-255/255
            motorA.go(i);
            delay(20);
          }
          for(int i=255;i>-255&&digitalRead(30);i--){ //255/255
            motorA.go(i);
            delay(20);
          }}
        }
        else{motorA.go(0);}

         if(digitalRead(28)){
          while(digitalRead(28)){
          for(int i=-255;i<255&&digitalRead(28);i++){   //-255/255
            motorB.go(i);
            delay(20);
          }
          for(int i=255;i>-255&&digitalRead(28);i--){ //255/255
            motorB.go(i);
            delay(20);
          }}
        }
        else{motorB.go(0);}

                 if(digitalRead(26)){
          while(digitalRead(26)){
          for(int i=-255;i<255&&digitalRead(26);i++){   //-255/255
            motorC.go(i);
            delay(20);
          }
          for(int i=255;i>-255&&digitalRead(26);i--){ //255/255
            motorC.go(i);
            delay(20);
          }}
        }
        else{motorC.go(0);}

                 if(digitalRead(24)){
          while(digitalRead(24)){
          for(int i=-255;i<255&&digitalRead(24);i++){   //-255/255
            motorD.go(i);
            delay(20);
          }
          for(int i=255;i>-255&&digitalRead(24);i--){ //255/255
            motorD.go(i);
            delay(20);
          }}
        }
        else{motorD.go(0);}


  }lcd.clear();}
void start_menu(){
  static long last_buton = 0;
  int x=0;
String text = "XLC_team 2018/2019 Slovakia Topolcany team:Adam,Jakub,Adam Robot name is DANCZI";
int dlska= text.length();

   while(menu){

    if(slide==1){
  if(digitalRead(28)){co();}//compas
  else if(digitalRead(26)){st();}//start
  else if(digitalRead(24)){ca();}//camera
else{ lcd.setCursor(0,0);lcd.print("1/4 CO STA CA BA");
  lcd.setCursor(5,1);lcd.write("DANCZI");}
    }
     else if(slide==2){
 
  if(digitalRead(28)){li();}//lights
  else if(digitalRead(26)){st();}//start
  else if(digitalRead(24)){ki();}//kicker
else{ lcd.setCursor(0,0);lcd.print("2/4 LI STA KI BA");
  lcd.setCursor(5,1);lcd.write("DANCZI");}
    }
   else if(slide==3){

  if(digitalRead(28)){mo();}//motors
  else if(digitalRead(26)){st();}//start
  else if(digitalRead(24)){}//dribler
else{ lcd.setCursor(0,0);lcd.print("3/4 MO STA DR BA");
  lcd.setCursor(5,1);lcd.write("DANCZI");}
    }
   else if(slide==4){
 lcd.setCursor(0,0);lcd.print("4/4 XLC_TEAMinfo");

      for(int x=0;(!digitalRead(30))&&x<dlska-15;x++){

      for(int stlpec=0;(!digitalRead(30))&&stlpec<16;stlpec++){
      lcd.setCursor(stlpec,1);
      lcd.print(text[x]);
      x++;
      }

      delay(700);
      x=x-16;
    

  }
    }
  if(digitalRead(30)){
    slide++;
    if(slide==5){
      slide=1;
      }
     delay(400);
     lcd.clear();
    }
      }



  }
  void LCD_setup(){

        pinMode(44, INPUT); //s5
        pinMode(24, INPUT); //s4
        pinMode(26, INPUT); //s3
        pinMode(28, INPUT); //s2
        pinMode(30, INPUT); //s1

        lcd.begin(16, 2);
        lcd.setCursor(2,0);
        lcd.write("XLC_Bardaci");
        lcd.setCursor(5,1);
        lcd.write("DANCZI");
        lcd.setCursor(0,0);
        delay(1500);
        lcd.clear();
    }
