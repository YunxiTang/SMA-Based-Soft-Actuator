
#include <EEPROM.h>                     //用于参数Kp,Ki,Kd,数据储存

#include <LiquidCrystal_I2C.h>         // 用于LCD显示

#include <Wire.h>                         //用于LCD显示

#include<PID_v1.h>                        //用于PID控制

#define OutputPin  11                     //定义Output输出引脚11，控制信号

#define listPin A0                        //定义菜单光标位置，从电位器读入

LiquidCrystal_I2C lcd(0x3f,16,2);       //设置LCD1602设备地址

double consKp = 0.5, consKi = 0.2, consKd = 0.1;   //靠近终点时的参数
double aggrKp, aggrKi, aggrKd;                     //未靠近时的参数
double gap;

union data                                         //共用体结构
{
  double a;
  byte b[8];
};

data p,i,d,s;                                       //pid参数存放的共用体

float list = 0.0;                                   //菜单光标位置变量，从电位器读入（0~1023）
int listval = 1;                                    //光标初始化在1处
char output[6];                                     //Output转换为字符output
char input[6];                                      //Input转换为字符input
char kp[6];
char ki[6];
char kd[6];
char setpoint[6];

String comdata = "";                               //空字符串变量,用于接收串口数据

double Setpoint = 0.0, Input = 0.0, Output = 0.0, Input_last = 0.0;                    //设定点，输入，输出, 上一次的输入输入值

//double OutputMax = 255, OutputMin = 0;                                               //输出最大值为255，最小值为0（默认）

PID myPid(&Input, &Output, &Setpoint, aggrKp, aggrKi, aggrKd,P_ON_M, DIRECT);          //设置初始pid模式

void setup()
{
    //PID::SetOutputLimits(OutputMin,OutputMax);       //限制输出范围
    Serial.begin(9600);                               //串口初始化
    Setpoint = 120;                                    //设置终点角度
    myPid.SetMode(AUTOMATIC);                          //pid模式设置为自动计算

    attachInterrupt(1,add,FALLING);                    //按键1按下时，参数加
    attachInterrupt(0,sub,FALLING);                    //按键2按下时，参数减

    lcd.init();                                        //初始化LCD
    lcd.backlight();                                   //设置LCD背景灯亮

    pinMode(listPin,INPUT);

    /********************************依次从EEPROM里面把掉电或reset前的Kp,Ki,Kd,Setpoint参数读取出来******************************/
    for(int k = 0;k <8; k++)
    {
      p.b[k] = EEPROM.read(k); 
    }
    aggrKp = p.a;
    
    for(int m = 8;m < 16;m++)
    {
      i.b[m-8] = EEPROM.read(m); 
    }
    aggrKi = i.a;
    
    for(int n = 16; n < 24;n++)
    {
      d.b[n-16] = EEPROM.read(n);
    }
    aggrKd = d.a;
    
    for(int w = 24; w < 32; w++)
    {
      s.b[w-24] = EEPROM.read(w);                           
    }
    Setpoint = s.a;

    /*************LED闪烁，程序开始运行*************/
    for(int z = 4; z >= 0; z -- )
    {
        analogWrite(OutputPin, 125);
        delay(100);
        analogWrite(OutputPin,0);
        delay(100);
    }
}

//循环
void loop()
{
   /**********************************pid控制部分**********************************/
   comdata = "";
   while(Serial.available() > 0)
   {
        comdata  += char(Serial.read());
        delay(2);
        if(comdata.length() == 3)
        {
          Input = comdata.toInt();
          break;                      
        }
    }                                                   //读取串口数据                                         
              
    gap = abs(Setpoint - Input);
    if(gap < 10)
    {
        myPid.SetTunings(consKp, consKi, consKd);       //靠近时
    }
    else
    {
        myPid.SetTunings(aggrKp, aggrKi, aggrKd);       //未靠近时
    }
    
    myPid.Compute();                                    //计算
    analogWrite(OutputPin, Output);                     //pin 11输出模拟电压
    
    
    /******************************菜单与数据显示部分***********************************/
    list = map(analogRead(listPin),0,1023,0,80);        // 读电位器
    
    mark(list);
    disp(kp);
    disp(ki);
    disp(kd);
    disp(setpoint);
    
    lcd.setCursor(0,0);
    lcd.print("I");
    lcd.setCursor(1,0);                                 // 设置显示指针位置（8，0） 
    dtostrf(Input,3,0,input);
    lcd.print(input);                                   //在LCD上输出input

    lcd.setCursor(0,1);
    lcd.print("O");
    lcd.setCursor(1,1);
    dtostrf(Output,3,0,output);                         //格式转换
    lcd.print(output);                                  //在LCD上输出output
    
    //Serial.println(Output);                           //打印输出值(串口被占用)
    Input_last = Input;                                 //记录输入值
    
    delay(500);                                         //延迟500ms
}

/*****************************************按键，参数加**********************************************/
void add()
{
  delay(500);                                           //延时消抖
  if(listval == 1)                                      //改变参数并回写，在中断里面回写，以防止EEPROM过度擦写
  {
    aggrKp = aggrKp + 0.01;
    p.a = aggrKp;
    for(int k = 0; k < 8; k++)
    {
      EEPROM.write(k,p.b[k]);
    }
  }
  else if(listval == 2)
  {
    aggrKi = aggrKi + 0.01;
    i.a = aggrKi;
    for(int m = 8;m < 16;m++)
    {
      EEPROM.write(m,i.b[m-8]);
    }
  }
  else if(listval == 3)
  {
    aggrKd = aggrKd + 0.01;
    d.a = aggrKd;
    for(int n = 16;n < 24;n++)
    {
      EEPROM.write(n,d.b[n-16]);
    }
  }
  else
  {
    Setpoint = Setpoint + 1;
    s.a = Setpoint;
    for(int w = 24;w < 32;w++)
    {
      EEPROM.write(w,s.b[w-24]);
    }
  }
}

/************************************按键，参数减********************************************/
void sub()
{
  delay(500);                                        //延时消抖
  if(listval == 1)                                   //改变参数并回写到EEPROM
  {
    if(aggrKp > 0)
    {
      aggrKp = aggrKp - 0.01;
    }
    else
    {
      aggrKp = 0.0;
      }
    p.a = aggrKp;
    for(int k = 0; k < 8; k++)
    {
      EEPROM.write(k,p.b[k]);
    }
  }
  else if(listval == 2)
  {
    if(aggrKi > 0)
    {
      aggrKi = aggrKi - 0.01;
    }
    else
    {
      aggrKi = 0.0;
      }
    i.a = aggrKi;
    for(int m = 8;m < 16;m++)
    {
      EEPROM.write(m,i.b[m-8]);
    }
  }
  else if(listval == 3)
  {
    if(aggrKd > 0)
    {
      aggrKd = aggrKd - 0.01;
    }
    else
    {
      aggrKd = 0.0;
     }
    d.a = aggrKd;
    for(int n = 16;n < 24;n++)
    {
      EEPROM.write(n,d.b[n-16]);
    }
  }
  else
  {
    if(Setpoint > 0)
    {
      Setpoint = Setpoint - 1;
    }
    else
    {
      Setpoint =0.0;
    }
    s.a = Setpoint;
    for(int w = 24;w < 32; w++)
    {
      EEPROM.write(w,s.b[w-24]);
    }
  }
}

/**********************************************指针显示函数*************************************************/
void mark(float y)
{
  lcd.setCursor(4,0);                             //清除上一时刻的箭头标志,用空格占位
  lcd.print(" ");
  lcd.setCursor(10,0);
  lcd.print(" ");
  lcd.setCursor(4,1);
  lcd.print(" ");
  lcd.setCursor(10,1);
  lcd.print(" ");                                  
  
  if(y >= 0 && y <= 20.0)                         //根据电位器值，显示箭头位置
  {
    listval = 1;
    lcd.setCursor(4,0);                               
  }
  else if(20.0 < y && y <= 40.0)
  {
    listval = 2;
    lcd.setCursor(10,0);                               
  }
  else if(40.0 < y && y <= 60)
  {
    listval = 3;
    lcd.setCursor(4,1);                               
  }
  else
  {
    listval = 4;
    lcd.setCursor(10,1);
  }
  lcd.print(">");
}

/**************************************pid参数显示函数*********************************************/
void disp(char x[6])
{
  if (x == kp)
  {
    lcd.setCursor(5,0);                               //在P后显示参数kp
    lcd.print("p");
    lcd.setCursor(6,0);
    dtostrf(aggrKp,1,2,kp);
    lcd.print(kp);
  }
  else if(x == ki)
  {
    lcd.setCursor(11,0);                              //在I后显示参数ki
    lcd.print("i");
    lcd.setCursor(12,0);
    dtostrf(aggrKi,1,2,ki);
    lcd.print(ki);
  }
  else if( x == kd)
  {
    lcd.setCursor(5,1);                               //在D后显示参数kd
    lcd.print("d");
    lcd.setCursor(6,1);
    dtostrf(aggrKd,1,2,kd);
    lcd.print(kd);
  }
  else                                                //在S后显示参数Setpoint
  {
    lcd.setCursor(11,1);
    lcd.print("S");
    lcd.setCursor(12,1);
    dtostrf(Setpoint,3,0,setpoint);
    lcd.print(setpoint);
  }
}