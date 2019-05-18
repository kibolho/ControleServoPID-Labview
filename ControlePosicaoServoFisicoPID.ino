// ControlePosicaoServo.ino

#include <LiquidCrystal.h>
#include <Keypad.h>


class PID{
public:
  
  double error;
  double sample;
  double lastSample;
  double kP, kI, kD;      
  double P, I, D;
  double pid;
  
  double setPoint;
  long lastProcess;
  
  PID(double _kP, double _kI, double _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }
  
  void addNewSample(double _sample){
    sample = _sample;
  }
  
  void setSetPoint(double _setPoint){
    setPoint = _setPoint;
  }
  
  double process(){
    // Implementação P ID
    error = setPoint - sample;
    float deltaTime = (millis() - lastProcess) / 1000.0;
    lastProcess = millis();
    
    //P
    P = error * kP;
    
    //I
    I = I + (error * kI) * deltaTime;
    
    //D
    D = (lastSample - sample) * kD / deltaTime;
    lastSample = sample;
    
    // Soma tudo
    pid = P + I + D;
    
    return pid;
  }
};

LiquidCrystal lcd (8, 9, 4, 5, 6, 7);
int in1 = 2;
int in2 = 3;

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
#define encoder A2
#define positionSet A1
byte rowPins[ROWS] = {31, 33, 35, 37}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {39, 41, 43, 45}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

int referencia, referenciaFinal;
int posicaoAtual;
int encoderValue;
int erro;
long lastClearLCD;
PID meuPid(1.0, 0.0, 0.2);

void setup() {
    lcd.begin(16, 2);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	Serial.begin(9600); //set up serial port
  	delay(500);
  encoderValue = map(analogRead(encoder),30,980,0,1023);
  posicaoAtual = map(encoderValue,0,1023,0,180);
  //referencia = posicaoAtual;
  lastClearLCD = millis();
}
/*
#define SOP '<'
#define EOP '>'

bool started = false;
bool ended = false;

char inData[3];
byte index;
*/

int pos = 0;    // variable to store the servo position
int motor = 0;

void loop()
{
  // Read all serial data available, as fast as possible
  /*
  while(Serial.available() > 0)
  {
    char inChar = Serial.read();
    if(inChar == SOP)
    {
       index = 0;
       inData[index] = '\0';
       started = true;
       ended = false;
    }
    else if(inChar == EOP)
    {
       ended = true;
       break;
    }
    else
    {
      if(index < 5)
      {
        inData[index] = inChar;
        index++;
        inData[index] = '\0';
      }
    }
  } 
  // We  are here either because all pending serial
  // data has been read OR because an end of
  // packet marker arrived. Which is it?
  if(started && ended)
  {
    // The end of packet marker arrived. Process the packet
    referenciaFinal = atoi(inData);
    if(referenciaFinal>180){
      referenciaFinal=180;
    }else if (referenciaFinal<0){
      referenciaFinal=0;
    }
    // Reset for the next packet
    started = false;
    ended = false;
    index = 0;
    inData[index] = '\0';
  }
  */
   while (Serial.available() > 0) {
    
    // look for the next valid integer in the incoming serial stream:
    motor = Serial.parseInt();
   
    // do it again:
    pos = Serial.parseInt();
  
    // look for the newline. That's the end of your  sentence:
    if (Serial.read() == '\n') {
              
       referenciaFinal = pos;             // tell servo to go to position in variable 'pos'
       delay(15);                       // waits 15ms for the servo to reach the position
     
      // print the three numbers in one string as hexadecimal:
      Serial.print("Data Response : ");
      Serial.print(motor, DEC);
      Serial.print(pos, DEC);
      
    }
  }
  
  char key = keypad.getKey();
  if(referencia>180){
  	referencia=0000;
  }
  if (key != NO_KEY){
  	if(key == '#'){
  		if(referencia<=180){
   			referenciaFinal = referencia;
   		}
   		referencia = 0;
  	}else{
  		int multiplicador;
  		switch (key) {
  			case '0':
  		      multiplicador = 0;
  		      break;
  		    case '1':
  		      multiplicador = 1;
  		      break;
  		    case '2':
  		  	  multiplicador = 2;
  		      break;
  		    case '3':
		  	  multiplicador = 3;
  		      break;
  		    case '4':
		  	  multiplicador = 4;
  		      break;
  		    case '5':
		  	  multiplicador = 5;
  		      break;
  		    case '6':
		  	  multiplicador = 6;
  		      break;
  		    case '7':
		  	  multiplicador = 7;
  		    break;
  		    case '8':
		  	  multiplicador = 8;
  		    break;
  		    case '9':
		  	  multiplicador = 9;
  		    break;
  		    default:
  		      break;
  		}
  		if(referencia<1){
  			referencia=multiplicador;
  		}else if(referencia<100){
  		   	referencia=referencia*10+multiplicador;
  		}
  	}
  }
  if((millis() - lastClearLCD)>200){
        lcd.clear();
        lastClearLCD = millis();
  }
  	encoderValue = map(analogRead(encoder),30,980,0,1023);
	posicaoAtual = map(encoderValue,0,1023,0,180);
	lcd.setCursor(0, 0);
	lcd.print("ENCODER: ");
	lcd.print(posicaoAtual);
	lcd.setCursor(0, 1);
	lcd.print("REF: ");
	lcd.print(referenciaFinal);
	lcd.print(" ");
	lcd.print(referencia);
        //Enviando posicao Atual
        Serial.println(posicaoAtual);
        
	meuPid.addNewSample(posicaoAtual); //0 a 180
	// Envia a referencia
	meuPid.setSetPoint(referenciaFinal); //0 a 180
	// Converte para controle
	erro = (meuPid.process());
	erro = map(erro,0,180,0,255);
	erro=8*erro;
        lcd.print(" ");
	lcd.print(erro);
	if(erro>255){
		erro=255;
	}else if(erro<-255){
		erro=-255;
	}
	if(erro<0){
		erro = -erro;
		analogWrite(in1, 0);
		analogWrite(in2, erro);
	}else if(erro>0){
		erro = erro;
		analogWrite(in1, erro);
		analogWrite(in2, 0);
	}
	else{
		digitalWrite(in1, HIGH);
		digitalWrite(in2, HIGH);
	}
}

