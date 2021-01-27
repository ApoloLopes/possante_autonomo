volatile unsigned long pulses1 = 0, //variavel que contará os pulsos do motor 1
                       pulses2 = 0, //variavel que contará os pulsos do motor 2
                       pulses11 = 0,
                       pulses22 = 0;

volatile int dist = 50;

volatile unsigned long counter = 0; //variavel que conta o tempo para a relaização dos calculos

volatile byte control1 = 0;
volatile byte control2 = 0;

volatile int errSum1=0, lastErr1=0; //variaveis para o auxilio do calculo dos pids
volatile int errSum2=0, lastErr2=0;


#define trigPin A0
#define echoPin A1


#define S0 1
#define S1 2
#define S2 3
#define S3 4
#define S4 5 
#define S5 6
#define S6 7
#define S7 8

byte estado;

void procTraj(){

  switch(estado){

    case S0:

    control1 = 0;

    control2 = 0;

    break;

    case S1:
     
     control1 = 8;
     control2 = 8;

     if(dist <= 20){

      control1 = 0;
      control2 = 0;

      estado = S2;

      pulses11=0;
      pulses22=0;

     }

     break;

     case S2:
     
     control1 = 8; 
     control2 = 0;

     if(pulses11>=90){

      control1 = 0;
      control2 = 0;
      
      estado = S3;

      
     }

     break;

     case S3:

     control1 = 8;
     control2 = 8;

     if(dist<=20){

      control1 = 0;
      control2 = 0;

      estado = S4;
      pulses11 = 0;
      pulses22 = 0;
     
     }

     break;

     case S4:

     control1 = 8;
     control2 = 0;

     if(pulses11>=90){

      control1 = 0;
      control2 = 0;

      estado = S5;
      
     }

     break;

     case S5:

     control1 = 8; 
     control2 = 8;

     if(dist<=20){

      control1 = 0;
      control2 = 0;

      estado = S6;
      
      pulses11 = 0; 
      pulses22 = 0;
      
     }
     
     break;

     case S6:

     control1 = 0;
     control2 = 8;

     if(pulses22>=90){

      control1 = 0;
      control2 = 0;

      estado = S7;
      
     }

     break;

     case S7:

     control1 = 8;
     control2 = 8;

     if(dist<=20){
      
      control1=0;
      control2=0;
      estado=S0;
     
     }
     
     break;
  }
}

long duracao;

long distancia_cm=0;


int lerSonar(){    

digitalWrite(trigPin,HIGH); //envia som 
delayMicroseconds(10);
digitalWrite(trigPin,LOW); //não envia o som e espera o retorno do som enviado
duracao = pulseIn(echoPin,HIGH); //Captura a duração em tempo do retorno do som.
distancia_cm = duracao/56; //Calcula a distância
return distancia_cm;             // Retorna a distância  

}

int pid_control1(double kp, double ki, double kd, int input, int setpoint){ //função pid para o motor 1 (input é o numero de pulsos lidos no intervalo de 40ms)
  
  int error = setpoint - input;
  
  errSum1 += (ki * error);
  
  int dErr = (error - lastErr1)*kd;

  int output = kp * error + errSum1 + dErr;
  
  if(output<0){

      output = 0;
  }

  if(output>1023){

    output = 1023;
  
  }
  
  lastErr1 = error;

  return output; 
 
}

int pid_control2(double kp, double ki, double kd, int input, int setpoint){ //função pid para o motor 2
  
  int error = setpoint - input;
  
  errSum2 += (ki * error);
  
  int dErr = (error - lastErr2)*kd;

  int output = kp * error + errSum2 + dErr;

  if(output<0){

      output = 0;
  }

  if(output>1023){

    output = 1023;
  
  }
  
  lastErr2 = error;

  return output; 
   
}

ISR(INT0_vect){            //função que conta os pulsos do motor 1
  
  cli();
  
    delayMicroseconds(200) ;   
    pulses1++;
    pulses11++;

  sei();

}


ISR(INT1_vect){            //função que conta os pulsos do motor 2
  
  cli();
  
    delayMicroseconds(200) ;    
    pulses2++;
    pulses22++;
   
  sei();

}

ISR(TIMER2_OVF_vect){ // funcao que atualiza as variaveis pids

  cli();

  counter++;

  if(counter==5){
    
    OCR1B = pid_control1(20.0, 15.0, 9.0, pulses1, control1);
    OCR1A = pid_control2(20.0, 15.0, 9.0, pulses2, control2);
    pulses1 = 0;
    pulses2 = 0;
    counter = 0;
    
  }
  
  sei();

}



void setup(){

  Serial.begin(9600);

  DDRB |= (1<<DDB1); // saida de pwm nos pinos 9 e 10
  DDRB |= (1<<DDB2);
  
  DDRD &= ~(1<<DDD2);     // configura PD2 como entrada para receber a interrupcao INT0
  DDRD &= ~(1<<DDD3);     // configura PD3 como entrada para receber a interrupcao INT1
  
  DDRD |= (1<<DDD4);      // configura os pinos PD4 a PD7 como saidas para controlar a direcao dos motores      
  DDRD |= (1<<DDD5);
  DDRD |= (1<<DDD6);
  DDRD |= (1<<DDD7);  

  PORTD |= (1<<PORTD4);     // configura a ponte h para girar os motores no mesmo sentido (para frente)
  PORTD &= ~(1<<PORTD5);
  PORTD |= (1<<PORTD6);
  PORTD &= ~(1<<PORTD7);
  

  EICRA |= (1 << ISC00);    // configura INT0 para trigar na borda de subida e de descida (multiplicando a resolucao dos encoders por um fator de 2)
  EICRA &= ~(1 << ISC01);
  EICRA |= (1 << ISC10);    // configura INT1 para trigar na borda de subida e de descida
  EICRA &= ~(1 << ISC11);
  
  EIMSK |= (1 << INT0);     // ativa INT0
  EIMSK |= (1 << INT1);     // ativa INT1

  //Configuração da interrupção por comparacação dos TIMERS 1 e 2

  TCCR2A = 0;
  TCCR2B = 0;
  TCCR1A = 0; //limpando os registradores
  TCCR1B = 0;

  TCNT1 = 0; // inicia o contador em 0
  TCNT2 = 0; //inicia a contagem em 0;

  TCCR2B |= (1<<CS22);
  TCCR2B |= (1<<CS21);  // PRESCALER DE 1024 TIMER 2
  TCCR2B |= (1<<CS20);
  
  TCCR1B |= (1<<CS12);
  TCCR1B &= ~(1<<CS11);
  TCCR1B &= ~(1<<CS10); // PRESCALER 256 TIMER 1

  
  TCCR1A |= (1<<WGM10);
  TCCR1A |=(1<<WGM11);
  TCCR1A |=(1<<WGM12); // FAST PWM MODE 10 BITS TIMER 1
  TCCR1A &= ~(1<<WGM13);

  TCCR1A |= (1<<COM1A1);
  TCCR1A &= ~(1<<COM1A0); //Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM (non-inverting mode)
  TCCR1A |= (1<<COM1B1);
  TCCR1A &= ~(1<<COM1B0);
    
  OCR1A = 0; // vetor que determinam o duty-cycle do pwm no pino 9 
  OCR1B = 0; // vetor que determinam o duty-cycle do pwm no pino 10

  TIMSK2 |= (1<<TOIE2); // ATIVA A INTERRUPÇÃO POR OVERFLOW DO TIMER 2

  pinMode(trigPin, OUTPUT); // TRIG - out
  pinMode(echoPin, INPUT);

  sei();

  estado = S1;

}
void loop(){
  
  dist = lerSonar();
  procTraj();
  
}
