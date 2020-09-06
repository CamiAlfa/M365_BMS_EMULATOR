#include <avr/wdt.h>
//this uses watchdog so arduino must have new bootloader
//pin lectura voltaje batería 
#define BattV_Pin A7
//pin blinky
boolean Blinky_status = false;
#define Blinky_Pin 13

#define BattV_Rbig    10000000
#define BattV_Rsmall  470000
//characteristics bateria
#define BattCap 6600
#define BattS 10
//voltaje minimo 280 =2.8v
#define Undervolt 280
#define Overvolt 421
//0x55,0xAA,0x03,0x22,0x01,0x10,0x10,0xB9,0xFF
//registros en memoria de la bateria
#define MastertoBATT 0x22
#define BATTtoMaster 0x25

//serial bateria string, 
#define BATTserialREG 0x10
#define BATTserialLEN 14
//version bms ex 0x0112 = v1.1.2
#define BATTbmsverREG 0x17
#define BATTbmsverLEN 2
//capacidad de fabrica de bateria en mAh
#define BATTfabcapREG 0x18
#define BATTfabcapLEN 2
//fecha a la batt 7 MSB->año, siguientes 4bits->mes, 5 LSB ->dia ex:
//b 0001_010=10, año 2010
//        b 1_000= 8 agosto
//            b  0_1111=15 
//  0001_0101_0000_1111=0x150F 
#define BATTdateREG 0x20
#define BATTfabcapLEN 2
//mAh restantes en la bateria
#define BATTremaincapREG 0x31
#define BATTremaincapLEN 2
//porcentaje de la batería en porcentaje
#define BATTbattREG 0x32
#define BATTbattLEN 2
//corriente de la batería en amperios*100, puede ser negativo(carga)
#define BATTcurrentREG 0x33
#define BATTcurrentLEN 2
//voltaje bateria en voltios*10
#define BATTvoltREG 0x34
#define BATTvoltLEN 2
//temperatura celdas byte MSB temp1 y LSB temp2, en grados mas 20. ex 31 = 11grados
#define BATTtempREG 0x35
#define BATTtempLEN 2
//Vida de la batería, 0-100, error si es menor a 60
#define BATThealthREG 0x3b
#define BATThealthLEN 2
//voltaje packs del 1 al 10 en miliVoltios
#define BATTpackLEN 2
#define BATTpack1REG 0x40
#define BATTpack2REG 0x41
#define BATTpack3REG 0x42
#define BATTpack4REG 0x43
#define BATTpack5REG 0x44
#define BATTpack6REG 0x45
#define BATTpack7REG 0x46
#define BATTpack8REG 0x47
#define BATTpack9REG 0x48
#define BATTpack10REG 0x49
//memoria que usaremos
#define memsize 0x80
volatile uint16_t memory[memsize];
//buffer mensajes de entrada
#define buffersize 0x88
//max buffersize needed known 136 or 0x88
volatile uint8_t input_buffer[buffersize];
volatile uint16_t input_buffer_head=0;
volatile uint16_t input_buffer_tail=0;
volatile uint8_t input_buffer_full=0;
//extract an element without flush
uint8_t parse_buffer(uint16_t index){
    uint16_t buffer_index=(input_buffer_tail+index);
    if (buffer_index>=buffersize) buffer_index-=buffersize;
    return input_buffer[(uint8_t)(buffer_index)];
}

//extract flush an element
uint8_t flush_buffer(uint16_t items){
    input_buffer_tail+=items;
    if (input_buffer_tail>=buffersize) input_buffer_tail-=buffersize;
    input_buffer_full=0;
    return input_buffer_tail;
}

//serial stuff
uint16_t send_serial_mem(uint8_t target, uint16_t _size){
    uint8_t par=_size&0x01;//miro si es par o no
    uint8_t cantidad=_size>>1;//division por 2
    uint16_t x;
    uint16_t suma=0;
    for (x=target;x<(target+cantidad);x++){
    //index=(target) & 0x7f;//truco para hacer un loop en la memoria, solo tenemos 128
        suma+=send_serial_16blsb(memory[x&0x7f]);
        suma+=send_serial_16bmsb(memory[x&0x7f]);
    }
    if (par!=0){
        suma+=send_serial_16blsb(memory[(uint8_t)((target+cantidad+1)&0x7f)]);
    }
    return suma;
}

uint16_t send_serial_16bmsb(uint16_t data){
    return send_serial_byte((uint8_t)((data>>8)&0xff));
}
uint16_t send_serial_16blsb(uint16_t data){
    return send_serial_byte((uint8_t)((data)&0xff));
}

uint16_t send_serial_byte(uint8_t data){
  Serial.write(data);
  return (uint16_t)data;
}
//estimar porcentaje con voltaje
uint8_t estimate_batt(uint16_t volt, uint16_t batt_s_count){
  static float filtered_volt=0; //en 0.01V
  float instant_volt=0; //en 0.01V
  const float filter_value = 0.003;
  uint16_t cell_volt;
  uint8_t estimated_percent=0;
  //first time I enter here I set the initial filtered volt
  if(filtered_volt==0)
    filtered_volt=volt;
  //volt value is noisy so I filter it
  filtered_volt=filtered_volt-(filter_value*(filtered_volt-(float)volt));
  //get cell volt
  cell_volt=filtered_volt/batt_s_count;//lo tengo en resolucion de 0.01v

  instant_volt=(float)volt/batt_s_count;
  if (instant_volt<=Undervolt){
    memory[0x30]|=0x0140;//activa flag
  }//no hay else, para que el error no se limpie
  
  if (instant_volt>=Overvolt){
    memory[0x30]|=0x0200;
  } else {
    memory[0x30]&=~0x0200;
  }
  
  if (cell_volt<=310){
    estimated_percent=0;
  } else if (cell_volt<=350){
    estimated_percent=(cell_volt-310)/4;
  } else if (cell_volt<=370){
    estimated_percent=(cell_volt-350)+10;
  } else if (cell_volt<=390){
    estimated_percent=((cell_volt-370)*3)+30;
  } else if (cell_volt<=410){
    estimated_percent=((cell_volt-390)/2)+90;
  } else {
    estimated_percent=100;
  }
  return estimated_percent;
}
//timer para refresh
unsigned long now_time;
unsigned long next_time=0;
void refresh(){
  now_time=millis();
  if (now_time>=next_time){
    next_time=now_time+1000;//shedule next time
//leer el voltaje de la batería
  int ADC_val_BATT = analogRead(BattV_Pin);//adc value
  float Batt_volt= ((float)5/(float)1024)*(float)ADC_val_BATT;//volt on pin


  
  Batt_volt=100*Batt_volt*((float)(BattV_Rsmall+BattV_Rbig)/(float)BattV_Rsmall);//volt on battery calculated with resistor divisor
  memory[BATTvoltREG]=(uint16_t)(Batt_volt);//save value on memory so scooter can read it
  //porcentaje y capacidad
  memory[BATTbattREG]=estimate_batt((uint16_t)(Batt_volt),BattS);
  memory[BATTremaincapREG]=(uint16_t)(((uint32_t)memory[BATTbattREG]*BattCap)/100); 
  //test
  //memory[BATTcurrentREG]++;
    digitalWrite(Blinky_Pin,Blinky_status);
  Blinky_status=!Blinky_status;
  }

}
//
void setup() {
    //watchdog
  //MCUSR = MCUSR & B11110111;
  wdt_disable(); // Desactivar el watchdog mientras se configura, para que no se resetee
  wdt_enable(WDTO_2S); // Configurar watchdog a 2 segundos
  // initialize serial:
  delay(1000);//espera a que todo sea estable
  Serial.begin(115200);
  pinMode(Blinky_Pin, OUTPUT);

 
  // initialize memory
  uint16_t x;
  for (x=0;x<memsize;x++){
      memory[x]=0;
  }
  //serial, must start as 3J
  memory[BATTserialREG]=((uint16_t)'3'|((uint16_t)'J')<<8);
  memory[BATTserialREG+1]=((uint16_t)'C'|((uint16_t)'a')<<8);
  memory[BATTserialREG+2]=((uint16_t)'m'|((uint16_t)'i')<<8);
  memory[BATTserialREG+3]=((uint16_t)'A'|((uint16_t)'l')<<8);
  memory[BATTserialREG+4]=((uint16_t)'f'|((uint16_t)'a')<<8);
  memory[BATTserialREG+5]=((uint16_t)'_'|((uint16_t)'B')<<8);
  memory[BATTserialREG+6]=((uint16_t)'M'|((uint16_t)'S')<<8);

  //version bms
  memory[BATTbmsverREG]=0x115;
  //capacidad de batería
  memory[BATTfabcapREG]=BattCap;
  //fecha batería 31 ago 2020
  memory[BATTdateREG]=(((uint16_t)20)<<9)|(((uint16_t)8)<<5)|(((uint16_t)31));
  //batt flags
  memory[0x30]=0x0001;
  //mah restantes
  //memory[BATTremaincapREG]=2600;//calculo
  //porcentaje
  //memory[BATTbattREG]=100;//calculo
  //corriente de la batería en centi-amperios, puede ser negativo(carga)
  memory[BATTcurrentREG]=0;
  //voltaje en centivoltios
  //memory[BATTvoltREG]=4100;//lo calculo con adc
  //temperatura celdas byte MSB temp1 y LSB temp2, en grados mas 20. ex 31 = 11grados
  memory[BATTtempREG]=(((uint16_t)(15+20))<<8)|(((uint16_t)(16+20))<<0);
  //Vida de la batería, 0-100, error si es menor a 60
  memory[BATThealthREG]=90;
  //voltaje de las baterías en milivoltios
  memory[BATTpack1REG]=3010;
  memory[BATTpack2REG]=3020;
  memory[BATTpack3REG]=3030;
  memory[BATTpack4REG]=3040;
  memory[BATTpack5REG]=3050;
  memory[BATTpack6REG]=3060;
  memory[BATTpack7REG]=3070;
  memory[BATTpack8REG]=3080;
  memory[BATTpack9REG]=3090;
  memory[BATTpack10REG]=3100;
}

void loop() {
  int rcvd_count,x;
  uint16_t chksum=0;
  uint16_t outchksum;
  static uint16_t expected=2;
  refresh();
rcvd_count=input_buffer_head-input_buffer_tail;
    if (rcvd_count<0) rcvd_count+=buffersize;
    if (rcvd_count==0 && input_buffer_full==1) rcvd_count=buffersize;

    if (rcvd_count>=(expected+6)){

        if (parse_buffer(0)!=0x55 && parse_buffer(1)!=0xaa){//mensaje invalido
            //correr buffer para tratar de sincronizar
            flush_buffer(1);
        } else {
            //cabecera correcta
            if ((parse_buffer(2)+6)<buffersize)expected=parse_buffer(2);
            else expected=2;
            if ((expected+6)<=rcvd_count){                   //mensaje completo
                chksum=0;
                for (x=expected+3;x>1;x--){                 //calculo del chksum
                    chksum+=parse_buffer(x);
                }
                chksum=chksum^0xffff;
                if (parse_buffer(expected+4)==(chksum&0xff) && parse_buffer(expected+5)==((chksum>>8)&0xff) && parse_buffer(3)==0x22){//checksum valido y dirigido a la batería
                  wdt_reset();
                    switch(parse_buffer(4)){//que tipo de comando recibo
                    case 0x01://lectura aqui es lo interesante
                        //send_serial_byte('R');
                        outchksum=0;
                        send_serial_byte(0x55);
                        send_serial_byte(0xaa);
                        outchksum+=send_serial_byte(parse_buffer(6)+2);
                        outchksum+=send_serial_byte(0x25);
                        outchksum+=send_serial_byte(0x01);
                        outchksum+=send_serial_byte(parse_buffer(5));
                        outchksum+=send_serial_mem(parse_buffer(5),parse_buffer(6));
                        outchksum^=0xffff;
                        send_serial_16blsb(outchksum);
                        send_serial_16bmsb(outchksum);
                        
                        break;
                    case 0x03://escritura
                        //send_serial_byte('W');
                        break;
                    case 0x07://tamaño fw
                        //send_serial_byte('7');
                        break;
                    case 0x08://parte fw
                        //send_serial_byte('8');
                        break;
                    case 0x09://tamaño fw
                        //send_serial_byte('9');
                        break;
                    case 0x0a://reset
                        //send_serial_byte('a');
                        break;
                    default:
                        //send_serial_byte('z');
                        break;
                    }
                    //quitar este mensaje del buffer
                    flush_buffer(expected+6);
                } else {                                    //checksum invalido
                    //vaciar buffer
                    flush_buffer(rcvd_count);
                }
            }
        }

    }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    uint8_t inChar = (uint8_t)Serial.read();
    if (input_buffer_full==0){
        input_buffer[input_buffer_head]=inChar;
        input_buffer_head++;
        if (input_buffer_head>=buffersize) input_buffer_head-=buffersize;
        if (input_buffer_head==input_buffer_tail)input_buffer_full=1;
      }
  }
}
