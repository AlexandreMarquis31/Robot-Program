#include <Servo.h>
#define MOTG_AV     0b10000     //PD4   = D4
#define MOTG_AR     0b100000    //PD5   = D5
#define MOTD_AV     0b1000000   //PD6   = D6
#define MOTD_AR     0b10000000  //PD7   = D7
#define MOTG_VIT    0b10        //PB1   = D9
#define MOTD_VIT    0b100       //PB2   = D10
#define PIN_SERVO   0b1000      //PB3   = D12
#define RING_BUFFER_MAX 1
typedef struct {
    size_t s_elem;
    size_t n_elem;
    void *buffer;
} rb_attr_t;

typedef unsigned int rbd_t;
struct ring_buffer
{
    size_t s_elem;
    size_t n_elem;
    uint8_t *buf;
    volatile size_t head;
    volatile size_t tail;
};
char rbmem[64];
ring_buffer rxBuffer;
static rbd_t _rbd;
//static char _rbmem[32];
static struct ring_buffer _rb[RING_BUFFER_MAX];
char vitesse = 0;
char rotation = 100;

//Servo servo;
/*
struct ring_buffer
{
    uint8_t s_elem;
    uint8_t n_elem;     
    uint8_t *buf;
    volatile uint8_t head ;
    volatile uint8_t tail;
};*/

int ring_buffer_init(rbd_t *rbd, rb_attr_t *attr)
{
    static int idx = 0;
    int err = -1; 
 
    if ((idx < RING_BUFFER_MAX) && (rbd != NULL) && (attr != NULL)) {
        if ((attr->buffer != NULL) && (attr->s_elem > 0)) {
            /* Check that the size of the ring buffer is a power of 2 */
            if (((attr->n_elem - 1) & attr->n_elem) == 0) {
                /* Initialize the ring buffer internal variables */
                _rb[idx].head = 0;
                _rb[idx].tail = 0;
                _rb[idx].buf = attr->buffer;
                _rb[idx].s_elem = attr->s_elem;
                _rb[idx].n_elem = attr->n_elem;
 
                *rbd = idx++;
                err= 0;
            }
        }
    }
 
    return err;
}

static int _ring_buffer_full(struct ring_buffer *rb)
{
    return ((rb->head - rb->tail) == rb->n_elem) ? 1 : 0;
}
 
static int _ring_buffer_empty(struct ring_buffer *rb)
{
    return ((rb->head - rb->tail) == 0U) ? 1 : 0;
}


ISR(USART_RX_vect){
  /*
  uint8_t data = UDR0;
  ring_buffer_put(&rxBuffer,&data);*/

  const char c = UDR0;
 
  
 
  ring_buffer_put(_rbd, &c);
  
}
/*
static int _ring_buffer_full(struct ring_buffer *rb)
{
    if ((rb->head - rb->tail) == rb->n_elem){
      return true;
    }
    else{
      return false;
    }
}
 
static int _ring_buffer_empty(struct ring_buffer *rb)
{
    if((rb->head - rb->tail) == 0){
      return true;
    }
    else{
      return false;
    }
}*/

int ring_buffer_put(rbd_t rbd, const void *data)
{
    int err = 0;
 
    if ((rbd < RING_BUFFER_MAX) && (_ring_buffer_full(&_rb[rbd]) == 0)) {
        const size_t offset = (_rb[rbd].head & (_rb[rbd].n_elem - 1)) * _rb[rbd].s_elem;
        memcpy(&(_rb[rbd].buf[offset]), data, _rb[rbd].s_elem);
        _rb[rbd].head++;
    } else {
        err = -1;
    }
 
    return err;
}

int ring_buffer_get(rbd_t rbd, void *data)
{
    int err = 0;
 
    if ((rbd < RING_BUFFER_MAX) && (_ring_buffer_empty(&_rb[rbd]) == 0)) {
        const size_t offset = (_rb[rbd].tail & (_rb[rbd].n_elem - 1)) * _rb[rbd].s_elem;
        memcpy(data, &(_rb[rbd].buf[offset]), _rb[rbd].s_elem);
        _rb[rbd].tail++;
    } else {
        err = -1;
    }
 
    return err;
}
/*
void ring_buffer_put(struct ring_buffer *rb, const void *data)
{
    if (_ring_buffer_full(rb)) {
        const size_t offset = (rb->head & (rb->n_elem - 1)) * rb->s_elem;
        memcpy(&(rb->buf[offset]), data, rb->s_elem);
        rb->head++;
    }
}

void ring_buffer_get(struct ring_buffer *rb, void *data)
{
    if (!(_ring_buffer_empty(rb))) {
        const size_t offset = (rb->tail & (rb->n_elem - 1)) * rb->s_elem;
        memcpy(data, &(rb->buf[offset]), rb->s_elem);
        rb->tail++;
    }
}*/

int uart_getchar(void)
{
    char c = -1;
 
    ring_buffer_get(_rbd, &c);
 
    return c;
}






void setup() {
   Serial.begin(9600);
   //rb_attr_t attr = {sizeof(_rbmem[0]), 32, _rbmem};

  /*
  ring_buffer rxBuffer;
  rxBuffer.s_elem=8;
  rxBuffer.n_elem=64; //doit etre une puissance de 2
  rxBuffer.head=0;
  rxBuffer.tail=0;
  rxBuffer.buf = rbmem;*/
  pinMode(LED_BUILTIN, OUTPUT);
  //servo.attach(PIN_SERVO);
  //DDRD = MOTD_AV|MOTD_AR|MOTG_AV|MOTG_AR;
  //PORTD = 0b0;
  //DDRB = MOTG_VIT|MOTD_VIT|PIN_SERVO;
  
  //TCCR1A = 1<<COM1A1|1<<COM1B1|1<<WGM10;
  //TCCR1B = 1<<CS12|1<<WGM12;
  //TCCR2A = 1<<COM2A1|1<<WGM20;
  //TCCR2B = 1<<CS12|1<<WGM12;
  /*
  //baudrate = 9600
  UBRR0L = 0b1011;
  //Enable receiver 
  UCSR0B = 1<<RXEN0|1<<RXCIE0;
  // Set frame format: 8data
  UCSR0C =1<<UCSZ00|1<<UCSZ01;
  */
 // interrupts();
  
  /*
  Serial.begin(9600);
  Serial.println(UCSR0A,BIN);
  Serial.println(UCSR0B,BIN);
  Serial.println(UCSR0C,BIN);
  Serial.println(UBRR0L,BIN);*/
  /*int baud = 9600;
uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
  UCSR0A = 1 << U2X0;
  //baud_setting=0b101;
  // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
  UBRR0H = baud_setting >> 8;
  UBRR0L = baud_setting;

  //_written = false;

  //set the data bits, parity, and stop bits
  UCSR0C = 1<<UCSZ00|1<<UCSZ01;;

  UCSR0B = 1<< RXEN0|1<<TXEN0|1<<RXCIE0|1<<UDRIE0;*/
}

void loop() { 
  /*
  while (!(UCSR0A & (1<<RXC0)));
  int data=UDR0;
  Serial.println(data);*/
  /*
 if(UCSR0A & (1<<RXC0)){
   uint8_t data = readSerial();
   //Serial.println(data);
   if (data == 254){
       data = readSerial(); 
       //Serial.println(data);
       vitesse = data-100;
       deplacement(vitesse,rotation);   
   }
   else if (data == 250) {
      data = readSerial();
      //Serial.println(data);
      rotation= data;
      deplacement(vitesse,rotation);
   } 
   else if (data == 253){
      data = readSerial();
      //Serial.println(data);
      int value = convert(data,0,200,2250,700);
      //servo.write(value);
      //delay(10);        
   }
 }
 */
 /*
 //digitalWrite(LED_BUILTIN, HIGH);
    uint8_t data = 0;
    uint8_t data2= 0;
    ring_buffer_get(_rbd,&data);
    ring_buffer_get(_rbd,&data2);
    if (data == 254){
       digitalWrite(LED_BUILTIN, HIGH);
       //Serial.println(data);
       vitesse = data2-100;
       deplacement(vitesse,rotation);   
     }
     else if (data == 250) {
       digitalWrite(LED_BUILTIN, HIGH);
       //Serial.println(data);
       rotation= data2;
       deplacement(vitesse,rotation);
     } 
     else if (data == 253){
      pinMode(LED_BUILTIN, OUTPUT);
    //digitalWrite(LED_BUILTIN, HIGH);
      //Serial.println(data);
      int value = convert(data2,0,200,2250,700);
      //servo.write(value);
      //delay(10);        
   
 }*/
}

void deplacement(int vitesse, int rotation){
  float diffMoteur = float(rotation-100)/100.0;
  int valeurAnalogueGauche =0;
  int valeurAnalogueDroite =0;
  if (diffMoteur <0){
    diffMoteur=-diffMoteur;
    valeurAnalogueGauche = convert(max(vitesse,-vitesse)*(1-diffMoteur),0,100,0,255);
    valeurAnalogueDroite = convert(max(vitesse,-vitesse),0,100,0,255);
  }
  else {
    valeurAnalogueGauche = convert(max(vitesse,-vitesse),0,100,0,255);
    valeurAnalogueDroite = convert(max(vitesse,-vitesse)*(1-diffMoteur),0,100,0,255); 
  }
  OCR1A = valeurAnalogueDroite;
  OCR1B = valeurAnalogueGauche;
  if (vitesse <0){
    PORTD=0b0;
    PORTD = MOTG_AR|MOTD_AR;
  }
  else if (vitesse >0){
    PORTD=0b0;
    PORTD = MOTG_AV|MOTD_AV;
 }
 else if (vitesse == 0 && rotation != 100){
    int valeurAnalogue = convert(max(rotation-100,-(rotation-100)),0,100,0,255);
    OCR1A = valeurAnalogue;
    OCR1B = valeurAnalogue;
    if (rotation >100){
      PORTD=0b0;
      PORTD = MOTG_AV|MOTD_AR;
    }
    else{
      PORTD=0b0;
      PORTD = MOTG_AR|MOTD_AV;
    }
 }
}

float convert(float x,float x_a,float x_b,float y_a,float y_b){
     return ((y_b-y_a/x_b-x_a)*(x-x_b))+y_b;
}

