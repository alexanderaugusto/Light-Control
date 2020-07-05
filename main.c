#include "msp430g2553.h"  // Biblioteca padrão do msp430g2553
#include <stdio.h> // Biblioteca que contem comandos como printf para mostrar dados com a UART

// Funções utilizadas abaixo da funçao MAIN devem ter seus prototipos declaras aqui
void configTimer0(void); 
void configTimer1(void);
void configInterrupt();
void configADC();
unsigned int leADC(unsigned int porta, unsigned int canal);
void configUART(void);
void UART_TX(char * tx_data);

// Variaveis
char mensagem_tx[20]; // Vetor que guarda a mensagem transmitida pela UART
char mensagem_rx[32]; // Vetor que guarda a mensagem recebida pela UART
int tam=0; // Variavel que guarda o total de caracteres inseridos 
int TAMANHO = 3; // Variavel para guardar o tamanho do Buffer de caracteres (total de caracteres a serem digitados)
int aux = 0; // Variavel aux
unsigned long int valorLimite = 0; // Variavel que guarda o valor limite do sensor de luz para acender o LED
int valorAD1 = 0; // Armazena os valores do sensor 1
int valorAD2; // Armazena os valores do sensor 2
int valorAD; // Pega a media dos valores do sensor 1 e 2
int valorPWM; // Guarda o valor maximo de luminosidade do LED inserido pelo usuario
unsigned char cont = 0; // contador para auxiliar na contagem do timer

// FUNCAO MAIN
void main( void )
{
  // DESABILITANDO O WATCHDOG 
  WDTCTL = WDTPW + WDTHOLD; // Desabilita o watchdog
  
  // Seta a frequencia de operação do MSP para 1Mhz
  BCSCTL1 = CALBC1_1MHZ;
  DCOCTL = CALDCO_1MHZ; 

  // Para o botao
  P2REN |= BIT0; // Habilitando o resistor interno para o pino P2.0 (botao)
  P2OUT |= BIT0; // Definindo o resistor com pull-up
  // Para o PWM
  P1DIR |= BIT6; // Setando o BIT6 como saida (PWM)
  P1SEL |= BIT6; // Configura o BIT6 como função de saida do sinal do Timer (PWM)
  
  // Iniciando todas as funções necessarioas
  configUART();
  configADC();
  configInterrupt();
  configTimer0();
  configTimer1();
 
  _BIS_SR(GIE); // Habilita a interrupção global
  
  // Zerando todos as posições do vetor de recepção de mensagem para não pegar valores desconhecidos
  mensagem_rx[0] = 0;
  mensagem_rx[1] = 0;
  mensagem_rx[2] = 0;
  TAMANHO = 2 // Setando o tamanho do Buffer de caracteres para 2 
  
  // Inserindo o valor limite de luminosidade pela UART que obrigatoriamente deve ser igual a 70
  UART_TX("Digite o limite de luminosidade ambiente: \r\n");
  do{        
      valorLimite = (( mensagem_rx[0]-48)*10 +(mensagem_rx[1]-48)*1);  // Recebe o valor digitado pelo usuario
  }while(valorLimite != 70);
  sprintf(mensagem_tx, "<%.3d>\n\r",valorLimite);
  UART_TX(mensagem_tx);
        
  // Zerando todas as posições do vetor de mensagem novamente para não pegar valores lixos
  mensagem_rx[0] = 0;
  mensagem_rx[1] = 0;
  mensagem_rx[2] = 0;
  TAMANHO = 3; // Setando o tamanho do buffer de caracteres para 3

  // Inserindo o valor maximo de luminosidade do led que deve obrigatoriamente estar entre 0 e 100
  UART_TX("Digite o valor maximo de luminosidade do led: \r\n");
  do{      
      valorPWM = ((mensagem_rx[0]-48)*100 +( mensagem_rx[1]-48)*10 +(mensagem_rx[2]-48)*1); // Pegando o valor inserido na UART 
  }while((valorPWM < 000) || (valorPWM > 100));
  sprintf(mensagem_tx, "<%.3d>\n\r",valorPWM);
  UART_TX(mensagem_tx);
        
  valorLimite = 45; 
  valorLimite =  (valorLimite*10.23); // convertendo o valor entrado de porcentagem para um valor de 0 a 1023 
  valorPWM = 80;
  
  for(;;){  
        valorAD1 = leADC(BIT4,INCH_4); // Lendo o valor que está no sensor de luz 1 (Que está conectado ao BIT4)
        valorAD2 = leADC(BIT3,INCH_3); // Lendo o valor que está no sensor de luz 2 (Que está conectado ao BIT3
        valorAD = (valorAD1 + valorAD2)/2; // Fazendo a media dos dois sensores
        
        //sprintf(mensagem_tx, "<%.3d>\n\r",valorAD);
        //UART_TX(mensagem_tx);     
  }
}
// FUNCAO DE CONFIGURACAO DO TIMER1
void configTimer1(void){
  TA1CTL |= MC_1; // Configurando para conter em modo crescente
  TA1CTL |= TASSEL_2; // Configura o clock para SMCLK (Sub-main Clock)
  TA1CTL |= ID_1; // Divide o clock por 2
  TA1CCR0 |= 49999; // Valor maximo da contagem
  TA1CCTL0 |= CCIE; // Habilita a interrupção do timer
}

// FUNCAO DE CONFIGURACAO DO TIMER0 COMO PWM
void configTimer0(void){
  TA0CCTL1 = OUTMOD_7; // Seleciona o modo SET/RESET
  TA0CCR0 = 19999; // Valor do periodo - 1 (Periodo  = 20000)
  TA0CCR1 = 0; // Tempo em que o sinal ficará ligado (inicialmente é zero)
  TA0CTL = TASSEL_2 + ID_0 + MC_1; // Utilizando o clock como SMCLK, Dividindo o clock por 1 (nao vai mudar) e configurando para contar em modo crescente
  TA0CCTL0 &= ~CCIE; // Desabilitando inicialmente o PWM
}

// FUNCAO DE CONFIGURACAO DE INTERRUPCAO
void configInterrupt(){  
  P2IES |= BIT0; // Configurando para acionar com a transição de descida do clock
  P2IE |= BIT0; // Habilitando a interrupção no BIT0
  P2IFG &=~ BIT0; // Limpa a flag de interrupção
}

// FUNCAO DE CONFIGURACAO DO CONVERSOR AD (COM SENSOR)
void configADC()
{
  ADC10CTL1 |= SHS_0 + ADC10SSEL_3; // Disparo do Sample-and-hold no bit SC + Configura o clock interno para SMCLK (5Mhz)
  ADC10CTL0 = SREF_0 + ADC10ON; // Referencia interna para VCC e GND + Liga o conversor
}

// FUNCAO PARA LER O CONVERSOR AD
unsigned int leADC(unsigned int porta, unsigned int canal) // Recebe a porta (BIT) e o canal (INCH_X)
{
  unsigned long int valor = 0;
  int media;
  ADC10AE0 |= porta;  // Configura a porta para funcionar como entrada analogica
  
  ADC10CTL0 &=~  ADC10ON; // Desliga o conversor
  ADC10CTL1 &=~ INCH_7; // Pino que queremos converter
  ADC10CTL1 |= canal; // Bit a ser convertido
  ADC10CTL0 |=  ADC10ON; // Iniciar a conversão
  
  for(int i = 0;i<100;i++){
    ADC10CTL0 |= ENC + ADC10SC; // Habilita e inicia a conversão
    while((ADC10CTL0 & ADC10IFG) == 0); // Espera a conversão AD terminar e quando finaliada seta o ADC10IFG para zero
    valor += ADC10MEM; // Armazena o valor lido na variavel valor (somando com o que tinha antes para tirar a media)
  }
  
   ADC10CTL0 &= ~ADC10IFG; // Zera a flag para proxima conversao
   valor = valor/100; // Divide o valor lido por 100 (pois passo 100 vezes no for)
   media = (int) valor; // atribui o valor a variavel media
   return media;
}

// FUNCAO PARA CONFIGURAR O UART
void configUART(void)
{
  P1SEL |= BIT1 + BIT2; // Seleciona o BIT1 e BIT2 para funcionarem como UART
  P1SEL2 |= BIT1 + BIT2; // Seleciona o BIT1 e BIT2 para funcionarem como UART
  UCA0CTL1 |= UCSSEL_2;  // Utiliza o clock principal
  UCA0BR0 = 104; // Configura o baudrate para 9600      Baudrate: tempo de duração de cada bit
  UCA0BR1 = 0; // Configura o baudrate para 9600
  UCA0MCTL = UCBRS0;  // Modulação UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST;  // Inicia a UART
  IE2 |= UCA0RXIE; 
}

//Função de envio de mensagens “STRINGS”
void UART_TX (char * tx_data) 
{
  unsigned int i=0;
  while(tx_data[i]) // Espera enviar todos caracteres da STRING
  {
    while ((UCA0STAT & UCBUSY)); //espera terminar o envio da ultima informação
    UCA0TXBUF = tx_data[i]; // envia o elemento na posição i
    i++; // incrementa posição do vetor
  }
}

// ROTINA DE INTERRUPÇÃO DO TIMER1
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
{
  // Se o botao for pressionado a o valor dos sensores passarem do valor limite inserido pelo usuario (70)
   if(((P2IN&BIT0) == 0) && (valorAD > valorLimite)){
      cont = 0;
    }else{
      cont++;
    }
    
   // Caso nao existe mais pessoas (botao nao pressionado) e o valor do sensor fique abaixo do valor limite, então deve-se desligar o LED
   // Conta 100 vezes para dar os 10 segundos (cada passagem aqui seria 100ms)
   if(cont >= 100){
    TACCR1 = 0; // Apaga o LED
   }
    
  TA1CCTL0 &=~ CCIFG; // Limpa a flag de interrupção
}

// ROTINA DE INTERRUPÇÃO EXTERNA (BOTAO)
#pragma vector = PORT2_VECTOR
__interrupt void INTERRUPCAO_EXTERNA_ISR(void)
{
  // Quando o botao for pressionado, se o valor do sensor for maios que o valor Limite, então acende o LED com o valor do PWM inserido pelo usuario
  if(valorAD > valorLimite){
    TA0CCR1 = valorPWM * 499; // Multiplica o valor por 499 para ficar na faixa dos 49999 que é o maximo
  } 

  P2IFG &=~ BIT0; // Limpa a flag de interrupção
}

// ROTINA DE INTERRUPÇÃO PARA RECEPÇÃO DE MENSAGENS
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  mensagem_rx[tam] = UCA0RXBUF; // TX -> RXed character
  
  // Este trecho é para recepção de mensagem que tenha valor maior que um
  tam++;
  if(tam == TAMANHO)
  {
    tam=0;
  }

  IFG2=IFG2&~UCA0RXIFG; // Limpa a flag de interrupção
} 
