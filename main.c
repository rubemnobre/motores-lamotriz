//C�DIGO PARA CONTROLE DE VELOCIDADE DE UM MOTOR DE INDU��O TRIF�SICO DE 0.25CV.
//TRABALHO DE CONCLUS�O DE CURSO
//ALUNO: LEONARDO DUARTE MILFONT MATR�CULA: 378771
//PROFESSORES ORIENTADORES : WILKLEY BEZERRA E DALTON HON�RIO

// INCLUDE FILES.
#include <math.h>
#include <stdio.h>
#include "F28x_Project.h"
#include "params.h"
#include "datastruct.h"

#define DPI 6.28318530717958647692

msg_data *data1 = (void *)(uint32_t)0x03FC00;
//VARI�VEIS GLOBAIS.

float t = 0;//Vari�vel para medi��o de tempo.
int saida = 0; //Vari�vel para controle do loop principal.
float b,c,d = 0; // Valores de moduladora para PWM em malha aberta.
float theta_atual_ma = 0; //�ngulo para aplica��o das transformadas de eixo em malha aberta.
float theta_ant_ma = 0; // Armazena valor anterior do �ngulo para aplica��o das transformadsa de eixo em malha aberta.
float theta_ant = 0; // Armazena valor anterior do �ngulo para aplica��o das transformadsa de eixo em malha fechada.
float theta_atual = 0; // Armazena valor anterior do �ngulo para aplica��o das transformadsa de eixo em malha feechada.

//Vari�veis de aux�lio para aplica��o das transformadas de eixo.
unsigned int pin12 = 0,cont_zero = 0; //Vari�veis de controle gerias.
int Ia_med, Ib_med, Ic_med, Ia_med0, Ib_med0, Ic_med0, I_d_AD, I_q_AD; //Tratamento das vari�veis medidas mo conversor AD.
float  ia, ib, ic;
float va, vb, vc;

//Correntes de refer�ncia nos eixos d/q.
float I_d =0;
float I_q =0;
float erro_cd =0; //Erro para o controlador (eixo d).
float erro_cq =0; //Erro para o controlador (eixo q).
float erro_cd_ant1 =0; //Erro para o controlador uma amostra anterior (eixo d).
float erro_cq_ant1 =0; //Erro para o controladoruyma amostra anterior (eixo q).
float v_atual_d =0; //Sa�da de tens�o atual do controlador (eixo d).
float v_atual_q =0; //Sa�da de tens�o atual do controlador (eixo q).
float v_d_ant =0; //Sa�da de tens�o  do controlador uma amostra anteriorr (eixo d).
float v_q_ant =0; //Sa�da de tens�o  do controlador uma amostra anterior (eixo q).
float I_atual_d=0; //Sa�da de corrente da planta controlada (eixo d).
float I_atual_q=0; //Sa�da de corrente da planta controlada (eixo q).
float I_d_ant =0; //Sa�da de corrente  do controlador uma amostra anteriorr (eixo d).
float I_q_ant =0; //Sa�da de corrente  do controlador uma amostra anterior (eixo q).


int Va_med = 0, Vb_med = 0, Vc_med = 0;

float Va =0; //tens�es de sa�da do modulador.
float Vb =0;
float Vc =0;
float theta_rad; // �ngulo calculado no integrador.
float ref_kd =0; //Valores de refer�ncia para os eixos d e q.
float ref_kq =0;
unsigned int ref_kd_AD =0; //Valores de refer�ncia para os eixos d e q em valores digitais.
unsigned int ref_kq_AD =0;
float ref_kq_ant =0;//Valores passados para a refer�ncia de corrente no eixo q.

//Vari�veis gerais.
float T = 0; //Constante de tempo do rotor.
float w=0;   //Velocidade mec�nica do rotor em rad/s.
float w_eletrica=0;  // Velocidade angular el�trica medida em rad/s.
unsigned int Posicao_ADC = 0; //Leitura da velocidade no m�dulo eqep.
float Rotor_Posicao = 0; //Posi��o angular do rotor.
float Rotor_Posicao_Ant = 0; //Mem�ria da posi��o angular do rotor.
float delta_posicao =0; //Varia��o da posi��o angular do rotor.
float delta_posicao_1 =0; //Varia��o da posi��o angular do rotor (vari�vel auxiliar).
float Velo = 0; //Medida de velocidade mec�nica do rotor em RPM.
float Velo_avg =0; //Leitura filtrada de velocidade.
unsigned long Velo_ADC =0 ; //Medida digital da velocidade.
float Velo_aux = 0; //Vari�vel auxiliar para controle de velocidade.
float wsl=0;  //Velocidade de escorregamento do rotor.
float w_tot =0; //Velocidade el�trica do rotor em rad/s.
float w_avg =0; //Velocidade angular depois do filtro.
float ref_Velo = 800; //Refer�ncia de velocidade para o controle do motor.
unsigned int ref_Velo_AD =0; //Refer�ncia digital de velocidade.
unsigned int ref = 5;   //Escolha da refer�ncia de velocidade.
float ref_Velo1 =0; //Refer�ncia de velocidade para o controle do motor.
float cont_velo =0; //Contador para a malha de velocidade.
float cont_velo_aux =0; //Contador para a refer�ncia de velocidade.
float erro_Velo =0; // Erro para a amlha de velocidade.
float erro_Velo_ant1 =0; //Valores passados para o erro da malha de velocidade.
float Velo_ant1 =0;//Valores passados para a velociddae do motor.

float wa = 0;

float V_a = 0, V_b = 0, V_c = 0;
float va_obs = 0, vb_obs = 0, vc_obs = 0, ia_obs = 0, ib_obs = 0, ic_obs = 0, refobs = 0, iq = 0.5;
int n_degrau = 0;
// FUNCTIONS
void InitEPwmS(void);
void LigaEPWMs(void);
void SetupTimers(void);
void ConfigureDAC(void);
void SetupADC(void);
void zero_sensores(void);
void SetupEQEP1(void);
__interrupt void adca1_isr(void);
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void timer0_isr(void);
__interrupt void eqep1_isr(void);
float ref_OBS(float, float, float, float, float, float);
// MAIN
void main(void){
    //INICIALIZA��O DO CONTROLE DO SISTEMA.
    EALLOW;
    DevCfgRegs.CPUSEL5.bit.SCI_A = 1; // Handoff do SCIA para a CPU02
    EDIS;

    InitSysCtrl();

    //CONFIGURA��O DOS PINOS DE ENTRADA E SA�DA.
    EALLOW;

    //CONFIGURA��O DOS PINOS DO PWM E CONTROLE DAS 6 CHAVES DO INVERSOR.
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0; //S1.
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0; //S2
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0; //S3
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0; //S4
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0; //S5
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0; //S6
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;

    //CHAVE CONTROLADOR.
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;


    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO34  = 0;
    GpioCtrlRegs.GPBCSEL1.bit.GPIO34 = 0b10;

    //MUDA_REFER�NCIA.
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;

    //EQEP.
    InitEQep1Gpio();

    GPIO_SetupPinMux(43, GPIO_MUX_CPU2, 0xF);
    GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(42, GPIO_MUX_CPU2, 0xF);
    GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC);

    EDIS;

    // HABILITANDO  PWM1, PWM2, PWM3
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM3=1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;
    //CpuSysRegs.PCLKCR4.bit.EQEP1 = 1;

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    DINT; //step1
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    EALLOW;

    // AS INTERRUP��ES QUE S�O USADAS S�O REMAPEADAS.
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;

    PieVectTable.TIMER0_INT = &timer0_isr;

    PieVectTable.EQEP1_INT = &eqep1_isr;

    PieVectTable.EPWM1_INT = &epwm1_isr;
    PieVectTable.EPWM2_INT = &epwm2_isr;
    PieVectTable.EPWM3_INT = &epwm3_isr;
    EDIS;

    // Step 4. Initialize the Device Peripherals:
    //

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;
    EDIS;
    ConfigureDAC();
    SetupEQEP1();
    InitEPwmS();
    SetupTimers();
    SetupADC();
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;
    EDIS;

    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    IER |= M_INT1 | M_INT3 | M_INT5;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER5.bit.INTx1 = 1;
    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    LigaEPWMs();

    while(1){
        //PINO DE CONTROLE PARA O CONTROLADOR.
        pin12 = GpioDataRegs.GPADAT.bit.GPIO14;

        //MODULADORA SENOIDAL PARA PWM EM MALHA ABERTA.
        b=4000 + 3200*__sin(theta_atual_ma);
        c=4000 + 3200*__sin(theta_atual_ma  -  ((DPI)/3));
        d=4000 + 3200*__sin(theta_atual_ma  -  ((2*DPI)/3));

        //LOOP.
        for(;;) if(saida==1) break;
        saida = 0;
    }
}

// INICIALIZA��O DOS PWMS.
void InitEPwmS(){
    //ePWM 1
    EPwm1Regs.TBPRD = 8000;                       // Set timer period 6kHz
    EPwm1Regs.TBPHS.bit.TBPHS = 0;                // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm1Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN; // Count up/down
    EPwm1Regs.TBCTL.bit.PHSEN     = TB_DISABLE; // Disable phase loading
    EPwm1Regs.TBCTL.bit.PHSDIR    = 1; // Dire��o Phase 1 Positivo - 0 Negativo
    EPwm1Regs.TBCTL.bit.SYNCOSEL  = TB_CTR_ZERO; // EPWMxSYNCI / SWFSYNC
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT: dividido por 1
    EPwm1Regs.TBCTL.bit.CLKDIV    = TB_DIV1; // Clock ratio to SYSCLKOUT: dividido por 1

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;   // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CAU = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CAD = AQ_CLEAR;

    // Active Low PWMs - Setup Deadband
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.IN_MODE  = DBA_ALL;
    EPwm1Regs.DBCTL.bit.OUTSWAP  = 0x3;
    EPwm1Regs.DBRED.bit.DBRED    = 20;
    EPwm1Regs.DBFED.bit.DBFED    = 20;

    //ePWM 2
    EPwm2Regs.TBPRD = 8000;                       // Set timer period 6kHz
    EPwm2Regs.TBPHS.bit.TBPHS = 0;                // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    //Setup TBCLK
    EPwm2Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN; // Count up/down
    EPwm2Regs.TBCTL.bit.PHSEN     = TB_DISABLE; // Disable phase loading
    EPwm2Regs.TBCTL.bit.PHSDIR    = 1; // Dire��o Phase 1 Positivo - 0 Negativo
    EPwm2Regs.TBCTL.bit.SYNCOSEL  = TB_CTR_ZERO; // EPWMxSYNCI / SWFSYNC
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT: dividido por 1
    EPwm2Regs.TBCTL.bit.CLKDIV    = TB_DIV1; // Clock ratio to SYSCLKOUT: dividido por 1

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;   // Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Set actions
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm2Regs.AQCTLB.bit.CAU = AQ_SET;
    EPwm2Regs.AQCTLB.bit.CAD = AQ_CLEAR;

    //Active Low PWMs - Setup Deadband
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBCTL.bit.OUTSWAP  = 0x3;
    EPwm2Regs.DBRED.bit.DBRED = 20;
    EPwm2Regs.DBFED.bit.DBFED = 20;

    //ePWM 3
    EPwm3Regs.TBPRD = 8000;                       // Set timer period 6kHz
    EPwm3Regs.TBPHS.bit.TBPHS = 0;                // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                     // Clear counter

    //Setup TBCLK
    EPwm3Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN; // Count up/down
    EPwm3Regs.TBCTL.bit.PHSEN     = TB_DISABLE; // Disable phase loading
    EPwm3Regs.TBCTL.bit.PHSDIR    = 1; // Dire��o Phase 1 Positivo - 0 Negativo
    EPwm3Regs.TBCTL.bit.SYNCOSEL  = TB_CTR_ZERO; // EPWMxSYNCI / SWFSYNC
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT: dividido por 1
    EPwm3Regs.TBCTL.bit.CLKDIV    = TB_DIV1; // Clock ratio to SYSCLKOUT: dividido por 1

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;   // Load registers every ZERO
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Set actions
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;
    EPwm3Regs.AQCTLB.bit.CAD = AQ_CLEAR;

    //Active Low PWMs - Setup Deadband
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBCTL.bit.OUTSWAP  = 0x3;
    EPwm3Regs.DBRED.bit.DBRED = 20;
    EPwm3Regs.DBFED.bit.DBFED = 20;

    //Setup EPwm interrupts
    EPwm1Regs.ETSEL.bit.INTSEL = 0b100;
    EPwm1Regs.ETSEL.bit.INTSELCMP = 0;
    EPwm1Regs.ETPS.bit.INTPSSEL = 0;
    EPwm1Regs.ETPS.bit.INTPRD = 1;

    EPwm2Regs.ETSEL.bit.INTSEL = 0b100;
    EPwm2Regs.ETSEL.bit.INTSELCMP = 0;
    EPwm2Regs.ETPS.bit.INTPSSEL = 0;
    EPwm2Regs.ETPS.bit.INTPRD = 1;

    EPwm3Regs.ETSEL.bit.INTSEL = 0b100;
    EPwm3Regs.ETSEL.bit.INTSELCMP = 0;
    EPwm3Regs.ETPS.bit.INTPSSEL = 0;
    EPwm3Regs.ETPS.bit.INTPRD = 1;
}

const int vpeak = 250;

int epwm1_high = 0;
__interrupt void epwm1_isr(){
    if(epwm1_high == 0){
        vc_obs = vpeak;
        epwm1_high = 1;
        EPwm1Regs.ETSEL.bit.INTSEL = 0b101;
    }else{
        vc_obs = 0;
        epwm1_high = 0;
        EPwm1Regs.ETSEL.bit.INTSEL = 0b100;
    }
    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}

int epwm2_high = 0;
__interrupt void epwm2_isr(){
    if(epwm2_high == 0){
        vb_obs = vpeak;
        epwm2_high = 1;
        EPwm2Regs.ETSEL.bit.INTSEL = 0b101;
    }else{
        vb_obs = 0;
        epwm2_high = 0;
        EPwm2Regs.ETSEL.bit.INTSEL = 0b100;
    }
    EPwm2Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}

int epwm3_high = 0;
__interrupt void epwm3_isr(){
    if(epwm3_high == 0){
        va_obs = vpeak;
        epwm3_high = 1;
        EPwm3Regs.ETSEL.bit.INTSEL = 0b101;
    }else{
        va_obs = 0;
        epwm3_high = 0;
        EPwm3Regs.ETSEL.bit.INTSEL = 0b100;
    }
    EPwm3Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}


//  LIGA PWMS ------------------------------------------------------------------
void LigaEPWMs(){
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0     = 0;      // Enable Pullup    // PWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO0    = 1;      // GPIO0 = PWM
    GpioCtrlRegs.GPADIR.bit.GPIO0     = 1;      // GPIO0 = output
    GpioCtrlRegs.GPAPUD.bit.GPIO1     = 0;      // Enable Pullup    // PWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO1    = 1;      // GPIO1 = PWM
    GpioCtrlRegs.GPADIR.bit.GPIO1     = 1;      // GPIO1 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO2     = 0;      // Enable Pullup    // PWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO2    = 1;      // GPIO2 = PWM
    GpioCtrlRegs.GPADIR.bit.GPIO2     = 1;      // GPIO2 = output
    GpioCtrlRegs.GPAPUD.bit.GPIO3     = 0;      // Enable Pullup    // PWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO3    = 1;      // GPIO3 = PWM
    GpioCtrlRegs.GPADIR.bit.GPIO3     = 1;      // GPIO3 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO4     = 0;      // Enable Pullup    // PWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO4    = 1;      // GPIO4 = PWM
    GpioCtrlRegs.GPADIR.bit.GPIO4     = 1;      // GPIO4 = output
    GpioCtrlRegs.GPAPUD.bit.GPIO5     = 0;      // Enable Pullup    // PWM3B
    GpioCtrlRegs.GPAMUX1.bit.GPIO5    = 1;      // GPIO5 = PWM
    GpioCtrlRegs.GPADIR.bit.GPIO5     = 1;      // GPIO5 = output

    EPwm1Regs.ETSEL.bit.INTEN = 1;
    EPwm2Regs.ETSEL.bit.INTEN = 1;
    EPwm3Regs.ETSEL.bit.INTEN = 1;
    EDIS;
}

//CONFIGURA��O DO TIMER 1.
void SetupTimers(void){
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer1, 200, Ts*1e6); //200MHz, 80us
    CpuTimer1Regs.TCR.all = 0x4000;

    ConfigCpuTimer(&CpuTimer0, 200, 160); //200MHz, 80us
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer0Regs.TCR.bit.TIE = 1;
    CpuTimer0Regs.TCR.bit.TIF = 1;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}

//CONFIGURA��O DO ADC.
void SetupADC(void){
    EALLOW;

    //write configurations
    AdcaRegs.ADCCTL2.bit.PRESCALE = 7; //set ADCCLK divider to /4.5
    AdcbRegs.ADCCTL2.bit.PRESCALE = 7; //set ADCCLK divider to /4.5
    AdccRegs.ADCCTL2.bit.PRESCALE = 7; //set ADCCLK divider to /4.5
    AdcdRegs.ADCCTL2.bit.PRESCALE = 7; //set ADCCLK divider to /4.5

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;
    AdcaRegs.ADCOFFTRIM.bit.OFFTRIM = 0;

    AdcbRegs.ADCCTL2.bit.RESOLUTION = 0;
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;
    AdcbRegs.ADCOFFTRIM.bit.OFFTRIM = 0;

    AdccRegs.ADCCTL2.bit.RESOLUTION = 0;
    AdccRegs.ADCCTL2.bit.SIGNALMODE = 0;
    AdccRegs.ADCOFFTRIM.bit.OFFTRIM = 0;

    AdcdRegs.ADCCTL2.bit.RESOLUTION = 0;
    AdcdRegs.ADCCTL2.bit.SIGNALMODE = 0;
    AdcdRegs.ADCOFFTRIM.bit.OFFTRIM = 0;

    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    EDIS;

    DELAY_US(2000);

    EALLOW;

    //Configura��o ADC-A
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3;      //SOC0 will convert pin A3
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 28;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 2;    //trigger on Timer1 SOCA/C

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 5;      //SOC1 will convert pin A4
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 28;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 2;    //trigger on Timer1 SOCA/C

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //make sure INT1 flag is cleared

    //Configura��o ADC-B
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 3;      //SOC1 will convert pin B3
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 28;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 2;    //trigger on TIMER1 SOCA/C

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 5;      //SOC1 will convert pin 4
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 28;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 2;    //trigger on Timer1 SOCA/C

    //Configura��o ADC-C
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 3;  //SOC2 will convert pin C3
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 28;
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 2;    //trigger on Timer1 SOCA

    AdccRegs.ADCSOC1CTL.bit.CHSEL = 4;  //SOC2 will convert pin C3
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 28;
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 2;    //trigger on Timer1 SOCA

    //Configura��o ADC-D
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 14;  //SOC3 will convert pin 14
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 28;
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 2;    //trigger on Timer1 SOCA
//
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 15;  //SOC3 will convert pin 14
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 28;
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 2;    //trigger on Timer1 SOCA

    EDIS;
}

//CONFIGURA��O DO DAC.

void ConfigureDAC(){
    EALLOW;

    //DAC-B
    DacbRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
    DacbRegs.DACCTL.bit.LOADMODE = 0;           // Load on next SYSCLK
    DacbRegs.DACVALS.all = 0x0800;              // Set mid-range
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC

    //DAC -A
    DacaRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
    DacaRegs.DACCTL.bit.LOADMODE = 0;           // Load on next SYSCLK
    DacaRegs.DACVALS.all = 0x0800;              // Set mid-range
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC

    EDIS;
}

unsigned long i_amostra = 0;
float pos;
int cont_controle = 0;
unsigned int tc = 0;

__interrupt void timer0_isr(){
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    theta_atual_ma = ((0.000160)*DPI*20) + theta_ant_ma; // Integrador discreto.
        theta_ant_ma = theta_atual_ma;

        if(theta_atual_ma > DPI){
            theta_atual_ma = theta_atual_ma -DPI;
            theta_ant_ma = theta_atual_ma;// Controle para manter Theta
        }
        if(theta_atual_ma < 0){
            theta_atual_ma = theta_atual_ma +DPI;
            theta_ant_ma = theta_atual_ma;// Controle para manter Theta
        }

        //LEITURA EQEP -POSI��O.
        Posicao_ADC  = EQep1Regs.QPOSCNT;

        //wa = (32.0*60.0/2048.0)/(EQep1Regs.QCPRD*6.4e-7);
        if(EQep1Regs.QEPSTS.bit.COEF == 0 && EQep1Regs.QEPSTS.bit.CDEF == 0){
            wa = (32.0*60.0/2048.0)/(EQep1Regs.QCPRD*128.0/200.0e6);
        }else{
            EQep1Regs.QEPSTS.bit.COEF = 0;
            EQep1Regs.QEPSTS.bit.CDEF = 0;
        }


        //POSI�AO ANGULAR.
        Rotor_Posicao = ((float)Posicao_ADC)*DPI/2048.0;
        delta_posicao = Rotor_Posicao - Rotor_Posicao_Ant;

        //ROTOR PARADO ??
        if(delta_posicao == 0||Velo == 0){
            Velo_aux = Velo;
        }

        //VELOCIDADE NEGATIVA??
        if (delta_posicao <0 || Velo<0){
            delta_posicao = delta_posicao_1;
        }

        //VELOCIDADE POSITIVA.
        if (delta_posicao>0 || Velo>0){
            //DERIVADA DISCRETA DA POSI��O = VELOCIDADE.
            w = (float)((delta_posicao)/(0.000160));
            Velo = (float)(w*(60/(DPI)));
            Velo = wa;
            // Filtro passa baixa para f32f
            Velo_avg = Velo_avg + 0.000160*100.0*(Velo - Velo_avg);
            w_avg = (Velo_avg*DPI)/(60);

            //Velocidade medida em valores digitais.
            Velo_ADC = (int)(2.048*Velo_avg);
        }
        Rotor_Posicao_Ant = Rotor_Posicao;
        Velo_ant1 = Velo_avg;

    //IN�CIO DA MALHA DE CONTROLE.
    if (pin12==1){
        //CAMPO ORIENTADO INDIRETO.
        float v_controle = Velo_avg;
        T = ((Llr+Lm)/Rr);
//        T = (Rr/(Llr+Lm));
        wsl = (ref_kq)/(T*ref_kd);
//        w_tot = wsl + 2*w_avg;
        w_tot = wsl + v_controle*DPI/60.0;

        theta_atual = ((160E-006)*w_tot) + theta_ant; // Integrador discreto.
        theta_ant = theta_atual;

        if(theta_atual > DPI){
            theta_atual = theta_atual -DPI;
            theta_ant = theta_atual;// Controle para manter Theta
        }
        if(theta_atual < 0){
            theta_atual = theta_atual +DPI;
            theta_ant = theta_atual;// Controle para manter Theta
        }

        // APLICA��O DAS TRANSFORMADAS DE EIXO DE CLARKE E PARK.
        // 2 - park.
        theta_rad = theta_atual;

        if(theta_rad > DPI){
            theta_rad = theta_rad - DPI;
        }
        if(theta_rad < -DPI){
            theta_rad = theta_rad + DPI;
        }

        I_d = 0.81649658092772603273242802490196*(ic*__cos(theta_rad) + ib*__cos(theta_rad - ((DPI)/3)) + ia*__cos(theta_rad -((2*DPI)/3)));
        I_q = 0.81649658092772603273242802490196*(-ic*__sin(theta_rad) - ib*__sin(theta_rad - ((DPI)/3)) - ia*__sin(theta_rad -((2*DPI)/3)));

        I_atual_d = I_d;
        I_atual_q = I_q;

        data1->id = I_d;
        data1->iq = I_q;

        //Correntes d e q em valores digitais.

        I_d_AD = (int)(1024*I_d) + 2048;
        I_q_AD = (int)(1024*I_q) + 2048;


        //MALHA DE VELOCIDADE.
        cont_velo ++;
        if(cont_velo==1225){
            //Refer�ncia degrau
            if(ref==1){
                ref_Velo = 1000;
                ref_Velo_AD = (int)(2.048*ref_Velo);
            }

            //Refer�ncia triangular
            if (ref==2){
                t = 0.096*cont_velo_aux;
                if(t<2){
                    ref_Velo = 100*(t) + 600 ;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=2){
                    ref_Velo = -100*(t) + 1000 ;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=4){
                    cont_velo_aux = 0;
                }
            }

            //Refer�ncia trapezoidal
            if (ref==3){
                t = 0.096*cont_velo_aux;
                if(t<1){
                    ref_Velo = 100*(t) + 600 ;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=1 && t<2){
                    ref_Velo = 800;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=2){
                    ref_Velo = -100*(t) + 1100 ;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=5 && t<6){
                    ref_Velo = 600;
                    ref_Velo_AD = (int)(2.048*ref_Velo);
                    cont_velo_aux ++;
                }
                if(t>=5 && t<6){
                    cont_velo_aux = 0;
                }
            }

            //Refer�ncia senoidal
            if(ref==4){
                t = 0.096*cont_velo_aux;
                ref_Velo = 600 + 200*__sin(DPI*0.25*t);
                cont_velo_aux++;
                if(t>=4){
                    cont_velo_aux = 0;
                }
            }
            if(ref==5){
                t = 0.096*cont_velo_aux;
                if(t >= 5){
                    ref_Velo = 1000;
                }else{
                    ref_Velo = 500;
                }
                cont_velo_aux++;
                if(t>=10){
                    cont_velo_aux = 0;
                }
                ref_Velo_AD = (int)(2.048*ref_Velo);
            }
            cont_velo = 0;

            //PI malha de velocidade
            double kp = 6.6667e-05;
            double ki = 6.6334e-05;

            erro_Velo = ref_Velo - v_controle;
            ref_kq = ref_kq_ant + ki*erro_Velo_ant1 + kp*erro_Velo;
            erro_Velo_ant1 = erro_Velo;
            ref_kq_ant = ref_kq;
        }
        ref_kd = 0.4;
        n_degrau++;
        if(n_degrau >= 6250*3){
            n_degrau = 0;
            if(iq == 0.5){
                iq = 0.7;
            }else{
                iq = 0.5;
            }
        }

        //REFER�NCIAS EIXO D E Q:
        ref_kd_AD = (int)(1024*ref_kd) + 2048;
        ref_kq_AD = (int)(1024*ref_kq) + 2048;

        //C�LCULO DOS ERROS
        erro_cd = ref_kd - I_d;
        erro_cq =  ref_kq - I_q;

        float kpi = 103, kii = 100.4;
        v_atual_d = v_d_ant + kpi*erro_cd - kii*erro_cd_ant1;
        v_atual_q = v_q_ant + kpi*erro_cq - kii*erro_cq_ant1;


        //Atualiza��o das vari�veis.
        erro_cd_ant1 = erro_cd;
        erro_cq_ant1 = erro_cq;

        I_q_ant = I_atual_q;
        v_d_ant = v_atual_d;
        v_q_ant = v_atual_q;

        //APLICA��O DAS TRANSFORMADAS INVERSAS DE CLARKE E PARK:
        Va =  0.81649658092772603273242802490196*(v_atual_d*__cos(theta_rad) - v_atual_q*__sin(theta_rad));
        Vb =  0.81649658092772603273242802490196*(v_atual_d*__cos(theta_rad - ((DPI)/3.0)) - v_atual_q*__sin(theta_rad - ((DPI)/3.0)));
        Vc =  0.81649658092772603273242802490196* (v_atual_d*__cos(theta_rad - ((2*DPI)/3.0)) - v_atual_q*__sin(theta_rad - ((2*DPI)/3.0)));

        //GANHO DE MODULA��O:
        Va = Va*16;
        Vb = Vb*16;
        Vc = Vc*16;

        //OFFSET:
        Va = Va + 4000;
        Vb = Vb + 4000;
        Vc = Vc + 4000;

        //SATURADOR:
        if (Va >= 8000){
            Va = 7950;
        }
        if (Va <= 0){
            Va = 50;
        }
        if (Vb >= 8000){
            Vb = 7950;
        }
        if (Vb <= 0 ){
            Vb = 50;
        }
        if (Vc >= 8000){
            Vc = 7950;
        }
        if (Vc <= 0){
            Vc = 50;
        }

        V_a = (Va - 4000)/16.0;
        V_b = (Vb - 4000)/16.0;
        V_c = (Vc - 4000)/16.0;

        //DacaRegs.DACVALS.all = (V_a*1024.0/1000.0) + 1024;
        //DacbRegs.DACVALS.all = (V_b*1024.0/1000.0) + 1024;

        //MODULA��O:
        EPwm1Regs.CMPA.bit.CMPA = Vc; // adjust duty for output EPWM1A
        EPwm2Regs.CMPA.bit.CMPA = Vb; // adjust duty for output EPWM2A
        EPwm3Regs.CMPA.bit.CMPA = Va; // adjust duty for output EPWM3A
        //SEQUENCIA CORRETA DE CIMA PRA BAIXO C,B,A
    }
    if (pin12==0){
        EPwm1Regs.CMPA.bit.CMPA = d; // adjust duty for output EPWM1A
        EPwm2Regs.CMPA.bit.CMPA = c; // adjust duty for output EPWM2A
        EPwm3Regs.CMPA.bit.CMPA = b; // adjust duty for output EPWM3A
    }

    saida = 1;
    i_amostra++;
    data1->j = i_amostra;
}

float ia_filt = 0, ib_filt = 0, ic_filt = 0;
float ga = 680, gb = 560, gc = 500;
__interrupt void adca1_isr(void){
    DINT;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    GpioDataRegs.GPADAT.bit.GPIO15 = 1;
    Ia_med0 = 2000;
    Ib_med0 = 2000;
    Ic_med0 = 2000;
    //LEITURA DO CONVERSOR AD.
    Ib_med = AdccResultRegs.ADCRESULT0;
    Ic_med = AdcbResultRegs.ADCRESULT0;
    Ia_med = AdcaResultRegs.ADCRESULT0;

    Va_med = AdccResultRegs.ADCRESULT1;
    Vb_med = AdcbResultRegs.ADCRESULT1;
    Vc_med = AdcdResultRegs.ADCRESULT1;

    va = Va_med * 250.0/670.0;
    vb = Vb_med * 250.0/670.0;
    vc = Vc_med * 250.0/670.0;

    // Ganho do ADC * Ganho do sensor;
    ic = (Ic_med - Ic_med0)/gc;
    ib = (Ib_med - Ib_med0)/gb;
    ia = (Ia_med - Ia_med0)/ga;

    float kk1 = 9.809831e-01, kk2 = 1.901685e-02;
    ia_filt = kk1*ia_filt + kk2*ia;
    ib_filt = kk1*ib_filt + kk2*ib;
    ic_filt = kk1*ic_filt + kk2*ic;

//    DacaRegs.DACVALS.all = (ib_filt*2048.0/3.0) + 1024;
//    DacbRegs.DACVALS.all = (ib*2048.0/3.0) + 1024;

    ia = ia_filt;
    ib = ib_filt;
    ic = ic_filt;

    refobs = ref_OBS(vc_obs, vb_obs, va_obs, ia, ib, ic);

    DacaRegs.DACVALS.all = Velo_avg * 2.048;
    DacbRegs.DACVALS.all = refobs * 2.048;

    data1->med_vel = Velo_avg;
    data1->obs_vel = refobs;
    data1->ref = ref_Velo;


//
//    DacaRegs.DACVALS.all = (va_obs*2048.0/2.0) + 1024;
//    DacbRegs.DACVALS.all = (vb_obs*2048.0/2.0) + 1024;
    GpioDataRegs.GPADAT.bit.GPIO15 = 0;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    EINT;
}

__interrupt void eqep1_isr(){


//    EQep1Regs.QCLR.bit.UTO = 1;
//    EQep1Regs.QCLR.bit.INT = 1;
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
}

void SetupEQEP1 (){
    EALLOW;
    EQep1Regs.QUPRD = 1;            // Unit Timer for 100Hz at 200 MHz
                                              // SYSCLKOUT
    EQep1Regs.QDECCTL.bit.QSRC = 00;      // QEP quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep1Regs.QEPCTL.bit.PCRM = 1;         // PCRM=01 mode - QPOSCNT reset on maximum position
    EQep1Regs.QEPCTL.bit.UTE = 1;         // Unit Timeout Enable
    EQep1Regs.QEPCTL.bit.QCLM = 1;        // Latch on unit time out
    EQep1Regs.QPOSMAX = 2000;               //0x7ff = 2048 contagens
    EQep1Regs.QDECCTL.bit.SWAP = 1;         // troca o sentido da contagem
    EQep1Regs.QEPCTL.bit.QPEN = 1;        // QEP enable

    EQep1Regs.QCAPCTL.bit.UPPS = 5;       // 1/32 for unit position
    EQep1Regs.QCAPCTL.bit.CCPS = 7;       // 1/128 for CAP clock
    EQep1Regs.QCAPCTL.bit.CEN = 1;        // QEP Capture Enable
    //EQep1Regs.QEINT.bit.UTO = 1; // 400 Hz interrupt for speed estimation
    EDIS;
}
