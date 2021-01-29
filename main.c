//CÓDIGO PARA CONTROLE DE VELOCIDADE DE UM MOTOR DE INDUÇÃO TRIFÁSICO DE 0.25CV.
//TRABALHO DE CONCLUSÃO DE CURSO
//ALUNO: LEONARDO DUARTE MILFONT MATRÍCULA: 378771
//PROFESSORES ORIENTADORES : WILKLEY BEZERRA E DALTON HONÓRIO

// INCLUDE FILES.
#include <math.h>
#include <stdio.h>
#include "F28x_Project.h"

#define DPI 6.28318530717958647692

//DAC:
//volatile struct DAC_REGS* DAC_PTR[4] = {0x0,&DacaRegs,&DacbRegs,&DaccRegs};
//VARIÁVEIS GLOBAIS.

float32 t = 0;//Variável para medição de tempo.
Uint16 saida =0; //Variável para controle do loop principal.
float32 b,c,d=0; // Valores de moduladora para PWM em malha aberta.
float32 theta_atual_ma=0; //Ângulo para aplicação das transformadas de eixo em malha aberta.
float32 theta_ant_ma=0; // Armazena valor anterior do ângulo para aplicação das transformadsa de eixo em malha aberta.
float32 theta_ant=0; // Armazena valor anterior do ângulo para aplicação das transformadsa de eixo em malha fechada.
float32 theta_atual=0; // Armazena valor anterior do ângulo para aplicação das transformadsa de eixo em malha feechada.

//Variáveis de auxílio para aplicação das transformadas de eixo.
float32 A1;
float32 B1;
float32 C1;
float32 A2;
float32 B2;
float32 C2;
float32 A3;
float32 B3;
float32 A4;
float32 B4;
float32 A5;
float32 B5;
Uint16 pin12=0,cont_zero=0; //Variáveis de controle gerias.
Uint16 I_ZERO_A[5] = {0,0,0,0,0}; //Zero sensores correntes Ia, Ib, Ic.
Uint16 I_ZERO_B[5] = {0,0,0,0,0};
Uint16 I_ZERO_C[5] = {0,0,0,0,0};
int16 Ia_med , Ib_med , Ic_med , Ia_med0 , Ib_med0 , Ic_med0 , Ia_off , Ib_off , Ic_off,I_d_AD,I_q_AD; //Tratamento das variáveis medidas mo conversor AD.
float32  Ia_g , Ib_g , Ic_g , I_a , I_b , I_c;

//Correntes de referência nos eixos d/q.
float32 I_d =0;
float32 I_q =0;
float32 erro_cd =0; //Erro para o controlador (eixo d).
float32 erro_cq =0; //Erro para o controlador (eixo q).
float32 erro_cd_ant1 =0; //Erro para o controlador uma amostra anterior (eixo d).
float32 erro_cq_ant1 =0; //Erro para o controladoruyma amostra anterior (eixo q).
float32 v_atual_d =0; //Saída de tensão atual do controlador (eixo d).
float32 v_atual_q =0; //Saída de tensão atual do controlador (eixo q).
float32 v_d_ant =0; //Saída de tensão  do controlador uma amostra anteriorr (eixo d).
float32 v_q_ant =0; //Saída de tensão  do controlador uma amostra anterior (eixo q).
float32 I_atual_d=0; //Saída de corrente da planta controlada (eixo d).
float32 I_atual_q=0; //Saída de corrente da planta controlada (eixo q).
float32 I_d_ant =0; //Saída de corrente  do controlador uma amostra anteriorr (eixo d).
float32 I_q_ant =0; //Saída de corrente  do controlador uma amostra anterior (eixo q).

float32 Va =0; //tensões de saída do modulador.
float32 Vb =0;
float32 Vc =0;
float32 theta_rad; // Ângulo calculado no integrador.
float32 ref_kd =0; //Valores de referência para os eixos d e q.
float32 ref_kq =0;
Uint16 ref_kd_AD =0; //Valores de referência para os eixos d e q em valores digitais.
Uint16 ref_kq_AD =0;
float32 ref_kq_ant =0;//Valores passados para a referência de corrente no eixo q.

//Parâmetros da máquina

float32 Rs = 35.58;
float32 Rr = 87.44;
float32 Lls = 0.16;
float32 Llr = 0.16;
float32 Ls = 1.044; //Lls +Lm
float32 Lr = 1.044; //Llr +Lm
float32 Lm = 0.884;

Uint16 p = 2; // pares de polos

//Parâmetros do Observador
float32 fsw = 6e3;                  // Frequência de chaveamento
float32 Ts = 80e-6;         // tempo de amostragem do observador 1/(fsw*100)
float32 sigma = 0.346442359;        // coeficiente total de dispersão 1 - (Lm^2/(Ls*Lr))
float32 Tr = 0.011939615;           // constante de tempo rotórica (Lr/Rr)
float32 fpb_a = 800;                    // parâmetro para FPB
float32 fpb_b = -792;                   // parâmetro para FPB (-0.99*a)

//Variáveis gerais.
float32 T = 0; //Constante de tempo do rotor.
float32 w=0;   //Velocidade mecânica do rotor em rad/s.
float32 w_eletrica=0;  // Velocidade angular elétrica medida em rad/s.
Uint16 Posicao_ADC = 0; //Leitura da velocidade no módulo eqep.
float32 Rotor_Posicao = 0; //Posição angular do rotor.
float32 Rotor_Posicao_Ant = 0; //Memória da posição angular do rotor.
float32 delta_posicao =0; //Variação da posição angular do rotor.
float32 delta_posicao_1 =0; //Variação da posição angular do rotor (variável auxiliar).
float32 Velo = 0; //Medida de velocidade mecânica do rotor em RPM.
float32 Velo_avg =0; //Leitura filtrada de velocidade.
Uint32 Velo_ADC =0 ; //Medida digital da velocidade.
float32 Velo_aux = 0; //Variável auxiliar para controle de velocidade.
float32 wsl=0;  //Velocidade de escorregamento do rotor.
float32 w_tot =0; //Velocidade elétrica do rotor em rad/s.
float32 w_avg =0; //Velocidade angular depois do filtro.
float32 ref_Velo = 600; //Referência de velocidade para o controle do motor.
Uint16 ref_Velo_AD =0; //Referência digital de velocidade.
Uint16 ref = 1;   //Escolha da referência de velocidade.
float32 ref_Velo1 =0; //Referência de velocidade para o controle do motor.
float32 cont_velo =0; //Contador para a malha de velocidade.
float32 cont_velo_aux =0; //Contador para a referência de velocidade.
float32 erro_Velo =0; // Erro para a amlha de velocidade.
float32 erro_Velo_ant1 =0; //Valores passados para o erro da malha de velocidade.
float32 Velo_ant1 =0;//Valores passados para a velociddae do motor.


// FUNCTIONS
void InitEPwmS(void);
void LigaEPWMs(void);
void SetupTimers(void);
void ConfigureDAC(void);
void SetupADC(void);
void zero_sensores(void);
void SetupEQEP1(void);
__interrupt void adca1_isr(void);
float32 ref_MRAS(float32, float32, float32, float32, float32, float32);
// MAIN
void main(void)

{
    //INICIALIZAÇÃO DO CONTROLE DO SISTEMA.

    InitSysCtrl();

    //CONFIGURAÇÃO DOS PINOS DE ENTRADA E SAÍDA.
    EALLOW;

    //CONFIGURAÇÃO DOS PINOS DO PWM E CONTROLE DAS 6 CHAVES DO INVERSOR.

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
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;

    //MUDA_REFERÊNCIA.

    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;

    //EQEP.
    InitEQep1Gpio();

    EDIS;

    // HABILITANDO  PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8, EPWM9 e EPWM10

    CpuSysRegs.PCLKCR2.bit.EPWM1=1; CpuSysRegs.PCLKCR2.bit.EPWM2=1; CpuSysRegs.PCLKCR2.bit.EPWM3=1; //CpuSysRegs.PCLKCR2.bit.EPWM4=1;
    //CpuSysRegs.PCLKCR2.bit.EPWM5=1; //CpuSysRegs.PCLKCR2.bit.EPWM6=1; //CpuSysRegs.PCLKCR2.bit.EPWM7=1; //CpuSysRegs.PCLKCR2.bit.EPWM8=1;
    //CpuSysRegs.PCLKCR2.bit.EPWM9=1;

    //
    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    //
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.

    DINT; //step1
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block //step2
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    EALLOW;

    // AS INTERRUPÇÕES QUE SÃO USADAS SÃO REMAPEADAS.
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;
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
    IER |= M_INT1;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    LigaEPWMs();

    // Step 6. IDLE loop. Just sit and loop forever (optional):


    while(1)
    {

        //PINO DE CONTROLE PARA O CONTROLADOR.
        pin12 = GpioDataRegs.GPADAT.bit.GPIO14;

        //MODULADORA SENOIDAL PARA PWM EM MALHA ABERTA.
        b=2000 + 1600*__sin(theta_atual_ma);
        c=2000 + 1600*__sin(theta_atual_ma  -  ((DPI)/3));
        d=2000 + 1600*__sin(theta_atual_ma  -  ((2*DPI)/3));

        //LOOP.
        for(;;){
            if(saida==1){
                break;
            }
        }
        saida = 0;
    }
}

// INICIALIZAÇÃO DOS PWMS.
void InitEPwmS(){
    //  ePWM 1 ----------------------------------------------------------------------------------------
    //  -----------------------------------------------------------------------------------------------
    EPwm1Regs.TBPRD = 8000;                       // Set timer period 6kHz
    EPwm1Regs.TBPHS.bit.TBPHS = 0;                // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm1Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN; // Count up/down
    EPwm1Regs.TBCTL.bit.PHSEN     = TB_DISABLE; // Disable phase loading
    EPwm1Regs.TBCTL.bit.PHSDIR    = 1; // Direção Phase 1 Positivo - 0 Negativo
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


    //  ePWM 2 ----------------------------------------------------------------------------------------
    //  -----------------------------------------------------------------------------------------------

    EPwm2Regs.TBPRD = 8000;                       // Set timer period 6kHz
    EPwm2Regs.TBPHS.bit.TBPHS = 0;                // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm2Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN; // Count up/down
    EPwm2Regs.TBCTL.bit.PHSEN     = TB_DISABLE; // Disable phase loading
    EPwm2Regs.TBCTL.bit.PHSDIR    = 1; // Direção Phase 1 Positivo - 0 Negativo
    EPwm2Regs.TBCTL.bit.SYNCOSEL  = TB_CTR_ZERO; // EPWMxSYNCI / SWFSYNC
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT: dividido por 1
    EPwm2Regs.TBCTL.bit.CLKDIV    = TB_DIV1; // Clock ratio to SYSCLKOUT: dividido por 1

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;   // Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm2Regs.AQCTLB.bit.CAU = AQ_SET;
    EPwm2Regs.AQCTLB.bit.CAD = AQ_CLEAR;

    // Active Low PWMs - Setup Deadband
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBCTL.bit.OUTSWAP  = 0x3;
    EPwm2Regs.DBRED.bit.DBRED = 20;
    EPwm2Regs.DBFED.bit.DBFED = 20;

    //  ePWM 3 ----------------------------------------------------------------------------------------
    //  -----------------------------------------------------------------------------------------------


    EPwm3Regs.TBPRD = 8000;                       // Set timer period 6kHz
    EPwm3Regs.TBPHS.bit.TBPHS = 0;                // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm3Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN; // Count up/down
    EPwm3Regs.TBCTL.bit.PHSEN     = TB_DISABLE; // Disable phase loading
    EPwm3Regs.TBCTL.bit.PHSDIR    = 1; // Direção Phase 1 Positivo - 0 Negativo
    EPwm3Regs.TBCTL.bit.SYNCOSEL  = TB_CTR_ZERO; // EPWMxSYNCI / SWFSYNC
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT: dividido por 1
    EPwm3Regs.TBCTL.bit.CLKDIV    = TB_DIV1; // Clock ratio to SYSCLKOUT: dividido por 1


    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;   // Load registers every ZERO
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;
    EPwm3Regs.AQCTLB.bit.CAD = AQ_CLEAR;

    // Active Low PWMs - Setup Deadband
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBCTL.bit.OUTSWAP  = 0x3;
    EPwm3Regs.DBRED.bit.DBRED = 20;
    EPwm3Regs.DBFED.bit.DBFED = 20;
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


    EDIS;
}

//CONFIGURAÇÃO DO TIMER 1.

void SetupTimers(void){
    //
    // Step 4. Initialize the Device Peripheral. This function can be
    //         found in F2837xD_CpuTimers.c
    //
    InitCpuTimers();   // For this example, only initialize the Cpu Timers

    //
    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    //
    //ConfigCpuTimer(&CpuTimer0, 200, 1000000);
    ConfigCpuTimer(&CpuTimer1, 200, 80);
    //ConfigCpuTimer(&CpuTimer2, 200, 1000000);

    //
    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed in
    // ConfigCpuTimer and InitCpuTimers (in F2837xD_cputimervars.h), the below
    // settings must also be updated.
    //
    //CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    //CpuTimer2Regs.TCR.all = 0x4000;

    //
}

//CONFIGURAÇÃO DO ADC.

void SetupADC(void){
    EALLOW;

    //write configurations
    AdcaRegs.ADCCTL2.bit.PRESCALE = 7; //set ADCCLK divider to /4.5
    AdcbRegs.ADCCTL2.bit.PRESCALE = 7; //set ADCCLK divider to /4.5
    AdccRegs.ADCCTL2.bit.PRESCALE = 7; //set ADCCLK divider to /4.5
    AdcdRegs.ADCCTL2.bit.PRESCALE = 7; //set ADCCLK divider to /4.5

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

    //Configuração ADC-A
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3;      //SOC0 will convert pin A3
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 28;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 2;    //trigger on Timer1 SOCA/C

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //make sure INT1 flag is cleared


    //  //Configuração ADC-B
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3;      //SOC1 will convert pin B3
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 28;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 2;    //trigger on TIMER1 SOCA/C



    //  //Configuração ADC-C
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 3;  //SOC2 will convert pin C3
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 28;
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 2;    //trigger on Timer1 SOCA

    ////  //Configuração ADC-D
    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 14;  //SOC3 will convert pin 14
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = 28;
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 2;    //trigger on Timer1 SOCA

    EDIS;
}

//CONFIGURAÇÃO DO DAC.

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

float32 va_obs = 0, vb_obs = 0, vc_obs = 0, ia_obs = 0, ib_obs = 0, ic_obs = 0, refmras = 0;

float32 Va_sg = 0, Vb_sg = 0, Vc_sg = 0;

int cont_controle = 0;

__interrupt void adca1_isr(void){
    //CONDIÇÃO INICIAL PARA O CÁLCULO DOS ZEROS DOS SENSORES.
    if (cont_zero<=5){
        zero_sensores();
    }
    //CÁLCULO DOS ZEROS DOS SENSORES.
    if (cont_zero==5){
        Ia_med0 = (I_ZERO_A[0] + I_ZERO_A[1] + I_ZERO_A[2] + I_ZERO_A[3] + I_ZERO_A[4])/5;
        Ib_med0 = (I_ZERO_B[0] + I_ZERO_B[1] + I_ZERO_B[2] + I_ZERO_B[3] + I_ZERO_B[4])/5;
        Ic_med0 = (I_ZERO_C[0] + I_ZERO_C[1] + I_ZERO_C[2] + I_ZERO_C[3] + I_ZERO_C[4])/5;
        cont_zero = cont_zero+1;
    }

    theta_atual_ma = ((0.000080)*DPI*20) + theta_ant_ma; // Integrador discreto.
    theta_ant_ma = theta_atual_ma;

    if(theta_atual_ma > DPI){
        theta_atual_ma = theta_atual_ma -DPI;
        theta_ant_ma = theta_atual_ma;// Controle para manter Theta
    }
    if(theta_atual_ma < 0){
        theta_atual_ma = theta_atual_ma +DPI;
        theta_ant_ma = theta_atual_ma;// Controle para manter Theta
    }

    //LEITURA EQEP -POSIÇÃO.
    Posicao_ADC  = (int)(EQep1Regs.QPOSCNT);

    //POSIÇAO ANGULAR.
    Rotor_Posicao = (float)(Posicao_ADC*DPI)/(2048);
    delta_posicao = Rotor_Posicao - Rotor_Posicao_Ant;

    //ROTOR PARADO ??
    if(delta_posicao ==0||Velo == 0){
        Velo_aux = Velo;
    }

    //VELOCIDADE NEGATIVA??
    if (delta_posicao <0 || Velo<0){
        delta_posicao = delta_posicao_1;
    }

    //VELOCIDADE POSITIVA.
    if (delta_posicao>0 || Velo>0){
        //DERIVADA DISCRETA DA POSIÇÃO = VELOCIDADE.
        w = (float)((delta_posicao)/(0.000080));
        Velo = (float)(w*(60/(DPI)));

        // Filtro passa baixa para f32f
        Velo_avg = Velo_avg + 0.000080*(Velo - Velo_avg);
        w_avg = (Velo_avg*DPI)/(60);

        //Velocidade medida em valores digitais.
        Velo_ADC = (int)(2.048*Velo_avg);
    }

    Rotor_Posicao_Ant = Rotor_Posicao;

    Velo_ant1 = Velo_avg;

    refmras = ref_MRAS(va_obs, vb_obs, vc_obs, ia_obs, ib_obs, ic_obs);
    int ref_dac = (int)(refmras*2048);
    //DacaRegs.DACVALS.all = ref_dac;

    DacaRegs.DACVALS.all = ref_Velo_AD;
    DacbRegs.DACVALS.all = Velo_ADC;

    //DacaRegs.DACVALS.all = ic_obs*1024/2 + 1024;
    //DacbRegs.DACVALS.all = ib_obs*1024/2 + 1024;

    //INÍCIO DA MALHA DE CONTROLE.
    if (cont_zero>5){
        if (pin12==1){
            //LEITURA DO CONVERSOR AD.
            Ic_med = AdcdResultRegs.ADCRESULT3;
            Ib_med = AdcaResultRegs.ADCRESULT0;
            Ia_med = AdccResultRegs.ADCRESULT2;

            //CORRECAO DOS VALORES;
            Ic_off = Ic_med - Ic_med0;//Offset
            Ib_off = Ib_med - Ib_med0;//Offset
            Ia_off = Ia_med - Ia_med0;//Offset

            //VALORES REAIS - TENSÃO;

            Ic_g = (float)(Ic_off*0.0008056640625); //ganho do ad 3.3/4096
            Ib_g = (float)(Ib_off*0.0008056640625); //ganho do ad 3.3/4095
            Ia_g = (float)(Ia_off*0.0008056640625); //ganho do ad 3.3/4096

            //VALORES REAIS - CORRENTE

            I_c = (float)(Ic_g*1.33);
            I_b = (float)(Ib_g*1.33);
            I_a = (float)(Ia_g*1.33);

            //valores de i para o observador
            ia_obs = I_a;
            ib_obs = I_b;
            ic_obs = I_c;

            if(cont_controle == 1){
                //CAMPO ORIENTADO INDIRETO.
                T = ((Llr+Lm)/Rr);
                wsl = (ref_kq)/(ref_kd*T);
                w_tot = wsl + w_avg;

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


                // APLICAÇÃO DAS TRANSFORMADAS DE EIXO DE CLARKE E PARK.

                // 2 - park.
                theta_rad = theta_atual;

                if(theta_rad > DPI){
                    theta_rad = theta_rad - DPI;
                }
                if(theta_rad < -DPI){
                    theta_rad = theta_rad + DPI;
                }

                A1=I_a*__cos(theta_rad);
                B1 = I_b*__cos(theta_rad - ((DPI)/3));
                C1 =I_c*__cos(theta_rad -((2*DPI)/3));
                A2=-I_a*__sin(theta_rad);
                B2 =- I_b*__sin(theta_rad - ((DPI)/3));
                C2 =-I_c*__sin(theta_rad -((2*DPI)/3));

                I_d = 0.81649658092772603273242802490196*(A1 + B1 + C1);
                I_q = 0.81649658092772603273242802490196*(A2 + B2 + C2);

                I_atual_d = I_d;
                I_atual_q = I_q;

                //Correntes d e q em valores digitais.

                I_d_AD = (int)(1024*I_d) + 2048;
                I_q_AD = (int)(1024*I_q) + 2048;


                //MALHA DE VELOCIDADE.

                cont_velo ++;

                //ref_Velo =600;

                //Referência de  velocidade em valores digitais.

                //ref_velo_AD


                if(cont_velo==1225){
                    //Referência degrau
                    if(ref==1){
                        ref_Velo_AD = (int)(2.048*ref_Velo);
                    }

                    //Referência triangular

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

                    //Referência trapezoidal

                    if (ref==3){
                        t = 0.096*cont_velo_aux;
                        if(t<2){
                            ref_Velo = 100*(t) + 600 ;
                            ref_Velo_AD = (int)(2.048*ref_Velo);
                            cont_velo_aux ++;
                        }

                        if(t>=2 && t<3){
                            ref_Velo = 800;
                            ref_Velo_AD = (int)(2.048*ref_Velo);
                            cont_velo_aux ++;
                        }

                        if(t>=3){
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

                    //Referência senoidal

                    if(ref==4){
                        t = 0.096*cont_velo_aux;
                        ref_Velo = 600 + 200*sin(DPI*0.25*t);
                        cont_velo_aux++;
                        if(t==4){
                            cont_velo_aux = 0;
                        }
                    }
                    cont_velo = 0;

                    //PI malha de velocidade
                    double ki = -0.0011;
                    double kp = 0.0011491;
                    erro_Velo = ref_Velo - Velo_avg;
                    ref_kq = ref_kq_ant + ki*erro_Velo_ant1 + kp*erro_Velo;
                    erro_Velo_ant1 = erro_Velo;
                    ref_kq_ant = ref_kq;

                    /*
                    * GPC malha de velocidade
                    //ref_Velo1 = ref_Velo*(9.317070813126087e-05);
                    ref_Velo1 = ref_Velo*(130.978479409080e-006);

                    //erro_Velo = ref_Velo1 - ((0.115347981981859*erro_Velo_ant1)  +  ((6.599753085671523e-04)*Velo_avg -(5.668046004358913e-04)*Velo_ant1));
                    erro_Velo = ref_Velo1 - (((152.705934971598e-003)*erro_Velo_ant1)  +  ((881.355078709949e-006)*Velo_avg -(750.376599300869e-006)*Velo_ant1));

                    ref_kq = erro_Velo + ref_kq_ant;

                    erro_Velo_ant1 = erro_Velo;

                    ref_kq_ant = ref_kq;
                    */
                }



                //REFERÊNCIAS EIXO D E Q:

                ref_kd=0.4;
                //ref_kq = 0.7;

                ref_kd_AD = (int)(1024*ref_kd) + 2048;
                ref_kq_AD = (int)(1024*ref_kq) + 2048;

                /*

                if (ref_kq>0.5)
                {
                GpioDataRegs.GPASET.bit.GPIO15 = 1;
                }

                if (ref_kq<=0.5)
                {
                GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;
                }

                */

                //CÁLCULO DOS ERROS

                erro_cd = ref_kd - I_d;
                erro_cq =  ref_kq - I_q;


                //LEI DE CONTROLE:

                //Controladores eixo d e q.

                v_atual_d =103*erro_cd -100.4 * erro_cd_ant1  + v_d_ant ;
                v_atual_q =103* erro_cq -100.4* erro_cq_ant1 + v_q_ant ;

                //Atualização das variáveis.

                erro_cd_ant1 = erro_cd;
                erro_cq_ant1 = erro_cq;

                I_q_ant = I_atual_q;
                v_d_ant = v_atual_d;
                v_q_ant = v_atual_q;

                //APLICAÇÃO DAS TRANSFORMADAS INVERSAS DE CLARKE E PARK:

                A3 = v_atual_d*__cos(theta_rad);
                B3 = - v_atual_q*__sin(theta_rad);
                A4 = v_atual_d*__cos(theta_rad - ((DPI)/3.0));
                B4 =- v_atual_q*__sin(theta_rad - ((DPI)/3.0));
                A5 = v_atual_d*__cos(theta_rad - ((2*DPI)/3.0));
                B5 =- v_atual_q*__sin(theta_rad - ((2*DPI)/3.0));
                Va =  0.81649658092772603273242802490196*(A3 + B3);
                Vb =  0.81649658092772603273242802490196*(A4 + B4);
                Vc =  0.81649658092772603273242802490196* (A5 + B5);

                //GANHO DE MODULAÇÃO:
                Va_sg = Va;
                Vb_sg = Vb;
                Vc_sg = Vc;
                Va = Va*8;
                Vb = Vb*8;
                Vc = Vc*8;

                va_obs = Va*250/4096;
                vb_obs = Vb*250/4096;
                vc_obs = Va*250/4096;

                //OFFSET:
                Va = Va + 2000;
                Vb = Vb + 2000;
                Vc = Vc + 2000;

                //SATURADOR:
                if (Va >= 4000){
                    Va = 3950;
                }

                if (Va <= 0){
                    Va = 50;
                }

                if (Vb>=4000){
                    Vb = 3950;
                }

                if (Vb<=0 ){
                    Vb = 50;
                }

                if (Vc>=4000){
                    Vc = 3950;
                }

                if (Vc<=0 ){
                    Vc = 50;
                }

                //MODULAÇÃO:
                EPwm1Regs.CMPA.bit.CMPA = Vc; // adjust duty for output EPWM1A
                EPwm2Regs.CMPA.bit.CMPA = Vb; // adjust duty for output EPWM2A
                EPwm3Regs.CMPA.bit.CMPA = Va; // adjust duty for output EPWM3A
                //SEQUENCIA CORRETA DE CIMA PRA BAIXO C,B,A
            }
        }
        if (pin12==0){
            if(cont_controle == 1){
                // Run Time (Note: Example execution of one run-time instant)
                //=========================================================
                EPwm1Regs.CMPA.bit.CMPA = d; // adjust duty for output EPWM1A
                EPwm2Regs.CMPA.bit.CMPA = c; // adjust duty for output EPWM2A
                EPwm3Regs.CMPA.bit.CMPA = b; // adjust duty for output EPWM3A
            }
        }
        cont_controle++;
        cont_controle %= 2;
        saida = 1;
    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void zero_sensores(){
    I_ZERO_C[cont_zero] = AdcdResultRegs.ADCRESULT3 ;
    I_ZERO_B[cont_zero] = AdcaResultRegs.ADCRESULT0 ;
    I_ZERO_A[cont_zero] = AdccResultRegs.ADCRESULT2 ;
    cont_zero = cont_zero + 1;
}

void SetupEQEP1(){
    EALLOW;
    EQep1Regs.QUPRD = 2000000;            // Unit Timer for 100Hz at 200 MHz
                  // SYSCLKOUT
    EQep1Regs.QDECCTL.bit.QSRC = 00;      // QEP quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep1Regs.QEPCTL.bit.PCRM = 01;         // PCRM=01 mode - QPOSCNT reset on maximum position
    EQep1Regs.QEPCTL.bit.UTE = 1;         // Unit Timeout Enable
    EQep1Regs.QEPCTL.bit.QCLM = 1;        // Latch on unit time out
    EQep1Regs.QPOSMAX = 0x7ff;               //0x7ff = 2048 contagens
    EQep1Regs.QDECCTL.bit.SWAP = 1;         // troca o sentido da contagem
    EQep1Regs.QEPCTL.bit.QPEN = 1;        // QEP enable
    EQep1Regs.QCAPCTL.bit.UPPS = 5;       // 1/32 for unit position
    EQep1Regs.QCAPCTL.bit.CCPS = 7;       // 1/128 for CAP clock
    EQep1Regs.QCAPCTL.bit.CEN = 1;        // QEP Capture Enable
    EDIS;
}

// Declarações dos anteriores para ref_mras
float32 xka1 = 0, xkb1 = 0, aux_alphak1 = 0, aux_betak1 = 0, dphia_k1 = 0, dphib_k1 = 0, phir_alpha_rtk1 = 0, phir_beta_rtk1 = 0, prodk1 = 0, wrk1 = 0, wrf = 0, wr = 0;

float32 aux_alpha = 0, aux_beta = 0, phir_alpha_rt = 0, phir_beta_rt = 0, dphir_alpha_rt = 0, dphir_beta_rt = 0, intprod = 0;

float32 ref_MRAS(float32 va_in, float32 vb_in, float32 vc_in, float32 ia_in, float32 ib_in, float32 ic_in){
    // ============ Modelo de Referência ============================================================
    // Entradas: va - tensão alpha; vb - tensão beta; ia - corrente alpha; ib - corrente beta;

    //transformação de clarke de v
    float32 va = (2.0/3.0)*(va_in - 0.5*vb_in -0.5*vc_in);
    float32 vb = (2.0/3.0)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);

    //transformação de clarke de i
    float32 ia = (2.0/3.0)*(ia_in - 0.5*ib_in -0.5*ic_in);
    float32 ib = (2.0/3.0)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);

    // Variável auxiliar para integração
    float32 xka = (va - Rs*ia);
    float32 xkb = (vb - Rs*ib);

    // Integração
    //float32 aux_alpha = aux_alphak1 + (Ts/2)*xka + (Ts/2)*xka1;
    //float32 aux_beta  = aux_betak1  + (Ts/2)*xkb + (Ts/2)*xkb1;
    aux_alpha = aux_alpha + (Ts/2)*xka + (Ts/2)*xka1;
    aux_beta  = aux_beta  + (Ts/2)*xkb + (Ts/2)*xkb1;

    float32 phir_alpha_st = (Lr/Lm)*(aux_alpha - sigma*Ls*ia );
    float32 phir_beta_st  = (Lr/Lm)*(aux_beta  - sigma*Ls*ib );


    // Modelo Adaptativo
    //float32 dphir_alpha_rt = -(1/Tr)*phir_alpha_rtk1 - wr*phir_beta_rtk1  + (Lm/Tr)*ia;
    //float32 dphir_beta_rt  = -(1/Tr)*phir_beta_rtk1  + wr*phir_alpha_rtk1 + (Lm/Tr)*ib;
    //float32 phir_alpha_rt  = phir_alpha_rtk1 + (Ts/2)*dphir_alpha_rt + (Ts/2)*dphia_k1;
    //float32 phir_beta_rt   = phir_beta_rtk1  + (Ts/2)*dphir_beta_rt  + (Ts/2)*dphib_k1;

    dphir_alpha_rt = -(1/Tr)*phir_alpha_rt - wr*phir_beta_rt  + (Lm/Tr)*ia;
    dphir_beta_rt  = -(1/Tr)*phir_beta_rt  + wr*phir_alpha_rt + (Lm/Tr)*ib;
    phir_alpha_rt  = phir_alpha_rt + (Ts/2)*dphir_alpha_rt + (Ts/2)*dphia_k1;
    phir_beta_rt   = phir_beta_rt  + (Ts/2)*dphir_beta_rt  + (Ts/2)*dphib_k1;


    // Mecanismo de adaptação
    float32 prodk = -(phir_beta_st*phir_alpha_rt - phir_alpha_st*phir_beta_rt);
    intprod += (prodk)*0.000080;
    wr = (wrk1 + fpb_b*prodk1 + fpb_a*prodk);


    // Filtro passa-baixas
    float32 alpha = 100;
    float32 kfilt = alpha;
    wrf = (wrf + Ts*kfilt*wr)/(1+Ts*alpha);

    // Atualização das variáveis
    xka1 = xka;
    xkb1 = xkb;
    aux_alphak1 = aux_alpha;
    aux_betak1  = aux_beta;
    dphia_k1 = dphir_alpha_rt;
    dphib_k1 = dphir_beta_rt;
    phir_alpha_rtk1 = phir_alpha_rt;
    phir_beta_rtk1 = phir_beta_rt;
    prodk1 = prodk;
    wrk1 = wr;
    return (60.0/DPI)*(1.0/p)*wr;  // conversão de rad/s para rpm
}
