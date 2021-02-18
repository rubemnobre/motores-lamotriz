/*
 * observador.c
 * Autores: Jucelino Taleires, Rubem Bezerra
 * Laboratório de Sistemas Motrizes
 */

#include<fpu32/fpu_vector.h>
#include "params.h"
#define DPI 6.28318530717958647692

// Declarações dos anteriores para ref_mras
float xka1 = 0, xkb1 = 0, dphia_k1 = 0, dphib_k1 = 0, prodk1 = 0, wrk1 = 0, wrf = 0, wr = 0, aux_alpha = 0, aux_beta = 0, phir_alpha_rt = 0, phir_beta_rt = 0;

float ref_MRAS(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in){
    // Transformação de clarke de v
    float valpha = (2.0/3.0)*(va_in - 0.5*vb_in -0.5*vc_in);
    float vbeta = (2.0/3.0)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);

    // Transformação de clarke de i
    float ialpha = (2.0/3.0)*(ia_in - 0.5*ib_in -0.5*ic_in);
    float ibeta = (2.0/3.0)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);

    // Variável auxiliar para integração
    float xka = (valpha - Rs*ialpha);
    float xkb = (vbeta - Rs*ibeta);

    // Integração
    aux_alpha = aux_alpha + (Ts/2)*xka + (Ts/2)*xka1;
    aux_beta  = aux_beta  + (Ts/2)*xkb + (Ts/2)*xkb1;

    float phir_alpha_st = (Lr/Lm)*(aux_alpha - sigma*Ls*ialpha );
    float phir_beta_st  = (Lr/Lm)*(aux_beta  - sigma*Ls*ibeta );

    // Modelo Adaptativo
    float dphir_alpha_rt = -(1/Tr)*phir_alpha_rt - wr*phir_beta_rt  + (Lm/Tr)*ialpha;
    float dphir_beta_rt  = -(1/Tr)*phir_beta_rt  + wr*phir_alpha_rt + (Lm/Tr)*ibeta;
    phir_alpha_rt  = phir_alpha_rt + (Ts/2)*dphir_alpha_rt + (Ts/2)*dphia_k1;
    phir_beta_rt   = phir_beta_rt  + (Ts/2)*dphir_beta_rt  + (Ts/2)*dphib_k1;

    // Mecanismo de adaptação
    float prodk = -(phir_beta_st*phir_alpha_rt - phir_alpha_st*phir_beta_rt);
    wr = (wrk1 + fpb_b*prodk1 + fpb_a*prodk);

    // Filtro passa-baixas
    wrf = (wrf + Ts*100*wr)/(1+Ts*100);

    // Atualização das variáveis
    xka1 = xka;
    xkb1 = xkb;

    dphia_k1 = dphir_alpha_rt;
    dphib_k1 = dphir_beta_rt;

    prodk1 = prodk;
    wrk1 = wr;
    return (60.0/DPI)*(1.0/p)*wr;  // conversão de rad/s para rpm
}


// Declaracoes para o observador SMO
float dialpha_est = 0, dibeta_est = 0, ialpha_est = 0, ibeta_est = 0, qalpha = 0, qbeta = 0, salpha = 1, sbeta = 1, salphak1 = 0, sbetak1 = 0, dflux_est_alpha = 0, dflux_est_beta= 0, flux_est_alpha= 0, flux_est_beta= 0;

float ref_SMO(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in){ // Observador de fluxo rotórico e de velocidade por modos deslizantes
    // Transformação de eixos de referência (invariante em amplitude)
    float valpha = (2/3)*(va_in - 0.5*vb_in - 0.5*vc_in);
    float vbeta  = -(2/3)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);

    float ialpha = (2/3)*(ia_in -0.5*ib_in - 0.5*ic_in);
    float ibeta  = -(2/3)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);

    // Observador de corrente
    // Derivadas das correntes em alpha/beta
    dialpha_est = qalpha - k1*ialpha_est + k2*valpha;
    dibeta_est  = qbeta  - k1*ibeta_est  + k2*vbeta;

    // Integração da corrente
    ialpha_est = ialpha_est + Ts*(dialpha_est);
    ibeta_est  = ibeta_est  + Ts*(dibeta_est);

    // Cálculo das funções "s" em alpha/beta
    salpha = ialpha_est - ialpha;
    sbeta  = ibeta_est  - ibeta;

    // Cálculo de q
    qalpha = qalpha - (1/Ts)*( (1+Ts*gamma1)*salpha - salphak1);
    qbeta  = qbeta  - (1/Ts)*( (1+Ts*gamma2)*sbeta - sbetak1 );

    // Cálculo de dq
    // dqalpha = - (1/Ts)*( (1+Ts*gamma1)*salpha - salphak1);
    // dqbeta  = - (1/Ts)*( (1+Ts*gamma2)*sbeta - sbetak1 );

    // Cálculo das derivadas dos fluxos
    dflux_est_alpha = - qalpha/gamma;
    dflux_est_beta  = - qbeta/gamma;

    // Integração dos fluxos
    flux_est_alpha = flux_est_alpha + Ts*dflux_est_alpha;
    flux_est_beta  = flux_est_beta  + Ts*dflux_est_beta;

    // Estimador de velocidade
    wr = -(flux_est_alpha*dflux_est_beta - flux_est_beta*dflux_est_alpha - Lm*(Rr/Lr)*(ibeta_est*flux_est_alpha - ialpha_est*flux_est_beta));
    float D = (flux_est_beta*flux_est_beta + flux_est_alpha*flux_est_alpha);
    if(D <= 0.04) D = 0.01;

    // Atualização das variáveis
    salphak1 = salpha;
    sbetak1 = sbeta;

    // Saída do observador
    wr = wr/D;
    return wr;
}

float ref_EKF(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in){ // Observador de fluxo rotórico e de velocidade filtro de Kalman estendido
    // Transformação de clarke de v
    float valpha = (2.0/3.0)*(va_in - 0.5*vb_in -0.5*vc_in);
    float vbeta = (2.0/3.0)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);

    // Transformação de clarke de i
    float ialpha = (2.0/3.0)*(ia_in - 0.5*ib_in -0.5*ic_in);
    float ib = (2.0/3.0)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);

    float uk[2][1] = { {valpha},
                       {vbeta} };

    float yk[2][1] = {
       {ialpha},
       {ib},
    };

    float Q[5][5] = {
        {1e-6,    0,    0,    0,    0},
        {   0, 1e-6,    0,    0,    0},
        {   0,    0, 1e-6,    0,    0},
        {   0,    0,    0, 1e-6,    0},
        {   0,    0,    0,    0, 1e-3},
    };

    /*
    R = 2*eye(2);

    // Iniciando variáveis
    if isempty(cont)
    Ptil = 1e6*eye(5);Phat = 1e6*eye(5);  xktil = 1*ones(5,1); xkhat = 0*ones(5,1);
    wrf =0; angk1 =0; angk2=0;
    end


    omega1 = p*xkhat(5);  // variável para atualizar matriz A'

    // Matriz A'
    Alinha = [(1 - Ts/Tlinha)         0           ((Ts*Lm)/(sigma*Ls*Lr*Tr))    ((omega1*Ts*Lm)/(sigma*Ls*Lr)) 0;
                    0           (1 - Ts/Tlinha) ((-omega1*Ts*Lm)/(sigma*Ls*Lr)) ((Ts*Lm)/(sigma*Ls*Lr*Tr))     0;
              (Ts*Lm/Tr)              0              (1 - Ts/Tr)                       -Ts*omega1              0;
                    0           (Ts*Lm/Tr)           Ts*omega1                          (1 - Ts/Tr)            0;
                    0                  0                   0                                0                  1];

    // Matriz B'
    Blinha = [(Ts/(sigma*Ls))         0;
                   0            (Ts/(sigma*Ls));
                   0                  0;
                   0                  0;
                   0                  0];

    // Matriz H
    Hk = [1 0 0 0 0;
          0 1 0 0 0];

    // ============== Filtro de Kalman estendido ===================================================================

    // 1) variáveis de estado
    xktil = Alinha*xkhat + Blinha*uk;

    phi_alpha = xktil(3);
    phi_beta = xktil(4);
    omega2 = p*xktil(5);

    // cálculo de G
    Gk =[(1 - Ts/Tlinha)        0        ((Ts*Lm)/(sigma*Ls*Lr*Tr)) ((omega2*Ts*Lm)/(sigma*Ls*Lr))    ((Ts*Lm*phi_beta)/(sigma*Ls*Lr));
                0         (1-Ts/Tlinha)  ((-omega2*Ts*Lm)/(sigma*Ls*Lr)) ((Ts*Lm)/(sigma*Ls*Lr*Tr))   ((-Ts*Lm*phi_alpha)/(sigma*Ls*Lr));
             (Ts*Lm/Tr)         0              (1 - Ts/Tr)                -Ts*omega2                       -Ts*phi_beta;
                0             Ts*Lm/Tr             Ts*omega2                  (1 - Ts/Tr)                    Ts*phi_alpha;
                0               0                   0                       0                                 1];

    // 2) covariância do erro
    Ptil = Gk*Phat*Gk'+Q;

    // 3) ganho de Kalman
    kk = Ptil*Hk'*inv(Hk*Ptil*Hk'+ R);

    // 4) Atualização dos estados preditos
    xkhat = xktil + kk*(yk - Hk*xktil);

    // 5) Atualização da covariância do erro
    Phat = (eye(5) - kk*Hk)*Ptil;

    // Saída estimada
    yhat = Hk*xkhat;

    trP = trace(Phat); // Traço da matriz de covariância
    phia = xkhat(3);
    phib = xkhat(4);
    w =  xktil(5);
    */
    float w = 0;
    return w;
}
