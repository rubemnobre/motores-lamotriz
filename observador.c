/*
 * observador.c
 * Autores: Jucelino Taleires, Rubem Bezerra
 * Laboratório de Sistemas Motrizes
 */

#include<fpu32/fpu_vector.h>
#include<math.h>
#include "F28x_Project.h"
#include "params.h"
#define DPI 6.28318530717958647692

#ifdef MRAS

// Declarações dos anteriores para ref_mras
float xka1 = 0, xkb1 = 0, dphia_k1 = 0, dphib_k1 = 0, prodk1 = 0, wrk1 = 0, wrf = 0, wr = 0, aux_alpha = 0, aux_beta = 0, phir_alpha_rt = 0, phir_beta_rt = 0;

float aux_alpha2 = 0, aux_alpha3 = 0, aux_beta2 = 0, aux_beta3 = 0;

float ref_MRAS(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in, float Velo){
    // Transformação de clarke de v
    // Mudanças: valpha e vbeta trocados e mecanismo de adaptação
    float vbeta = (2.0/3.0)*(va_in - 0.5*vb_in -0.5*vc_in);
    float valpha = (2.0/3.0)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);

    // Transformação de clarke de i
    float ialpha = (2.0/3.0)*(ia_in - 0.5*ib_in -0.5*ic_in);
    float ibeta = (2.0/3.0)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);

    // Variável auxiliar para integração
    float xka = (valpha - Rs*ialpha);
    float xkb = (vbeta - Rs*ibeta);

    // Integração pura
//    aux_alpha = aux_alpha + (Ts/2)*xka + (Ts/2)*xka1;
//    aux_beta  = aux_beta  + (Ts/2)*xkb + (Ts/2)*xkb1;
//
//    float phir_alpha_st = (Lr/Lm)*(aux_alpha - sigma*Ls*ialpha );
//    float phir_beta_st  = (Lr/Lm)*(aux_beta  - sigma*Ls*ibeta );

    // Integração do fluxo com correção
    float g1 = 9.993602e-01, g2 = 3.198976e-06, g3 = 9.993602e-01, g4 = 6.397952e-04;
    aux_alpha = g1*aux_alpha + g2*xka;
    aux_beta  = g1*aux_beta  + g2*xkb;

    if(aux_alpha3 >= 0.5) aux_alpha3 = 0.5;
    if(aux_alpha3 <= -0.5) aux_alpha3 = -0.5;
    if(aux_beta3 >= 0.5) aux_beta3 = 0.5;
    if(aux_beta3 <= -0.5) aux_beta3 = -0.5;

    aux_alpha2 = g3*aux_alpha2 + g4*aux_alpha3;
    aux_beta2  = g3*aux_beta2  + g4*aux_beta3;

    aux_alpha3 = aux_alpha + aux_alpha2;
    aux_beta3  = aux_beta  + aux_beta2;


    float phir_alpha_st = (Lr/Lm)*(aux_alpha3 - sigma*Ls*ialpha );
    float phir_beta_st  = (Lr/Lm)*(aux_beta3  - sigma*Ls*ibeta );
    //DacaRegs.DACVALS.all = phir_beta_st * 1024.0 / 2.0 + 1024;
    //float pos = atan2(phir_alpha_st, phir_beta_st);
    //DacbRegs.DACVALS.all = pos * 1024.0 / 7 + 1024;

    // Modelo Adaptativo
    float dphir_alpha_rt = -(1/Tr)*phir_alpha_rt - wr*phir_beta_rt  + (Lm/Tr)*ialpha;
    float dphir_beta_rt  = -(1/Tr)*phir_beta_rt  + wr*phir_alpha_rt + (Lm/Tr)*ibeta;
    phir_alpha_rt  = (phir_alpha_rt + (Ts/2)*dphir_alpha_rt + (Ts/2)*dphia_k1);
    phir_beta_rt   = phir_beta_rt  + (Ts/2)*dphir_beta_rt  + (Ts/2)*dphib_k1;

    // Mecanismo de adaptação
    float prodk = -(phir_beta_st*phir_alpha_rt - phir_alpha_st*phir_beta_rt);
    //float prodk = -(phir_beta_st*phir_beta_rt - phir_alpha_st*phir_alpha_rt);

    wr = (wrk1 + fpb_b*prodk1 + fpb_a*prodk);
    //wr = Velo;

    //DacaRegs.DACVALS.all = 1*(phir_alpha_rt * 2048.0 / 5.0) + 1024; // Amarelo
    //DacbRegs.DACVALS.all = 1*(phir_alpha_st * 2048.0 / 5.0) + 1024; // Azul

    // Filtro passa-baixas
    wrf = (wrf + Ts*10*wr)/(1+Ts*10);
    //DacbRegs.DACVALS.all = wrf * 1024.0 / 1000.0 + 1024;

    // Atualização das variáveis
    xka1 = xka;
    xkb1 = xkb;

    dphia_k1 = dphir_alpha_rt;
    dphib_k1 = dphir_beta_rt;

    prodk1 = prodk;
    //wr = Velo*DPI*p/60.0;
    wrk1 = wr;
    //return pos;
    return (60.0/DPI)*(1.0/p)*wrf;  // conversão de rad/s para rpm
}

#endif

#ifdef SMO

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

#endif

#ifdef EKF

int init = 1;

static float Phat[25];
static float xkhat[5];

float ref_EKF(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in){ // Observador de fluxo rotórico e de velocidade filtro de Kalman estendido
    if(init){
        int i;
        static const float fv2[25] = { 1e6, 0, 0, 0, 0,
                                       0, 1e6, 0, 0, 0,
                                       0, 0, 1e6, 0, 0,
                                       0, 0, 0, 1e6, 0,
                                       0, 0, 0, 0, 1e6};

        for (i = 0; i < 25; i++){
            Phat[i] = fv2[i];
        }

        for (i = 0; i < 5; i++){
            xkhat[i] = 0;
        }
        init = 0;
    }

    // Transformação de eixos de referência (invariante em amplitude)
    float valpha = (2/3)*(va_in - 0.5*vb_in - 0.5*vc_in);
    float vbeta  = -(2/3)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);

    float ialpha = (2/3)*(ia_in -0.5*ib_in - 0.5*ic_in);
    float ibeta  = -(2/3)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);

    float omega1;
    float Gk[25];
    int r1;
    float b_va[2];
    static const int iv0[5] = { 0, 0, 0, 0, 1 };

    float xktil[5];
    float fv0[5];
    int r2;
    float b_Gk[25];
    float a[5];
    static const float b_a[10] = { 5.64055972E-6, 0, 0, 0, 0,
                                   0, 5.64055972E-6, 0, 0, 0 };

    float c[4];
    int k;
    float kk[10];
    float b_c[10];
    float a22;
    static const int c_a[10] = { 1, 0, 0, 1, 0,
                                 0, 0, 0, 0, 0 };

    float Ptil[25];
    static const float fv1[25] = { 1E-6, 0, 0, 0, 0,
                                   0, 1E-6, 0, 0, 0,
                                   0, 0, 1E-6, 0, 0,
                                   0, 0, 0, 1E-6, 0,
                                   0, 0, 0, 0, 1e-3 };

    static const int iv1[4] = { 2, 0, 0, 2 };

    static const int b[10] = { 1, 0, 0, 0, 0,
                               0, 1, 0, 0, 0};

    float d_a[2];
    float b_ia[2];

    static const int iv2[25] = { 1, 0, 0, 0, 0,
                                 0, 1, 0, 0, 0,
                                 0, 0, 1, 0, 0,
                                 0, 0, 0, 1, 0,
                                 0, 0, 0, 0, 1};

    omega1 = 2 * xkhat[4];

    Gk[0] = 0.999445677;
    Gk[5] = 0;
    Gk[10] = 0.00040002176;
    Gk[15] = omega1 * 1.66666666E-6 * 0.884 / 0.308480024;
    Gk[20] = 0;
    Gk[1] = 0;
    Gk[6] = 0.999445677;
    Gk[11] = -omega1 * 1.66666666E-6 * 0.884 / 0.308480024;
    Gk[16] = 0.00040002176;
    Gk[21] = 0;
    Gk[2] = 0.000123398713;
    Gk[7] = 0;
    Gk[12] = 0.999860406;
    Gk[17] = -1.66666666E-6 * omega1;
    Gk[22] = 0;
    Gk[3] = 0;
    Gk[8] = 0.000123398713;
    Gk[13] = 1.66666666E-6 * omega1;
    Gk[18] = 0.999860406;
    Gk[23] = 0;

    for (r1 = 0; r1 < 5; r1++) {
        Gk[4 + 5 * r1] = iv0[r1];
    }

    b_va[0] = valpha;
    b_va[1] = vbeta;
    for (r1 = 0; r1 < 5; r1++) {
        fv0[r1] = 0;
        for (r2 = 0; r2 < 5; r2++) {
            fv0[r1] += Gk[r1 + 5 * r2] * xkhat[r2];
        }
        a[r1] = 0;
        for (r2 = 0; r2 < 2; r2++) {
            a[r1] += b_a[r1 + 5 * r2] * b_va[r2];
        }
        xktil[r1] = fv0[r1] + a[r1];
        b_Gk[4 + 5 * r1] = iv0[r1];
    }

    omega1 = 2 * xktil[4];

    /*  cálculo de G */
    b_Gk[0] = 0.999445677;
    b_Gk[5] = 0;
    b_Gk[10] = 0.00040002176;
    b_Gk[15] = omega1 * 1.66666666E-6 * 0.884 / 0.308480024;
    b_Gk[20] = 1.47333333E-6 * xktil[3] / 0.308480024;
    b_Gk[1] = 0;
    b_Gk[6] = 0.999445677;
    b_Gk[11] = -omega1 * 1.66666666E-6 * 0.884 / 0.308480024;
    b_Gk[16] = 0.00040002176;
    b_Gk[21] = -1.47333333E-6 * xktil[2] / 0.308480024;
    b_Gk[2] = 0.000123398713;
    b_Gk[7] = 0;
    b_Gk[12] = 0.999860406;
    b_Gk[17] = -1.66666666E-6 * omega1;
    b_Gk[22] = -1.66666666E-6 * xktil[3];
    b_Gk[3] = 0;
    b_Gk[8] = 0.000123398713;
    b_Gk[13] = 1.66666666E-6 * omega1;
    b_Gk[18] = 0.999860406;
    b_Gk[23] = 1.66666666E-6 * xktil[2];

    /*  2) covariância do erro */
    /*  3) ganho de Kalman */
    for (r1 = 0; r1 < 5; r1++) {
        for (r2 = 0; r2 < 5; r2++) {
            Gk[r1 + 5 * r2] = 0;
            for (k = 0; k < 5; k++) {
                Gk[r1 + 5 * r2] += b_Gk[r1 + 5 * k] * Phat[k + 5 * r2];
            }
        }

        for (r2 = 0; r2 < 5; r2++) {
            omega1 = 0;
            for (k = 0; k < 5; k++) {
                omega1 += Gk[r1 + 5 * k] * b_Gk[r2 + 5 * k];
            }
            Ptil[r1 + 5 * r2] = omega1 + fv1[r1 + 5 * r2];
        }

        for (r2 = 0; r2 < 2; r2++) {
            b_c[r1 + 5 * r2] = 0;
            for (k = 0; k < 5; k++) {
                b_c[r1 + 5 * r2] += Ptil[r1 + 5 * k] * (float)b[k + 5 * r2];
            }
        }
    }

    for (r1 = 0; r1 < 2; r1++) {
        for (r2 = 0; r2 < 5; r2++) {
            kk[r1 + (r2 << 1U)] = 0;
            for (k = 0; k < 5; k++) {
                kk[r1 + (r2 << 1U)] += (float)c_a[r1 + (k << 1U)] * Ptil[k + 5 * r2];
            }
        }

        for (r2 = 0; r2 < 2; r2++) {
            omega1 = 0;
            for (k = 0; k < 5; k++) {
                omega1 += kk[r1 + (k << 1U)] * (float)b[k + 5 * r2];
            }

            c[r1 + (r2 << 1U)] = omega1 + (float)iv1[r1 + (r2 << 1U)];
        }
    }

    if ((float)fabs(c[1]) > (float)fabs(c[0])){
        r1 = 1;
        r2 = 0;
    }else{
        r1 = 0;
        r2 = 1;
    }

    omega1 = c[r2] / c[r1];
    a22 = c[2 + r2] - omega1 * c[2 + r1];
    for (k = 0; k < 5; k++) {
        kk[k + 5 * r1] = b_c[k] / c[r1];
        kk[k + 5 * r2] = (b_c[5 + k] - kk[k + 5 * r1] * c[2 + r1]) / a22;
        kk[k + 5 * r1] -= kk[k + 5 * r2] * omega1;
    }

    /*  4) Atualização dos estados preditos */
    b_va[0] = ialpha;
    b_va[1] = ibeta;
    for (r1 = 0; r1 < 2; r1++) {
        d_a[r1] = 0;
        for (r2 = 0; r2 < 5; r2++) {
            d_a[r1] += (float)c_a[r1 + (r2 << 1U)] * xktil[r2];
        }

        b_ia[r1] = b_va[r1] - d_a[r1];
    }

    /*  5) Atualização da covariância do erro */
    for (r1 = 0; r1 < 5; r1++) {
        omega1 = 0;
        for (r2 = 0; r2 < 2; r2++) {
            omega1 += kk[r1 + 5 * r2] * b_ia[r2];
        }

        xkhat[r1] = xktil[r1] + omega1;
        for (r2 = 0; r2 < 5; r2++) {
            omega1 = 0;
            for (k = 0; k < 2; k++) {
                omega1 += kk[r1 + 5 * k] * (float)c_a[k + (r2 << 1U)];
            }

            Gk[r1 + 5 * r2] = (float)iv2[r1 + 5 * r2] - omega1;
        }

        for (r2 = 0; r2 < 5; r2++) {
            Phat[r1 + 5 * r2] = 0;
            for (k = 0; k < 5; k++) {
                Phat[r1 + 5 * r2] += Gk[r1 + 5 * k] * Ptil[k + 5 * r2];
            }
        }
    }
    return xktil[4];
}

#endif
