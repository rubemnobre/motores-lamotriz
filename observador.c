/*
 * observador.c
 * Autores: Jucelino Taleires, Rubem Bezerra
 * Laboratório de Sistemas Motrizes
 */

#include "params.h"

#define DPI 6.28318530717958647692

// Declarações dos anteriores para ref_mras
float xka1 = 0, xkb1 = 0, dphia_k1 = 0, dphib_k1 = 0, prodk1 = 0, wrk1 = 0, wrf = 0, wr = 0, aux_alpha = 0, aux_beta = 0, phir_alpha_rt = 0, phir_beta_rt = 0;

float ref_MRAS(float va_in, float vb_in, float vc_in, float ia_in, float ib_in, float ic_in){
    // ============ Modelo de Referência ============================================================
    // Entradas: va - tensão alpha; vb - tensão beta; ia - corrente alpha; ib - corrente beta;

    //transformação de clarke de v
    float va = (2.0/3.0)*(va_in - 0.5*vb_in -0.5*vc_in);
    float vb = (2.0/3.0)*(0.86602540378443864676*vb_in - 0.86602540378443864676*vc_in);

    //transformação de clarke de i
    float ia = (2.0/3.0)*(ia_in - 0.5*ib_in -0.5*ic_in);
    float ib = (2.0/3.0)*(0.86602540378443864676*ib_in - 0.86602540378443864676*ic_in);

    //Variável auxiliar para integração
    float xka = (va - Rs*ia);
    float xkb = (vb - Rs*ib);

    //Integração
    aux_alpha = aux_alpha + (Ts/2)*xka + (Ts/2)*xka1;
    aux_beta  = aux_beta  + (Ts/2)*xkb + (Ts/2)*xkb1;

    float phir_alpha_st = (Lr/Lm)*(aux_alpha - sigma*Ls*ia );
    float phir_beta_st  = (Lr/Lm)*(aux_beta  - sigma*Ls*ib );


    //Modelo Adaptativo
    float dphir_alpha_rt = -(1/Tr)*phir_alpha_rt - wr*phir_beta_rt  + (Lm/Tr)*ia;
    float dphir_beta_rt  = -(1/Tr)*phir_beta_rt  + wr*phir_alpha_rt + (Lm/Tr)*ib;
    phir_alpha_rt  = phir_alpha_rt + (Ts/2)*dphir_alpha_rt + (Ts/2)*dphia_k1;
    phir_beta_rt   = phir_beta_rt  + (Ts/2)*dphir_beta_rt  + (Ts/2)*dphib_k1;


    //Mecanismo de adaptação
    float prodk = -(phir_beta_st*phir_alpha_rt - phir_alpha_st*phir_beta_rt);
    wr = (wrk1 + fpb_b*prodk1 + fpb_a*prodk);


    //Filtro passa-baixas
    wrf = (wrf + Ts*100*wr)/(1+Ts*100);

    //Atualização das variáveis
    xka1 = xka;
    xkb1 = xkb;

    dphia_k1 = dphir_alpha_rt;
    dphib_k1 = dphir_beta_rt;

    prodk1 = prodk;
    wrk1 = wr;
    return (60.0/DPI)*(1.0/p)*wr;  // conversão de rad/s para rpm
}

