/*
 * params.h
 *
 */

#ifndef PARAMS_H_
#define PARAMS_H_

//Par�metros da m�quina

#define Rs 35.58
#define Rr 87.44
#define Lls 0.16
#define Llr 0.16
#define Ls 1.044 //Lls +Lm
#define Lr 1.044 //Llr +Lm
#define Lm 0.884

#define p 2 // pares de polos

//Par�metros do Observador
#define fsw 6e3            // Frequ�ncia de chaveamento
#define Ts 80e-6           // tempo de amostragem do observador 1/(fsw*100)
#define sigma 0.346442359  // coeficiente total de dispers�o 1 - (Lm^2/(Ls*Lr))
#define Tr 0.011939615     // constante de tempo rot�rica (Lr/Rr)
#define fpb_a 800          // par�metro para FPB
#define fpb_b -792         // par�metro para FPB (-0.99*a)


// Definicoes para o observador SMO
#define Pn   180        // Pot�ncia nominal (0.25 cv)
#define Vn   380        // Tens�o de linha da rede el�trica
#define fn   60         // Frequ�ncia nominal da rede el�trica
#define rend 0.64       // Rendimento
#define fp   0.65       // Fator de Pot�ncia
#define Sn   432.69231  // Pn/(rend*fp)

#define H 0.00045       // Momento de in�rcia
#define F 0.0001        // Coeficiente de atrito

#define k2 2.764830516
#define k1 98.37266975
#define gamma 2.341101701
#define gamma1 0.01
#define gamma2 0.01

#endif /* PARAMS_H_ */
