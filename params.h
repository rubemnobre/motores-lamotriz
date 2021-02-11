/*
 * params.h
 *
 *  Created on: 10 de fev de 2021
 *      Author: rubem
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

#endif /* PARAMS_H_ */