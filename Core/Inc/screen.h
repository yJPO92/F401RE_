/*
 * screen.h
 *
 *  Created on: 25 mai 2019
 *      Author: Jean
 */
 /*
 * code VT100 :  \x1B = ESC
 * \x1B[2J          efface ecran
 * \x1B[H           reset curseur
 * \x1B[?25l        curseur invisible
 * \x1B[?25h        curseur visible
 * Esc[Line;ColH    position curseur
 */

#ifndef INC_SCREEN_H_
#define INC_SCREEN_H_


#define clrscr "\x1B[2J"
#define homescr "\x1B[H"
#define gotoscr "\x1B[3;0"    // row 3, col 0

/*
#define mmenu1 "\x1B[?25h\n\
----------------------------------\n\
 R (ADC1 analog values) \n\
 C (start toggle LED1 avec time)\n\
 L (start toggle LED1 sans time)\n\
 S (stop toggle LED1)\n\
 D (display Real Time)\n\
 \n\
 0-9|a|b (affiche le chiffre)\n\
 +|- (change de chiffre)\n\
\n\
\n\
 W (saisie de 2 touches)\n\
\n\
 Menu | Quit\n"

#define mmenu2 " Faire un choix ...\n"
*/

//efface ecran, reset cursor, curseur invisible, 1crlf, 1 point, 1crlf
//en col 2 dessine une box
//retour en haut ecran
#define screen1 "\x1B[2J\x1B[H\x1B[?25l\
\n\
.               Horloge de Jean92\n\
+--------+---------+---------+---------+---------+\n\
|                  jj-mm-aa                      |\n\
|                  hh:mm:ss                      |\n\
+------------------------------------------------+\n\
.\x1B[H"


#endif /* INC_SCREEN_H_ */
