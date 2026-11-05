/* ========================================
 * music.c
 * ========================================
 */

#include "car.h"

// Frequencies in Hz for the notes
#define C4  261
#define E4  330
#define F4  349
#define G4  392
#define Ab4 415
#define A4  440
#define B4  494
#define C5  523
#define D5  587
#define Eb5 622
#define E5  659
#define F5  698
#define G5  784

// Duration in milliseconds
#define D 300  // Waltz beat (dotted quarter)
#define H 600  // Half note
#define Q 400  // Quarter note
#define E 200  // Eighth note
#define S 100  // Sixteenth note

#define BPM144    144
#define BEAT(bpm) (60000 / (bpm))

#define W144  (BEAT(BPM144) * 4)      // 1666
#define H144  (BEAT(BPM144) * 2)      // 832
#define DQ144 (BEAT(BPM144) * 3 / 2)  // 624
#define Q144  (BEAT(BPM144))          // 416
#define E144  (BEAT(BPM144) / 2)      // 208
#define S144  (BEAT(BPM144) / 4)      // 104

void Music_FurElise(void) {
    Sound_Play(E5, E);
    Sound_Play(Eb5, E);
    Sound_Play(E5, E);
    Sound_Play(Eb5, E);
    Sound_Play(E5, E);
    Sound_Play(B4, E);
    Sound_Play(D5, E);
    Sound_Play(C5, E);
    Sound_Play(A4, Q);

    Sound_Play(C4, E);
    Sound_Play(E4, E);
    Sound_Play(A4, E);
    Sound_Play(B4, Q);

    Sound_Play(E4, E);
    Sound_Play(Ab4, E);
    Sound_Play(B4, E);
    Sound_Play(C5, Q);
}

void Music_ViennaWaltz(void) {
    Sound_Play(G4, D);
    Sound_Play(B4, D);
    Sound_Play(D5, D);

    Sound_Play(G4, D);
    Sound_Play(B4, D);
    Sound_Play(D5, D);

    Sound_Play(C5, D);
    Sound_Play(E5, D);
    Sound_Play(G5, D);

    Sound_Play(B4, D);
    Sound_Play(D5, D);
    Sound_Play(F5, D);

    Sound_Play(G4, H);
    Sound_Play(C5, H);
}

void Music_Technopolis(void) {
    Sound_Play(C5, E144);
    Sound_Play(B4, E144);
    Sound_Play(A4, E144);
    Sound_Play(G4, Q144);
    Sound_Play(E4, Q144);

    Sound_Play(G4, Q144);
    Sound_Play(A4, E144);
    Sound_Play(A4, H144);

    Sound_Play(B4, Q144);

    Sound_Play(C5, E144);
    Sound_Play(B4, E144);
    Sound_Play(A4, E144);
    Sound_Play(G4, Q144);
    Sound_Play(E4, E144);
    Sound_Play(G4, E144);
    Sound_Play(E5, 1300);
    Sound_Play(D5, Q144);
}

/* [] END OF FILE */
