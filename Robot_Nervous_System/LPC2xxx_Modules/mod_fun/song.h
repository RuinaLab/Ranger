/*

	song.h
	
	Nicolas Williamson - July 2009
	
*/

#ifndef __H_SONG__
#define __H_SONG__

/**
  One note of the song.
*/
typedef struct song_note{
	unsigned int length; /**< The full length of the note (in ms). */
	unsigned int duration; /**< The duration of the sounds during the note (in ms). */
	unsigned int frequency; /**< The frequency of the note. */
} SONG_NOTE;

/**
  The possible states of the song player.
*/
typedef enum song_states{
	SONG_PLAY = 0, /**< Play the song at normal speed. */
	SONG_STOP, /**< Stops and resets the song. */
	SONG_PAUSE, /**< Pauses the current song. Play begins where it left off. */
	SONG_FAST_FORWARD /**< Plays the song at double speed. */
} SONG_STATE;

#define NOTE_A4 440
#define NOTE_As4 466
#define NOTE_Bb4 466
#define NOTE_B4	494
#define NOTE_C4	523
#define NOTE_Cs4 554
#define NOTE_Db4 554
#define NOTE_D4	587
#define NOTE_Ds4 622
#define NOTE_Eb4 622
#define NOTE_E4	659
#define NOTE_F4	698
#define NOTE_Fs4 740
#define NOTE_Gb4 740
#define NOTE_G4	784
#define NOTE_Gs4 830
#define NOTE_Ab5 830
#define NOTE_A5	880
#define NOTE_As5 932
#define NOTE_Bb5 932
#define NOTE_B5 988
#define NOTE_C5	1046
#define NOTE_Cs5 1108
#define NOTE_Db5 1108
#define NOTE_D5	1175
#define NOTE_Ds5 1244
#define NOTE_Eb5 1244
#define NOTE_E5	1318
#define NOTE_F5	1397
#define NOTE_Fs5 1480
#define NOTE_Gb5 1480
#define NOTE_G5	1568
#define NOTE_Gs5 1661
#define NOTE_Ab6 1661
#define NOTE_A6	1760
#define NOTE_C6	2093

#define NOTE_WHOLE (1600) //length of whole note, ms
#define NOTE_EIGHTH (NOTE_WHOLE/8)
#define NOTE_QUARTER (NOTE_WHOLE/4)
#define NOTE_HALF (NOTE_WHOLE/2)
#define NOTE_THIRD (NOTE_WHOLE/3)
#define NOTE_SIXTH (NOTE_WHOLE/6)
#define NOTE_DOTTED (3*NOTE_EIGHTH)

//functions
void song_set(SONG_NOTE* song, unsigned int length);
void song_play(void);
void song_stop(void);
void song_pause(void);
void song_update(void);
void song_fast_forward(void);

#endif
