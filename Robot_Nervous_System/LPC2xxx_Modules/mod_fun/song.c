/**

	@file song.c
  
  A song playing module.
  Plays sets of notes made of freq and duration.
  
  Mario Song:
  @code
  SONG_NOTE buzzer_mario[74] = {
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_E5},//intro
  	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_E5},
  	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_E5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_E5},
  	{NOTE_HALF,NOTE_QUARTER,NOTE_G5},
  	{NOTE_HALF,NOTE_QUARTER,NOTE_G4},//end intro
  	
  	{NOTE_DOTTED,NOTE_DOTTED,NOTE_C5},
  	{NOTE_DOTTED,NOTE_EIGHTH,NOTE_G4},
  	{NOTE_DOTTED,NOTE_DOTTED,NOTE_E4},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_B5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Bb5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
  	{NOTE_SIXTH,NOTE_SIXTH,NOTE_G4},
  	{NOTE_SIXTH,NOTE_SIXTH,NOTE_E5},
  	{NOTE_SIXTH,NOTE_SIXTH,NOTE_G5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_F5},
  	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_G5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_E5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_D5},
  	{NOTE_DOTTED,NOTE_DOTTED,NOTE_B5},//repeat
  	
  	{NOTE_DOTTED,NOTE_DOTTED,NOTE_C5},
  	{NOTE_DOTTED,NOTE_EIGHTH,NOTE_G4},
  	{NOTE_DOTTED,NOTE_DOTTED,NOTE_E4},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_B5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Bb5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
  	{NOTE_SIXTH,NOTE_SIXTH,NOTE_G4},
  	{NOTE_SIXTH,NOTE_SIXTH,NOTE_E5},
  	{NOTE_SIXTH,NOTE_SIXTH,NOTE_G5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_A5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_F5},
  	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_G5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_E5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_D5},
  	{5*NOTE_EIGHTH,NOTE_DOTTED,NOTE_B5},//repeat
  	
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_G5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Fs5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_F5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_Ds5},
  	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_E5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Gs4},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_A5},
  	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_C5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_A5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C5},
  	{NOTE_DOTTED,NOTE_EIGHTH,NOTE_D5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_G5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Fs5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_F5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_Ds5},
  	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_E5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_C6},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C6},
  	{3*NOTE_QUARTER,NOTE_HALF,NOTE_C6},
  	
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_G5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Fs5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_F5},
  	{NOTE_QUARTER,NOTE_QUARTER,NOTE_Ds5},
  	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_E5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_Gs4},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_A5},
  	{NOTE_QUARTER,NOTE_EIGHTH,NOTE_C5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_A5},
  	{NOTE_EIGHTH,NOTE_EIGHTH,NOTE_C5},
  	{NOTE_DOTTED,NOTE_EIGHTH,NOTE_D5},
  	{NOTE_DOTTED,NOTE_QUARTER,NOTE_Eb5},
  	{NOTE_DOTTED,NOTE_DOTTED,NOTE_D5},
  	{NOTE_HALF,NOTE_HALF,NOTE_C5}
  	};
  @endcode
  
  Cornell Alma Mater:
  @code
  SONG_NOTE alma_mater[26] = {
    {NOTE_DOTTED, NOTE_DOTTED, NOTE_Ab5},
    {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_Bb5},
    {NOTE_DOTTED, NOTE_DOTTED, NOTE_C5},
    {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_Bb5},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_Ab5},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_F4},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_F4},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_Eb4},
    {NOTE_DOTTED, NOTE_DOTTED, NOTE_Bb5},
    {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_Ab5},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_G4},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_Ab5},
    {NOTE_WHOLE, 3*NOTE_QUARTER, NOTE_Bb5},
    
    {NOTE_DOTTED, NOTE_DOTTED, NOTE_Ab5},
    {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_Bb5},
    {NOTE_DOTTED, NOTE_DOTTED, NOTE_C5},
    {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_Bb5},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_Ab5},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_F4},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_F4},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_Eb4},
    {NOTE_DOTTED, NOTE_DOTTED, NOTE_Bb5},
    {NOTE_EIGHTH, NOTE_EIGHTH, NOTE_C5},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_Db4},
    {NOTE_QUARTER, NOTE_QUARTER, NOTE_G5},
    {NOTE_WHOLE, 3*NOTE_QUARTER, NOTE_Ab5}
    };
	@endcode
  
	@author Nicolas Williamson 
  @date July 2009
	
*/

#include <includes.h>

SONG_STATE song_state = SONG_STOP; /**< The current state of the song module. */

unsigned int song_index; /**< Where in the song we are. */
unsigned int song_wait; /**< How long to wait before the next note. */
SONG_NOTE* song_notes; /**< The current song of notes playing. */
SONG_NOTE song_current_note; /**< The current note being played. */
unsigned int song_length; /**< The total number of notes in the song. */

/**
  Sets the song to play.
  @param song The array of notes in the song.
  @param length The number of notes in the song (the length of the array).
*/
void song_set(SONG_NOTE* song, unsigned int length){
	song_notes = song;
	song_length = length;
	song_wait = 0;
	song_index = 0;
	song_current_note = song_notes[song_index];
}

/**
  Runs the state machine which controls the song playing.
*/
void song_update(void){
	switch (song_state){
		case SONG_FAST_FORWARD: song_wait++;
		case SONG_PLAY:
			song_wait++;
			if (song_wait >= song_current_note.length){
				song_current_note = song_notes[song_index];
				song_index++;
				if (song_index >= song_length){song_index = 0;}
				song_wait = 0;
			}
			if (song_wait >= song_current_note.duration - 50){
				buzzer_off();
			} else{ 
				buzzer_set_frequency(song_current_note.frequency);
				buzzer_on();
			}
			break;
		case SONG_STOP: buzzer_off(); break;
		case SONG_PAUSE: buzzer_off(); break;
	}
}	

/**
  Stops the song player.
*/
void song_stop(void){
	song_wait = 0;
	song_index = 0;
	song_current_note = song_notes[song_index];
	song_state = SONG_STOP;
}

/**
  Starts the song player.
*/
void song_play(void){
	song_state = SONG_PLAY;
}

/**
  Pauses the song player.
*/
void song_pause(void){
	song_state = SONG_PAUSE;
}

/**
  Fast Forwards the song player. Doesn't stop it playing.
*/
void song_fast_forward(void){
	song_state = SONG_FAST_FORWARD;
}

