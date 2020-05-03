/*
 * harkkaMEGA.c
 *
 * Created: 23.4.2020 14.59.55
 * Author : Elias, Aleksi, Aku
 */ 

#include <avr/io.h>
#include "lcd.h" // LCD Library by Peter Fleury.

#define CHECK_SENSOR 0
#define ALARM 1
#define KEYPAD 2

#define F_CPU 16000000UL

const char message1_connection_established[] = "Alarm connected successfully";
const char message2_movement_detected[] = "Movement detected";
const char message3_give_password[] = "Type password to reset";
const char message4_password_correct[] = "Password given, alarm will reset";
const char message5_password_time_out[] = "Password time-out error.";

int8_t g_state = 0;

/* Fucntion declarations */
void display_message(int message_number, int password_length); 

int main(void)
{
	// LCD initialization.
	lcd_init(LCD_DISP_ON);
    /* Replace with your application code */
    while (1) 
    {
		switch (g_state) 
		{
			case CHECK_SENSOR:
				display_message(0, 0);
				/*Todo*/
				break;
			case ALARM:
				/*Todo*/
				break;
			case KEYPAD:
				/*Todo*/
				break;
		}
		
		display_message(0, 0);
    }
}


/* Displays the string matching the message number. Also displays password as '*' depending on password_length. */
void display_message(int message_number, int password_length) 
{
	/* Clearing the LCD before displaying new message. */
	lcd_clrscr();
	switch (message_number)
	{
		case 1:
			lcd_puts(message1_connection_established);
			break;
		case 2:
			lcd_puts(message2_movement_detected);
			break;
		case 3:
			lcd_puts(message3_give_password);
			lcd_gotoxy(0,1);
			switch (password_length)
			{
				case 0:
				lcd_puts("Password: ");
				break;
				case 1:
				lcd_puts("Password: *");
				break;
				case 2:
				lcd_puts("Password: **");
				break;
				case 3:
				lcd_puts("Password: ***");
				break;
				case 4:
				lcd_puts("Password: ****");
				break;
				default:
				lcd_puts("Password: Error");
			}
			break;
		case 4:
			lcd_puts(message4_password_correct);
			break;
		case 5:
			lcd_puts(message5_password_time_out);
			break;
		default:
			lcd_puts("Unknown value passed to LCD function.");
	}

}

