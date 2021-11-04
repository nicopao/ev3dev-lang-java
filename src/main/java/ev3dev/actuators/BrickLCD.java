package ev3dev.actuators;

import lejos.hardware.lcd.GraphicsLCD;

/**
 * Interface that extends GraphicsLCD to provide APIs for easy 
 * string printing to LCD, without having to implement TextLCD interface.
 * @author nicopao
 */

public interface BrickLCD extends GraphicsLCD {

	/**
	 * Prints a string on LCD. 
	 * By default:
	 * <ul>
	 * <li>anchor point is TOP-LEFT</li>
	 * <li>text color is black</li>
	 * <li>bg color is white</li> 
	 * </ul>
	 * @param str - string to print
	 * @param x - horizontal position in characters
	 * @param y - vertical position in lines
	 */
	void drawString(String str, int x, int y);
	
	/**
	 * Prints a string on LCD. 
	 * By default, anchor point is TOP-LEFT
	 * @param str - string to print
	 * @param x - horizontal position in characters
	 * @param y - vertical position in lines
	 * @param inverted - if false, print black text on white bg; 
	 * 	if true, white text on black bg 
	 */
	void drawString(String str, int x, int y, boolean inverted);
}
