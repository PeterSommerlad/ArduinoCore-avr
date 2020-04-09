/*
 WCharacter.h - Character utility functions for Wiring & Arduino
 Copyright (c) 2010 Hernando Barragan.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef Character_h
#define Character_h

#include <ctype.h>

// PS: improve safety by only accepting char and casting to unsigned char
// int values out of range would cause UB anyway...

// WCharacter.h prototypes
inline bool isAlphaNumeric(char c) __attribute__((always_inline));
inline bool isAlpha(char c) __attribute__((always_inline));
inline bool isAscii(char c) __attribute__((always_inline));
inline bool isWhitespace(char c) __attribute__((always_inline));
inline bool isControl(char c) __attribute__((always_inline));
inline bool isDigit(char c) __attribute__((always_inline));
inline bool isGraph(char c) __attribute__((always_inline));
inline bool isLowerCase(char c) __attribute__((always_inline));
inline bool isPrintable(char c) __attribute__((always_inline));
inline bool isPunct(char c) __attribute__((always_inline));
inline bool isSpace(char c) __attribute__((always_inline));
inline bool isUpperCase(char c) __attribute__((always_inline));
inline bool isHexadecimalDigit(char c) __attribute__((always_inline));
inline char toAscii(char c) __attribute__((always_inline));
inline char toLowerCase(char c) __attribute__((always_inline));
inline char toUpperCase(char c)__attribute__((always_inline));


// Checks for an alphanumeric character. 
// It is equivalent to (isalpha(c) || isdigit(c)).
inline bool isAlphaNumeric(char c)
{
  return  isalnum(static_cast<unsigned char>(c));
}


// Checks for an alphabetic character. 
// It is equivalent to (isupper(c) || islower(c)).
inline bool isAlpha(char c)
{
  return  isalpha(static_cast<unsigned char>(c));
}


// Checks whether c is a 7-bit unsigned char value 
// that fits into the ASCII character set.
inline bool isAscii(char c)
{
  return  isascii(static_cast<unsigned char>(c));
}


// Checks for a blank character, that is, a space or a tab.
inline bool isWhitespace(char c)
{
  return  isblank (static_cast<unsigned char>(c));
}


// Checks for a control character.
inline bool isControl(char c)
{
  return  iscntrl (static_cast<unsigned char>(c));
}


// Checks for a digit (0 through 9).
inline bool isDigit(char c)
{
  return  isdigit (static_cast<unsigned char>(c));
}


// Checks for any printable character except space.
inline bool isGraph(char c)
{
  return  isgraph (static_cast<unsigned char>(c));
}


// Checks for a lower-case character.
inline bool isLowerCase(char c)
{
  return islower (static_cast<unsigned char>(c));
}


// Checks for any printable character including space.
inline bool isPrintable(char c)
{
  return  isprint (static_cast<unsigned char>(c));
}


// Checks for any printable character which is not a space 
// or an alphanumeric character.
inline bool isPunct(char c)
{
  return  ispunct (static_cast<unsigned char>(c));
}


// Checks for white-space characters. For the avr-libc library, 
// these are: space, formfeed ('\f'), newline ('\n'), carriage 
// return '\r'), horizontal tab ('\t'), and vertical tab ('\v').
inline bool isSpace(char c)
{
  return  isspace (static_cast<unsigned char>(c));
}


// Checks for an uppercase letter.
inline bool isUpperCase(char c)
{
  return  isupper (static_cast<unsigned char>(c));
}


// Checks for a hexadecimal digits, i.e. one of 0 1 2 3 4 5 6 7 
// 8 9 a b c d e f A B C D E F.
inline bool isHexadecimalDigit(char c)
{
  return  isxdigit (static_cast<unsigned char>(c));
}


// Converts c to a 7-bit unsigned char value that fits into the 
// ASCII character set, by clearing the high-order bits.
inline char toAscii(char c)
{
  return toascii(static_cast<unsigned char>(c));
}


// Warning:
// Many people will be unhappy if you use this function. 
// This function will convert accented letters into random 
// characters.

// Converts the letter c to lower case, if possible.
inline char toLowerCase(char c)
{
  return tolower(static_cast<unsigned char>(c));
}


// Converts the letter c to upper case, if possible.
inline char toUpperCase(char c)
{
  return toupper(static_cast<unsigned char>(c));
}

#endif
