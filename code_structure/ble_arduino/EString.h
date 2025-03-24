#ifndef ESTRING_STRING_H
#define ESTRING_STRING_H

#define MAX_MSG_SIZE 151 // +1 to account for null terminal

/**
 * A character array with functions to set, append and empty the array contents
 */
class EString
{
  public:
    char char_array[MAX_MSG_SIZE];
    char temp_array[40]; // Account for max of int32, '-' and '\0'

    EString()
    {
    };

    /**
     * Empty the contents of the character array
     */
    void clear()
    {
        char_array[0] = '\0';
    }

    /**
     * Get the character array
     *
     * @return character array
     */
    const char* c_str()
    {
        return char_array;
    }

    /**
     * Set the character array
     *
     * @param value_array a const character array or string literal
     */
    void set(const char* value_array)
    {
        strcpy(char_array, value_array);
    }

    /**
     * Set the character array
     * 
     * @param  msg_uint_array   a uint8 array containing the BLE characteristic value
     * @param  length           length of the uint8 array
     */
    void set(const uint8_t* uint_value_array, int length)
    {
        for (int i = 0; i < length; i++) {
            char_array[i] = (char)uint_value_array[i];
        }
        // valueLength does not count NULL character
        char_array[length] = '\0';
    }

    /**
     * Get the length of the character array
     *
     * @return length of the character array
     */
    int get_length()
    {
        return strlen(char_array);
    }

    /**
     * Append an interger to the character array
     *
     * @param value integer value to append
     */
    void append(int value)
    {
        itoa(value, temp_array, 10);
        strcat(char_array, temp_array);
    }

    /**
     * Append a string literal to the character array
     *
     * @param value const character array or string literal to append
     */
    void append(const char* array)
    {
        strcat(char_array, array);
    }

    /**
     * Append a character array to the given character array
     *
     * @param value character array to append
     */
    void append(char* array)
    {
        strcat(char_array, array);
    }

    /**
     * Append a float to the character array
     *
     * @param value float value to append
     */
    void append(float value)
    {
        int integer_part = (int)(value);
        int decimal_part = 1000 * abs(value - integer_part); //10^3 for 3 decimal places

        // Account for negative values
        if (value < 0) {
            strcat(char_array, "-");
        }

        // Write absolute integer value
        append(abs(integer_part));
        strcat(char_array, ".");

        // Account for leading zeros
        if (decimal_part < 100) {
            append(0);
        }
        if (decimal_part < 10) {
            append(0);
        }

        // Write decimal value
        append(decimal_part);
    }

    /**
     * Append a double to the character array
     *
     * @param value double value to append
     */
    void append(double value)
    {
        int integer_part = (int)(value);
        int decimal_part = 1000 * abs(value - integer_part); //10^3 for 3 decimal places

        // Account for negative values
        if (value < 0) {
            strcat(char_array, "-");
        }

        // Write absolute integer value
        append(abs(integer_part));
        strcat(char_array, ".");

        // Account for leading zeros
        if (decimal_part < 100) {
            append(0);
        }
        if (decimal_part < 10) {
            append(0);
        }

        // Write decimal value
        append(decimal_part);
    }
};

#endif // ESTRING_STRING_H
