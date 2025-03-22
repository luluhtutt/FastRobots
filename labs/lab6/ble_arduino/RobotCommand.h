#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H

#define DELIM_SIZE 9

/**
 * Extract command type and values from a robot command string
 */
class RobotCommand
{
  protected:
    char* token_ptr;
    char delimiters[DELIM_SIZE];
    char cmd_array[MAX_MSG_SIZE];

  public:
    /**
     * Default Constructor
     * The delimiters are set to ":|", which are used to tokenize the robot command string
     */
    RobotCommand()
    {
        strcpy(delimiters, ":|");
    };

    /**
     * Constructor 
     * @param   Delimiters character array specifying the delimiters used to tokenize 
     *                      the robot command string
     */
    RobotCommand(char* Delimiters)
    {
        strcpy(delimiters, Delimiters);
    };

    /**
     * Constructor 
     * @param   Delimiters const character array or string literal specifying the delimiters
     *                      used to tokenize the robot command string
     */
    RobotCommand(const char* Delimiters)
    {
        strcpy(delimiters, Delimiters);
    };

    /**
     * Set the robot command string from the BLE characteristic value
     * Only the first MAX_MSG_SIZE-1 characters are copied into the array 
     * 
     * @param  msg_uint_array   a uint8 array containing the BLE characteristic value
     * @param  length           length of the uint8 array
     */
    void set_cmd_string(const uint8_t* msg_uint_array, int length)
    {
        if (length < MAX_MSG_SIZE) {
            for (int i = 0; i < length; i++) {
                cmd_array[i] = (char)msg_uint_array[i];
            }
            // valueLength does not count null character
            cmd_array[length] = '\0';
        } else {
            Serial.print("Robot Command string larger than MAX_MSG_SIZE=");
            Serial.print(MAX_MSG_SIZE);
            Serial.println(" bytes, copying only the first MAX_MSG_SIZE-1 bytes");

            for (int i = 0; i < MAX_MSG_SIZE; i++) {
                cmd_array[i] = (char)msg_uint_array[i];
            }
            cmd_array[MAX_MSG_SIZE - 1] = '\0';
        }
    }

    /**
     * Set the robot command string based on a const character array or string literal
     * Only the first MAX_MSG_SIZE-1 characters are copied into the array 
     * 
     * @param  msg_array a const character array or string literal
     */
    void set_cmd_string(const char* msg_array)
    {
        if (strlen(msg_array) < MAX_MSG_SIZE)
            strcpy(cmd_array, msg_array);
        else {
            //Ref: https://www.cplusplus.com/reference/cstring/strncpy/
            strncpy(cmd_array, msg_array, MAX_MSG_SIZE - 1);
            // May not be null terminated, so add one
            cmd_array[MAX_MSG_SIZE - 1] = '\0';
        }
    }

    /**
     * Extract the command type from the robot command string
     * By default, the delimiter ":" is used to mark the end of the command type substring
     *
     * @param  cmd_type integer variable storing the command type
     * @return a boolean indicating whether the operation was successful
     */
    bool get_command_type(int& cmd_type)
    {
        token_ptr = strtok(cmd_array, delimiters);
        if (token_ptr != NULL) {
            cmd_type = atoi(token_ptr);
            return true;
        } else {
            Serial.println("Invalid command type");
            return false;
        }
    }

    /**
     * Extract the next integer value from the robot command string.
     * By default, the delimiter "|" is used to mark the end of the value substring
     *
     * @param  value integer variable storing the next value
     * @return a boolean indicating whether the operation was successful
     */
    bool get_next_value(int& value)
    {
        token_ptr = strtok(NULL, delimiters);
        if (token_ptr != NULL) {
            value = atoi(token_ptr);
            return true;
        } else {
            Serial.println("ERROR: No more tokens");
            return false;
        }
    }

    /**
     * Extract the next float value from the robot command string.
     * By default, the delimiter "|" is used to mark the end of the value substring
     *
     * @param  value float variable storing the next value
     * @return a boolean indicating whether the operation was successful
     */
    bool get_next_value(float& value)
    {
        token_ptr = strtok(NULL, delimiters);
        if (token_ptr != NULL) {
            value = atof(token_ptr);
            return true;
        } else {
            Serial.println("ERROR: No more tokens");
            return false;
        }
    }

    /**
     * Extract the next string (character array) value from the robot command string
     * By default, the delimiter "|" is used to mark the end of the value substring
     *
     * @param  value char array storing the next value
     * @return a boolean indicating whether the operation was successful
     */
    bool get_next_value(char* char_arr)
    {
        token_ptr = strtok(NULL, delimiters);
        if (token_ptr != NULL) {
            strcpy(char_arr, token_ptr);
            return true;
        } else {
            Serial.println("ERROR: No more tokens");
            return false;
        }
    }
};

#endif // ROBOT_COMMAND_H