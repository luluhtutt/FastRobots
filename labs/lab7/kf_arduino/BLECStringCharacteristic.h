#ifndef _BLE_CSTRING_CHARACTERISTIC_H_
#define _BLE_CSTRING_CHARACTERISTIC_H_

#include <BLECharacteristic.h>

class BLECStringCharacteristic : public BLECharacteristic
{
  public:
    BLECStringCharacteristic(const char* uuid, unsigned char properties, int valueSize)
      : BLECharacteristic(uuid, properties, valueSize){};

    int writeValue(const char* value)
    {
        return BLECharacteristic::writeValue(value);
    }

    int setValue(const char* value)
    {
        return writeValue(value);
    }

    void valueInto(char* char_arr)
    {
        const uint8_t* val = BLECharacteristic::value();
        int length = BLECharacteristic::valueLength();

        for (int i = 0; i < length; i++) {
          char_arr[i] = (char)val[i];
        }

        // valueLength does not count null character
        char_arr[length] = '\0';
    }

    const uint8_t* value()
    {
        const uint8_t* val = BLECharacteristic::value();
        return val;
    }

  private:
};

#endif // _BLE_CSTRING_CHARACTERISTIC_H_