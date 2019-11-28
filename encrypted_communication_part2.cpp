/*
    CMPUT 274 Assignment #2 Part 2
    Mackenzie Malainey - 1570494
    Lora Ma - 1570935
*/
#include "Arduino.h"

#define ARDUINO_MODE_PIN 13

/**
 * Description:
 * Writes an uint32_t to Serial3, starting from the least - significant bit
 * and finishing with the most significant byte.
 * (FUNCTION PROVIDED FROM ASSIGNMENT INFORMATION)
 *
 * Arguments:
 * num (uint32_t): 32 bit unsigned integer to print to serial3
 */
void uint32_to_serial3 (uint32_t num)
{
    Serial3.write(num >> 0);
    Serial3.write(num >> 8);
    Serial3.write(num >> 16);
    Serial3.write(num >> 24);
}

/**
 * Description:
 * Reads an uint32_t from Serial3, starting from the least - significant
 * and finishing with the most significant byte.
 * (FUNCTION PROVIDED FROM ASSIGNMENT INFORMATION)
 *
 * Returns:
 * num (uint32_t): 32 bit unsigned integer read from Serial3
 */
uint32_t uint32_from_serial3()
{
    uint32_t num = 0;
    num = num | ((uint32_t)Serial3.read()) << 0;
    num = num | ((uint32_t)Serial3.read()) << 8;
    num = num | ((uint32_t)Serial3.read()) << 16;
    num = num | ((uint32_t)Serial3.read()) << 24;
    return num ;

}

/**
 * Description:
 * Sets up modules and configures necessary pins
 */
void setup()
{
    // Initialize arduino and serial modules, as well as pins
    init();
    Serial.begin(9600);
    Serial3.begin(9600);

    pinMode(ARDUINO_MODE_PIN, INPUT);
}

/**
 * Description:
 * Performs modular multiplication [formula: (a * b) % m] using 32 bit integers
 * 
 * Parameters:
 * a (uint32_t): multiplicand
 * b (uint32_t): multiplier
 * m (uint32_t): modulus
 * 
 * Returns:
 * result (uint32_t): result of the modular multiplication
 */
uint32_t mulmod(uint32_t a, uint32_t b, uint32_t m)
{
    uint32_t n = 1;
    uint32_t ans = 0;
    uint32_t last_n = 1;
    b = b % m;
    while (n <= a)
    {
        if ((n & a) > 0)
        {
        while (last_n < n)
        {
            b = (2*b) % m;
            last_n <<= 1;
        }
        ans = (ans + b) % m;
        }
        n <<= 1;
    }
    return ans;
}

/**
 * Description:
 * Performs fast modular exponentiation [formula: ((base)^power) % mod]
 *
 * Arguments:
 * base (uint32_t): base for exponentation
 * power (uint32_t): exponent for exponentation
 * mod (uint32_t): number to perform modulus around
 *
 * Returns:
 * ans (uint32_t): Result of the expression ((base)^power) % mod
 */
uint32_t powmod(uint32_t base, uint32_t power, uint32_t mod)
{
    uint32_t ans = 1;
    uint32_t pow_x = base % mod;

    while (power > 0)
    {
        if (power & 1 == 1)
        {
            ans = mulmod(pow_x, ans, mod);
        }

        pow_x = mulmod(pow_x, pow_x, mod);

        power >>= 1;
    }

    return ans;
}

/**
 * Description:
 * Main entry point of the program
 */
int main(){

    setup();

    while(true)
    {
        if (Serial.available() > 0)
        {
            // Read from computer input
            char input = Serial.read();

            // Encrypt byte
            if (input == '\r' )
            {
                Serial.println();
                uint32_t encryptedR = powmod('\r', e, m);
                uint32_to_serial3(encryptedR);
                uint32_t encryptedN = powmod('\n', e, m);
                uint32_to_serial3(encryptedN);
            } else {
                Serial.print(input);
                uint32_t encrypted = powmod(input, e, m);
                uint32_to_serial3(encrypted);
            }
        }

        if (Serial3.available() > 3)
        {
            uint32_t read_input = uint32_from_serial3();
            char decrypted = (char)powmod(read_input, d, n);
            Serial.print(decrypted);
        }
    }

    Serial.flush();
    Serial3.flush();
    return 0;
}
