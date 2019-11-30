/*
    CMPUT 274 Assignment #2 Part 2
    Mackenzie Malainey - 1570494
    Lora Ma - 1570935
*/
#include "Arduino.h"

#define ARDUINO_MODE_PIN 13


struct RsaKey
{
    uint32_t privateKey;
    uint32_t publicKey;
    uint32_t modulus;
};

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
        if ((power & 1) == 1)
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
 * Generates a random k-bit number (up to 32 bits)
 *
 * Arguments:
 * k (uint8_t): How many bits the number should be
 *
 * Returns:
 * K-bit number (uint32_t): generated k-bit number
 */
uint32_t generateNumber(uint8_t k)
{
    uint32_t num = 0;
    for (uint8_t i = 0; i < k - 1; i++)
    {
        num |= ((uint32_t)(analogRead(1) & 1)) << i;
    }

    num += 1ul << k;
    return num;
}

/**
 *
 */
bool isPrime(uint32_t num)
{
    if(num % 2 == 0)
    {
        return false;
    }

    for (uint32_t p = 3; p*p <= num; p += 2)
    {
        if (num % p == 0)
        {
            return false;
        }
    }

    return true;
}

/**
 * Description:
 * Find greatest common divisor (GCD) of two numbers
 *
 * Arguments:
 * a, b (uint32_t): Numbers to find GCD of.
 *
 * Returns:
 * gcd (uint32_t): GCD of a and b
 */
uint32_t gcd(uint32_t a, uint32_t b, int32_t& d)
{
    int32_t s_pre = 1;
    int32_t s = 0;

    int32_t t_pre = 0;
    int32_t t = 1;

    int32_t r_pre = a;
    int32_t r = b;

    while(r > 0)
    {
        int32_t q = r_pre / r;

        int32_t r_next = r_pre - q * r;
        int32_t s_next = s_pre - q * s;
        int32_t t_next = t_pre - q * t;

        r_pre = r;
        s_pre = s;
        t_pre = t;

        r = r_next;
        s = s_next;
        t = t_next;
    }

    d = s_pre;

    return a * s_pre + b * t_pre;
}

uint32_t good_gcd(uint32_t a, uint32_t b)
{
    while (b > 0)
    {
        a = a % b;
        unsigned int x = a;
        a = b;
        b = x;
    }
    return a;
}

int32_t reduce_mod(int32_t x, uint32_t m)
{
    if (x < 0)
    {
        x += ((-x / m) + 1)*m;
    }
    return x % m;
}

/**
 * Description:
 * Generates RSA encryption keys and modulus
 *
 * Returns:
 * RSA info (RsaKey): A struct containing the RSA private and public keys, and modulus
 */
RsaKey generateKey()
{
    RsaKey key;

    uint32_t p;
    do
    {
        p = generateNumber(14);
    } while (!isPrime(p));

    uint32_t q;
    do
    {
        q = generateNumber(15);
    } while (!isPrime(q));

    key.modulus = p*q;

    uint32_t phi_n = (p - 1)*(q - 1);

    int32_t d = 0;
    uint32_t e;
    do
    {
        e = generateNumber(15);
    } while (gcd(e, phi_n, d) != 1);

    d = reduce_mod(d, phi_n);

    key.publicKey = e;
    key.privateKey = (uint32_t)d;

    return key;
}
/*
* Description:
* Client message processing
*/
void dataEx(RsaKey mykeys, RsaKey theirkeys) {
    if (Serial.available() > 0){
       // Read from computer input
       char input = Serial.read();

       // Encrypt byte
       if (input == '\r' )
       {
           Serial.println();
           uint32_t encryptedR = powmod('\r', theirkeys.publicKey, theirkeys.modulus);
           uint32_to_serial3(encryptedR);
           uint32_t encryptedN = powmod('\n', theirkeys.publicKey, theirkeys.modulus);
           uint32_to_serial3(encryptedN);
       } else {
           Serial.print(input);
           uint32_t encrypted = powmod(input, theirkeys.publicKey, theirkeys.modulus);
           uint32_to_serial3(encrypted);
       }
    }
    if (Serial3.available() > 3)
    {
        uint32_t read_input = uint32_from_serial3();
        char decrypted = (char)powmod(read_input, mykeys.privateKey, mykeys.modulus);
        Serial.print(decrypted);
    }
}


/*
* Description:
* Waits for a certain number of bytes on Serial 3 or timeout
*
* Arguments:
* nbytes: the number of bytes we want
* timeout: timeout period (ms); specifying a negaitve number turns off timeouts
*
* Returns:
* true if the required number of bytes have arrived
*/
bool wait_on_serial3( uint8_t  nbytes , long  timeout )
{
    unsigned  long  deadline = millis () + timeout;// wraparound  not a problem
    while (Serial3.available()<nbytes  && (timeout <0 || millis()<deadline))
    {
        delay (1); // be nice , no busy  loop
    }
    return  Serial3.available () >=nbytes;
}
/**
 * Description:
 * Main entry point of the program
 */
int main(){
    setup();
    enum states {start, waitingForKey, waitingForAck, dataExchange};
    states currentState = start;

    RsaKey myKeys = generateKey();
    RsaKey theirKeys;


    if (digitalRead(ARDUINO_MODE_PIN) == LOW) {

        while(true) {
            bool exchanged;
            switch(currentState) {
                case start:
                    Serial3.write('C');
                    uint32_to_serial3(myKeys.publicKey);
                    uint32_to_serial3(myKeys.modulus);
                    currentState = waitingForAck;
                    exchanged = false;
                    break;
                case waitingForAck:
                    if(wait_on_serial3(9, 1000)) {
                        if((char)Serial3.read() == 'A') {
                            theirKeys.publicKey = uint32_from_serial3();
                            theirKeys.modulus = uint32_from_serial3();

                            Serial3.write('A');
                            exchanged = true;
                        }
                    }
                    if(exchanged) {
                        currentState = dataExchange;
                    }
                    else {
                        currentState = start;
                    }
                    break;

                case dataExchange:
                    dataEx(myKeys, theirKeys);
                    break;

                default:
                    currentState = start;
                    break;

            }
        }
    }
    else {

        while(true) {
            switch(currentState) {
                case start:
                    if(Serial3.available()) {
                        if((char)Serial3.read() == 'C') {
                            currentState = waitingForKey;
                        }
                    }
                    break;
                case waitingForKey:

                    if(wait_on_serial3(8, 1000)) {
                        theirKeys.publicKey = uint32_from_serial3();
                        theirKeys.modulus = uint32_from_serial3();
                        Serial3.write('A');

                        uint32_to_serial3(myKeys.publicKey);
                        uint32_to_serial3(myKeys.modulus);
                        currentState = waitingForAck;
                    } else {
                        currentState = start;
                    }
                    break;
                case waitingForAck:

                if(wait_on_serial3(1, 1000)) {
                    if(Serial3.read() == 'A') {
                        currentState = dataExchange;
                    }
                    else if(Serial3.read() == 'C') {

                        currentState = waitingForKey;
                    }
                    else {
                        currentState = start;

                    }
                    break;
                case dataExchange:

                    dataEx(myKeys, theirKeys);
                    break;
                default:
                    currentState = start;
                    break;
                }

            }
        }

    }
    Serial.flush();
    Serial3.flush();
    return 0;
}
