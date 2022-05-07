/*
  This is a library written for the VEML6075 UVA/UVB/UV index Sensopr
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14748
  Written by Jim Lindblom @ SparkFun Electronics, May 23, 2018

  The VEML6075 senses UVA and UVB light, which allows for a calculation
  of the UV index.

  This library handles the initialization, configuration and monitoring of the
  UVA and UVB intensity, and calculation of the UV index.

  https://github.com/sparkfunX/SparkFun_VEML6075_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DEBUG_VEML6075

#include <Wire.h>

typedef uint16_t veml6075_t;

//  Valid VEML6075 addresses
typedef enum
{
    VEML6075_ADDRESS = 0x10,
    VEML6075_ADDRESS_INVALID = 0xFF
} VEML6075_Address_t;

// VEML6075 error code returns:
typedef enum
{
    VEML6075_ERROR_READ = -4,
    VEML6075_ERROR_WRITE = -3,
    VEML6075_ERROR_INVALID_ADDRESS = -2,
    VEML6075_ERROR_UNDEFINED = -1,
    VEML6075_ERROR_SUCCESS = 1
} VEML6075_error_t;
const VEML6075_error_t VEML6075_SUCCESS = VEML6075_ERROR_SUCCESS;

struct test
{
    float a;
    float b;
};

class VEML6075
{
public:
    enum class IntegrationTime : unsigned int
    {
        IT_50MS = 0,
        IT_100MS = 1,
        IT_200MS = 2,
        IT_400MS = 3,
        IT_800MS = 4,
    };

#pragma pack(1)
    struct Configuration
    {
        bool shutdown : 1;
        bool active_force : 1;
        bool trigger : 1;
        bool high_dynamic : 1;
        IntegrationTime integration_time : 3;
        uint8_t RESERVED : 1;
    };
    static_assert(sizeof(Configuration) == 1);
    static_assert(std::is_trivially_copyable_v<Configuration>);
#pragma pack()

    VEML6075();

    // begin initializes the Wire port and I/O expander
    bool begin(void);
    // give begin a TwoWire port to specify the I2C port
    VEML6075_error_t begin(TwoWire &wirePort);

    // setDebugStream to enable library debug statements
    void setDebugStream(Stream &debugPort = Serial);

    bool isConnected(void);

    VEML6075_error_t getConfiguration(Configuration *out);
    VEML6075_error_t setConfiguration(const Configuration &conf);

    VEML6075_error_t trigger(void);

    VEML6075_error_t powerOn(bool enable = true);
    VEML6075_error_t shutdown(bool shutdown = true);

    VEML6075_error_t get(float *uva, float *uvb, float *index, float *rawa = nullptr, float *rawb = nullptr, float *visible = nullptr, float *ir = nullptr);

    VEML6075_error_t deviceID(uint8_t *id);
    VEML6075_error_t deviceAddress(uint8_t *address);

private:
    float calcUva(const float rawa, const float visiblecomp, const float ircomp);
    float calcUvb(const float rawb, const float visiblecomp, const float ircomp);
    float calcUvindex(const float uva, const float uvb);

    void updateConfiguration(const Configuration &conf);

    VEML6075_error_t rawUva(uint16_t *out);
    VEML6075_error_t rawUvb(uint16_t *out);
    VEML6075_error_t visibleCompensation(uint16_t *out); // uvComp1
    VEML6075_error_t irCompensation(uint16_t *out);      // uvComp2

    // VEML6075 registers:
    typedef enum
    {
        REG_UV_CONF = 0x00,
        REG_UVA_DATA = 0x07,
        REG_UVB_DATA = 0x09,
        REG_UVCOMP1_DATA = 0x0A,
        REG_UVCOMP2_DATA = 0x0B,
        REG_ID = 0x0C
    } VEML6075_REGISTER_t;

    TwoWire *_i2cPort =
        nullptr; // The generic connection to user's chosen I2C hardware
    Stream *_debugPort = nullptr;
    VEML6075_Address_t _deviceAddress = VEML6075_ADDRESS_INVALID;

    unsigned int _integrationTime = 0;
    float _aResponsivity;
    float _bResponsivity;
    bool _hdEnabled = false;

    VEML6075_error_t _connected(void);

    // I2C Read/Write
    VEML6075_error_t
    readI2CBuffer(uint8_t *dest, VEML6075_REGISTER_t startRegister, uint16_t len);
    VEML6075_error_t
    writeI2CBuffer(uint8_t *src, VEML6075_REGISTER_t startRegister, uint16_t len);
    VEML6075_error_t readI2CRegister(veml6075_t *dest,
                                     VEML6075_REGISTER_t registerAddress);
    VEML6075_error_t writeI2CRegister(veml6075_t data,
                                      VEML6075_REGISTER_t registerAddress);
};