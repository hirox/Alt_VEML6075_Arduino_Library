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

#include <SparkFun_VEML6075_Arduino_Library.h>

#ifdef DEBUG_VEML6075
#define VEML6075_DEBUG(x)     \
    if (_debugPort != NULL)   \
    {                         \
        _debugPort->print(x); \
    }
#define VEML6075_DEBUGLN(x)     \
    if (_debugPort != NULL)     \
    {                           \
        _debugPort->println(x); \
    }
#define STORAGE(x) (x)
#else
#define VEML6075_DEBUG(x)
#define VEML6075_DEBUGLN(x)
#define STORAGE(x) (x)
#endif

#define VEML6075_REGISTER_LENGTH 2 // 2 bytes per register
#define NUM_INTEGRATION_TIMES 5

#define VEML6075_DEVICE_ID 0x26

const float HD_SCALAR = 2.0;

const float UV_ALPHA = 1.0;
const float UV_BETA = 1.0;
const float UV_GAMMA = 1.0;
const float UV_DELTA = 1.0;

const float UVA_A_COEF = 2.22F;
const float UVA_B_COEF = 1.33F;
const float UVA_C_COEF = 2.95F;
const float UVA_D_COEF = 1.74F;

const float UVA_RESPONSIVITY_100MS_UNCOVERED = 0.001111;
const float UVB_RESPONSIVITY_100MS_UNCOVERED = 0.00125;

const float UVA_RESPONSIVITY[NUM_INTEGRATION_TIMES] = {
    UVA_RESPONSIVITY_100MS_UNCOVERED / 0.5016286645, // 50ms
    UVA_RESPONSIVITY_100MS_UNCOVERED,                // 100ms
    UVA_RESPONSIVITY_100MS_UNCOVERED / 2.039087948,  // 200ms
    UVA_RESPONSIVITY_100MS_UNCOVERED / 3.781758958,  // 400ms
    UVA_RESPONSIVITY_100MS_UNCOVERED / 7.371335505   // 800ms
};

const float UVB_RESPONSIVITY[NUM_INTEGRATION_TIMES] = {
    UVB_RESPONSIVITY_100MS_UNCOVERED / 0.5016286645, // 50ms
    UVB_RESPONSIVITY_100MS_UNCOVERED,                // 100ms
    UVB_RESPONSIVITY_100MS_UNCOVERED / 2.039087948,  // 200ms
    UVB_RESPONSIVITY_100MS_UNCOVERED / 3.781758958,  // 400ms
    UVB_RESPONSIVITY_100MS_UNCOVERED / 7.371335505   // 800ms
};

VEML6075::VEML6075()
{
    _aResponsivity = UVA_RESPONSIVITY_100MS_UNCOVERED;
    _bResponsivity = UVB_RESPONSIVITY_100MS_UNCOVERED;
}

bool VEML6075::begin(void) { return begin(Wire) == VEML6075_ERROR_SUCCESS; }

VEML6075_error_t VEML6075::begin(TwoWire &wirePort)
{
    uint8_t systemControl = 0;
    VEML6075_error_t err;

    _deviceAddress = VEML6075_ADDRESS;
    _i2cPort = &wirePort;
    _i2cPort->begin();

    err = _connected();
    if (err != VEML6075_ERROR_SUCCESS)
    {
        return err;
    }

    Configuration conf;
    err = getConfiguration(&conf);
    if (err != VEML6075_ERROR_SUCCESS)
        return err;

    // Power on and default settings
    conf.shutdown = false;
    conf.integration_time = IntegrationTime::IT_100MS;
    conf.high_dynamic = false;
    conf.active_force = false;
    return setConfiguration(conf);
}

void VEML6075::setDebugStream(Stream &debugPort) { _debugPort = &debugPort; }

bool VEML6075::isConnected(void)
{
    return _connected() == VEML6075_ERROR_SUCCESS;
}

VEML6075_error_t VEML6075::getConfiguration(Configuration *out)
{
    veml6075_t reg;
    auto err = readI2CRegister(&reg, VEML6075::REG_UV_CONF);
    if (err == VEML6075_ERROR_SUCCESS)
    {
        *reinterpret_cast<uint8_t *>(out) = reg & 0xFF;
        updateConfiguration(*out);
    }
    return err;
}

VEML6075_error_t VEML6075::setConfiguration(const Configuration &conf)
{
    veml6075_t reg = *reinterpret_cast<const uint8_t *>(&conf);
    auto err = writeI2CRegister(reg, VEML6075::REG_UV_CONF);
    if (err == VEML6075_ERROR_SUCCESS)
        updateConfiguration(conf);
    return err;
}

void VEML6075::updateConfiguration(const Configuration &conf)
{
    _aResponsivity = UVA_RESPONSIVITY[static_cast<uint8_t>(conf.integration_time)];
    _bResponsivity = UVB_RESPONSIVITY[static_cast<uint8_t>(conf.integration_time)];
    _last_conf = conf;
}

VEML6075_error_t VEML6075::shutdown(bool shutdown)
{
    Configuration conf;
    VEML6075_error_t err = getConfiguration(&conf);
    if (err != VEML6075_ERROR_SUCCESS)
        return err;

    conf.shutdown = shutdown;

    return setConfiguration(conf);
}

VEML6075_error_t VEML6075::trigger(void)
{
    Configuration conf;
    VEML6075_error_t err = getConfiguration(&conf);
    if (err != VEML6075_ERROR_SUCCESS)
        return err;

    conf.trigger = true;

    return setConfiguration(conf);
}

float VEML6075::calcUva(const float rawa, const float visiblecomp, const float ircomp)
{
    return rawa -
           ((UVA_A_COEF * UV_ALPHA * visiblecomp) / UV_GAMMA) -
           ((UVA_B_COEF * UV_ALPHA * ircomp) / UV_DELTA);
}

float VEML6075::calcUvb(const float rawb, const float visiblecomp, const float ircomp)
{
    return rawb -
           ((UVA_C_COEF * UV_BETA * visiblecomp) / UV_GAMMA) -
           ((UVA_D_COEF * UV_BETA * ircomp) / UV_DELTA);
}

float VEML6075::calcUvindex(const float uva, const float uvb)
{
    float uvia = uva * (1.F / UV_ALPHA) * _aResponsivity;
    float uvib = uvb * (1.F / UV_BETA) * _bResponsivity;
    auto index = (uvia + uvib) / 2.F;
    if (_last_conf.high_dynamic)
    {
        index *= HD_SCALAR;
    }

    return index;
}

VEML6075_error_t VEML6075::get(float *uva, float *uvb, float *uvindex, float *rawa, float *rawb, float *visible, float *ir)
{
    VEML6075_error_t err;
    uint16_t ra;
    uint16_t rb;
    uint16_t viscomp;
    uint16_t ircomp;

    err = rawUva(&ra);
    if (err != VEML6075_ERROR_SUCCESS)
        return err;
    err = rawUvb(&rb);
    if (err != VEML6075_ERROR_SUCCESS)
        return err;
    err = visibleCompensation(&viscomp);
    if (err != VEML6075_ERROR_SUCCESS)
        return err;
    err = irCompensation(&ircomp);
    if (err != VEML6075_ERROR_SUCCESS)
        return err;

    float a = calcUva(ra, viscomp, ircomp);
    float b = calcUvb(rb, viscomp, ircomp);

    if (uva)
        *uva = a;
    if (uvb)
        *uvb = b;
    if (uvindex)
        *uvindex = calcUvindex(a, b);
    if (rawa)
        *rawa = ra;
    if (rawb)
        *rawb = rb;
    if (visible)
        *visible = viscomp;
    if (ir)
        *ir = ircomp;
    return VEML6075_ERROR_SUCCESS;
}

VEML6075_error_t VEML6075::rawUva(uint16_t *out)
{
    return readI2CRegister(out, VEML6075::REG_UVA_DATA);
}

VEML6075_error_t VEML6075::rawUvb(uint16_t *out)
{
    return readI2CRegister(out, VEML6075::REG_UVB_DATA);
}

VEML6075_error_t VEML6075::visibleCompensation(uint16_t *out)
{
    return readI2CRegister(out, VEML6075::REG_UVCOMP1_DATA);
}

VEML6075_error_t VEML6075::irCompensation(uint16_t *out)
{
    return readI2CRegister(out, VEML6075::REG_UVCOMP2_DATA);
}

VEML6075_error_t VEML6075::_connected(void)
{
    uint8_t id;
    VEML6075_error_t err = deviceID(&id);
    if (err != VEML6075_ERROR_SUCCESS)
    {
        VEML6075_DEBUGLN(("Connect err: " + String(err)));
        return err;
    }
    if (id != VEML6075_DEVICE_ID)
    {
        VEML6075_DEBUGLN(("Connect read err: " + String(id)));
        return VEML6075_ERROR_INVALID_ADDRESS;
    }
    VEML6075_DEBUGLN(("Connect success!"));

    return VEML6075_ERROR_SUCCESS;
}

VEML6075_error_t VEML6075::deviceID(uint8_t *id)
{
    VEML6075_error_t err;
    veml6075_t devID = 0;

    err = readI2CRegister(&devID, VEML6075::REG_ID);

    if (err != VEML6075_ERROR_SUCCESS)
    {
        return err;
    }
    VEML6075_DEBUGLN(("Device ID: " + String(devID, HEX)));
    *id = (uint8_t)(devID & 0x00FF);
    return err;
}

VEML6075_error_t VEML6075::readI2CBuffer(uint8_t *dest,
                                         VEML6075_REGISTER_t startRegister,
                                         uint16_t len)
{
    VEML6075_DEBUGLN((STORAGE("(readI2CBuffer): len ") + String(len) +
                      STORAGE(" @ reg 0x") + String(startRegister, HEX)));
    _i2cPort->beginTransmission((uint8_t)_deviceAddress);
    _i2cPort->write(startRegister);
    if (_i2cPort->endTransmission(false) != 0)
    {
        VEML6075_DEBUGLN(STORAGE("    ERR (readI2CBuffer): End transmission"));
        return VEML6075_ERROR_READ;
    }

    _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)len);
    for (int i = 0; i < len; i++)
    {
        dest[i] = _i2cPort->read();
        VEML6075_DEBUGLN(
            (STORAGE("    ") + String(i) + STORAGE(": 0x") + String(dest[i], HEX)));
    }

    return VEML6075_ERROR_SUCCESS;
}

VEML6075_error_t VEML6075::writeI2CBuffer(const uint8_t *src,
                                          VEML6075_REGISTER_t startRegister,
                                          uint16_t len)
{
    VEML6075_DEBUGLN((STORAGE("(writeI2CBuffer): len ") + String(len) +
                      STORAGE(" @ reg 0x") + String(startRegister, HEX)));
    _i2cPort->beginTransmission((uint8_t)_deviceAddress);
    _i2cPort->write(startRegister);
    for (int i = 0; i < len; i++)
    {
        _i2cPort->write(src[i]);
        VEML6075_DEBUGLN(
            (STORAGE("    ") + String(i) + STORAGE(": 0x") + String(src[i], HEX)));
    }
    if (_i2cPort->endTransmission(true) != 0)
    {
        return VEML6075_ERROR_WRITE;
    }
    return VEML6075_ERROR_SUCCESS;
}

VEML6075_error_t
VEML6075::readI2CRegister(veml6075_t *dest,
                          const VEML6075_REGISTER_t registerAddress)
{
    VEML6075_error_t err;
    uint8_t tempDest[2];
    err = readI2CBuffer(tempDest, registerAddress, VEML6075_REGISTER_LENGTH);
    if (err == VEML6075_ERROR_SUCCESS)
    {
        *dest = (tempDest[0]) | ((veml6075_t)tempDest[1] << 8);
    }
    return err;
}

VEML6075_error_t
VEML6075::writeI2CRegister(const veml6075_t data,
                           const VEML6075_REGISTER_t registerAddress)
{
    uint8_t d[2];
    // Write LSB first
    d[0] = (uint8_t)(data & 0x00FF);
    d[1] = (uint8_t)((data & 0xFF00) >> 8);
    return writeI2CBuffer(d, registerAddress, VEML6075_REGISTER_LENGTH);
}