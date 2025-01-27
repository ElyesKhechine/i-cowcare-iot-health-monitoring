/** Based on ST MicroElectronics LIS2DH datasheet http://www.st.com/web/en/resource/technical/document/datasheet/DM00042751.pdf
* 18/06/2014 by Conor Forde <me@conorforde.com>
* Updates should be available at https://github.com/Snowda/LIS2DH
*
* Changelog:
*     ... - ongoing development release
*     ... - May 2018 - Update by Disk91 / Paul Pinault to make it working
*           maintained at https://github.com/disk91/LIS2DH
* NOTE: THIS IS ONLY A PARIAL RELEASE. 
* THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE DEVELOPMENT AND IS MISSING MOST FEATURES. 
* PLEASE KEEP THIS IN MIND IF YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.
*/

#ifndef _LIS2DH_H_
#define _LIS2DH_H_

//Registers
#define LIS2DH_STATUS_REG_AUX   0x07
#define LIS2DH_OUT_TEMP_L     0x0C
#define LIS2DH_OUT_TEMP_H     0x0D
#define LIS2DH_INT_COUNTER_REG  0x0E
#define LIS2DH_WHO_AM_I     0x0F
#define LIS2DH_CTRL_REG0      0x1E        // Not documented register : bit 1 => connect / disconnect SDO,SA0 internal pullup
#define LIS2DH_TEMP_CFG_REG   0x1F
#define LIS2DH_CTRL_REG1    0x20
#define LIS2DH_CTRL_REG2    0x21
#define LIS2DH_CTRL_REG3    0x22
#define LIS2DH_CTRL_REG4    0x23
#define LIS2DH_CTRL_REG5    0x24
#define LIS2DH_CTRL_REG6    0x25
#define LIS2DH_REFERENCE    0x26
#define LIS2DH_STATUS_REG2    0x27
#define LIS2DH_OUT_X_L      0x28
#define LIS2DH_OUT_X_H      0x29
#define LIS2DH_OUT_Y_L      0x2A
#define LIS2DH_OUT_Y_H      0x2B
#define LIS2DH_OUT_Z_L      0x2C
#define LIS2DH_OUT_Z_H      0x2D
#define LIS2DH_FIFO_CTRL_REG  0x2E
#define LIS2DH_FIFO_SRC_REG   0x2F
#define LIS2DH_INT1_CFG     0x30
#define LIS2DH_INT1_SOURCE    0x31
#define LIS2DH_INT1_THS     0x32
#define LIS2DH_INT1_DURATION  0x33
#define LIS2DH_INT2_CFG      0x34
#define LIS2DH_INT2_SOURCE    0x35
#define LIS2DH_INT2_THS     0x36
#define LIS2DH_INT2_DURATION  0x37
#define LIS2DH_CLICK_CFG    0x38
#define LIS2DH_CLICK_SRC    0x39
#define LIS2DH_CLICK_THS    0x3A
#define LIS2DH_TIME_LIMIT     0x3B
#define LIS2DH_TIME_LATENCY   0x3C
#define LIS2DH_TIME_WINDOW    0x3D
#define LIS2DH_ACT_THS      0x3E
#define LIS2DH_ACT_DUR      0x3F

//Register Masks

//STATUS_AUX_REG masks
#define LIS2DH_TOR_MASK     0x40
#define LIS2DH_TDA_MASK     0x04

//INT_COUNTER masks
//what goes here?

//WHO_AM_I masks
#define LIS2DH_I_AM_VALUE   0x33

// TEMP_CFG_REG masks
#define LIS2DH_TEMP_EN_MASK   0xC0

// CTRL_REG1 masks
#define LIS2DH_REG0_SA0PULLUP_MASK 0x80
#define LIS2DH_REG0_SA0PULLUP_ENABLE 0x00
#define LIS2DH_REG0_SA0PULLUP_DISABLE 0x80

// CTRL_REG1 masks
#define LIS2DH_ODR_MASK     0xF0
#define LIS2DH_LPEN_MASK    0x08
#define LIS2DH_Z_EN_MASK    0x04
#define LIS2DH_Y_EN_MASK    0x02
#define LIS2DH_X_EN_MASK    0x01
#define LIS2DH_XYZ_EN_MASK  0x07

#define LIS2DH_ODR_SHIFT      4
#define LIS2DH_ODR_POWER_DOWN 0x00
#define LIS2DH_ODR_1HZ        0x01
#define LIS2DH_ODR_10HZ       0x02
#define LIS2DH_ODR_25HZ       0x03
#define LIS2DH_ODR_50HZ       0x04
#define LIS2DH_ODR_100HZ      0x05
#define LIS2DH_ODR_200HZ      0x06
#define LIS2DH_ODR_400HZ      0x07
#define LIS2DH_ODR_1620HZ     0x08
#define LIS2DH_ODR_1344HZ     0x09
#define LIS2DH_ODR_5376HZ     0x09
#define LIS2DH_ODR_MAXVALUE   0x09


// CTRL_REG2 masks
#define LIS2DH_HPM_MASK         0xC0
#define LIS2DH_HPCF_MASK        0x30
#define LIS2DH_FDS_MASK         0x08
#define LIS2DH_HPCLICK_MASK     0x04
#define LIS2DH_HPIA2_MASK       0x02        // Apply filtering on interrupt 2
#define LIS2DH_HPIA1_MASK       0x01        // Apply filtering on interrupt 1

#define LIS2DH_HPM_SHIFT          6
#define LIS2DH_HPM_NORMAL_RESET   0x00      // In this mode - when reading on of the XL/XH_REFERENCE register the current acceleration is reset on the corresponding axe (manuela reset)
#define LIS2DH_HPM_REFSIG         0x01      // In this mode acceleration is the difference with the XL/XH_REFERENCE content for each axis
#define LIS2DH_HPM_NORMAL2        0x02      // In this mode I assume we have no filtering
#define LIS2DH_HPM_AUTORESET      0x03      // In this mode the interrupt event will reset the filter
#define LIS2DH_HPM_MAXVALUE       0x03

#define LIS2DH_HPCF_SHIFT         4
#define LIS2DH_HPCF_ODR_50        0x00      // F cut = ODR Freq / 50
#define LIS2DH_HPCF_ODR_100       0x01
#define LIS2DH_HPCF_ODR_9         0x02
#define LIS2DH_HPCF_ODR_400       0x03
#define LIS2DH_HPCF_MAXVALUE      0x03

// CTRL_REG3 masks
#define LIS2DH_I1_CLICK           0x80      // Interrupt on click on INT1
#define LIS2DH_I1_IA1             0x40      // Interrupt from IA1 on INT1
#define LIS2DH_I1_IA2             0x20      // Interrupt from IA2 on INT1
#define LIS3DH_I1_ZYXDA           0x10      // Any Data axis on INT1
#define LIS2DH_I1_WTM             0x04      // FiFo Watermark on INT1
#define LIS2DH_I1_OVERRUN         0x02      // FiFo Overrun on INT1
#define LIS2DH_I1_INTERRUPT_NONE  0x00

// CTRL_REG6 masks
#define LIS2DH_I2_MASK            0xF8       // Mask for interrupt
#define LIS2DH_I2_CLICK           0x80       // Click interrupt
#define LIS2DH_I2_IA1             0x40       // Interrupt 1 function
#define LIS2DH_I2_IA2             0x20       // Interrupt 2 function
#define LIS2DH_I2_BOOT            0x10       // Boot interrupt
#define LIS2DH_I2_ACTIVITY        0x08       // Activity interrupt
#define LIS2DH_I2_INTERRUPT_NONE  0x00
#define LIS2DH_INT_POLARITY       0x02        // Interupt polarity => 0 active high / 1 active low



// CTRL_REG4 masks
#define LIS2DH_BDU_MASK     0x80
#define LIS2DH_BLE_MASK     0x40
#define LIS2DH_FS_MASK      0x30
#define LIS2DH_HR_MASK      0x08
#define LIS2DH_ST_MASK      0x06
#define LIS2DH_SIM_MASK     0x01

#define LIS2DH_FS_SHIFT     4
#define LIS2DH_FS_SCALE_2G  0x00
#define LIS2DH_FS_SCALE_4G  0x01
#define LIS2DH_FS_SCALE_8G  0x02
#define LIS2DH_FS_SCALE_16G 0x03
#define LIS2DH_FS_MAXVALUE  0x03

#define LIS2DH_RESOLUTION_8B        0x01      // Different resolution mode => Low Power
#define LIS2DH_RESOLUTION_10B       0x02      // Normal mode (10b)
#define LIS2DH_RESOLUTION_12B       0x03      // High Resolution mode (12b)
#define LIS2DH_RESOLUTION_MAXVALUE  0x03


// CTRL_REG5 masks
#define LIS2DH_BOOT_MASK      0x80
#define LIS2DH_FIFO_EN_MASK   0x40
#define LIS2DH_LIR_INT1_MASK  0x08
#define LIS2DH_D4D_INT1_MASK  0x04
#define LIS2DH_LIR_INT2_MASK  0x02
#define LIS2DH_D4D_INT2_MASK  0x01


// REF masks
// none

// STATUS_REG masks
#define LIS2DH_STATUS_ZYXOR     0x80      // X, Y, Z data overrun => a new set of data has overwritten the previous set
#define LIS2DH_STATUS_ZOR       0x40      // Z overrun
#define LIS2DH_STATUS_YOR       0x20      // Y overrun
#define LIS2DH_STATUS_XOR       0x10      // X overrun
#define LIS2DH_STATUS_ZYXDA     0x08      // X, Y, Z data available => a new set of data is availbale
#define LIS2DH_STATUS_ZDA       0x04      // Z data available
#define LIS2DH_STATUS_YDA       0x02      // Y data available
#define LIS2DH_STATUS_XDA       0x01      // X data available

// FIFO_CTRL_REG masks
#define LIS2DH_FM_MASK          0xC0
#define LIS2DH_TR_MASK          0x20
#define LIS2DH_FTH_MASK         0x1F

#define LIS2DH_FM_SHIFT         6
#define LIS2DH_FM_BYPASS        0x00      // No FIFO at all, the acceleration are not stored in FIFO
#define LIS2DH_FM_FIFO          0x01      // FIFO is used until being full after that no more data are added until clearing it by switching to bypass
#define LIS2DH_FM_STREAM        0x02      // FIFO is used and when full the older data are replaced by the new one.
#define LIS2DH_FM_STREAMFIFO    0x03      // In this mode the Interrupt Generator will automatically swicth the mode from STREAM to FiFo
#define LIS2DH_FM_MAXVALUE      0x03    

#define LIS2DH_TR_SHIFT         5
#define LIS2DH_TR_INT1          0x00
#define LIS2DH_TR_INT2          0x01
#define LIS2DH_TR_MAXVALUE      0x01

#define LIS2DH_FTH_SHIFT        0
#define LIS2DH_FTH_MAXVALUE     32

// FIFO_SRC_REG masks
#define LIS2DH_WTM_MASK          0x80
#define LIS2DH_OVRN_FIFO_MASK    0x40
#define LIS2DH_EMPTY_MASK        0x20
#define LIS2DH_FSS_MASK          0x1F

// INT1/2_CFG masks
#define LIS2DH_AOI_MASK         0x80
#define LIS2DH_6D_MASK          0x40
#define LIS2DH_INT_MODE_MASK    0xC0
#define LIS2DH_ZHIE_MASK        0x20
#define LIS2DH_ZLIE_MASK        0x10
#define LIS2DH_YHIE_MASK        0x08
#define LIS2DH_YLIE_MASK        0x04
#define LIS2DH_XHIE_MASK        0x02
#define LIS2DH_XLIE_MASK        0x01
#define LIS2DH_INTEVENT_MASK    0x3F


#define LIS2DH_INT_MODE_SHIFT   6
#define LIS2DH_INT_MODE_OR      0x00      // If one of the event triggers, the interrupt is fired
#define LIS2DH_INT_MODE_AND     0x02      // When all the event have triggered, the inteerupt is fired
#define LIS2DH_INT_MODE_MOV     0x01      // Movement recognition => when the orientation move from and unknown zone to a known zone, duration is ODR
#define LIS2DH_INT_MODE_POS     0x03      // Interrupt fired when the orientation is in a known zone and stay until we stay in this zone => Position
#define LIS2DH_INT_MODE_MAXVALUE  0x03

#define LIS2DH_INTEVENT_SHIFT   0
#define LIS2DH_INTEVENT_Z_HIGH  0x20      // Fire interrupt on Z high event of direction recognotion
#define LIS2DH_INTEVENT_Z_LOW   0x10      // Fire interrupt on Z low event of direction recognotion
#define LIS2DH_INTEVENT_Y_HIGH  0x08      // Fire interrupt on Y high event of direction recognotion
#define LIS2DH_INTEVENT_Y_LOW   0x04      // Fire interrupt on Y low event of direction recognotion
#define LIS2DH_INTEVENT_X_HIGH  0x02      // Fire interrupt on X high event of direction recognotion
#define LIS2DH_INTEVENT_X_LOW   0x01      // Fire interrupt on X low event of direction recognotion
#define LIS2DH_INTEVENT_NONE    0x00      // No interrupt
#define LIS2DH_INTEVENT_ALL     0x3F      // No interrupt
#define LIS2DH_INTEVENT_MAXVALUE 0x3F


// INT1/2_SRC masks
#define LIS2DH_INT_IA_MASK    0x40
#define LIS2DH_ZH_MASK        0x20
#define LIS2DH_ZL_MASK        0x10
#define LIS2DH_YH_MASK        0x08
#define LIS2DH_YL_MASK        0x04
#define LIS2DH_XH_MASK        0x02
#define LIS2DH_XL_MASK        0x01
#define LIS2DH_INT_ALL_MASK   0x7F
#define LIS2DH_INT_POS_MASK   0x3F

// INT1/2_THS masks
#define LIS2DH_THS_MASK       0x7F
#define LIS2DH_THS_SHIFT      0
#define LIS2DH_THS_MAXVALUE   0x7F

// INT1/2_DURATION masks
#define LIS2DH_D_MASK         0x7F
#define LIS2DH_DUR_SHIFT      0
#define LIS2DH_DUR_MAXVALUE   0x7F

// CLICK_CFG masks
#define LIS2DH_ZD_MASK      0x20
#define LIS2DH_ZS_MASK      0x10
#define LIS2DH_YD_MASK      0x08
#define LIS2DH_YS_MASK      0x04
#define LIS2DH_XD_MASK      0x02
#define LIS2DH_XS_MASK      0x01
#define LIS2DH_CLICEVENT_MASK         0x3F      // Interrupt mask

#define LIS2DH_CLICEVENT_SHIFT        0
#define LIS2DH_CLICEVENT_DBLE_Z       0x20      // Fire interrupt when detect double clic on Z - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_SINGLE_Z     0x10      // Fire interrupt when detect single clic on Z - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_DBLE_Y       0x08      // Fire interrupt when detect double clic on Y - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_SINGLE_Y     0x04      // Fire interrupt when detect single clic on Y - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_DBLE_X       0x02      // Fire interrupt when detect double clic on X - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_SINGLE_X     0x01      // Fire interrupt when detect single clic on X - based on getting higher accel than the preset threshold
#define LIS2DH_CLICEVENT_MAXVALUE     0x3F


// CLICK_SRC masks
                                        // Basically we have 6 axis (X,Y,Z) in both positive and negative direction
                                        // we also have two type of clics => Single and Double
                                        // This set of bit allow to identify all these different kind of clics
                                        
#define LIS2DH_CLK_IA_MASK    0x40      // An interrupt is active on one of the following bits
#define LIS2DH_DCLICK_MASK    0x20      // Double clic detected 
#define LIS2DH_SCLICK_MASK    0x10      // Single clic detected
#define LIS2DH_SIGN_MASK      0x08      // 0 - Positive detection / 1 - Negative detection (direction of the clic)
#define LIS2DH_Z_CLICK_MASK   0x04      // Clic on Z axis
#define LIS2DH_Y_CLICK_MASK   0x02      // Clic on Y axis
#define LIS2DH_X_CLICK_MASK   0x01      // Clic on X axis
#define LIS2DH_CLICK_SRC_MASK 0x7F

#define LIS2DH_CLIC_NONE        0x0000  // For each of the possible clic combination. ( they can be cumulated )
#define LIS2DH_CLIC_DBL_X_POS   0x0001
#define LIS2DH_CLIC_DBL_X_NEG   0x0002
#define LIS2DH_CLIC_SIN_X_POS   0x0004
#define LIS2DH_CLIC_SIN_X_NEG   0x0008
#define LIS2DH_CLIC_DBL_Y_POS   0x0010
#define LIS2DH_CLIC_DBL_Y_NEG   0x0020
#define LIS2DH_CLIC_SIN_Y_POS   0x0040
#define LIS2DH_CLIC_SIN_Y_NEG   0x0080
#define LIS2DH_CLIC_DBL_Z_POS   0x0100
#define LIS2DH_CLIC_DBL_Z_NEG   0x0200
#define LIS2DH_CLIC_SIN_Z_POS   0x0400
#define LIS2DH_CLIC_SIN_Z_NEG   0x0800



// CLICK_THS masks
#define LIS2DH_CLK_THS_MASK       0x7F
#define LIS2DH_CLK_THS_SHIFT      0
#define LIS2DH_CLK_THS_MAXVALUE   0x7F

#define LIS2DH_CLK_INTDUR_MASK        0x80
#define LIS2DH_CLK_INTDUR_SHIFT       7
#define LIS2DH_CLK_INTDUR_UNTILREAD   0x00    // Click interrupt is pending until the register CLICK_SRC has been read
#define LIS2DH_CLK_INTDUR_LATWINDOW   0x01    // Click interrupt is automatically cancel after the latency window duration ( TIME_LATENCY register )
#define LIS2DH_CLK_INTDUR_MAXVALUE    0x01

// TIME_LIMIT masks
#define LIS2DH_TLI_MASK               0x7F
#define LIS2DH_TLI_SHIFT              0
#define LIS2DH_TLI_MAXVALUE           0x7F

// TIME_LATENCY masks
#define LIS2DH_TIME_LATENCY_MASK         0xFF
#define LIS2DH_TIME_LATENCY_SHIFT        0
#define LIS2DH_TIME_LATENCY_MAXVALUE     0xFF

// TIME_WINDOW masks
#define LIS2DH_TIME_WINDOW_MASK          0xFF
#define LIS2DH_TIME_WINDOW_SHIFT         0
#define LIS2DH_TIME_WINDOW_MAXVALUE      0xFF

// ACT_THS masks
#define LIS2DH_ACT_THS_MASK       0x7F
#define LIS2DH_ACT_THS_SHIFT      0
#define LIS2DH_ACT_THS_MAXVALUE   0x7F

// ACT_DUR masks
#define LIS2DH_ACT_DUR_MASK       0xFF
#define LIS2DH_ACT_DUR_SHIFT      0
#define LIS2DH_ACT_DUR_MAXVALUE   0xFF


#define LIS2DH_DEFAULT_ADDRESS  0x18

#define LIS2DH_LIS2DH_SA0_LOW     LIS2DH_DEFAULT_ADDRESS
#define LIS2DH_LIS2DH_SA0_HIGH    (LIS2DH_DEFAULT_ADDRESS+1)

#define LIS2DH_INTERRUPT1     1
#define LIS2DH_INTERRUPT2     2

// 6D position of the Object. Assuming the TOP is component top side
// We assume the position entry is the direction the TOP side is looking
#define LIS2DH_POSITION_TOP_ON_NONE     0x00
#define LIS2DH_POSITION_TOP_ON_TOP      0x20
#define LIS2DH_POSITION_TOP_ON_BOTTOM   0x10
#define LIS2DH_POSITION_TOP_ON_RIGHT    0x02
#define LIS2DH_POSITION_TOP_ON_LEFT     0x01     
#define LIS2DH_POSITION_TOP_ON_FRONT    0x04
#define LIS2DH_POSITION_TOP_ON_BACK     0x08

#define LIS2DH_TEMPERATURE_INVALID      0x1FFF

class LIS2DH {
 public:

    // Main function to be use
    LIS2DH();
    bool init(void);                                                                                                      // Default Init 8b, 10Hz, 2G
    bool init(int resolution, int frequency, int scale);
    bool initPosition6D();                                                                                               // Configure the 6D position function on INT1
    bool reinit();                                                                                                        // Do not change the device config but restore the lis2dh12 structure  
    void dumpConfig(void);

    uint8_t getPendingMotions( int16_t * _buffer, uint8_t size);                                                          // Get all the FiFo pending measure for X,Y,Z 
    uint8_t getPendingAcceleration( int16_t * _buffer, uint8_t size);                                                     // Get all the Fifo pending measure in Mg ( x,y,z )
    uint8_t getPendingForces( int16_t * _buffer, uint8_t size);                                                           // Get all the Fifo pending measure into a force in Mg array |A|

    bool runBackgroundTiltDetection(uint16_t forceMg);                                                                    // Configure Interrupt for being fired if the force is detected any axis
    bool hasTiltDetected();                                                                                               // Return true when a tilt has been detected and clear the interrupt

    uint8_t getPosition6D();                                                                                              // Return one of the 6D positions

    // Low level functions
    int16_t getAxisX(void);                                                                                               // Get the last measured X acceleration
    int16_t getAxisY(void);                                                                                               // Get the last measured Y acceleration
    int16_t getAxisZ(void);                                                                                               // Get the last measured Z acceleration
    void getMotion(int16_t* ax, int16_t* ay, int16_t* az);                                                                // Get the last measured X,Y,Z acceleration
    int8_t getAxisX_LR(void);                                                                                             // Get the last measured X acceleration in LowpoweR mode (8 bits)
    int8_t getAxisY_LR(void);                                                                                             // Get the last measured Y acceleration in LowpoweR mode (8 bits)
    int8_t getAxisZ_LR(void);                                                                                             // Get the last measured Z acceleration in LowpoweR mode (8 bits)
    void getMotion_LR(int8_t* ax, int8_t* ay, int8_t* az);                                                                // Get the last measured X,Y,Z acceleration in LowpowerR mode (8bits)


    bool tempHasOverrun(void);
    bool tempDataAvailable(void);
    int16_t getTemperature(void);
    bool whoAmI(void);
    bool getTempEnabled(void);
    bool setTempEnabled(bool enable);
    uint8_t getDataRate(void);
    bool setDataRate(uint8_t rate);
    bool enableLowPower(void);
    bool disableLowPower(void);
    bool isLowPowerEnabled(void);
    bool setPowerDown(void);
    bool setSleepNWakeUpThreshold(uint8_t raw);
    bool setSleepNWakeUpThresholdMg( uint8_t mg, const uint8_t scale);
    bool setSleepNWakeUpDuration(uint8_t raw);
    bool setSleepNWakeUpDurationMs(uint8_t _int, uint32_t ms, const uint8_t odr);
    
    bool enableAxisX(void);
    bool disableAxisX(void);
    bool isXAxisEnabled(void);
    bool enableAxisY(void);
    bool disableAxisY(void);
    bool isYAxisEnabled(void);
    bool enableAxisZ(void);
    bool disableAxisZ(void);
    bool isZAxisEnabled(void);
    uint8_t getHPFilterMode();
    bool setHPFilterMode(uint8_t mode);
    uint8_t getHPFilterCutOff();
    bool setHPFilterCutOff(uint8_t mode);
    bool enableHPClick(void);
    bool disableHPClick(void);
    bool isHPClickEnabled(void);
    bool enableHPIA1(void);
    bool disableHPIA1(void);
    bool isHPIA1Enabled(void);
    bool enableHPIA2(void);
    bool disableHPIA2(void);
    bool isHPIA2Enabled(void);
    bool enableHPFDS(void);
    bool disableHPFDS(void);
    bool isHPFDSEnabled(void);
    bool enableAxisXYZ(void);
    bool disableAxisXYZ(void);
    
    bool enableInterruptInt1(uint8_t _int);
    bool disableInterruptInt1(uint8_t _int);
    bool enableInterruptInt2(uint8_t _int);
    bool disableInterruptInt2(uint8_t _int);
    bool disableAllInterrupt();
    bool setInterruptPolarity(uint8_t polarity);
    bool triggerSelect(uint8_t triggerMode);
    bool intWorkingMode(uint8_t _int, uint8_t _mode);                                 // Set the interrupt mode (OR, AND, 6D Movement, 6D Positions) see LIS2DH_INT_MODE_XXX
    bool enableInterruptEvent(uint8_t _int, uint8_t _intEvent);                       // Select the interrupt event source X,Y,Z Higher or Lower see LIS2DH_INTEVENT_XX
    bool isInterruptFired(uint8_t _int);
    bool isInterruptZHighFired(uint8_t _int);
    bool isInterruptZLowFired(uint8_t _int);
    bool isInterruptYHighFired(uint8_t _int);
    bool isInterruptYLowFired(uint8_t _int);
    bool isInterruptXHighFired(uint8_t _int);
    bool isInterruptXLowFired(uint8_t _int);
    bool setInterruptThreshold(uint8_t _int, uint8_t raw);                            // Set the raw limit for an interrupt to be fired
    bool setInterruptThresholdMg(uint8_t _int, uint8_t mg, const uint8_t scale);      // Set the limit in mG for an interrupt to be fired
    bool setInterruptDuration(uint8_t _int, uint8_t raw);
    bool setInterruptDurationMs(uint8_t _int, uint32_t ms, const uint8_t odr);    
    bool enableLatchInterrupt(uint8_t _int, bool enable);                             // When true, Interrupt is cleared only when INTx_SRC register is read

    bool setLittleEndian();
    bool setBitEndian();
    bool setContinuousUpdate(bool continuous);
    bool setAccelerationScale(uint8_t scale);
    uint8_t getAccelerationScale();
    bool setHighResolutionMode(bool hr);
    bool isHighResolutionMode();

    bool reboot();
    bool enableFifo(bool enable);

    bool setReference(uint8_t ref);
    uint8_t getDataStatus();
    bool setResolutionMode(uint8_t res);
    uint8_t getResolutionMode();

    bool setFiFoMode(uint8_t fifoMode);
    uint8_t getFiFoMode();
    bool setFiFoThreshold(uint8_t threshold);
    bool isFiFoWatermarkExceeded();
    bool isFiFoFull();
    bool isFiFoEmpty();
    uint8_t getFiFoSize();

    bool enableInterruptEvent(uint8_t _intEvent);
    bool isClickInterruptFired();
    bool isDoubleClickFired();
    bool isSimpleClickFired();
    bool isClickFiredOnZ();
    bool isClickFiredOnY();
    bool isClickFiredOnX();
    bool isSignClickFired();
    uint16_t getClickStatus();
    bool setClickThreshold(uint8_t ths);
    bool setClickThresholdMg(uint16_t mg, const uint8_t scale);
    bool setClickInterruptMode(uint8_t _mode);
    bool setClickTimeLimit(uint8_t raw);
    bool setClickTimeLatency(uint8_t raw);
    bool setClickTimeWindow(uint8_t raw);

 private:
    void readSetting();                                                                                                   // Restore scale, resolution, frequency based on chip current config

    bool getAcceleration(const uint8_t resolution, const uint8_t scale, int16_t * ax, int16_t * ay, int16_t * az);        // return acceleration in mg instead of raw values
    bool getAcceleration(int16_t * x, int16_t * y, int16_t * z);                                                          // equivalent but use internal known config for this.
 
    bool writeRegister(const uint8_t register_addr, const uint8_t value);
    bool writeRegisters(const uint8_t msb_register, const uint8_t msb_value, const uint8_t lsb_register, const uint8_t lsb_value);
    bool writeMaskedRegister8(const uint8_t register_addr, const uint8_t mask, const bool value);
    bool writeMaskedRegisterI(const int register_addr, const int mask, const int value);
    uint8_t readRegister(const uint8_t register_addr);
    uint16_t readRegisters(const uint8_t msb_register, const uint8_t lsb_register);
    uint8_t readMaskedRegister(const uint8_t register_addr, const uint8_t mask);
    uint8_t readFifo(int16_t * _buffer,const uint8_t maxSz);
    
    bool convertMgToRaw(uint8_t * _raw, uint16_t mg, uint8_t scale);
    bool convertMsToRaw(uint8_t * _raw, uint32_t ms, const uint8_t odr);
    uint8_t _address;

    uint8_t _resolution;        // store the current resolution (bits)
    uint8_t _frequency;         // store the current frequency (Hz)
    uint8_t _scale;             // store the current scale (xG)
    uint8_t _fifoMode;          // store the current fifo mode
    uint8_t _odr;               // store the current ODR mode
};


// Logger wrapper
#define LIS2DH_LOG_LEVEL 5

#if LIS2DH_LOG_LEVEL >= 5
#define LIS2DH_LOG_DEBUG(x) Serial.printf x
#else
#define LIS2DH_LOG_DEBUG(x) 
#endif

#if LIS2DH_LOG_LEVEL >= 4
#define LIS2DH_LOG_INFO(x) Serial.printf x
#else
#define LIS2DH_LOG_INFO(x) 
#endif

#if LIS2DH_LOG_LEVEL >= 3
#define LIS2DH_LOG_WARN(x) Serial.printf x
#else
#define LIS2DH_LOG_WARN(x) 
#endif

#if LIS2DH_LOG_LEVEL >= 2
#define LIS2DH_LOG_ERROR(x) Serial.printf x
#else
#define LIS2DH_LOG_ERROR(x) 
#endif

#if LIS2DH_LOG_LEVEL >= 1
#define LIS2DH_LOG_ANY(x) Serial.printf x
#else
#define LIS2DH_LOG_ANY(x) 
#endif

#endif /* _LIS2DH_H_ */