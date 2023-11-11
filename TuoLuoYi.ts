
/*
load dependency
"MPU6050": "file:../pxt-mpu6050"
*/

/*
 * MPU6050 block
 */
//% weight=0 color=#9900CC icon="\uf13d" block="陀螺仪"
namespace MPU6050 {


    export enum REGISTER {
        // 陀螺仪采样率地址
        SMPLRT_DIV = 0x19,
        // 低通滤波频率地址
        CONFIG = 0x1a,
        // 陀螺仪自检及测量范围
        GYRO_CONFIG = 0x1b,
        // 加速计自检、测量范围及高通滤波频率
        ACCEL_CONFIG = 0x1c,
        // 电源管理地
        PWR_MGMT = 0x6b,
        ACCEL_X = 0x3b,
        ACCEL_Y = 0x3d,
        ACCEL_Z = 0x3f,
        TEMPATURE = 0x41,
        GYRO_X = 0x43,
        GYRO_Y = 0x45,
        GYRO_Z = 0x47
    }

    export enum AXIS {
        X = 1,
        Y = 2,
        Z = 3
    }


    export enum MPU6050_I2C_ADDRESS {
        ADDR_0x68 = 0x68,
        ADDR_0x69 = 0x69
    }

    let SMPLRT = 0x07 // 陀螺仪采样率 125Hz
    let CONFIG = 0x06 // 低通滤波频率 5Hz
    let GYRO_CONFIG = 0x18 // 典型值：0x18(不自检，2000deg/s) 
    let ACCEL_CONFIG = 0x01 // 典型值：0x01(不自检，2G，5Hz)
    let X_ACCEL_OFFSET = 0
    let Y_ACCEL_OFFSET = 0
    let Z_ACCEL_OFFSET = 0
    let X_GYRO_OFFSET = 0
    let Y_GYRO_OFFSET = 0
    let Z_GYRO_OFFSET = 0



    function i2cWrite(addr: number, reg: number, value: number): void {
        let buf = pins.createBuffer(2);
        buf[0] = reg;
        buf[1] = value;
        pins.i2cWriteBuffer(addr, buf);
    }

    function i2cRead(addr: number, reg: number): number {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function checkAddress(addr: MPU6050_I2C_ADDRESS): boolean {
        switch (addr) {
            case MPU6050_I2C_ADDRESS.ADDR_0x68:
                return true
            case MPU6050_I2C_ADDRESS.ADDR_0x69:
                return true
            default:
                return false
        }
        return false
    }

    /**
     * 初始化MPU6050
     * @param addr [0-1] choose address; eg: MPU6050.MPU6050_I2C_ADDRESS.ADDR_0x68
    */
    //% blockId="MPU6050_initMPU6050"
    //% block="初始化地址 %addr 的MPU6050"
    //% weight=85
    export function initMPU6050(addr: MPU6050_I2C_ADDRESS) {
        if (checkAddress(addr)) {
            i2cWrite(addr, REGISTER.PWR_MGMT, 0)
        }
    }

    /**
     * 复位MPU6050
     * @param addr [0-1] choose address; eg: MPU6050.MPU6050_I2C_ADDRESS.ADDR_0x68
    */
    //% blockId="MPU6050_resetMPU6050"
    //% block="重置地址 %addr 的MPU6050"
    //% weight=85
    export function resetMPU6050(addr: MPU6050_I2C_ADDRESS) {
        if (checkAddress(addr)) {
            i2cWrite(addr, REGISTER.PWR_MGMT, 1)
            i2cWrite(addr, REGISTER.SMPLRT_DIV, SMPLRT)
            i2cWrite(addr, REGISTER.CONFIG, CONFIG)
            i2cWrite(addr, REGISTER.GYRO_CONFIG, GYRO_CONFIG)
            i2cWrite(addr, REGISTER.ACCEL_CONFIG, ACCEL_CONFIG)
        }
    }


    /**
     *Read byte from MPU6050 register
     * @param reg  register of MPU6050; eg: 0, 15, 23
    */
    function readByte(addr: MPU6050_I2C_ADDRESS, reg: REGISTER): number {
        let val2 = i2cRead(addr, reg);
        return val2;
    }

    /**
     *Read data from MPU6050 register
     * @param reg  register of MPU6050; eg: 0, 15, 23
    */
    function readWord(addr: MPU6050.MPU6050_I2C_ADDRESS, reg: REGISTER): number {
        let valh = i2cRead(addr, reg);
        let vall = i2cRead(addr, reg + 1);
        let val3 = (valh << 8) + vall
        return val3
    }

    /**
     *Read data from MPU6050 register
     * @param reg  register of MPU6050; eg: 0, 15, 23
    */
    function readWord2C(addr: MPU6050_I2C_ADDRESS, reg: REGISTER): number {
        let val4 = readWord(addr, reg)
        if (val4 > 0x8000) {
            return -((65535 - val4) + 1)
        } else {
            return val4
        }
    }


    /**
    *  读取温度
   */
    //% blockId=MPU6050_readTempature 
    //% block="读取温度，设备地址 %addr"
    //% weight=75
    export function readTempature(addr: MPU6050_I2C_ADDRESS): number {
        if (checkAddress(addr)) {
            let value = readWord2C(addr, REGISTER.TEMPATURE)
            return 36.53 + value / 340;
        } else {
            return 0
        }
    }

    /**
     * 倾斜角度
    */
    //% blockId=MPU6050_get_angle
    //% block="获取设备 |%addr| 方向 |%axis| 的倾斜角度"
    //% weight=75
    export function getAngle(addr: MPU6050_I2C_ADDRESS, axis: AXIS): number {
        if (checkAddress(addr)) {
            switch (axis) {
                case AXIS.X:
                    return Math.acos(getAccel(addr, axis)) * 57.29577
                case AXIS.Y:
                    return Math.acos(getAccel(addr, axis)) * 57.29577
                case AXIS.Z:
                    return Math.acos(getAccel(addr, axis)) * 57.29577
                default:
                    return 0
            }
        }
        return 0
    }


    /**
     * 获取加速度 单位g
    */
    //% blockId=MPU6050_get_accel
    //% block="获取设备 |%addr| 方向 |%axis| 的加速度"
    //% weight=75
    export function getAccel(addr: MPU6050_I2C_ADDRESS, axis: AXIS): number {
        if (checkAddress(addr)) {
            switch (axis) {
                case AXIS.X:
                    return (readWord2C(addr, REGISTER.ACCEL_X) + X_ACCEL_OFFSET) / 16384.0
                case AXIS.Y:
                    return (readWord2C(addr, REGISTER.ACCEL_Y) + Y_ACCEL_OFFSET) / 16384.0
                case AXIS.Z:
                    return (readWord2C(addr, REGISTER.ACCEL_Z) + Z_ACCEL_OFFSET) / 16384.0
                default:
                    return 0
            }
        }
        return 0
    }

    /**
     * 设置加速度 偏差
    */
    //% blockId=MPU6050_set_accel_offset
    //% block="设置设备 |%addr| 方向 |%axis| 的加速度偏移 %offset"
    //% weight=75
    export function setAccelOffset(axis: AXIS, offset: number) {
        switch (axis) {
            case AXIS.X:
                X_ACCEL_OFFSET = offset
                break
            case AXIS.Y:
                Y_ACCEL_OFFSET = offset
                break
            case AXIS.Z:
                Z_ACCEL_OFFSET = offset
                break
            default:
                break
        }
        return
    }

    /**
     * 设置角速度 偏差
    */
    //% blockId=MPU6050_set_gyro_offset
    //% block="设置设备 |%addr| 方向 |%axis| 的角速度偏移 %offset"
    //% weight=75
    export function setGyroOffset(axis: AXIS, offset: number) {
        switch (axis) {
            case AXIS.X:
                X_GYRO_OFFSET = offset
                break
            case AXIS.Y:
                Y_GYRO_OFFSET = offset
                break
            case AXIS.Z:
                Z_GYRO_OFFSET = offset
                break
            default:
                break
        }
        return
    }


    /**
     * 获取角速度
    */
    //% blockId=MPU6050_get_gyro
    //% block="获取设备 |%addr| 方向 |%axis| 的角速度"
    //% weight=75
    export function getGyro(addr: MPU6050_I2C_ADDRESS, axis: AXIS): number {
        if (checkAddress(addr)) {
            switch (axis) {
                case AXIS.X:
                    return (readWord2C(addr, REGISTER.GYRO_X) + X_GYRO_OFFSET)
                case AXIS.Y:
                    return (readWord2C(addr, REGISTER.GYRO_Y) + Y_GYRO_OFFSET)
                case AXIS.Z:
                    return (readWord2C(addr, REGISTER.GYRO_Z) + Z_GYRO_OFFSET)
                default:
                    return 0
            }
        }
        return 0
    }

}
