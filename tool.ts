
enum 超声波测距返回类型 {
    //% block="μs"
    MicroSeconds,
    //% block="cm"
    Centimeters,
    //% block="inches"
    Inches
}
//% color=#9900CC weight=10 icon="" groups='["超声波", "陀螺仪"]'
namespace 方位测量 {


    /**
     * 超声波测距
     * @param maxCmDistance eg: 100
     */
    //% group="超声波"
    //% blockId=sonar_ping block="超声波测距|trig %trig echo %echo 返回类型 %unit 最大距离 %maxCmDistance"
    export function ping(trig: DigitalPin, echo: DigitalPin, unit: 超声波测距返回类型, maxCmDistance = 500): number {
        // send pulse
        pins.setPull(trig, PinPullMode.PullNone);
        pins.digitalWritePin(trig, 0);
        control.waitMicros(2);
        pins.digitalWritePin(trig, 1);
        control.waitMicros(10);
        pins.digitalWritePin(trig, 0);

        // read pulse
        const d = pins.pulseIn(echo, PulseValue.High, maxCmDistance * 58);

        switch (unit) {
            case 超声波测距返回类型.Centimeters: return Math.idiv(d, 58);
            case 超声波测距返回类型.Inches: return Math.idiv(d, 148);
            default: return d;
        }
    }


    // 陀螺仪代码
    let 角度z = 0
    let 上次时间 = control.millis()

    /**
     * 陀螺仪获取偏航初始化
     */
    //% group="陀螺仪"
    //% blockId=PianHang block="偏航初始化"
    export function initMPU6050(){
        MPU6050.initMPU6050(MPU6050.MPU6050_I2C_ADDRESS.ADDR_0x68)
       
        for (let index = 0; index < 5; index++) {
            角度z += MPU6050.getGyro(MPU6050.MPU6050_I2C_ADDRESS.ADDR_0x68, MPU6050.AXIS.Z)
            basic.pause(100)
        }
        角度z = 0
        for (let index = 0; index < 20; index++) {
            角度z += MPU6050.getGyro(MPU6050.MPU6050_I2C_ADDRESS.ADDR_0x68, MPU6050.AXIS.Z)
            basic.pause(100)
        }
        MPU6050.setGyroOffset(MPU6050.AXIS.Z, 0 - 角度z / 20)
        角度z = 0
        control.inBackground(function () {
            while (true) {
                角度z += MPU6050.getGyro(MPU6050.MPU6050_I2C_ADDRESS.ADDR_0x68, MPU6050.AXIS.Z) * (control.millis() - 上次时间)
                上次时间 = control.millis()
                basic.pause(8)
            }
        })
    }
    /**
     * 获取当前偏航值
     */
    //% blockId=get_z block="获取偏航值"
    export function get_z():number{
        return 角度z * 0.00000844
    }
    

 



}


//% color="#2c3e50" weight=10 icon=""
namespace 显示{
    /**
     * @param i 进度 eg: 20
     */
    //% block="显示25以内的进度 | 进度 %i"
    export function ShowNum(i:number=20){
        let rt_s = images.createImage(`
. . . . .
. . . . .
. . . . .
. . . . .
. . . . .
`)
        for(let j=0;j<i;j++){
            rt_s.setPixel(j % 5, Math.floor(j / 5), true)
        }
        rt_s.showImage(0)
    }
}

//% color="#2c3e50" weight=10 icon=""
namespace 算法{

    // let PID_data: { Kp: number, Ki: number, Kd: number, previousError: number, integral:number}={
    //     Kd:1,
    //     Ki:1,
    //     Kp:1,
        
    // }
    export class PID{
        public Kd:number
        public Ki:number
        public Kp:number
        public previousError:number
        public integral:number[]
        public add_integral(x:number){
            this.integral.push(x)
            if (this.integral.length>10){
                this.integral.shift()
            }
        }
        public sum_integral(){
            let sum = 0;

            for (let i = 0; i < this.integral.length; i++) {
                sum += this.integral[i];
            }
        
            return sum/this.integral.length
        }
    }
    /**
     * @param controlVariable 传感器测量值
     * @param setpoint 标准值
     * @param Kp 比例系数 eg: 1
     * @param Ki 积分系数 eg: 1
     * @param Kd 微分系数 eg: 1
     */
    //% block="PID计算|PID数据 %PID_Data 传感器测量值 %controlVariable 标准值 %setpoint"
    export function do_PID(PID_Data: PID, controlVariable: number, setpoint: number):number{
        let error = setpoint - controlVariable;
        let proportional = PID_Data.Kp * error;

        PID_Data.add_integral(error);
        let integralValue = PID_Data.Ki * PID_Data.sum_integral();

        let derivative = PID_Data.Kd * (error - PID_Data.previousError);
        PID_Data.previousError = error;

        let output = proportional + integralValue + derivative;
        return output;
    }
    /**
     * @param Kp 比例系数 eg: 1
     * @param Ki 积分系数 eg: 1
     * @param Kd 微分系数 eg: 1
     */
    //% block="创建PID数据|比例系数 %Kp 积分系数 %Ki 微分系数 %Kd"
    export function new_PID(Kp: number, Ki: number, Kd: number):PID{
        let pid_data =new PID()
        pid_data.Kp=Kp
        pid_data.Ki=Ki
        pid_data.Kd=Kd
        pid_data.previousError=0
        pid_data.integral=[]
        return pid_data
    }
    /**
     * @param controlVariable 传感器测量值 
     * @param setpoint 标准值 
     * @param Kp 比例系数 eg: 1 
     * @param Ki 积分系数 eg: 1 
     * @param Kd 微分系数 eg: 1 
     */
    //% block="PID算法|传感器测量值 %controlVariable 标准值 %setpoint 比例系数 %Kp 积分系数 %Ki 微分系数 %Kd"
    export function calculatePID(controlVariable: number, setpoint:number,Kp:number=1, Ki:number=1, Kd:number=1):number {
        let previousError = 0;
        let integral = 0;
        const error = setpoint - controlVariable;
        const proportional = Kp * error;

        integral += error;
        const integralValue = Ki * integral;

        const derivative = Kd * (error - previousError);
        previousError = error;

        const output = proportional + integralValue + derivative;
        return output;
    }
}

namespace spike{
    let _msg_data=''
    let _need_clear=false
    let _num=16

    function get_data_num():string{
        if(_num==255){
            _num=16
        }
        _num=_num+1
        return decimalToHex(_num);
    }


    function decimalToHex(num: number): string {

        const hexChars = '0123456789ABCDEF';
        let hexString = '';

        while (num > 0) {
            const remainder = num % 16;
            hexString = hexChars[remainder] + hexString;
            num = Math.floor(num / 16);
        }

        return hexString || '0';
    }
    /**
         * @param str 传感器测量值
         */
    //% block="发送消息|消息编号 %id|字符串 %msg"
    export function 发送消息(id:string,msg: string) {
        let data_head ='FF9703'
        let data_num= get_data_num()
        let data_id=CRC32(id)
        let data_msg=msg.split('').map(char => decimalToHex(char.charCodeAt(0))).join('');
        let data=data_head+data_num+data_id+data_msg
        let len=decimalToHex(data.length/2)
        if(len.length==1){
            len='0'+len
        }
        data=len+data

        return 发送AT命令(`AT+BLEADVDATA="${data}"`)
    }

    /**
    * @param cmd 命令
    * @param time_out 超时时间 eg:50
    */
    //% block="发送AT命令|命令 %cmd 超时时间ms %time_out"
    export function 发送AT命令(cmd:string,time_out:number=50):number {
        serial.writeLine(cmd+"\r\n")
        for(let i=0;i<time_out;i++){
            basic.pause(1)
            if(_msg_data.includes('OK')){
                _need_clear = true
                return 0
            }
            if (_msg_data.includes('ERROR')){
                _need_clear = true
                return 1
            }
        }
        return 2
    }
    //% block="初始化蓝牙设置"
    export function 初始化蓝牙设置():number {
        serial.redirect(
            SerialPin.P15,
            SerialPin.P16,
            BaudRate.BaudRate115200
        )
        loops.everyInterval(1, function () {
            let msg = serial.readLine()
            if (msg.includes('busy')) {
                msg = ''
            }
            if (msg.includes('AT')) {
                msg = ''
            }
            if (_need_clear) {
                _msg_data = msg
                _need_clear = false
            }
            else {
                _msg_data = _msg_data + msg
            }
        })
        if(发送AT命令('AT+BLEINIT=2')!=0){
            return 1
        }
        if (发送AT命令('AT+BLEADVPARAM=32,64,3,0,4,0,1,"00:00:00:00:00:00"') != 0) {
            return 2
        }
        if (发送AT命令('AT+BLEADVSTART') != 0) {
            return 3
        }
        return 0
    }
    /**
     * @param txt 传感器测量值 
     */
    //% block="CRC32算法|字符串 %txt"
    export function CRC32(txt: string): string {
        function crc32(str: string) {
            let crcTable = (function () {
                let c = 0
                let table = [];


                for (let n = 0; n != 256; ++n) {
                    c = n;
                    c = ((c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1));
                    c = ((c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1));
                    c = ((c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1));
                    c = ((c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1));
                    c = ((c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1));
                    c = ((c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1));
                    c = ((c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1));
                    c = ((c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1));
                    table[n] = c;
                }

                return table;
            })();

            let crc = 0 ^ (-1);

            for (let i = 0; i < str.length; ++i) {
                crc = (crc >>> 8) ^ crcTable[(crc ^ str.charCodeAt(i)) & 0xFF];
            }

            return decimalToHex((crc ^ (-1)) >>> 0);
        };
        function reversePairs(str:string):string {
            let reversed = '';
            for (let i = str.length - 2; i >= 0; i -= 2) {
                reversed += str.substr(i, 2);
            }
            return reversed;
        }
        return reversePairs(crc32(txt))
    }
}
