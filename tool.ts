
enum 超声波测距返回类型 {
    //% block="μs"
    MicroSeconds,
    //% block="cm"
    Centimeters,
    //% block="inches"
    Inches
}
//% color="#2c3e50" weight=10 icon=""
namespace 方位测量 {


    /**
     * 超声波测距
     * @param maxCmDistance eg: 100
     */
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
