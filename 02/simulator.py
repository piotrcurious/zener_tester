import time
import math
import random

class ADT75:
    def __init__(self, addr=0x48):
        self.addr = addr
        self.temp = 25.0 # Initial ambient temp

    def read_temp(self):
        return self.temp

class Peltier:
    def __init__(self, heat_capacity=100.0, ambient_temp=25.0):
        self.temp = ambient_temp
        self.ambient_temp = ambient_temp
        self.heat_capacity = heat_capacity
        self.cooling_power = 0
        self.heating_power = 0
        self.dt = 0.1 # Simulation time step

    def update(self, cooling_duty, heating_duty):
        # Very simple thermal model
        # Power proportional to duty
        p_cool = cooling_duty * 0.5
        p_heat = heating_duty * 0.5

        # Temp change = (P_heat - P_cool + (T_amb - T_curr)*k) / C
        heat_leak = (self.ambient_temp - self.temp) * 0.1
        dT = (p_heat - p_cool + heat_leak) / self.heat_capacity
        self.temp += dT * self.dt
        return self.temp

class StepUpConverter:
    def __init__(self):
        self.duty = 0
        self.v_in = 5.0
        self.v_out = 0

    def update(self, duty):
        self.duty = duty
        # Simplified: Vout = Vin / (1 - D)
        # But limited and with real-world inefficiency
        d = duty / 1023.0
        if d > 0.9: d = 0.9 # Limit duty
        self.v_out = self.v_in / (1.0001 - d)
        if self.v_out > 50.0: self.v_out = 50.0 # Clamp to 50V
        return self.v_out

class ZenerDiode:
    def __init__(self, v_z=12.0, temp_coeff=0.01):
        self.v_z_nom = v_z
        self.temp_coeff = temp_coeff # V/C change

    def get_voltage_drop(self, v_supply, current_temp):
        v_z_actual = self.v_z_nom + (current_temp - 25.0) * self.temp_coeff
        if v_supply < v_z_actual:
            return v_supply # Diode not conducting
        else:
            return v_z_actual # Diode clamping (simplified)

class ArduinoMock:
    def __init__(self):
        self.peltier = Peltier()
        self.converter = StepUpConverter()
        self.zener = ZenerDiode(v_z=15.0) # 15V Zener
        self.adt75 = ADT75()
        self.pwm = {0: 0, 1: 0, 2: 0} # CH_COOLING, CH_HEATING, CH_ZENER
        self.analog = {34: 0, 35: 0} # PIN_ANA_CONV_VOLT, PIN_ANA_ZENER_VOLT

    def ledcWrite(self, channel, duty):
        self.pwm[channel] = duty

    def analogRead(self, pin):
        # Convert physical voltage back to 0-4095 range
        # Note: assuming 1/11 divider in the .ino
        physical_v = 0
        if pin == 34:
            physical_v = self.converter.v_out
        elif pin == 35:
            physical_v = self.zener.get_voltage_drop(self.converter.v_out, self.peltier.temp)

        # V_pin = V_phys / 11
        v_pin = physical_v / 11.0
        if v_pin > 3.3: v_pin = 3.3 # Clamp to ESP32 limit
        return int(v_pin * (4095.0 / 3.3))

    def update_physics(self):
        self.peltier.update(self.pwm[0], self.pwm[1])
        self.adt75.temp = self.peltier.temp
        self.converter.update(self.pwm[2])

def map_val(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class PID:
    def __init__(self, kp, ki, kd, setpoint=0, out_min=-1023, out_max=1023):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0
        self.prev_error = 0

    def compute(self, input_val):
        error = self.setpoint - input_val
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return max(min(output, self.out_max), self.out_min)

def simulate_system():
    arduino = ArduinoMock()
    # Mocking a 15V Zener with 0.01V/C temp coeff
    arduino.zener = ZenerDiode(v_z=15.0, temp_coeff=0.01)

    temp_pid = PID(kp=100.0, ki=1.0, kd=10.0, out_min=-1023, out_max=1023)

    temp_start = 10.0
    temp_end = 40.0
    steps = 4

    print(f"{'Target T':>10} | {'Actual T':>10} | {'PWM Duty':>10} | {'SupplyV':>10} | {'ZenerV':>10}")
    print("-" * 65)

    for i in range(steps):
        target = temp_start + i * (temp_end - temp_start) / (steps - 1)
        temp_pid.setpoint = target

        # Settle temperature (simulated)
        for _ in range(500):
            arduino.update_physics()
            output = temp_pid.compute(arduino.adt75.temp)
            if output > 0:
                arduino.ledcWrite(1, abs(output))
                arduino.ledcWrite(0, 0)
            else:
                arduino.ledcWrite(0, abs(output))
                arduino.ledcWrite(1, 0)

        # Sweep Zener for this temperature
        for d in [200, 400, 600, 800]: # Sample duties
            arduino.ledcWrite(2, d)
            arduino.update_physics()

            v_conv = arduino.analogRead(34) * (3.3/4095.0) * 11.0
            v_zener = arduino.analogRead(35) * (3.3/4095.0) * 11.0

            print(f"{target:10.2f} | {arduino.adt75.temp:10.2f} | {d:10d} | {v_conv:10.2f} | {v_zener:10.2f}")

if __name__ == "__main__":
    simulate_system()
