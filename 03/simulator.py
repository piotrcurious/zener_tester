import time
import math
import random
import sys

# Simulation of the Zener Tester Hardware and Environment

class ADT75:
    def __init__(self, addr=0x48):
        self.addr = addr
        self.temp = 25.0 # Initial ambient temp

    def read_temp(self):
        # Add a tiny bit of noise to the reading
        return self.temp + random.uniform(-0.03, 0.03)

class Peltier:
    def __init__(self, heat_capacity=150.0, ambient_temp=25.0):
        self.temp = ambient_temp
        self.ambient_temp = ambient_temp
        self.heat_capacity = heat_capacity
        self.k_leak = 0.05
        self.p_coeff = 0.8
        self.dt = 0.1 # Simulation time step

    def update(self, cooling_duty, heating_duty):
        # Peltier effect is somewhat non-linear but we'll use a linear approximation
        p_cool = cooling_duty * self.p_coeff
        p_heat = heating_duty * self.p_coeff

        # Newton's law of cooling for heat leak
        heat_leak = (self.ambient_temp - self.temp) * self.k_leak

        # dQ/dt = P_heat - P_cool + P_leak
        dT = (p_heat - p_cool + heat_leak) / self.heat_capacity
        self.temp += dT * self.dt
        return self.temp

class StepUpConverter:
    def __init__(self):
        self.v_in = 5.0
        self.v_out = 5.0

    def update(self, duty):
        # Simplified: Vout = Vin / (1 - D)
        # PWM_MAX is 1023
        d = duty / 1023.0
        if d > 0.92: d = 0.92 # Realistic limit for a simple boost

        # Target voltage with some "droop" under load (simulated)
        target_v = self.v_in / (1.0 - d)

        # Slow response of the output capacitor
        self.v_out += (target_v - self.v_out) * 0.2

        if self.v_out > 50.0: self.v_out = 50.0
        return self.v_out

class ZenerDiode:
    def __init__(self, v_z=15.0, temp_coeff=0.012):
        self.v_z_nom = v_z
        self.temp_coeff = temp_coeff # V/C change

    def get_voltage_drop(self, v_supply, current_temp):
        # Breakdown voltage increases with temperature for Zeners > 5V
        v_z_actual = self.v_z_nom + (current_temp - 25.0) * self.temp_coeff

        if v_supply < v_z_actual:
            # Below breakdown, it's just a high impedance (modeled as supply voltage)
            # In reality there's a tiny leakage but we'll ignore it for now
            return v_supply
        else:
            # Clamping at Vz. We'll add a small series resistance (slope)
            r_z = 10.0 # 10 Ohms
            i_z = (v_supply - v_z_actual) / (r_z + 100.0) # 100 Ohm current limit resistor
            return v_z_actual + i_z * r_z

class ArduinoMock:
    def __init__(self):
        self.peltier = Peltier()
        self.converter = StepUpConverter()
        self.zener = ZenerDiode(v_z=15.0)
        self.adt75 = ADT75()
        self.pwm = {0: 0, 1: 0, 2: 0} # CH_COOLING, CH_HEATING, CH_ZENER

    def ledcWrite(self, channel, duty):
        self.pwm[channel] = duty

    def analogRead(self, pin):
        physical_v = 0
        if pin == 34: # PIN_ANA_CONV_VOLT
            physical_v = self.converter.v_out
        elif pin == 35: # PIN_ANA_ZENER_VOLT
            physical_v = self.zener.get_voltage_drop(self.converter.v_out, self.peltier.temp)

        # 1/11 voltage divider
        v_pin = physical_v / 11.0
        if v_pin > 3.3: v_pin = 3.3
        return int(v_pin * (4095.0 / 3.3))

    def update_physics(self):
        self.peltier.update(self.pwm[0], self.pwm[1])
        self.adt75.temp = self.peltier.temp
        self.converter.update(self.pwm[2])

class PID:
    def __init__(self, kp, ki, kd, setpoint=25.0, out_min=-1023, out_max=1023):
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
        # Anti-windup
        if self.integral * self.ki > self.out_max: self.integral = self.out_max / self.ki
        if self.integral * self.ki < self.out_min: self.integral = self.out_min / self.ki

        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return max(min(output, self.out_max), self.out_min)

def simulate():
    arduino = ArduinoMock()
    temp_pid = PID(kp=80.0, ki=0.5, kd=20.0) # Adjusted PID for more stability

    temp_start = 10.0
    temp_end = 60.0
    temp_step = 10.0
    zener_steps = 64 # Reduced from 256 for faster simulation but still good graph

    print("# TargetTemp,ActualTemp,Duty,ConvVolt,ZenerVolt")

    for target_t in [temp_start + i * temp_step for i in range(int((temp_end-temp_start)/temp_step) + 1)]:
        temp_pid.setpoint = target_t

        # Settle temperature
        # 2000 steps of 0.1s = 200s simulated time
        for _ in range(2000):
            arduino.update_physics()
            output = temp_pid.compute(arduino.adt75.read_temp())
            if output > 0:
                arduino.ledcWrite(1, int(abs(output))) # HEATING
                arduino.ledcWrite(0, 0)                # COOLING
            else:
                arduino.ledcWrite(0, int(abs(output))) # COOLING
                arduino.ledcWrite(1, 0)                # HEATING

        # Perform Zener Sweep
        # Step increment calculation matching .ino: (PWM_MAX / zenerSteps + 1)
        pwm_max = 1023
        step_inc = (pwm_max // zener_steps) + 1

        for d in range(0, pwm_max + 1, step_inc):
            arduino.ledcWrite(2, d)
            # Short stabilization for the converter
            for _ in range(10):
                arduino.update_physics()

            raw_conv = arduino.analogRead(34)
            raw_zener = arduino.analogRead(35)

            v_conv = raw_conv * (3.3 / 4095.0) * 11.0
            v_zener = raw_zener * (3.3 / 4095.0) * 11.0

            print(f"{target_t:.1f},{arduino.adt75.temp:.2f},{d},{v_conv:.3f},{v_zener:.3f}")

        print("") # Blank line for gnuplot compatibility (per step)

    print("# ALL SWEEPS COMPLETE")

if __name__ == "__main__":
    simulate()
