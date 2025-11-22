import io
import base64
import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from flask import Flask, render_template_string, request, jsonify
from scipy.interpolate import RegularGridInterpolator

app = Flask(__name__)

print("Initializing application... Configuring UI and physics...")

# --- VEHICLE DATABASE ---
VEHICLES = {
    'ferrari': {'name': 'Ferrari 488', 'm': 1400.0, 'b': 20.0, 'u_max': 8000.0},
    'fiat': {'name': 'Fiat Punto', 'm': 1100.0, 'b': 55.0, 'u_max': 2500.0},
    'truck': {'name': 'Truck', 'm': 15000.0, 'b': 150.0, 'u_max': 12000.0}
}

# --- FUZZY LOGIC CONFIGURATION ---
max_error = 25.0
zakres_e = np.arange(-max_error, max_error + 0.1, 0.1)
max_delta_e = 5.0
zakres_ce = np.arange(-max_delta_e, max_delta_e + 0.1, 0.1)
max_delta_u = 600.0
zakres_cu = np.arange(-max_delta_u, max_delta_u + 1, 1.0)

e_in = ctrl.Antecedent(zakres_e, 'e')
ce_in = ctrl.Antecedent(zakres_ce, 'ce')
cu_out = ctrl.Consequent(zakres_cu, 'cu')

# Linguistic variables
names = ['DU', 'SU', 'MU', 'Z', 'MD', 'SD', 'DD']
e_in.automf(7, names=names)
ce_in.automf(7, names=names)
cu_out.automf(7, names=names)

# Rule Base
rule_table = [
    ['DU', 'DU', 'DU', 'DU', 'SU', 'MU', 'Z'],
    ['DU', 'DU', 'DU', 'SU', 'MU', 'Z', 'MD'],
    ['DU', 'DU', 'SU', 'MU', 'Z', 'MD', 'SD'],
    ['DU', 'SU', 'MU', 'Z', 'MD', 'SD', 'DD'],
    ['SU', 'MU', 'Z', 'MD', 'SD', 'DD', 'DD'],
    ['MU', 'Z', 'MD', 'SD', 'DD', 'DD', 'DD'],
    ['Z', 'MD', 'SD', 'DD', 'DD', 'DD', 'DD']
]

reguly_lista = []
for i_e, row in enumerate(rule_table):
    for i_ce, val in enumerate(row):
        reguly_lista.append(ctrl.Rule(e_in[names[i_e]] & ce_in[names[i_ce]], cu_out[val]))

system_sterowania = ctrl.ControlSystem(reguly_lista)
fuzzy_sim = ctrl.ControlSystemSimulation(system_sterowania)

# --- PRE-CALCULATION ---
print("Generating control map...")
e_grid = np.linspace(-max_error, max_error, 30)
ce_grid = np.linspace(-max_delta_e, max_delta_e, 30)
cu_matrix = np.zeros((len(e_grid), len(ce_grid)))

for i, e_val in enumerate(e_grid):
    for j, ce_val in enumerate(ce_grid):
        fuzzy_sim.input['e'] = e_val
        fuzzy_sim.input['ce'] = ce_val
        try:
            fuzzy_sim.compute()
            cu_matrix[i, j] = fuzzy_sim.output['cu']
        except:
            cu_matrix[i, j] = 0.0

fuzzy_interpolator = RegularGridInterpolator((e_grid, ce_grid), cu_matrix, bounds_error=False, fill_value=None)
print("Ready.")


# --- SIMULATION PHYSICS ---
class Vehicle:
    def __init__(self, m, b, u_max, dt):
        self.m = m;
        self.b = b;
        self.u_max = u_max;
        self.dt = dt;
        self.v = 0.0

    def step(self, u):
        u = np.clip(u, 0, self.u_max)
        dv_dt = (u - self.b * self.v) / self.m
        self.v += dv_dt * self.dt
        self.v = max(0.0, self.v)
        return self.v


class PID:
    def __init__(self, Kp, Ki, Kd, dt, u_max):
        self.Kp = Kp;
        self.Ki = Ki;
        self.Kd = Kd;
        self.dt = dt;
        self.u_max = u_max
        self.integral = 0.0;
        self.prev_error = 0.0

    def compute(self, sp, pv):
        error = sp - pv
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -2000, 2000)
        derivative = (error - self.prev_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return np.clip(output, 0, self.u_max)


def run_simulation(vehicle_key, Kp, Ki, Kd, target_speed_kmh, sim_time):
    car = VEHICLES.get(vehicle_key, VEHICLES['fiat'])
    dt = 0.15

    # Dynamic Simulation Time
    T_sim = float(sim_time)
    N = int(T_sim / dt) + 1
    time = np.linspace(0, T_sim, N)

    target_speed_ms = target_speed_kmh / 3.6
    setpoint_ms = np.zeros(N)
    setpoint_ms[int(3 / dt):] = target_speed_ms

    pid = PID(Kp, Ki, Kd, dt, car['u_max'])
    veh_pid = Vehicle(car['m'], car['b'], car['u_max'], dt)
    veh_fuz = Vehicle(car['m'], car['b'], car['u_max'], dt)

    u_fuz_acc = 0.0
    prev_e = 0.0
    res_v_pid_ms, res_v_fuz_ms = np.zeros(N), np.zeros(N)
    res_u_pid, res_u_fuz = np.zeros(N), np.zeros(N)

    for k in range(N):
        sp = setpoint_ms[k]

        # PID
        vp = veh_pid.v
        up = pid.compute(sp, vp)
        veh_pid.step(up)
        res_v_pid_ms[k] = vp;
        res_u_pid[k] = up

        # Fuzzy
        vf = veh_fuz.v
        e = sp - vf
        ce = (e - prev_e) / dt
        du = fuzzy_interpolator((e, ce))
        u_fuz_acc += du
        uf = np.clip(u_fuz_acc, 0, car['u_max'])
        veh_fuz.step(uf)
        prev_e = e
        res_v_fuz_ms[k] = vf;
        res_u_fuz[k] = uf

    res_v_pid_kmh = res_v_pid_ms * 3.6
    res_v_fuz_kmh = res_v_fuz_ms * 3.6
    setpoint_kmh = setpoint_ms * 3.6

    # PLOTTING
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

    # Speed
    ax1.plot(time, setpoint_kmh, 'k--', label=f'Target ({target_speed_kmh} km/h)', alpha=0.5)
    ax1.plot(time, res_v_pid_kmh, 'b-', label='PID', linewidth=1.5)
    ax1.plot(time, res_v_fuz_kmh, 'r-', label='Fuzzy', linewidth=1.5)
    ax1.set_title(f'Vehicle: {car["name"]}', fontsize=12)
    ax1.set_ylabel('Speed [km/h]')
    ax1.legend(loc='lower right')
    ax1.grid(True, alpha=0.3)

    # Force
    ax2.plot(time, res_u_pid, 'b-', alpha=0.4, label='PID Output')
    ax2.plot(time, res_u_fuz, 'r-', alpha=0.4, label='Fuzzy Output')
    ax2.set_ylabel('Force [N]')
    ax2.set_xlabel('Time [s]')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    buf = io.BytesIO()
    plt.tight_layout(pad=1.5)
    plt.savefig(buf, format='png', dpi=100)
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.getvalue()).decode('utf-8')


# --- HTML TEMPLATE ---
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Cruise Control Simulation</title>
    <script src="https://cdn.tailwindcss.com"></script>
</head>
<body class="bg-gray-100 text-gray-800 font-sans h-screen overflow-hidden flex flex-col">

    <!-- HEADER -->
    <header class="bg-blue-800 text-white p-4 shadow-md flex justify-center items-center relative">
        <h1 class="text-2xl font-bold">Cruise Control: PID vs Fuzzy Logic</h1>
    </header>

    <div class="flex flex-1 overflow-hidden">
        <!-- LEFT PANEL -->
        <div class="w-80 bg-white shadow-lg p-6 overflow-y-auto z-10 flex-shrink-0">
            <h2 class="font-bold text-lg mb-4 text-gray-700 border-b pb-2">Control Panel</h2>
            <form id="controlForm" class="space-y-6">

                <!-- Vehicle -->
                <div>
                    <label class="block text-xs font-bold uppercase text-gray-500 mb-1">Vehicle</label>
                    <select id="vehicle" name="vehicle" class="w-full p-2 border rounded bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500 cursor-pointer">
                        <option value="ferrari">Ferrari 488 (Sport)</option>
                        <option value="fiat" selected>Fiat Punto (City)</option>
                        <option value="truck">Truck (Heavy)</option>
                    </select>
                </div>

                <!-- Speed -->
                <div class="bg-gray-50 p-3 rounded border border-gray-200">
                    <div class="flex justify-between text-sm mb-1">
                        <span class="font-bold text-gray-600">Target Speed</span>
                        <span class="font-mono font-bold text-blue-700"><span id="val_speed">72</span> km/h</span>
                    </div>
                    <input type="range" id="speed" name="speed" min="10" max="180" step="1" value="72" class="w-full accent-green-600 h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer">
                </div>

                <!-- Time -->
                <div class="bg-gray-50 p-3 rounded border border-gray-200">
                    <div class="flex justify-between text-sm mb-1">
                        <span class="font-bold text-gray-600">Simulation Time</span>
                        <span class="font-mono font-bold text-purple-700"><span id="val_time">70</span> s</span>
                    </div>
                    <input type="range" id="time" name="time" min="10" max="250" step="10" value="70" class="w-full accent-purple-600 h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer">
                </div>

                <!-- PID -->
                <div class="bg-blue-50 p-4 rounded-lg border border-blue-100">
                    <p class="text-xs font-bold text-blue-800 mb-3 uppercase">PID Settings</p>
                    <div class="mb-4">
                        <div class="flex justify-between text-sm mb-1"><span>Kp (Proportional)</span><span id="val_kp" class="font-mono font-bold">300</span></div>
                        <input type="range" id="kp" name="kp" min="10" max="1000" step="10" value="300" class="w-full accent-blue-600 h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer">
                    </div>
                    <div class="mb-4">
                        <div class="flex justify-between text-sm mb-1"><span>Ki (Integral)</span><span id="val_ki" class="font-mono font-bold">15</span></div>
                        <input type="range" id="ki" name="ki" min="0" max="100" step="0.5" value="15" class="w-full accent-blue-600 h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer">
                    </div>
                    <div>
                        <div class="flex justify-between text-sm mb-1"><span>Kd (Derivative)</span><span id="val_kd" class="font-mono font-bold">50</span></div>
                        <input type="range" id="kd" name="kd" min="0" max="200" step="1" value="50" class="w-full accent-blue-600 h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer">
                    </div>
                </div>
            </form>
        </div>

        <!-- PLOT -->
        <div class="flex-1 bg-gray-50 p-2 flex items-center justify-center relative">
            <div id="loading" class="absolute top-4 right-4 bg-white px-3 py-1 rounded shadow text-xs text-blue-600 font-bold hidden z-20">
                Calculating...
            </div>
            <div class="w-full h-full flex items-center justify-center overflow-hidden">
                <img id="plotImage" src="" alt="Simulation Plot" class="max-w-full max-h-full object-contain shadow-lg bg-white rounded">
            </div>
        </div>
    </div>

    <script>
        document.addEventListener('DOMContentLoaded', function() {
            const form = document.getElementById('controlForm');
            const plotImg = document.getElementById('plotImage');
            const loading = document.getElementById('loading');
            let timeout = null;

            function updateVal(id) { 
                const el = document.getElementById(id);
                if(el) document.getElementById('val_' + id).innerText = el.value; 
            }

            // Listeners
            ['kp', 'ki', 'kd', 'speed', 'time'].forEach(id => {
                const el = document.getElementById(id);
                if (el) {
                    el.addEventListener('input', () => {
                        updateVal(id);
                        clearTimeout(timeout);
                        timeout = setTimeout(updateSimulation, 100);
                    });
                }
            });

            const vehicleSelect = document.getElementById('vehicle');
            if (vehicleSelect) {
                vehicleSelect.addEventListener('change', function() {
                    updateSimulation();
                });
            }

            function updateSimulation() {
                loading.classList.remove('hidden');
                const formData = new FormData(form);

                fetch('/simulate', { method: 'POST', body: formData })
                .then(r => r.json())
                .then(d => { 
                    plotImg.src = 'data:image/png;base64,' + d.image; 
                    loading.classList.add('hidden'); 
                })
                .catch(e => { 
                    console.error("Error:", e); 
                    loading.classList.add('hidden'); 
                });
            }

            updateSimulation();
        });
    </script>
</body>
</html>
"""


@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route('/simulate', methods=['POST'])
def simulate():
    try:
        v = request.form.get('vehicle', 'fiat')
        kp = float(request.form.get('kp', 300))
        ki = float(request.form.get('ki', 15))
        kd = float(request.form.get('kd', 50))
        speed_kmh = float(request.form.get('speed', 72.0))
        sim_time = float(request.form.get('time', 70.0))

        return jsonify({'image': run_simulation(v, kp, ki, kd, speed_kmh, sim_time)})
    except Exception as e:
        print(e);
        return jsonify({'error': str(e)}), 500


if __name__ == '__main__':
    app.run(debug=True, use_reloader=False)
