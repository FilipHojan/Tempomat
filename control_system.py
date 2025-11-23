import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from flask import Flask, render_template_string, request, jsonify
from scipy.interpolate import RegularGridInterpolator
import json

app = Flask(__name__)

print("--- SYSTEM TEMPOMATU (WEB) ---")
print("Inicjalizacja...")

# --- BAZA DANYCH POJAZDÓW ---
VEHICLES = {
    'ferrari': {'name': 'Ferrari 488', 'm': 1400.0, 'b': 20.0, 'u_max': 8000.0},
    'fiat': {'name': 'Fiat Punto', 'm': 1100.0, 'b': 55.0, 'u_max': 2500.0},
    'truck': {'name': 'Truck (TIR)', 'm': 15000.0, 'b': 150.0, 'u_max': 12000.0}
}

# --- KONFIGURACJA ROZMYTA ---
max_error = 25.0
zakres_e = np.arange(-max_error, max_error + 0.1, 0.1)
max_delta_e = 5.0
zakres_ce = np.arange(-max_delta_e, max_delta_e + 0.1, 0.1)
max_delta_u = 600.0
zakres_cu = np.arange(-max_delta_u, max_delta_u + 1, 1.0)

names = ['DU', 'SU', 'MU', 'Z', 'MD', 'SD', 'DD']

rule_table = [
    ['DU', 'DU', 'DU', 'DU', 'SU', 'MU', 'Z'],
    ['DU', 'DU', 'DU', 'SU', 'MU', 'Z', 'MD'],
    ['DU', 'DU', 'SU', 'MU', 'Z', 'MD', 'SD'],
    ['DU', 'SU', 'MU', 'Z', 'MD', 'SD', 'DD'],
    ['SU', 'MU', 'Z', 'MD', 'SD', 'DD', 'DD'],
    ['MU', 'Z', 'MD', 'SD', 'DD', 'DD', 'DD'],
    ['Z', 'MD', 'SD', 'DD', 'DD', 'DD', 'DD']
]


# --- BUILDER ---
def build_interpolator(mf_type='tri'):
    e = ctrl.Antecedent(zakres_e, 'e')
    ce = ctrl.Antecedent(zakres_ce, 'ce')
    cu = ctrl.Consequent(zakres_cu, 'cu')

    if mf_type == 'tri':
        e.automf(7, names=names)
        ce.automf(7, names=names)
        cu.automf(7, names=names)
    elif mf_type == 'trap':
        def make_traps(variable, universe):
            step = (universe.max() - universe.min()) / (len(names) - 1)
            top_width = step * 0.3
            for i, label in enumerate(names):
                center = universe.min() + i * step
                a = center - step
                b = center - top_width
                c = center + top_width
                d = center + step
                variable[label] = fuzz.trapmf(variable.universe, [a, b, c, d])

        make_traps(e, zakres_e)
        make_traps(ce, zakres_ce)
        make_traps(cu, zakres_cu)

    rules = []
    for i_e, row in enumerate(rule_table):
        for i_ce, val in enumerate(row):
            rules.append(ctrl.Rule(e[names[i_e]] & ce[names[i_ce]], cu[val]))

    system = ctrl.ControlSystem(rules)
    sim = ctrl.ControlSystemSimulation(system)

    e_grid = np.linspace(-max_error, max_error, 30)
    ce_grid = np.linspace(-max_delta_e, max_delta_e, 30)
    cu_matrix = np.zeros((len(e_grid), len(ce_grid)))

    for i, e_val in enumerate(e_grid):
        for j, ce_val in enumerate(ce_grid):
            sim.input['e'] = e_val
            sim.input['ce'] = ce_val
            try:
                sim.compute()
                cu_matrix[i, j] = sim.output['cu']
            except:
                cu_matrix[i, j] = 0.0

    return RegularGridInterpolator((e_grid, ce_grid), cu_matrix, bounds_error=False, fill_value=None)


INTERPOLATORS = {
    'tri': build_interpolator('tri'),
    'trap': build_interpolator('trap')
}
print("Gotowe. Serwer aktywny.")


# --- FIZYKA I PID ---
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


# --- SYMULACJA ---
def run_simulation(vehicle_key, Kp, Ki, Kd, target_speed_kmh, sim_time, mf_type):
    car = VEHICLES.get(vehicle_key, VEHICLES['fiat'])
    dt = 0.15
    T_sim = float(sim_time)
    N = int(T_sim / dt) + 1
    time = np.linspace(0, T_sim, N)

    target_speed_ms = target_speed_kmh / 3.6
    setpoint_ms = np.zeros(N)
    setpoint_ms[int(3 / dt):] = target_speed_ms

    pid = PID(Kp, Ki, Kd, dt, car['u_max'])
    veh_pid = Vehicle(car['m'], car['b'], car['u_max'], dt)
    veh_fuz = Vehicle(car['m'], car['b'], car['u_max'], dt)

    current_fuzzy = INTERPOLATORS.get(mf_type, INTERPOLATORS['tri'])

    u_fuz_acc = 0.0
    prev_e = 0.0

    res_v_pid = []
    res_v_fuz = []
    res_u_pid = []
    res_u_fuz = []

    setpoint_kmh = (setpoint_ms * 3.6).tolist()

    for k in range(N):
        sp = setpoint_ms[k]

        # PID
        vp = veh_pid.v
        up = pid.compute(sp, vp)
        veh_pid.step(up)

        # Fuzzy
        vf = veh_fuz.v
        e = sp - vf
        ce = (e - prev_e) / dt
        du = current_fuzzy((e, ce))
        u_fuz_acc += du
        uf = np.clip(u_fuz_acc, 0, car['u_max'])
        veh_fuz.step(uf)
        prev_e = e

        res_v_pid.append(float(vp * 3.6))
        res_v_fuz.append(float(vf * 3.6))
        res_u_pid.append(float(up))
        res_u_fuz.append(float(uf))

    # Tworzenie danych dla Plotly
    mf_label = "Triangular" if mf_type == 'tri' else "Trapezoidal"
    time_list = time.tolist()

    data = [
        # --- GÓRNY WYKRES (PRĘDKOŚĆ) ---
        {
            'x': time_list, 'y': setpoint_kmh, 'type': 'scatter', 'mode': 'lines',
            'name': f'Target ({target_speed_kmh} km/h)',
            'line': {'color': 'black', 'dash': 'dash', 'width': 2},
            'xaxis': 'x', 'yaxis': 'y'
        },
        {
            'x': time_list, 'y': res_v_pid, 'type': 'scatter', 'mode': 'lines',
            'name': 'PID Speed',
            'line': {'color': 'blue', 'width': 2},
            'xaxis': 'x', 'yaxis': 'y'
        },
        {
            'x': time_list, 'y': res_v_fuz, 'type': 'scatter', 'mode': 'lines',
            'name': f'Fuzzy Speed ({mf_label})',
            'line': {'color': 'red', 'width': 2},
            'xaxis': 'x', 'yaxis': 'y'
        },

        # --- DOLNY WYKRES (SIŁA) ---
        {
            'x': time_list, 'y': res_u_pid, 'type': 'scatter', 'mode': 'lines',
            'name': 'PID Force',
            'line': {'color': 'rgba(0,0,255,0.4)', 'width': 1},
            'xaxis': 'x', 'yaxis': 'y2',
            'showlegend': True
        },
        {
            'x': time_list, 'y': res_u_fuz, 'type': 'scatter', 'mode': 'lines',
            'name': 'Fuzzy Force',
            'line': {'color': 'rgba(255,0,0,0.4)', 'width': 1},
            'xaxis': 'x', 'yaxis': 'y2',
            'showlegend': True
        }
    ]

    layout = {
        # Tytuł wykresu - Wyśrodkowany, uproszczony
        'title': {
            'text': f'Simulation: {car["name"]}',
            'font': {'size': 18},
            'x': 0.5,  # Środek
            'xanchor': 'center'
        },
        'grid': {'rows': 2, 'columns': 1, 'pattern': 'independent'},

        # OŚ Y1 (Górna - Prędkość)
        'yaxis': {
            'title': '<b>Speed [km/h]</b>',
            'domain': [0.55, 1],
            'zeroline': False
        },

        # OŚ Y2 (Dolna - Siła)
        'yaxis2': {
            'title': '<b>Force [N]</b>',
            'domain': [0, 0.45],
            'zeroline': False
        },

        # OŚ X (Główna)
        'xaxis': {
            'title': '<b>Time [s]</b>',
            'anchor': 'y2',
            'domain': [0, 1],
            'showticklabels': True
        },

        'margin': {'l': 60, 'r': 10, 't': 50, 'b': 60},

        # Legenda
        'legend': {
            'orientation': 'v',
            'yanchor': 'middle',
            'y': 0.5,
            'xanchor': 'right',
            'x': 1.15
        },

        'autosize': True
    }

    return json.dumps({'data': data, 'layout': layout})


# --- HTML TEMPLATE ---
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Cruise Control Simulation</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <!-- PLOTLY JS -->
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
</head>
<body class="bg-gray-100 text-gray-800 font-sans h-screen overflow-hidden flex flex-col">

    <!-- HEADER -->
    <header class="bg-blue-800 text-white p-3 shadow-md flex items-center justify-center relative shrink-0 px-6">
        <h1 class="text-2xl font-bold">Cruise Control: PID vs FUZZY</h1>
    </header>

    <div class="flex flex-1 overflow-hidden">
        <!-- SIDEBAR -->
        <div class="w-72 bg-white shadow-lg p-4 overflow-y-auto z-10 flex-shrink-0 border-r">
            <h2 class="font-bold text-base mb-3 text-gray-700 border-b pb-1">Control Panel</h2>
            <form id="controlForm" class="space-y-4 text-sm">

                <div>
                    <label class="block text-xs font-bold uppercase text-gray-500 mb-1">Vehicle</label>
                    <select id="vehicle" name="vehicle" class="w-full p-1.5 border rounded bg-gray-50 focus:ring-1 focus:ring-blue-500">
                        <option value="ferrari">Ferrari 488 (Sport)</option>
                        <option value="fiat" selected>Fiat Punto (City)</option>
                        <option value="truck">Truck (Heavy)</option>
                    </select>
                </div>

                <div>
                    <label class="block text-xs font-bold uppercase text-indigo-800 mb-1">Membership Function</label>
                    <select id="mf_type" name="mf_type" class="w-full p-1.5 border rounded bg-indigo-50 border-indigo-100 focus:ring-1 focus:ring-indigo-500">
                        <option value="tri" selected>Triangular (Standard)</option>
                        <option value="trap">Trapezoidal (Stable)</option>
                    </select>
                </div>

                <div class="bg-gray-50 p-2 rounded border border-gray-200">
                    <div class="flex justify-between mb-1">
                        <span class="font-semibold text-gray-600">Target</span>
                        <span class="font-mono font-bold text-blue-700"><span id="val_speed">72</span> km/h</span>
                    </div>
                    <input type="range" id="speed" name="speed" min="10" max="180" step="1" value="72" class="w-full accent-green-600 h-1.5 bg-gray-200 rounded-lg appearance-none cursor-pointer">
                </div>

                <div class="bg-gray-50 p-2 rounded border border-gray-200">
                    <div class="flex justify-between mb-1">
                        <span class="font-semibold text-gray-600">Sim Time</span>
                        <span class="font-mono font-bold text-purple-700"><span id="val_time">70</span> s</span>
                    </div>
                    <input type="range" id="time" name="time" min="10" max="250" step="10" value="70" class="w-full accent-purple-600 h-1.5 bg-gray-200 rounded-lg appearance-none cursor-pointer">
                </div>

                <div class="bg-blue-50 p-3 rounded-lg border border-blue-100 space-y-3">
                    <p class="text-xs font-bold text-blue-800 uppercase">PID Settings</p>
                    <div>
                        <div class="flex justify-between mb-1"><span>Kp</span><span id="val_kp" class="font-mono font-bold">300</span></div>
                        <input type="range" id="kp" name="kp" min="10" max="1000" step="10" value="300" class="w-full accent-blue-600 h-1.5 bg-gray-200 rounded-lg cursor-pointer">
                    </div>
                    <div>
                        <div class="flex justify-between mb-1"><span>Ki</span><span id="val_ki" class="font-mono font-bold">15</span></div>
                        <input type="range" id="ki" name="ki" min="0" max="100" step="0.5" value="15" class="w-full accent-blue-600 h-1.5 bg-gray-200 rounded-lg cursor-pointer">
                    </div>
                    <div>
                        <div class="flex justify-between mb-1"><span>Kd</span><span id="val_kd" class="font-mono font-bold">50</span></div>
                        <input type="range" id="kd" name="kd" min="0" max="200" step="1" value="50" class="w-full accent-blue-600 h-1.5 bg-gray-200 rounded-lg cursor-pointer">
                    </div>
                </div>
            </form>
        </div>

        <!-- PLOT AREA -->
        <div class="flex-1 bg-white p-0 relative flex flex-col">
            <div id="loading" class="absolute top-4 right-4 bg-white/80 backdrop-blur px-3 py-1 rounded shadow text-xs text-blue-600 font-bold hidden z-50 border">
                Computing...
            </div>
            <!-- Plotly Container -->
            <div id="chart" class="w-full h-full"></div>
        </div>
    </div>

    <script>
        document.addEventListener('DOMContentLoaded', function() {
            const form = document.getElementById('controlForm');
            const loading = document.getElementById('loading');
            let timeout = null;

            function updateVal(id) { 
                const el = document.getElementById(id);
                if(el) document.getElementById('val_' + id).innerText = el.value; 
            }

            ['kp', 'ki', 'kd', 'speed', 'time'].forEach(id => {
                const el = document.getElementById(id);
                if (el) {
                    el.addEventListener('input', () => {
                        updateVal(id);
                        clearTimeout(timeout);
                        timeout = setTimeout(updateSimulation, 150);
                    });
                }
            });

            ['vehicle', 'mf_type'].forEach(id => {
                const el = document.getElementById(id);
                if (el) {
                    el.addEventListener('change', function() {
                        updateSimulation();
                    });
                }
            });

            function updateSimulation() {
                loading.classList.remove('hidden');
                const formData = new FormData(form);

                fetch('/simulate', { method: 'POST', body: formData })
                .then(r => r.json())
                .then(response => { 
                    const chartData = JSON.parse(response);
                    const config = {responsive: true, displayModeBar: false};
                    Plotly.react('chart', chartData.data, chartData.layout, config);
                    loading.classList.add('hidden'); 
                })
                .catch(e => { 
                    console.error("Error:", e); 
                    loading.classList.add('hidden'); 
                });
            }

            updateSimulation();

            window.onresize = function() {
                Plotly.Plots.resize('chart');
            };
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
        mf_type = request.form.get('mf_type', 'tri')
        kp = float(request.form.get('kp', 300))
        ki = float(request.form.get('ki', 15))
        kd = float(request.form.get('kd', 50))
        speed_kmh = float(request.form.get('speed', 72.0))
        sim_time = float(request.form.get('time', 70.0))

        return jsonify(run_simulation(v, kp, ki, kd, speed_kmh, sim_time, mf_type))
    except Exception as e:
        print(f"Server Error: {e}")
        return jsonify({'error': str(e)}), 500


if __name__ == '__main__':
    app.run(debug=True, use_reloader=False)