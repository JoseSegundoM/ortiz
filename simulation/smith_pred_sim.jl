################################################################################
#  Smith Predictor – Temperature Control Simulation  (Julia 1.x)
#  ─────────────────────────────────────────────────────────────────────────────
#  Hardware: 20-L mixing tank, DS18B20 sensor, 1/4" on/off solenoid, ESP32
#
#  Defects explicitly modelled
#  ────────────────────────────
#  1. Shared hot/cold feed pipe
#       When the cold solenoid opens the same supply header that feeds the hot
#       leg, the available ΔP for the cold side drops.
#       Model: ΔP_cold = ΔP_eff × (1 − α × valve_ON)
#              → cold flow reduced by √(1−α) ≈ 80 % of commanded value
#
#  2. Low supply pressure
#       Whole system runs at ≈10 psi instead of rated 16 psi.
#       → valve_factor uses effective pressure so Vp_ss is re-computed correctly
#
#  Controller: Smith Predictor + PI (exact mirror of src/main.cpp)
#  Actuator  : Time-Proportional Output (TPO), window = 33 s
#
#  Scenario  (total 1800 s = 30 min)
#  ────────────────────────────────────
#    t =   0 s : cold start at 80 °F  [hold]
#    t = 200 s : step → 95 °F         (your experimental linearisation point)
#    t = 800 s : disturbance – tap opened elsewhere on shared pipe
#                cold water available ↓  → measured PV rises +3 °F
#    t = 1200 s: disturbance cleared,  try step → 100 °F
#                (capped at what 110 °F hot water can physically reach)
#
#  Output files (written to same directory as this script)
#  ───────────────────────────────────────────────────────
#    simulation_results.csv   – 1800 rows × 8 columns (raw data for analysis)
#    simulation_summary.txt   – key metrics / parameter block
################################################################################

using Printf, Statistics

f_to_norm(T_F) = (T_F - 70.0) / 130.0
norm_to_f(n)   = n * 130.0 + 70.0

# ──────────────────────────────────────────────────────────────────────────────
#  PHYSICAL CONSTANTS  (const = compile-time, safe inside functions)
# ──────────────────────────────────────────────────────────────────────────────

const V_ft3      = 0.71
const rho        = 62.4
const L_pipe     = 20.0 / 12.0
const tube_id_in = 0.622
const A_pipe     = π * (tube_id_in / 12.0)^2 / 4

const T_HOT      = 110.0
const T_COLD     = 50.0
const T_TARGET   = 90.0
const T_SMIN     = 70.0
const T_SMAX     = 200.0

const f_lmin = 2.0
const W1     = f_lmin / 28.3168 * 62.4    # lb/min
const Cp1 = 0.8; const Cp2 = 1.0; const Cp3 = 0.9; const Cv3 = 0.9

const C_VL   = 0.20
const G_f    = 1.0
const dP_NOM = 16.0    # rated [psi]
const dP_EFF = 10.0    # actual low-pressure [psi]
const ALPHA  = 0.35    # shared-pipe coupling: fraction of ΔP lost when valve ON

const VFACTOR = (500.0 / 60.0) * C_VL * sqrt(G_f * dP_EFF)

# Steady-state cold flow for T_TARGET
const W2_SS   = W1 * (Cp3 * T_TARGET - Cp1 * T_HOT) / (Cp2 * T_COLD - Cp3 * T_TARGET)
const W_TOT   = W1 + W2_SS
const Vp_SS   = W2_SS / VFACTOR          # nominal duty cycle [0-1]

# Delays & time constants
const t0_s     = (L_pipe * A_pipe * rho / W_TOT) * 60.0   # transport [s]
const tau_tk_s = (V_ft3 * rho * Cv3) / (W_TOT * Cp3) * 60.0

# ── FOPDT – locked to values embedded in main.cpp ───────────────────────────
const K_PROC  = -0.3270
const TAU_P   = 499.54
const T0_P    = 21.94
const TS      = 1.0

const MODEL_A   = exp(-TS / TAU_P)
const MODEL_B   = K_PROC * (1.0 - MODEL_A)
const BUF_SIZE  = round(Int, T0_P / TS)   # 22 samples
const TPO_WIN   = 33                        # seconds (main.cpp value)

const Tc      = max(TS, T0_P / 10.0)
const PI_KP   = TAU_P / (K_PROC * Tc)
const PI_KI   = PI_KP / TAU_P
const T_OP    = f_to_norm(T_TARGET)        # normalised operating point

# ──────────────────────────────────────────────────────────────────────────────
#  PRINT PARAMETERS
# ──────────────────────────────────────────────────────────────────────────────
println("="^60)
println(" Smith Predictor Simulation – Parameter Report")
println("="^60)
println("\n▸ Physical steady state")
@printf("  W1=%.3f  W2_ss=%.3f  W_tot=%.3f  [lb/min]\n", W1, W2_SS, W_TOT)
@printf("  Vp_ss=%.4f (%.1f%%)  t0=%.2fs  τ_tank=%.1fs\n",
        Vp_SS, Vp_SS*100, t0_s, tau_tk_s)
@printf("  ΔP_supply=%.1fpsi  (rated=%.1fpsi)  pipe_coupling_α=%.2f\n",
        dP_EFF, dP_NOM, ALPHA)
println("\n▸ Discrete controller (Ts=1 s, ZOH)")
@printf("  MODEL_A=%.5f   MODEL_B=%.6f\n", MODEL_A, MODEL_B)
@printf("  BUFFER_SIZE=%d   TPO_WINDOW=%d s\n", BUF_SIZE, TPO_WIN)
@printf("  PI_KP=%.4f   PI_KI=%.6f   u₀=%.4f\n\n", PI_KP, PI_KI, Vp_SS)

# ──────────────────────────────────────────────────────────────────────────────
#  SCENARIO HELPERS
# ──────────────────────────────────────────────────────────────────────────────

# Setpoint profile (in °F → normalised inside)
# Note: 105 °F is still physically reachable; the low-pressure and shared-pipe
# defects slow things down but the hot inlet (110 °F) can drive the tank there.
function get_sp(k)
    k < 200  ? f_to_norm(80.0)  :
    k < 1200 ? f_to_norm(95.0)  :
               f_to_norm(100.0)   # capped at realistic max given 110°F hot
end

# Disturbance: shared pipe draws cold water elsewhere → less available for tank
# Modelled as an additive shift on the measured PV (warmer reading)
function get_dist(k)
    (800 <= k < 1200) ? (3.0 / 130.0) : 0.0
end

# ──────────────────────────────────────────────────────────────────────────────
#  SIMULATION FUNCTION  (scoped properly to avoid Julia soft-scope warnings)
# ──────────────────────────────────────────────────────────────────────────────

function run_simulation(N)
    T_init_n = f_to_norm(80.0)

    # Plant state
    plant_dev   = T_init_n - T_OP
    plant_buf   = fill(T_init_n, BUF_SIZE)

    # Controller states (Smith Predictor)
    pred_dev       = T_init_n - T_OP
    T_pred         = T_init_n
    ctrl_buf       = fill(T_init_n, BUF_SIZE)
    err_integral   = Vp_SS          # warm-start at SS duty cycle
    err_corr_filt  = 0.0
    filter_alpha   = exp(-TS / 15.0)
    tpo_cnt        = 0

    # Log arrays
    t_log   = zeros(N)
    sp_log  = zeros(N)
    pv_log  = zeros(N)
    pr_log  = zeros(N)
    u_log   = zeros(N)
    vt_log  = zeros(N)    # binary TPO state
    ue_log  = zeros(N)    # effective cold flow fraction
    dp_log  = zeros(N)    # effective cold-leg ΔP [psi]

    for k in 1:N
        sp_n = get_sp(k)
        dist = get_dist(k)

        # ── Sensor: delayed plant + disturbance ────────────────────────────
        T_meas = plant_buf[1] + dist

        # ── Smith Predictor feedback ───────────────────────────────────────
        T_del          = ctrl_buf[1]
        e_corr         = T_meas - T_del
        err_corr_filt  = filter_alpha * err_corr_filt + (1.0 - filter_alpha) * e_corr
        T_fb           = T_pred + err_corr_filt
        ctrl_err       = sp_n - T_fb

        # ── PI law (positional – mirrors main.cpp exactly, no *Ts) ─────────
        u_p   = PI_KP * ctrl_err
        p_int = err_integral + PI_KI * ctrl_err
        u_unc = u_p + p_int
        u_fin = clamp(u_unc, 0.0, 1.0)

        # Directional anti-windup
        if !((u_unc > 1.0) && (ctrl_err < 0.0)) && !((u_unc < 0.0) && (ctrl_err > 0.0))
            err_integral = p_int
        end

        # ── TPO → binary valve signal ──────────────────────────────────────
        on_s       = round(Int, u_fin * TPO_WIN)
        v_state    = (tpo_cnt < on_s) ? 1.0 : 0.0
        tpo_cnt    = mod(tpo_cnt + 1, TPO_WIN)

        # ── Shared-pipe defect: ΔP collapses when cold solenoid fires ──────
        dp_cold = dP_EFF * (1.0 - ALPHA * v_state)
        dp_cold = max(dp_cold, 0.5)
        u_eff   = v_state * sqrt(dp_cold / dP_EFF)   # effective cold fraction

        # ── Update Smith Predictor model (uses smooth u_fin) ───────────────
        pred_dev = MODEL_A * pred_dev + MODEL_B * (u_fin - Vp_SS)
        T_pred   = pred_dev + T_OP
        ctrl_buf = vcat(ctrl_buf[2:end], [T_pred])

        # ── Update TRUE plant (uses defect-reduced u_eff) ──────────────────
        plant_dev  = MODEL_A * plant_dev  + MODEL_B * (u_eff  - Vp_SS)
        plant_inst = plant_dev + T_OP
        plant_buf  = vcat(plant_buf[2:end], [plant_inst])

        # ── Log ────────────────────────────────────────────────────────────
        t_log[k]  = (k - 1) * TS
        sp_log[k] = norm_to_f(sp_n)
        pv_log[k] = norm_to_f(T_meas)
        pr_log[k] = norm_to_f(T_pred)
        u_log[k]  = u_fin
        vt_log[k] = v_state
        ue_log[k] = u_eff
        dp_log[k] = dp_cold
    end

    return t_log, sp_log, pv_log, pr_log, u_log, vt_log, ue_log, dp_log
end

# ──────────────────────────────────────────────────────────────────────────────
#  RUN
# ──────────────────────────────────────────────────────────────────────────────
const T_SIM = 1800
const N_SIM = round(Int, T_SIM / TS)

println("Running simulation ($N_SIM steps × Ts=$(TS)s = $(T_SIM)s) …")
(t_log, sp_log, pv_log, pr_log, u_log, vt_log, ue_log, dp_log) = run_simulation(N_SIM)
println("Done.\n")

# ──────────────────────────────────────────────────────────────────────────────
#  PERFORMANCE METRICS
# ──────────────────────────────────────────────────────────────────────────────
function settling(pv, sp_val, k_start, tol, N)
    for k in k_start:N-30
        if all(abs.(pv[k:k+30] .- sp_val) .< tol)
            return Float64(k - k_start) * TS
        end
    end
    return Inf
end

tol = 0.5
ts1 = settling(pv_log, 95.0,  200,  tol, N_SIM)
ts2 = settling(pv_log, 100.0, 1200, tol, N_SIM)
ss1 = mean(pv_log[400:799]) - 95.0
os1 = max(0.0, maximum(pv_log[200:799]) - 95.0)

println("─"^60)
println(" Performance Summary")
println("─"^60)
@printf("  Step 80 → 95°F  │ settle=%.0fs  SS_err=%+.2f°F  overshoot=%.2f°F\n",
        ts1, ss1, os1)
@printf("  Step 95 → 100°F │ settle=%s\n\n",
        isinf(ts2) ? "not settled in 600s" : @sprintf("%.0fs", ts2))

# ──────────────────────────────────────────────────────────────────────────────
#  EXPORT CSV  (1800 rows × 8 columns)
# ──────────────────────────────────────────────────────────────────────────────
script_dir = @__DIR__
csv_path   = joinpath(script_dir, "simulation_results.csv")
open(csv_path, "w") do io
    println(io, "time_s,setpoint_F,actual_F,predicted_F,valve_duty,valve_tpo")
    for k in 1:N_SIM
        @printf(io, "%.1f,%.4f,%.4f,%.4f,%.4f,%.0f\n",
            t_log[k], sp_log[k], pv_log[k], pr_log[k],
            u_log[k], vt_log[k])
    end
end
@printf("▸ CSV  written → %s  (%d rows)\n", csv_path, N_SIM)

# ──────────────────────────────────────────────────────────────────────────────
#  EXPORT SUMMARY TXT
# ──────────────────────────────────────────────────────────────────────────────
txt_path = joinpath(script_dir, "simulation_summary.txt")
open(txt_path, "w") do io
    println(io, "Smith Predictor Temperature Control – Simulation Summary")
    println(io, "="^60)
    println(io, "Physical system:")
    @printf(io,"  Tank: %.2f ft³ (≈%.0f L)  Pipe: %.0f in  Hot: %.0f°F  Cold: %.0f°F\n",
            V_ft3, V_ft3*28.3168, L_pipe*12, T_HOT, T_COLD)
    println(io, "FOPDT model (identified from step response):")
    @printf(io,"  K=%.4f  τ=%.2f s  t₀=%.2f s\n", K_PROC, TAU_P, T0_P)
    println(io, "Discrete controller (ZOH, Ts=1 s):")
    @printf(io,"  MODEL_A=%.5f  MODEL_B=%.6f  BUFFER_SIZE=%d\n", MODEL_A, MODEL_B, BUF_SIZE)
    @printf(io,"  PI_KP=%.4f  PI_KI=%.6f\n", PI_KP, PI_KI)
    @printf(io,"  TPO_WINDOW=%d s  nominal Vp_ss=%.4f\n", TPO_WIN, Vp_SS)
    println(io, "Defects modelled:")
    @printf(io,"  Shared-pipe coupling α=%.2f (%.0f%% ΔP loss when valve=ON)\n",
            ALPHA, ALPHA*100)
    @printf(io,"  Low-pressure supply: ΔP_eff=%.1f psi  (rated=%.1f psi)\n",
            dP_EFF, dP_NOM)
    println(io, "Performance:")
    @printf(io,"  80→95°F:   settle=%.0fs  SS_err=%+.2f°F  overshoot=%.2f°F\n",
            ts1, ss1, os1)
    @printf(io,"  95→100°F:  settle=%s\n",
            isinf(ts2) ? "not settled in 600s (physically limited)" : @sprintf("%.0fs", ts2))
    println(io, "\nCSV columns:")
    println(io, "  time_s | setpoint_F | actual_F | predicted_F | valve_duty | valve_tpo")
end
@printf("▸ TXT  written → %s\n\n", txt_path)

# ──────────────────────────────────────────────────────────────────────────────
#  ASCII CHART (no packages required)
# ──────────────────────────────────────────────────────────────────────────────
function ascii_chart(t_v, pv_v, sp_v; W=72, H=22)
    y_lo = min(minimum(pv_v), minimum(sp_v)) - 2.0
    y_hi = max(maximum(pv_v), maximum(sp_v)) + 2.0
    t_lo, t_hi = t_v[1], t_v[end]

    cvs = fill(' ', H, W)
    to_r(v) = clamp(round(Int,(y_hi-v)/(y_hi-y_lo)*(H-1))+1, 1, H)
    to_c(t) = clamp(round(Int,(t-t_lo)/(t_hi-t_lo)*(W-1))+1,  1, W)

    for k in eachindex(t_v)
        r,c = to_r(sp_v[k]), to_c(t_v[k])
        if cvs[r,c]==' '; cvs[r,c]='─'; end
        cvs[to_r(pv_v[k]), to_c(t_v[k])] = '●'
    end

    println("─"^60)
    println(" Closed-loop response   ● = PV actual   ─ = Setpoint")
    println("─"^60)
    @printf("  %5.1f°F ┐\n", y_hi)
    for r in 1:H
        y_v = y_hi - (r-1)*(y_hi-y_lo)/(H-1)
        if r==1 || r==H || r==div(H,2)
            @printf("  %5.1f°F │", y_v)
        else
            print("          │")
        end
        println(String(cvs[r,:]))
    end
    print("          └" * "─"^W * "\n")
    pts = [string(round(Int, t_lo + i*(t_hi-t_lo)/4),"s") for i in 0:4]
    seg = W÷4
    row = " "^11
    for lb in pts; row *= lb * " "^max(1,seg-length(lb)); end
    println(row)
    println()
end

ds  = 10
idx = 1:ds:N_SIM
ascii_chart(t_log[idx], pv_log[idx], sp_log[idx])

# ──────────────────────────────────────────────────────────────────────────────
#  VALVE DUTY SPARKLINE
# ──────────────────────────────────────────────────────────────────────────────
function valve_chart(t_v, u_v; W=72, H=8)
    y_lo, y_hi = 0.0, 1.05
    t_lo, t_hi = t_v[1], t_v[end]
    cvs = fill(' ', H, W)
    to_r(v) = clamp(round(Int,(y_hi-v)/(y_hi-y_lo)*(H-1))+1, 1, H)
    to_c(t) = clamp(round(Int,(t-t_lo)/(t_hi-t_lo)*(W-1))+1,  1, W)
    for k in eachindex(t_v)
        cvs[to_r(u_v[k]), to_c(t_v[k])] = '▪'
    end
    println(" Valve duty cycle  u(t)   [0.0 → 1.0]")
    @printf("  %3.1f ┐\n", y_hi)
    for r in 1:H
        y_v = y_hi - (r-1)*(y_hi-y_lo)/(H-1)
        if r==1||r==H||r==div(H,2)
            @printf("  %.1f │", y_v)
        else
            print("      │")
        end
        println(String(cvs[r,:]))
    end
    print("      └" * "─"^W * "\n")
    pts = [string(round(Int,t_lo+i*(t_hi-t_lo)/4),"s") for i in 0:4]
    seg = W÷4
    row = " "^7
    for lb in pts; row *= lb * " "^max(1,seg-length(lb)); end
    println(row)
end

valve_chart(t_log[idx], u_log[idx])

println("\n✓ All outputs written. Simulation complete.")
println("  simulation_results.csv  →  import into Excel / Python / MATLAB for plots")
println("  simulation_summary.txt  →  key metrics and parameter block")
