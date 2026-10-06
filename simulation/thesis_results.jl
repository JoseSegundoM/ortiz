################################################################################
#  thesis_results.jl  –  Publication-quality figures for thesis Chapters 3 & 4
#  Smith Predictor Temperature Control — 20-L Lab Tank (ESP32 prototype)
#
#  Run:  julia simulation/thesis_results.jl
#  Out:  docs/figures/fig_*.png   (300 dpi, thesis-ready)
#        docs/figures/table_*.txt (LaTeX-ready table data)
################################################################################

using Printf, Statistics, Plots, LaTeXStrings

# ── output directory ─────────────────────────────────────────────────────────
const FIG_DIR = joinpath(@__DIR__, "..", "docs", "figures")
mkpath(FIG_DIR)
fig_path(name) = joinpath(FIG_DIR, name)

# ── common plot theme ─────────────────────────────────────────────────────────
theme(:default)
default(
    fontfamily  = "Computer Modern",
    titlefontsize = 11,
    guidefontsize = 10,
    tickfontsize  = 9,
    legendfontsize = 7,
    legend_foreground_color = :gray40,
    legend_background_color = RGBA(1,1,1,0.75),
    linewidth   = 2,
    grid        = true,
    gridalpha   = 0.3,
    framestyle  = :box,
    size        = (720, 450),
    dpi         = 300,
)

println("="^65)
println(" Thesis Results Generator — Smith Predictor Temperature Control")
println("="^65)

# ══════════════════════════════════════════════════════════════════════════════
#  §1  SYSTEM PARAMETERS  (locked to firmware src/main.cpp)
# ══════════════════════════════════════════════════════════════════════════════

# Physical prototype
const V_ft3      = 0.71          # tank volume  [ft3]  approx 20 L
const T_HOT      = 110.0         # hot inlet    [degF]
const T_COLD     = 50.0          # cold inlet   [degF]
const T_TARGET   = 90.0          # operating pt [degF]
const T_OP_NORM  = (T_TARGET - 70.0) / 130.0   # normalised

# FOPDT model — identified from step test (Smith two-point method)
const K_PROC  = -0.3270          # static gain  [frac TO / frac CO]
const TAU_P   = 499.54           # time const   [s]
const T0_P    = 21.94            # dead time    [s]

# Sampling / actuator
const TS      = 1.0              # sampling period [s]
const TPO_WIN = 33               # TPO window [s]

# Discrete model  (ZOH)
const MODEL_A = exp(-TS / TAU_P)
const MODEL_B = K_PROC * (1.0 - MODEL_A)
const BUF_SZ  = round(Int, T0_P / TS)   # 22 samples

# Valve steady-state operating point
const C_VL    = 0.20
const dP_EFF  = 10.0
const VFACTOR = (500.0/60.0) * C_VL * sqrt(dP_EFF)
const f_lmin  = 2.0
const W1      = f_lmin / 28.3168 * 62.4
const Cp1=0.8; const Cp2=1.0; const Cp3=0.9
const W2_SS   = W1 * (Cp3 * T_TARGET - Cp1 * T_HOT) / (Cp2 * T_COLD - Cp3 * T_TARGET)
const Vp_SS   = W2_SS / VFACTOR          # approx 0.189

f_to_norm(T) = (T - 70.0) / 130.0
norm_to_f(n) = n * 130.0 + 70.0

println("\n FOPDT Model  K=$(K_PROC)  tau=$(TAU_P) s  t0=$(T0_P) s")
println(" Discrete     MODEL_A=$(round(MODEL_A,digits=5))  MODEL_B=$(round(MODEL_B,sigdigits=4))")
println(" Buffer size  $(BUF_SZ) samples  TPO window $(TPO_WIN) s")
println(" Steady-state valve position  Vp_ss=$(round(Vp_SS,digits=4))\n")

# ══════════════════════════════════════════════════════════════════════════════
#  §2  PI GAIN COMPUTATION FOR DIFFERENT Tc VALUES
# ══════════════════════════════════════════════════════════════════════════════

function pi_gains(Tc)
    Kc = TAU_P / (K_PROC * Tc)
    Ki = Kc / TAU_P
    return Kc, Ki
end

Tc_cases = [2*T0_P, T0_P, T0_P/2, T0_P/4, T0_P/10]
Tc_labels = ["Tc=2L", "Tc=L", "Tc=L/2", "Tc=L/4", "Tc=L/10 (FW)"]

println("-"^65)
println("  PI gain table")
println("-"^65)
@printf("  %-14s %10s %12s\n", "Tc setting", "KP", "KI")
for (i, Tc) in enumerate(Tc_cases)
    Kp, Ki = pi_gains(Tc)
    @printf("  %-14s %+10.4f %+12.6f\n", Tc_labels[i], Kp, Ki)
end
println()

# ══════════════════════════════════════════════════════════════════════════════
#  §3  SIMULATION ENGINE  (Smith Predictor + PI, mirrors main.cpp exactly)
# ══════════════════════════════════════════════════════════════════════════════

function run_sim(;
    N       = 1800,
    Tc      = T0_P / 10.0,
    sp_prof = k -> k < 200  ? f_to_norm(80.0) : f_to_norm(95.0),
    dist_f  = k -> 0.0,
    alpha_pp = 0.0,
)
    PI_KP, PI_KI = pi_gains(Tc)
    T_init = f_to_norm(80.0)
    plant_dev   = T_init - T_OP_NORM
    plant_buf   = fill(T_init, BUF_SZ)
    pred_dev    = T_init - T_OP_NORM
    T_pred      = T_init
    ctrl_buf    = fill(T_init, BUF_SZ)
    err_int     = Vp_SS
    err_corr_f  = 0.0
    filt_a      = exp(-TS / 15.0)
    tpo_cnt     = 0

    t_v=zeros(N); sp_v=zeros(N); pv_v=zeros(N)
    pr_v=zeros(N); u_v=zeros(N); vt_v=zeros(N)

    for k in 1:N
        sp_n  = sp_prof(k)
        dist  = dist_f(k)
        T_meas = plant_buf[1] + dist

        T_del         = ctrl_buf[1]
        e_corr        = T_meas - T_del
        err_corr_f    = filt_a * err_corr_f + (1.0 - filt_a) * e_corr
        T_fb          = T_pred + err_corr_f
        ctrl_err      = sp_n - T_fb

        u_p    = PI_KP * ctrl_err
        p_int  = err_int + PI_KI * ctrl_err
        u_unc  = u_p + p_int
        u_fin  = clamp(u_unc, 0.0, 1.0)

        if !((u_unc > 1.0) && (ctrl_err < 0.0)) && !((u_unc < 0.0) && (ctrl_err > 0.0))
            err_int = p_int
        end

        on_s    = round(Int, u_fin * TPO_WIN)
        v_state = (tpo_cnt < on_s) ? 1.0 : 0.0
        tpo_cnt = mod(tpo_cnt + 1, TPO_WIN)

        dp_cold = dP_EFF * (1.0 - alpha_pp * v_state)
        dp_cold = max(dp_cold, 0.5)
        u_eff   = v_state * sqrt(dp_cold / dP_EFF)

        pred_dev = MODEL_A * pred_dev + MODEL_B * (u_fin - Vp_SS)
        T_pred   = pred_dev + T_OP_NORM
        ctrl_buf = vcat(ctrl_buf[2:end], [T_pred])

        plant_dev = MODEL_A * plant_dev + MODEL_B * (u_eff - Vp_SS)
        plant_inst = plant_dev + T_OP_NORM
        plant_buf  = vcat(plant_buf[2:end], [plant_inst])

        t_v[k]  = (k-1)*TS
        sp_v[k] = norm_to_f(sp_n)
        pv_v[k] = norm_to_f(T_meas)
        pr_v[k] = norm_to_f(T_pred)
        u_v[k]  = u_fin
        vt_v[k] = v_state
    end
    return t_v, sp_v, pv_v, pr_v, u_v, vt_v
end

function run_plain_pi(; N=1800, Tc=T0_P, sp_prof=k->k<200 ? f_to_norm(80.0) : f_to_norm(95.0))
    PI_KP, PI_KI = pi_gains(Tc)
    T_init = f_to_norm(80.0)
    plant_dev = T_init - T_OP_NORM
    plant_buf = fill(T_init, BUF_SZ)
    err_int   = Vp_SS
    tpo_cnt   = 0

    t_v=zeros(N); sp_v=zeros(N); pv_v=zeros(N); u_v=zeros(N)
    for k in 1:N
        sp_n   = sp_prof(k)
        T_meas = plant_buf[1]
        ctrl_err = sp_n - T_meas
        u_p    = PI_KP * ctrl_err
        p_int  = err_int + PI_KI * ctrl_err
        u_unc  = u_p + p_int
        u_fin  = clamp(u_unc, 0.0, 1.0)
        if !((u_unc > 1.0) && (ctrl_err < 0.0)) && !((u_unc < 0.0) && (ctrl_err > 0.0))
            err_int = p_int
        end

        on_s    = round(Int, u_fin * TPO_WIN)
        v_state = (tpo_cnt < on_s) ? 1.0 : 0.0
        tpo_cnt = mod(tpo_cnt + 1, TPO_WIN)

        plant_dev  = MODEL_A * plant_dev + MODEL_B * (v_state - Vp_SS)
        plant_inst = plant_dev + T_OP_NORM
        plant_buf  = vcat(plant_buf[2:end], [plant_inst])

        t_v[k]  = (k-1)*TS
        sp_v[k] = norm_to_f(sp_n)
        pv_v[k] = norm_to_f(T_meas)
        u_v[k]  = u_fin
    end
    return t_v, sp_v, pv_v, u_v
end

# ══════════════════════════════════════════════════════════════════════════════
#  §4  PERFORMANCE METRICS
# ══════════════════════════════════════════════════════════════════════════════

function perf_metrics(t_v, pv_v, sp_v, k_start; tol_pct=5.0)
    N      = length(pv_v)
    sp_val = sp_v[min(k_start + 30, N)]
    sp0    = pv_v[max(k_start - 1, 1)]
    dsp    = sp_val - sp0

    tol = abs(dsp) * tol_pct / 100.0
    ts  = Inf
    for k in k_start:N-30
        if all(abs.(pv_v[k:k+30] .- sp_val) .< tol)
            ts = (k - k_start) * TS
            break
        end
    end

    if dsp > 0
        os_abs = max(0.0, maximum(pv_v[k_start:min(k_start+600, N)]) - sp_val)
    else
        os_abs = max(0.0, sp_val - minimum(pv_v[k_start:min(k_start+600, N)]))
    end
    os_pct = abs(dsp) > 0 ? os_abs / abs(dsp) * 100.0 : 0.0

    ss_end = min(k_start + 1000, N)
    ss_err = mean(pv_v[max(k_start+300, ss_end-100):ss_end]) - sp_val

    w = k_start : min(k_start + 899, N)
    e_vec = pv_v[w] .- sp_v[w]
    iae = sum(abs.(e_vec)) * TS
    ise = sum(e_vec .^ 2) * TS

    return (ts=ts, os_pct=os_pct, ss_err=ss_err, iae=iae, ise=ise)
end

# ══════════════════════════════════════════════════════════════════════════════
#  §5  RUN ALL SIMULATIONS
# ══════════════════════════════════════════════════════════════════════════════

println("Running simulations ...")
sp_cmp = k -> k < 200 ? f_to_norm(80.0) : f_to_norm(95.0)
N_cmp  = 1200

res_smith = [run_sim(N=N_cmp, Tc=Tc, sp_prof=sp_cmp, dist_f=k->0.0)
             for Tc in Tc_cases[1:4]]

res_pi    = [run_plain_pi(N=N_cmp, Tc=Tc, sp_prof=sp_cmp)
             for Tc in Tc_cases[1:4]]

t_f, sp_f, pv_f, pr_f, u_f, vt_f = run_sim(N=1800, Tc=Tc_cases[5])

println("  Done.\n")

# ══════════════════════════════════════════════════════════════════════════════
#  §6  FIGURE 3.A – FOPDT STEP RESPONSE
# ══════════════════════════════════════════════════════════════════════════════

println("Generating Figure 3.A – FOPDT Step Response ...")
let
    N = 800; du = 0.05
    t = collect(0.0:TS:(N-1)*TS)
    y = zeros(N)
    for k in 1:N
        t_eff = max(0.0, (k-1)*TS - T0_P)
        y[k] = K_PROC * du * (1.0 - exp(-t_eff / TAU_P))
    end

    T_ini = norm_to_f(T_OP_NORM)
    T_ss  = norm_to_f(T_OP_NORM + K_PROC * du)
    dT    = T_ss - T_ini

    p = plot(t, norm_to_f.(T_OP_NORM .+ y),
        label = "Modelo FOPDT",
        color = :royalblue, lw = 2.5,
        xlabel = "Tiempo [s]",
        ylabel = "Temperatura [\u00b0F]",
        title  = "Fig. 3.A - Validacion del modelo FOPDT  (K=$(K_PROC),  \u03c4=$(TAU_P) s,  t0=$(T0_P) s)",
        legend = :topright,
    )
    hline!(p, [T_ss], ls=:dash, color=:gray50, lw=1.2, label="Estado estacionario")
    vline!(p, [T0_P], ls=:dot, color=:orange, lw=1.5, label="t\u2080 = $(T0_P) s")
    vline!(p, [T0_P + TAU_P], ls=:dashdot, color=:green, lw=1.2, label="t\u2080+\u03c4 (63.2%)")
    savefig(p, fig_path("fig_3A_fopdt_step.png"))
    println("  OK fig_3A_fopdt_step.png")
end

# ══════════════════════════════════════════════════════════════════════════════
#  §7  FIGURE 3.B – CONTROLLER COMPARISON (Tc = L)
# ══════════════════════════════════════════════════════════════════════════════

println("Generating Figure 3.B – Controller Comparison (Tc=L) ...")
let
    case = 2
    (t_s, sp_s, pv_s, pr_s, u_s, _) = res_smith[case]
    (t_p, sp_p, pv_p, u_p) = res_pi[case]

    p = plot(t_s, sp_s,
        label = "Valor de referencia (SP)", color = :black, ls = :dash, lw = 1.5,
        xlabel = "Tiempo [s]", ylabel = "Temperatura [\u00b0F]",
        title  = "Fig. 3.B - Comparacion PI vs Predictor Smith  (Tc = L = $(round(T0_P,digits=1)) s)",
        legend = :bottomright,
    )
    plot!(p, t_p, pv_p, label="Controlador PI simple",      color=:steelblue, lw=2.0)
    plot!(p, t_s, pv_s, label="Predictor Smith+PI",         color=:crimson,   lw=2.0)
    plot!(p, t_s, pr_s, label="Modelo Smith (sin retardo)", color=:seagreen, lw=1.5, ls=:dashdot)
    savefig(p, fig_path("fig_3B_controller_comparison.png"))
    println("  OK fig_3B_controller_comparison.png")
end

# ══════════════════════════════════════════════════════════════════════════════
#  §8  FIGURES 4.1 – 4.4  (four Tc cases, thesis style)
# ══════════════════════════════════════════════════════════════════════════════

println("Generating Figures 4.1-4.4 ...")
for (i, case) in enumerate(1:4)
    (t_s, sp_s, pv_s, pr_s, u_s, _) = res_smith[case]
    (t_p, sp_p, pv_p, u_p) = res_pi[case]

    p = plot(t_s, sp_s,
        label = "Valor de referencia (SP)", color = :black, ls = :dash, lw = 1.5,
        xlabel = "Tiempo [s]", ylabel = "Temperatura [\u00b0F]",
        title  = "Fig. 4.$(i) - Respuesta al escalon  $(Tc_labels[case])",
        legend = :bottomright,
    )
    plot!(p, t_p, pv_p, label="Controlador PI",       color=:steelblue, lw=2.0)
    plot!(p, t_s, pv_s, label="Predictor Smith+PI",   color=:crimson,   lw=2.0)
    savefig(p, fig_path("fig_4_$(i)_$(replace(replace(Tc_labels[case]," "=>"_"),"/"=>"_")).png"))
    println("  OK fig_4_$(i)_$(Tc_labels[case])")
end

# ══════════════════════════════════════════════════════════════════════════════
#  §9  FIGURE 4.C – FULL 30-min SCENARIO
# ══════════════════════════════════════════════════════════════════════════════

println("Generating Figure 4.C – 30-min scenario ...")
let
    # Top panel — keep only data traces in legend; annotate events as text
    p1 = plot(t_f, sp_f,
        label = "SP", color = :black, ls = :dash, lw = 1.5,
        xlabel = "", ylabel = "Temperatura [\u00b0F]",
        title  = "Fig. 4.C - Escenario completo 30 min  (Tc = t\u2080/10)",
        legend = :bottomright,
        legendfontsize = 7,
        size = (820, 580),
    )
    plot!(p1, t_f, pv_f, label="PV (medida)",           color=:steelblue, lw=2.0)
    plot!(p1, t_f, pr_f, label="Prediccion Smith",       color=:seagreen,  lw=1.5, ls=:dashdot)
    # Event lines — no legend entry, annotate instead
    vline!(p1, [200],  ls=:dot, color=:orange, lw=1.3, label="")
    # Annotations above the lines
    ylims_p1 = (78.0, 97.0)
    annotate!(p1, 205,  96.0, text("Escalon 80\u219295\u00b0F",  6, :left, :orange))
    ylims!(p1, ylims_p1...)

    # Bottom panel
    p2 = plot(t_f, u_f,
        label = "u(t) comandado", color = :darkorange, lw = 1.8,
        xlabel = "Tiempo [s]", ylabel = "Ciclo de trabajo [0-1]",
        ylims  = (-0.05, 1.10),
        legend = :topright,
        legendfontsize = 7,
    )
    ds = 5; idx = 1:ds:length(t_f)
    plot!(p2, t_f[idx], vt_f[idx] .* 0.95,
        seriestype=:steppre, label="Valvula ON/OFF (TPO)",
        color=:gray60, lw=0.8, alpha=0.6)
    vline!(p2, [200], ls=:dot, color=:gray40, lw=1.0, label="")

    combo = plot(p1, p2, layout=(2,1), size=(820, 580))
    savefig(combo, fig_path("fig_4C_full_scenario.png"))
    println("  OK fig_4C_full_scenario.png")
end

# ══════════════════════════════════════════════════════════════════════════════
#  §10  FIGURE 4.D – METRICS vs Tc
# ══════════════════════════════════════════════════════════════════════════════

println("Generating Figure 4.D – Metrics vs Tc ...")
let
    sp_step = k -> k < 200 ? f_to_norm(80.0) : f_to_norm(95.0)
    sp_arr  = [norm_to_f(sp_step(k)) for k in 1:N_cmp]
    tc_ratio = [2.0, 1.0, 0.5, 0.25]

    ts_smith = Float64[]; os_smith = Float64[]
    ts_pi    = Float64[]; os_pi    = Float64[]

    for (i, case) in enumerate(1:4)
        (_, _, pv_s, _, _, _) = res_smith[case]
        (_, _, pv_p, _) = res_pi[case]
        ms = perf_metrics(collect(1:N_cmp)*TS, pv_s, sp_arr, 201)
        mp = perf_metrics(collect(1:N_cmp)*TS, pv_p, sp_arr, 201)
        push!(ts_smith, isinf(ms.ts) ? N_cmp*TS : ms.ts)
        push!(os_smith, ms.os_pct)
        push!(ts_pi, isinf(mp.ts) ? N_cmp*TS : mp.ts)
        push!(os_pi, mp.os_pct)
    end

    p1 = plot(tc_ratio, ts_smith,
        label="Predictor Smith", marker=:circle, color=:crimson, lw=2,
        xlabel="Tc / L", ylabel="Tiempo de establecimiento [s]",
        title="Tiempo de establecimiento vs Tc/L")
    plot!(p1, tc_ratio, ts_pi,
        label="PI simple", marker=:square, color=:steelblue, lw=2, ls=:dash)

    p2 = plot(tc_ratio, os_smith,
        label="Predictor Smith", marker=:circle, color=:crimson, lw=2,
        xlabel="Tc / L", ylabel="Sobreimpulso [%]",
        title="Sobreimpulso vs Tc/L")
    plot!(p2, tc_ratio, os_pi,
        label="PI simple", marker=:square, color=:steelblue, lw=2, ls=:dash)

    combo = plot(p1, p2, layout=(1,2), size=(800, 360))
    savefig(combo, fig_path("fig_4D_metrics_vs_Tc.png"))
    println("  OK fig_4D_metrics_vs_Tc.png")
end

# ══════════════════════════════════════════════════════════════════════════════

#  §11  TABLES
# ══════════════════════════════════════════════════════════════════════════════

println("\nGenerating Tables ...")

open(fig_path("table_3_1b_prototype_params.txt"), "w") do io
    println(io, "% Table 3.1b - Physical Parameters of the Laboratory Prototype")
    println(io, "\\begin{tabular}{llr}")
    println(io, "\\hline")
    println(io, "\\textbf{Parameter} & \\textbf{Description} & \\textbf{Value} \\\\")
    println(io, "\\hline")
    rows = [
        ("V",                  "Tank volume",                              "0.71 ft3 (approx 20 L)"),
        ("T_hot",              "Hot inlet temperature",                    "110 degF (43.3 degC)"),
        ("T_cold",             "Cold inlet temperature",                   "50 degF (10 degC)"),
        ("T_SP",               "Operating setpoint",                       "90 degF (32.2 degC)"),
        ("C_VL",               "Solenoid valve flow coeff.",               "0.20 gpm/sqrt(psi)"),
        ("Delta_P (eff)",      "Supply pressure (actual)",                 "10 psi"),
        ("W1",                 "Hot water mass flow",                      @sprintf("%.2f lb/min", W1)),
        ("W2_ss",              "Cold water SS mass flow",                  @sprintf("%.3f lb/min", W2_SS)),
        ("Vp_ss",              "Valve duty at SS (90 degF)",               @sprintf("%.4f", Vp_SS)),
        ("K",                  "FOPDT static gain",                        "$(K_PROC) frac TO/CO"),
        ("tau",                "FOPDT time constant",                      "499.54 s"),
        ("t0",                 "FOPDT dead time",                          "21.94 s"),
        ("Ts",                 "Sampling period",                          "1.0 s"),
        ("d = t0/Ts",          "Buffer size (Smith predictor delay)",      "22 samples"),
        ("N_TPO",              "TPO window",                               "33 s"),
    ]
    for (var, desc, val) in rows
        @printf(io, "\\texttt{%s} & %s & %s \\\\\n", var, desc, val)
    end
    println(io, "\\hline")
    println(io, "\\end{tabular}")
end
println("  OK table_3_1b_prototype_params.txt")

open(fig_path("table_3_2b_controller_params.txt"), "w") do io
    println(io, "% Table 3.2b - Discrete Controller Parameters")
    println(io, "\\begin{tabular}{llr}")
    println(io, "\\hline")
    println(io, "\\textbf{Parameter} & \\textbf{Description} & \\textbf{Value} \\\\")
    println(io, "\\hline")
    Tc_fw = T0_P / 10.0
    Kp_fw, Ki_fw = pi_gains(Tc_fw)
    rows = [
        ("MODEL\\_A",      "Discrete pole  a = exp(-Ts/tau)",   @sprintf("%.5f", MODEL_A)),
        ("MODEL\\_B",      "Discrete gain  b = K*(1-a)",         @sprintf("%.6f", MODEL_B)),
        ("BUFFER\\_SIZE",  "Delay buffer length",                "22 samples"),
        ("TPO\\_WINDOW",   "Time-proportional output window",    "33 s"),
        ("PI\\_KP",        "Prop. gain (Tc = t0/10)",           @sprintf("%.4f", Kp_fw)),
        ("PI\\_KI",        "Integral gain (positional form)",    @sprintf("%.6f", Ki_fw)),
        ("Vp\\_ss",        "Integral init / SS duty cycle",      @sprintf("%.4f", Vp_SS)),
    ]
    for (var, desc, val) in rows
        @printf(io, "\\texttt{%s} & %s & %s \\\\\n", var, desc, val)
    end
    println(io, "\\hline")
    println(io, "\\end{tabular}")
end
println("  OK table_3_2b_controller_params.txt")

open(fig_path("table_4_1_performance.txt"), "w") do io
    println(io, "% Table 4.1 - Performance Metrics: PI vs Smith Predictor")
    println(io, "% Step: 80->95 degF, +/-5% settling criterion")
    println(io, "\\begin{tabular}{lcccc}")
    println(io, "\\hline")
    println(io, " & Smith ts [s] & Smith OS [%] & PI ts [s] & PI OS [%] \\\\")
    println(io, "\\hline")

    sp_step = k -> k < 200 ? f_to_norm(80.0) : f_to_norm(95.0)
    sp_arr  = [norm_to_f(sp_step(k)) for k in 1:N_cmp]

    for (i, case) in enumerate(1:4)
        (_, _, pv_s, _, _, _) = res_smith[case]
        (_, _, pv_p, _) = res_pi[case]
        ms = perf_metrics(collect(1:N_cmp)*TS, pv_s, sp_arr, 201)
        mp = perf_metrics(collect(1:N_cmp)*TS, pv_p, sp_arr, 201)
        ts_s = isinf(ms.ts) ? ">$(N_cmp)" : @sprintf("%.0f", ms.ts)
        ts_p = isinf(mp.ts) ? ">$(N_cmp)" : @sprintf("%.0f", mp.ts)
        @printf(io, "%s & %s & %.1f & %s & %.1f \\\\\n",
            Tc_labels[case], ts_s, ms.os_pct, ts_p, mp.os_pct)
    end
    println(io, "\\hline")
    println(io, "\\end{tabular}")
end
println("  OK table_4_1_performance.txt")

open(fig_path("table_4_2_ise_iae.txt"), "w") do io
    println(io, "% Table 4.2 - ISE/IAE Comparison (Step 80->95 degF, 900-s window)")
    println(io, "\\begin{tabular}{lcccc}")
    println(io, "\\hline")
    println(io, " & Smith ISE & Smith IAE & PI ISE & PI IAE \\\\")
    println(io, "\\hline")

    sp_step = k -> k < 200 ? f_to_norm(80.0) : f_to_norm(95.0)
    sp_arr  = [norm_to_f(sp_step(k)) for k in 1:N_cmp]

    for (i, case) in enumerate(1:4)
        (_, _, pv_s, _, _, _) = res_smith[case]
        (_, _, pv_p, _) = res_pi[case]
        ms = perf_metrics(collect(1:N_cmp)*TS, pv_s, sp_arr, 201)
        mp = perf_metrics(collect(1:N_cmp)*TS, pv_p, sp_arr, 201)
        @printf(io, "%s & %.1f & %.1f & %.1f & %.1f \\\\\n",
            Tc_labels[case], ms.ise, ms.iae, mp.ise, mp.iae)
    end
    println(io, "\\hline")
    println(io, "\\end{tabular}")
end
println("  OK table_4_2_ise_iae.txt")

# ══════════════════════════════════════════════════════════════════════════════
#  §12  CONSOLE PERFORMANCE SUMMARY
# ══════════════════════════════════════════════════════════════════════════════

println("\n" * "="^65)
println(" Performance Summary - Step 80->95 degF, +/-5% criterion")
println("="^65)
@printf("  %-14s %8s %8s %8s %8s\n", "Tc", "ts_Smith", "OS_Smith", "ts_PI", "OS_PI")
println("  " * "-"^50)

sp_step = k -> k < 200 ? f_to_norm(80.0) : f_to_norm(95.0)
sp_arr  = [norm_to_f(sp_step(k)) for k in 1:N_cmp]

for (i, case) in enumerate(1:4)
    (_, _, pv_s, _, _, _) = res_smith[case]
    (_, _, pv_p, _) = res_pi[case]
    ms = perf_metrics(collect(1:N_cmp)*TS, pv_s, sp_arr, 201)
    mp = perf_metrics(collect(1:N_cmp)*TS, pv_p, sp_arr, 201)

    ts_s = isinf(ms.ts) ? ">1000" : @sprintf("%.0fs", ms.ts)
    ts_p = isinf(mp.ts) ? ">1000" : @sprintf("%.0fs", mp.ts)
    @printf("  %-14s %8s %7.1f%% %8s %7.1f%%\n",
        Tc_labels[case], ts_s, ms.os_pct, ts_p, mp.os_pct)
end

println("\n Full-scenario Smith Predictor (Tc=L/10, firmware Tc=$(round(T0_P/10,digits=2)) s):")
m_full = perf_metrics(t_f, pv_f, sp_f, 201)
@printf("  Step 80->95 degF   ts=%.0fs  OS=%.1f%%  SS_err=%+.2f degF\n",
    m_full.ts, m_full.os_pct, m_full.ss_err)

println("\nAll figures and tables written to: docs/figures/")
