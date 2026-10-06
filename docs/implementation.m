% Simulation Parameters
TotalTime = 600;          % Total simulation time in seconds
Ts = 1.0;                 % 1-second sampling time
NumSteps = TotalTime/Ts;

% Plant and Model Coeffs (From FOPDT parameters)
MODEL_A = 0.98943;
MODEL_B = -0.00570;

% PI Controller Gains
PI_KP = -128.3886;
PI_KI = -1.364892;

% Operating Points
T_op_norm = 10.0 / 130.0;
u_op = 0.256;

% TPO configuration
TPO_WINDOW = 6;

% Feedback filter configuration (simulates sensor lag filtering of TPO ripple)
err_corr_filtered = 0.0;
filter_alpha = exp(-Ts / 15.0);

% True Plant States (Simulating the 14-second transport delay)
plant_buffer = ones(1, 14) * T_op_norm;
delay_buffer = ones(1, 14) * T_op_norm; % Your controller's time capsule

T_predicted = T_op_norm;
T_predicted_dev = 0.0;
T_plant_instant = T_op_norm;
T_plant_instant_dev = 0.0;
error_integral = 0.256;

% Logging variables for plotting
log_sp = zeros(1, NumSteps);
log_pv = zeros(1, NumSteps);
log_pred = zeros(1, NumSteps);
log_u = zeros(1, NumSteps);
log_u_tpo = zeros(1, NumSteps);

% Run Loop
for k = 1:NumSteps
    % Setpoint profile: step change from 80 F to 90 F
    if k >= 100
        T_setpoint = 20.0 / 130.0;
    else
        T_setpoint = T_op_norm;
    end
    log_sp(k) = T_setpoint;

    % 1. Read Delayed Plant Output (Reality)
    T_actual = plant_buffer(1);
    log_pv(k) = T_actual;

    % 2. Run your exact C-code Smith Predictor step
    T_delayed = delay_buffer(1);
    error_correction = T_actual - T_delayed;
    
    % Filter the error correction to eliminate high-frequency TPO ripple feedback
    err_corr_filtered = (filter_alpha * err_corr_filtered) + ((1 - filter_alpha) * error_correction);
    T_feedback = T_predicted + err_corr_filtered;
    control_error = T_setpoint - T_feedback;

    % 3. Calculate PI and Anti-Windup Clamping
    u_proportional = PI_KP * control_error;
    potential_integral = error_integral + (PI_KI * control_error);
    u_unclamped = u_proportional + potential_integral;
    u_final = max(0.0, min(1.0, u_unclamped));
    log_u(k) = u_final;

    % Anti-windup
    saturated_high = (u_unclamped > 1.0) && (control_error < 0.0);
    saturated_low = (u_unclamped < 0.0) && (control_error > 0.0);
    if ~saturated_high && ~saturated_low
        error_integral = potential_integral;
    end

    % 4. Update internal delay-free model (using continuous signal)
    T_predicted_dev = (MODEL_A * T_predicted_dev) + (MODEL_B * (u_final - u_op));
    T_predicted = T_predicted_dev + T_op_norm;
    log_pred(k) = T_predicted;

    % TPO modulation for the physical solenoid valve
    tpo_counter = mod(k-1, TPO_WINDOW);
    valve_state = double(tpo_counter < round(u_final * TPO_WINDOW));
    log_u_tpo(k) = valve_state;

    % 5. Shift Conveyor Belts (Update Buffers)
    plant_buffer = [plant_buffer(2:end), T_plant_instant];
    delay_buffer = [delay_buffer(2:end), T_predicted];

    % 6. Update actual delay-free plant state for next step (using TPO valve state)
    T_plant_instant_dev = (MODEL_A * T_plant_instant_dev) + (MODEL_B * (valve_state - u_op));
    T_plant_instant = T_plant_instant_dev + T_op_norm;
end

% Plotting results to verify performance
time_vec = (0:NumSteps-1) * Ts;
fig = figure('Name', 'Smith Predictor Controller Simulation', 'NumberTitle', 'off', 'Visible', 'off');

subplot(2,1,1);
plot(time_vec, log_sp, 'k--', 'LineWidth', 1.5); hold on;
plot(time_vec, log_pv, 'b-', 'LineWidth', 2);
plot(time_vec, log_pred, 'g-.', 'LineWidth', 1.2);
grid on;
title('Closed-loop Response of the Smith Predictor (Normalized)');
xlabel('Time [s]');
ylabel('Temperature Fraction (0.0 - 1.0)');
legend('Setpoint', 'Actual Temperature (PV)', 'Predicted Temp (No Delay)', 'Location', 'Best');

subplot(2,1,2);
plot(time_vec, log_u_tpo, 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5); hold on;
plot(time_vec, log_u, 'r-', 'LineWidth', 1.5);
grid on;
title('Control Action / Valve Opening (CO)');
xlabel('Time [s]');
ylabel('Duty Cycle (0.0 - 1.0)');
legend('Valve State (TPO)', 'Average Duty Cycle (u)', 'Location', 'Best');

% Save plot to the same directory as this script
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
save_path = fullfile(script_dir, 'simulation_results.png');
saveas(fig, save_path);
fprintf('Simulation finished and plot saved to %s.\n', save_path);