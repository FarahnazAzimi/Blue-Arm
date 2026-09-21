%% ========================================================
%  DYNAMICS - Lagrangian Method (All 3 Joints)
%  3-DOF RRR Robotic Arm - "Blue Arm" Project
%  --------------------------------------------------------
%  WHY THIS FILE MATTERS:
%  Kinematics (the other 2 files) answer "where is the arm?" - this
%  file answers a different question: "how much torque does each
%  servo need to hold or move the arm?" This is not just theory for
%  this project - it directly EXPLAINS a real problem hit during
%  build: the shoulder servo strained and eventually failed when the
%  arm was near-horizontal. This file shows mathematically WHY that
%  happens (gravity torque is highest near horizontal, lowest near
%  vertical) - matching exactly what was found by hand-testing the
%  real robot, before this analysis was ever written.
%
%  WHAT THIS FILE SHOWS:
%  1. Kinetic and potential energy of the FULL 3-joint arm
%  2. The equations of motion in standard robotics form:
%       M(q)*q_ddot + C(q,q_dot)*q_dot + G(q) = tau
%  3. Static holding torque for ALL THREE joints (base, shoulder, elbow)
%  4. A direct comparison to the MG995 rated torque
%  5. A 3D visualization of the arm at its highest/lowest-torque pose
%  --------------------------------------------------------
%  IMPORTANT NOTE ON MASSES:
%  The link masses below (m1, m2, m3) are ESTIMATES, not yet measured
%  with a scale. For a fully accurate report, weigh each link
%  (including its servo and any brackets) with a kitchen scale and
%  replace these numbers - the SHAPE of the results (WHERE torque is
%  highest) will stay the same, but the exact numbers will become
%  fully trustworthy instead of approximate.
%  --------------------------------------------------------
%  ANGLE CONVENTION USED IN THIS FILE:
%  This file uses REAL PHYSICAL angles directly (not the "commanded"
%  servo angles used in Kinematics_FK_IK.m). Dynamics depends on the
%  arm's true physical configuration, not on the calibration mapping
%  between commanded and real angles - that mapping is a control-layer
%  detail, separate from the physics itself.
%    theta1 = base rotation, around the vertical axis
%    theta2 = real shoulder angle from horizontal
%    theta3 = real elbow bend, relative to the upper arm
%% ========================================================

clc; clear all; close all;

%% --------------------------------------------------------
%  SECTION 1 - Parameters
%% --------------------------------------------------------
L1 = 0.08;             % m - base to shoulder
L2 = 0.21;             % m - shoulder to elbow
L3 = 0.28;             % m - elbow to gripper tip

% Link masses (kg) - ESTIMATED, replace with real measured values (see header)
m1 = 0.15;   % kg - base column + shoulder servo (MG995)
m2 = 0.15;   % kg - upper arm bracket + elbow servo (MG995)
m3 = 0.20;   % kg - forearm + gripper mechanism + gripper servo (SG90) - heaviest
             %      tip mass matters most for torque, since it has the longest lever arm

g = 9.81;    % m/s^2

MG995_RATED_TORQUE_KGCM = 11;  % kg.cm - MG995 datasheet spec (base + shoulder + elbow)

fprintf('=== Blue Arm - Full Lagrangian Dynamics (3 Joints) ===\n');
fprintf('Links (m): L1=%.2f  L2=%.2f  L3=%.2f\n', L1,L2,L3);
fprintf('Masses(kg) [ESTIMATED]: m1=%.3f  m2=%.3f  m3=%.3f\n\n', m1,m2,m3);

%% --------------------------------------------------------
%  SECTION 2 - Symbolic Variables (all 3 joints)
%% --------------------------------------------------------
syms t1 t2 t3 real
syms dt1 dt2 dt3 real

q  = [t1;  t2;  t3 ];
dq = [dt1; dt2; dt3];

%% --------------------------------------------------------
%  SECTION 3 - Center of Mass of Each Link (full 3D, x-y-z)
%  t1 rotates everything about the vertical (Z) axis.
%% --------------------------------------------------------
phi3 = t2 - t3;   % absolute forearm angle from horizontal

% Link 1 (base column): fixed under rotation about its own axis
rc1 = [0; 0; L1/2];

% Link 2 (upper arm)
rc2 = [ cos(t1)*(L2/2)*cos(t2);
         sin(t1)*(L2/2)*cos(t2);
         L1 + (L2/2)*sin(t2) ];

% Link 3 (forearm + gripper)
rc3 = [ cos(t1)*(L2*cos(t2) + (L3/2)*cos(phi3));
         sin(t1)*(L2*cos(t2) + (L3/2)*cos(phi3));
         L1 + L2*sin(t2) + (L3/2)*sin(phi3) ];

%% --------------------------------------------------------
%  SECTION 4 - Jacobians and Kinetic Energy -> Inertia Matrix M(q)
%% --------------------------------------------------------
Jc1 = jacobian(rc1, q);
Jc2 = jacobian(rc2, q);
Jc3 = jacobian(rc3, q);

fprintf('Computing Inertia Matrix M(q)...\n');
M_sym = m1*(Jc1'*Jc1) + m2*(Jc2'*Jc2) + m3*(Jc3'*Jc3);
M_sym = simplify(M_sym);
fprintf('M(q) computed - size: %dx%d\n', size(M_sym,1), size(M_sym,2));

M_check = simplify(M_sym - M_sym');
if all(all(M_check == 0))
    fprintf('[PASS] M(q) is symmetric (expected for any valid inertia matrix).\n');
else
    fprintf('[CHECK] M(q) is not symmetric - review the derivation above.\n');
end

%% --------------------------------------------------------
%  SECTION 5 - Potential Energy -> Gravity Vector G(q)
%  This is the MOST IMPORTANT part for explaining our real shoulder
%  problem - G(q) tells us exactly how much torque gravity demands
%  at each joint, for any given pose.
%% --------------------------------------------------------
fprintf('Computing Gravity Vector G(q)...\n');

P_sym = m1*g*rc1(3) + m2*g*rc2(3) + m3*g*rc3(3);
P_sym = simplify(P_sym);

G_sym = simplify(jacobian(P_sym, q)');
fprintf('G(q) computed.\n');

fprintf('\n=== Why the base (theta1) needs ZERO gravity torque ===\n');
fprintf('G(1) (base row) = '); disp(G_sym(1));
fprintf('This is proven, not assumed: rotating the base only spins the\n');
fprintf('arm sideways - none of the link heights (the "z" terms above)\n');
fprintf('depend on theta1, so lifting nothing, gravity torque is exactly\n');
fprintf('zero at the base. This matches the real robot: the base servo\n');
fprintf('never strained during testing, unlike the shoulder.\n');

%% --------------------------------------------------------
%  SECTION 6 - Coriolis/Centrifugal Matrix C(q,q_dot)
%  (included for completeness - for a slow-moving pick-and-place arm
%  like this one, gravity (G) dominates by far; C matters much more
%  for fast-moving industrial arms)
%% --------------------------------------------------------
fprintf('\nComputing Coriolis Matrix C(q,q_dot)...\n');
n = 3;
C_sym = sym(zeros(n,n));
for i = 1:n
    for j = 1:n
        c_ij = sym(0);
        for k = 1:n
            c_ijk = (1/2) * ( diff(M_sym(i,j), q(k)) + diff(M_sym(i,k), q(j)) - diff(M_sym(j,k), q(i)) );
            c_ij = c_ij + c_ijk * dq(k);
        end
        C_sym(i,j) = c_ij;
    end
end
C_sym = simplify(C_sym);
fprintf('C(q,q_dot) computed.\n');

%% --------------------------------------------------------
%  SECTION 7 - THE KEY RESULT: Static Torque for ALL 3 Joints
%  This directly recreates, mathematically, the problem we found by
%  hand-testing the real robot: the shoulder needs the MOST torque
%  near horizontal (low t2) and the LEAST near vertical (high t2).
%  The base and elbow are shown alongside for completeness.
%% --------------------------------------------------------
fprintf('\n=== Section 7: Static Holding Torque - ALL 3 JOINTS ===\n');
fprintf('(base t1 held at 0 - does not affect gravity torque, elbow t3=0 for this sweep)\n\n');

t2_range_deg = 20:5:90;   % covers our real safe shoulder range and beyond
tau1_vals = zeros(size(t2_range_deg));
tau2_vals = zeros(size(t2_range_deg));
tau3_vals = zeros(size(t2_range_deg));

for idx = 1:length(t2_range_deg)
    t2_rad = deg2rad(t2_range_deg(idx));
    G_i = double(subs(G_sym, [t1, t2, t3], [0, t2_rad, 0]));
    tau1_vals(idx) = G_i(1);
    tau2_vals(idx) = G_i(2);
    tau3_vals(idx) = G_i(3);
end

fprintf('%-10s %11s %11s %11s %11s %11s\n', ...
        't2(deg)','tau1(kg.cm)','tau2(kg.cm)','tau3(kg.cm)','tau2 %%MG995','tau3 %%MG995');
fprintf('%s\n', repmat('-',1,68));
for idx = 1:length(t2_range_deg)
    tau1_kgcm = tau1_vals(idx) * 100 / 9.81;
    tau2_kgcm = tau2_vals(idx) * 100 / 9.81;
    tau3_kgcm = tau3_vals(idx) * 100 / 9.81;
    pct2 = 100 * tau2_kgcm / MG995_RATED_TORQUE_KGCM;
    pct3 = 100 * tau3_kgcm / MG995_RATED_TORQUE_KGCM;
    fprintf('%-10.0f %11.4f %11.4f %11.4f %10.1f%% %10.1f%%\n', ...
            t2_range_deg(idx), tau1_kgcm, tau2_kgcm, tau3_kgcm, pct2, pct3);
end

[worst_torque, worst_idx] = max(abs(tau2_vals));
fprintf('\n*** Worst case: shoulder angle = %d deg, tau2 = %.2f kg.cm (%.0f%% of MG995 rating) ***\n', ...
        t2_range_deg(worst_idx), worst_torque*100/9.81, 100*worst_torque*100/9.81/MG995_RATED_TORQUE_KGCM);
fprintf('This confirms what was found by hand-testing: near-horizontal\n');
fprintf('shoulder angles demand the most torque, which is why our original\n');
fprintf('shoulder servo strained and needed replacement at low angles.\n');
fprintf('Base torque (tau1) is exactly zero at every angle - proven in Section 5.\n');
fprintf('Elbow torque (tau3) stays low here because it is held straight (t3=0)\n');
fprintf('in this sweep - it becomes more significant as the arm folds further.\n');

%% --------------------------------------------------------
%  SECTION 8 - Visualization
%% --------------------------------------------------------
figure('Name','Blue Arm - Full Dynamics Analysis', 'Color','white','Position',[50,50,1400,450]);

% Plot 1: torque vs shoulder angle, all 3 joints
subplot(1,3,1);
plot(t2_range_deg, tau1_vals*100/9.81, 'g-^', 'LineWidth',2); hold on;
plot(t2_range_deg, tau2_vals*100/9.81, 'b-o', 'LineWidth',2);
plot(t2_range_deg, tau3_vals*100/9.81, 'm-s', 'LineWidth',2);
yline(MG995_RATED_TORQUE_KGCM, 'r--', 'LineWidth', 1.5);
xlabel('Shoulder Angle t2 (deg)');
ylabel('Required Torque (kg.cm)');
title('Torque vs Shoulder Angle - All 3 Joints');
legend('tau1 (base)','tau2 (shoulder)','tau3 (elbow)','MG995 rated','Location','best');
grid on;

% Plot 2: 3D pose at the worst-case (highest torque) angle
subplot(1,3,2);
hold on; grid on; axis equal;
t1_p = 0; t2_p = deg2rad(t2_range_deg(worst_idx)); t3_p = 0;
phi3_p = t2_p - t3_p;
P0 = [0;0;0];
P1 = [0;0;L1];
P2 = P1 + [cos(t1_p)*L2*cos(t2_p); sin(t1_p)*L2*cos(t2_p); L2*sin(t2_p)];
P3 = P2 + [cos(t1_p)*L3*cos(phi3_p); sin(t1_p)*L3*cos(phi3_p); L3*sin(phi3_p)];
pts = [P0,P1,P2,P3];
plot3(pts(1,:),pts(2,:),pts(3,:),'-o','Color','b','LineWidth',3,'MarkerFaceColor','b','MarkerSize',8);
quiver3(mean([P1(1),P2(1)]),mean([P1(2),P2(2)]),mean([P1(3),P2(3)]), 0,0,-0.05,0, 'r','LineWidth',2,'MaxHeadSize',2);
text(0.01,0,mean([P1(3),P2(3)]),'g (gravity)','Color','r','FontSize',9);
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
title(sprintf('Worst-case pose (shoulder=%d deg)', t2_range_deg(worst_idx)));
view(45,30);

% Plot 3: 3D pose at the BEST case (lowest torque, near vertical), for comparison
subplot(1,3,3);
hold on; grid on; axis equal;
[best_torque, best_idx] = min(abs(tau2_vals));
t2_p2 = deg2rad(t2_range_deg(best_idx));
phi3_p2 = t2_p2 - 0;
P0b = [0;0;0];
P1b = [0;0;L1];
P2b = P1b + [L2*cos(t2_p2); 0; L2*sin(t2_p2)];
P3b = P2b + [L3*cos(phi3_p2); 0; L3*sin(phi3_p2)];
ptsb = [P0b,P1b,P2b,P3b];
plot3(ptsb(1,:),ptsb(2,:),ptsb(3,:),'-o','Color','g','LineWidth',3,'MarkerFaceColor','g','MarkerSize',8);
quiver3(mean([P1b(1),P2b(1)]),0,mean([P1b(3),P2b(3)]), 0,0,-0.05,0, 'r','LineWidth',2,'MaxHeadSize',2);
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
title(sprintf('Best-case pose (shoulder=%d deg)', t2_range_deg(best_idx)));
view(45,30);

fprintf('\nDynamics_Lagrange.m complete.\n');
