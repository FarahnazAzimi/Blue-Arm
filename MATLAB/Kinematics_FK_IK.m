%% ========================================================
%  KINEMATICS - Forward & Inverse, with Dual Solutions & 3D Plots
%  3-DOF RRR Robotic Arm - "Blue Arm" Project
%  --------------------------------------------------------
%  WHY THIS FILE MATTERS:
%  This is the file that actually matches what runs on the real robot
%  (see kinematics.py and esp32_standalone_firmware.ino). While
%  RobotModeling.m shows the IDEAL geometric theory, this file adds
%  the CALIBRATION found by testing the physical arm - because no real
%  servo is mounted at a perfect, exact angle. Without this
%  calibration, commanding "go to this position" would miss by
%  several centimeters.
%
%  WHAT THIS FILE SHOWS:
%  1. Forward Kinematics (FK): given joint angles -> where is the tip?
%  2. Inverse Kinematics (IK): given a target position -> what angles?
%     Includes BOTH mathematical solutions (elbow-up and elbow-down) -
%     see the important hardware note below.
%  3. A round-trip test proving FK and IK agree with each other
%  4. 3D plots: FK configurations, elbow-up vs elbow-down, pick-and-
%     place, and the full calibrated reachable workspace
%  --------------------------------------------------------
%  IMPORTANT HARDWARE NOTE ON DUAL SOLUTIONS:
%  A 2-link arm mathematically has TWO ways to reach most points
%  ("elbow-up" and "elbow-down"). This is true in theory for any
%  2-link geometry. However, THIS particular robot's elbow servo can
%  only bend in ONE physical direction (commanded range 25-100 degrees,
%  confirmed by testing - see project notes on elbow calibration).
%  This means only the ELBOW-UP solution is mechanically achievable on
%  the real hardware. The elbow-down solution is shown here for
%  academic completeness (this is standard robotics theory, and a
%  differently-built arm could use it), but is NOT something this
%  specific robot can physically do.
%% ========================================================

clc; clear all; close all;

%% --------------------------------------------------------
%  SECTION 1 - Real Measured Link Parameters (same as RobotModeling.m)
%% --------------------------------------------------------
GROUND_OFFSET = 5;   % cm
L1 = 8;              % cm
L2 = 21;             % cm
L3 = 28;             % cm

%% --------------------------------------------------------
%  SECTION 2 - Calibration (found by testing the real physical robot)
%% --------------------------------------------------------
THETA2_OFFSET = 15.5;  % degrees - shoulder: real angle = commanded - this
THETA3_OFFSET = 49.7;  % degrees - elbow: real bend = commanded + this

POS_CORRECTION_X = -5;  % cm - found by real pick-and-place testing
POS_CORRECTION_Y =  4;  % cm
POS_CORRECTION_Z = -7;  % cm

%% --------------------------------------------------------
%  SECTION 3 - Real Safe Joint Limits (commanded angle ranges)
%% --------------------------------------------------------
JOINT1_MIN = 15;  JOINT1_MAX = 170;  % base (deg)
JOINT2_MIN = 45;  JOINT2_MAX = 90;   % shoulder (deg)
JOINT3_MIN = 25;  JOINT3_MAX = 100;  % elbow (deg)

fprintf('=== Blue Arm - Calibrated Kinematics ===\n');
fprintf('Links (cm): GROUND=%d L1=%d L2=%d L3=%d\n', GROUND_OFFSET,L1,L2,L3);
fprintf('Calibration: THETA2_OFFSET=%.1f  THETA3_OFFSET=%.1f\n', THETA2_OFFSET, THETA3_OFFSET);
fprintf('Position correction: X=%d  Y=%d  Z=%d cm\n\n', POS_CORRECTION_X, POS_CORRECTION_Y, POS_CORRECTION_Z);

%% ========================================================
%  SECTION 4 - Forward Kinematics Test Cases
%% ========================================================
fprintf('=== Section 4: FK Test Cases ===\n');
fprintf('%-18s %8s %8s %8s %10s %10s %10s\n', ...
        'Config','t1(deg)','t2(deg)','t3(deg)','Px(cm)','Py(cm)','Pz(cm)');
fprintf('%s\n', repmat('-',1,82));

tests_deg = [
     90,  90,  25;    % HOME position
     90,  45, 100;    % shoulder low, elbow fully folded
    170,  70,  60;    % base rotated to one side
     15,  70,  60;    % base rotated to the other side
];
configs = {'HOME','Shldr low/Elbow fold','Base right','Base left'};

for i = 1:size(tests_deg,1)
    [Px, Py, Pz] = forward_kinematics(tests_deg(i,1), tests_deg(i,2), tests_deg(i,3), ...
                                       GROUND_OFFSET, L1, L2, L3, THETA2_OFFSET, THETA3_OFFSET);
    fprintf('%-18s %8.1f %8.1f %8.1f %10.4f %10.4f %10.4f\n', ...
            configs{i}, tests_deg(i,1), tests_deg(i,2), tests_deg(i,3), Px, Py, Pz);
end

%% ========================================================
%  SECTION 5 - IK Round-Trip Test (elbow-up only, matches real hardware)
%% ========================================================
fprintf('\n=== Section 5: IK Round-Trip Test ===\n');
fprintf('%-12s %9s %9s %9s %9s %9s %9s %10s\n', ...
        'Config','t1_in','t2_in','t3_in','t1_out','t2_out','t3_out','PosErr*');
fprintf('%s\n', repmat('-',1,88));

roundtrip_deg = [
     90,  90,  25;
     90,  60,  70;
    120,  50,  90;
     40,  80,  40;
];
rt_names = {'Config A','Config B','Config C','Config D'};

for i = 1:size(roundtrip_deg,1)
    t1o = roundtrip_deg(i,1); t2o = roundtrip_deg(i,2); t3o = roundtrip_deg(i,3);
    [Px, Py, Pz] = forward_kinematics(t1o, t2o, t3o, GROUND_OFFSET, L1, L2, L3, THETA2_OFFSET, THETA3_OFFSET);
    [t1r, t2r, t3r, valid] = inverse_kinematics_dual(Px, Py, Pz, GROUND_OFFSET, L1, L2, L3, ...
                                                       THETA2_OFFSET, THETA3_OFFSET, 0, 0, 0, +1);
    pos_err = NaN;
    if valid
        [Px2, Py2, Pz2] = forward_kinematics(t1r, t2r, t3r, GROUND_OFFSET, L1, L2, L3, THETA2_OFFSET, THETA3_OFFSET);
        pos_err = sqrt((Px-Px2)^2 + (Py-Py2)^2 + (Pz-Pz2)^2);
    end
    fprintf('%-12s %9.1f %9.1f %9.1f %9.1f %9.1f %9.1f %10.6f\n', ...
            rt_names{i}, t1o, t2o, t3o, t1r, t2r, t3r, pos_err);
end
fprintf('*PosErr uses ZERO position correction here on purpose, to test\n');
fprintf('ONLY whether FK and IK math agree with each other (they should\n');
fprintf('give ~0.000000 cm error). The position correction is a separate,\n');
fprintf('deliberate real-world fix, applied in Sections 7-8 below.\n');

%% ========================================================
%  SECTION 6 - Pick and Place Preview
%  --------------------------------------------------------
%  IMPORTANT: two different angle sets are shown below, on purpose.
%  "Diagram angles" (zero correction) are what makes the plotted arm
%  land EXACTLY on the target marker - useful for a clear, readable
%  figure. "Real robot angles" (with correction) are what you would
%  actually SEND to the physical robot, so it compensates for the
%  small real-world offset we measured earlier. These are supposed to
%  be slightly different - that difference IS the correction working.
%% ========================================================
fprintf('\n=== Section 6: Pick and Place Preview ===\n');

P_pick  = [20, 15, 5];    % cm - a real measured object position
P_place = [15, 25, 5];    % cm - a real measured target position

% Diagram-accurate angles (zero correction) - used for the plot below
[t1_pk,t2_pk,t3_pk,ok1] = inverse_kinematics_dual(P_pick(1),P_pick(2),P_pick(3), ...
    GROUND_OFFSET,L1,L2,L3,THETA2_OFFSET,THETA3_OFFSET, 0,0,0, +1);
[t1_pl,t2_pl,t3_pl,ok2] = inverse_kinematics_dual(P_place(1),P_place(2),P_place(3), ...
    GROUND_OFFSET,L1,L2,L3,THETA2_OFFSET,THETA3_OFFSET, 0,0,0, +1);

% Real robot angles (with correction) - what you'd actually command
[t1_pk_real,t2_pk_real,t3_pk_real,~] = inverse_kinematics_dual(P_pick(1),P_pick(2),P_pick(3), ...
    GROUND_OFFSET,L1,L2,L3,THETA2_OFFSET,THETA3_OFFSET, ...
    POS_CORRECTION_X,POS_CORRECTION_Y,POS_CORRECTION_Z, +1);
[t1_pl_real,t2_pl_real,t3_pl_real,~] = inverse_kinematics_dual(P_place(1),P_place(2),P_place(3), ...
    GROUND_OFFSET,L1,L2,L3,THETA2_OFFSET,THETA3_OFFSET, ...
    POS_CORRECTION_X,POS_CORRECTION_Y,POS_CORRECTION_Z, +1);

if ok1
    fprintf('PICK  [%4.0f,%4.0f,%4.0f]cm\n', P_pick(1),P_pick(2),P_pick(3));
    fprintf('  Diagram angles (plot below): t1=%7.2f t2=%7.2f t3=%7.2f deg\n', t1_pk,t2_pk,t3_pk);
    fprintf('  Real robot angles (send this to the firmware): t1=%7.2f t2=%7.2f t3=%7.2f deg\n', ...
            t1_pk_real,t2_pk_real,t3_pk_real);
else
    fprintf('PICK target is unreachable.\n');
end
if ok2
    fprintf('PLACE [%4.0f,%4.0f,%4.0f]cm\n', P_place(1),P_place(2),P_place(3));
    fprintf('  Diagram angles (plot below): t1=%7.2f t2=%7.2f t3=%7.2f deg\n', t1_pl,t2_pl,t3_pl);
    fprintf('  Real robot angles (send this to the firmware): t1=%7.2f t2=%7.2f t3=%7.2f deg\n', ...
            t1_pl_real,t2_pl_real,t3_pl_real);
else
    fprintf('PLACE target is unreachable.\n');
end

%% ========================================================
%  SECTION 7 - Elbow-Up vs Elbow-Down Dual Solutions
%% ========================================================
fprintf('\n=== Section 7: Elbow-Up vs Elbow-Down ===\n');

Px_t = 25; Py_t = 20; Pz_t = 10;
fprintf('Target: Px=%.1f Py=%.1f Pz=%.1f cm\n', Px_t, Py_t, Pz_t);

[t1_up,t2_up,t3_up,v1] = inverse_kinematics_dual(Px_t,Py_t,Pz_t,GROUND_OFFSET,L1,L2,L3, ...
    THETA2_OFFSET,THETA3_OFFSET,0,0,0,+1);
[t1_dn,t2_dn,t3_dn,v2] = inverse_kinematics_dual(Px_t,Py_t,Pz_t,GROUND_OFFSET,L1,L2,L3, ...
    THETA2_OFFSET,THETA3_OFFSET,0,0,0,-1);

if v1
    fprintf('ELBOW-UP   (matches real hardware): t1=%7.2f t2=%7.2f t3=%7.2f deg\n', t1_up,t2_up,t3_up);
    up_in_range = (t2_up>=JOINT2_MIN && t2_up<=JOINT2_MAX && t3_up>=JOINT3_MIN && t3_up<=JOINT3_MAX);
    fprintf('  Within real joint limits: %d\n', up_in_range);
end
if v2
    fprintf('ELBOW-DOWN (theory only, not achievable on this hardware): t1=%7.2f t2=%7.2f t3=%7.2f deg\n', t1_dn,t2_dn,t3_dn);
    down_in_range = (t2_dn>=JOINT2_MIN && t2_dn<=JOINT2_MAX && t3_dn>=JOINT3_MIN && t3_dn<=JOINT3_MAX);
    fprintf('  Within real joint limits: %d  (expected to usually be 0/false)\n', down_in_range);
end

%% ========================================================
%  SECTION 8 - Visualization
%% ========================================================
figure('Name','Kinematics - FK, IK Dual Solutions, Pick and Place', ...
       'Color','white','Position',[50,50,1400,500]);

% Plot 1: FK configurations
subplot(1,3,1);
hold on; grid on;
clr = {'b','r','g','m'};
for i = 1:size(tests_deg,1)
    plot_robot_3d(tests_deg(i,1), tests_deg(i,2), tests_deg(i,3), ...
                  GROUND_OFFSET, L1, L2, L3, THETA2_OFFSET, THETA3_OFFSET, clr{i}, configs{i});
end
xlabel('X(cm)'); ylabel('Y(cm)'); zlabel('Z(cm)');
title('FK - Configurations'); view(45,30); axis equal;

% Plot 2: IK dual solutions
subplot(1,3,2);
hold on; grid on;
if v1, plot_robot_3d(t1_up,t2_up,t3_up,GROUND_OFFSET,L1,L2,L3,THETA2_OFFSET,THETA3_OFFSET,'b','Elbow-UP (real)'); end
if v2, plot_robot_3d(t1_dn,t2_dn,t3_dn,GROUND_OFFSET,L1,L2,L3,THETA2_OFFSET,THETA3_OFFSET,'r','Elbow-DOWN (theory)'); end
plot3(Px_t,Py_t,Pz_t,'k*','MarkerSize',14,'LineWidth',2);
text(Px_t+1,Py_t+1,Pz_t+2,'Target','FontSize',9);
xlabel('X(cm)'); ylabel('Y(cm)'); zlabel('Z(cm)');
title('IK - Elbow-Up (real) vs Elbow-Down (theory)'); view(45,30); axis equal;

% Plot 3: Pick and place
subplot(1,3,3);
hold on; grid on;
if ok1, plot_robot_3d(t1_pk,t2_pk,t3_pk,GROUND_OFFSET,L1,L2,L3,THETA2_OFFSET,THETA3_OFFSET,'b','PICK'); end
if ok2, plot_robot_3d(t1_pl,t2_pl,t3_pl,GROUND_OFFSET,L1,L2,L3,THETA2_OFFSET,THETA3_OFFSET,'r','PLACE'); end
plot3(P_pick(1), P_pick(2), P_pick(3), 'bs','MarkerSize',12,'LineWidth',2);
plot3(P_place(1),P_place(2),P_place(3),'r^','MarkerSize',12,'LineWidth',2);
xlabel('X(cm)'); ylabel('Y(cm)'); zlabel('Z(cm)');
title('Pick and Place'); view(45,30); axis equal;

fprintf('\nKinematics_FK_IK.m complete.\n');
fprintf('(Full 3D workspace plot lives in RobotModeling.m - not repeated here.)\n');

%% ========================================================
%  LOCAL FUNCTIONS (must be at end of script - MATLAB rule)
%% ========================================================

function [Px, Py, Pz] = forward_kinematics(t1_cmd, t2_cmd, t3_cmd, GROUND_OFFSET, L1, L2, L3, THETA2_OFFSET, THETA3_OFFSET)
    % Converts COMMANDED angles (deg) to the REAL gripper tip position (cm).
    % Mirrors kinematics.py's forward_kinematics() exactly.
    t1 = deg2rad(t1_cmd);
    t2 = deg2rad(t2_cmd - THETA2_OFFSET);
    t3 = deg2rad(t3_cmd + THETA3_OFFSET);
    phi3 = t2 - t3;

    r = L2*cos(t2) + L3*cos(phi3);
    z = GROUND_OFFSET + L1 + L2*sin(t2) + L3*sin(phi3);

    Px = r * cos(t1);
    Py = r * sin(t1);
    Pz = z;
end

function [t1_cmd, t2_cmd, t3_cmd, valid] = inverse_kinematics_dual(x, y, z, GROUND_OFFSET, L1, L2, L3, ...
                                                                     THETA2_OFFSET, THETA3_OFFSET, ...
                                                                     CORR_X, CORR_Y, CORR_Z, elbow_sign)
    % Converts a REAL desired position (cm) to COMMANDED angles (deg).
    % elbow_sign = +1 -> elbow-up (matches the real hardware)
    % elbow_sign = -1 -> elbow-down (theory only - see file header note)
    valid = true;

    x = x + CORR_X;
    y = y + CORR_Y;
    z = z + CORR_Z;

    t1_cmd = rad2deg(atan2(y, x));

    r = sqrt(x^2 + y^2);
    z_rel = z - GROUND_OFFSET - L1;
    D = sqrt(r^2 + z_rel^2);

    if D > (L2+L3) || D < abs(L2-L3)
        t1_cmd = 0; t2_cmd = 0; t3_cmd = 0; valid = false;
        return;
    end

    cos_gamma = (L2^2 + L3^2 - D^2) / (2*L2*L3);
    cos_gamma = max(-1, min(1, cos_gamma));
    gamma = acos(cos_gamma);  % principal value, 0..180 deg
    theta3_real_bend_deg = elbow_sign * (180 - rad2deg(gamma));

    cos_beta = (L2^2 + D^2 - L3^2) / (2*L2*D);
    cos_beta = max(-1, min(1, cos_beta));
    beta = rad2deg(acos(cos_beta));

    angle_to_target_deg = rad2deg(atan2(z_rel, r));
    theta2_real_deg = angle_to_target_deg + elbow_sign*beta;

    t2_cmd = theta2_real_deg + THETA2_OFFSET;
    t3_cmd = theta3_real_bend_deg - THETA3_OFFSET;
end

function plot_robot_3d(t1_cmd, t2_cmd, t3_cmd, GROUND_OFFSET, L1, L2, L3, THETA2_OFFSET, THETA3_OFFSET, color, label)
    % Draws the 3-segment arm in 3D for a given set of COMMANDED angles.
    t1 = deg2rad(t1_cmd);
    t2 = deg2rad(t2_cmd - THETA2_OFFSET);
    t3 = deg2rad(t3_cmd + THETA3_OFFSET);
    phi3 = t2 - t3;

    P0 = [0; 0; 0];
    P1 = [0; 0; GROUND_OFFSET + L1];
    P2 = P1 + [ cos(t1)*L2*cos(t2); sin(t1)*L2*cos(t2); L2*sin(t2) ];
    P3 = P2 + [ cos(t1)*L3*cos(phi3); sin(t1)*L3*cos(phi3); L3*sin(phi3) ];

    pts = [P0, P1, P2, P3];
    plot3(pts(1,:),pts(2,:),pts(3,:),'-o', ...
          'Color',color,'LineWidth',2.5,'MarkerFaceColor',color,'MarkerSize',7);
    text(P3(1),P3(2),P3(3)+1.5,label,'FontSize',9,'Color',color);
end
