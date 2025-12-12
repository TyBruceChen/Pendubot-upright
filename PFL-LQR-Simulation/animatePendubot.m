function animatePendubot(t, x, varargin)
% animatePendubot(t, x)
% x = [q1; q2; dq1; dq2]

% Optional parameter: slow motion factor
if nargin > 2
    slow = varargin{1};
else
    slow = 0.5;
end

% link lengths
l1 = 1.0;
l2 = 1.0;

figure; hold on; axis equal;
axis([-2 2 -2 2]);
grid on;

% base point
Ox = 0; Oy = 0;

h1 = plot([0 0],[0 0],'LineWidth',3,'Color','b');   % Link 1
h2 = plot([0 0],[0 0],'LineWidth',3,'Color','r');   % Link 2
pt = plot(0,0,'ko','MarkerSize',6,'MarkerFaceColor','k'); % elbow

for k = 1:length(t)
    
    target_check(x(k,:))
    q1 = x(k,1);
    q2 = x(k,2);

    % Kinematics
    x1 = Ox + l1*cos(q1);
    y1 = Oy + l1*sin(q1);

    x2 = x1 + l2*cos(q1 + q2);
    y2 = y1 + l2*sin(q1 + q2);

    % update graphics
    set(h1, 'XData', [Ox x1], 'YData', [Oy y1]);
    set(h2, 'XData', [x1 x2], 'YData', [y1 y2]);
    set(pt, 'XData', x1, 'YData', y1);
    title(['t:', num2str(t(k))]);
    drawnow;

    pause(slow * (t(2)-t(1)));
end
end

function target_check(x)
x(1) = normalize_angle_rad(x(1));
x(2) = normalize_angle_rad(x(2));
if abs(sum(x(1:4)- [pi/2,0,0,0])) <= 0.001
    %display(abs(sum(x(1:2)- [pi/2, 0])));
    display(x);
end
end

function normalized_angle = normalize_angle_rad(angle_rad)
    % This function normalizes an angle in radians to the range -pi to pi
    normalized_angle = mod(angle_rad + pi, 2*pi) - pi;
end