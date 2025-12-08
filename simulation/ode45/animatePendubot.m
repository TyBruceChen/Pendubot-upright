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
title('t=');

for k = 1:length(t)

    q1 = x(1,k);
    q2 = x(2,k);

    % Kinematics
    x1 = Ox + l1*cos(q1);
    y1 = Oy + l1*sin(q1);

    x2 = x1 + l2*cos(q1 + q2);
    y2 = y1 + l2*sin(q1 + q2);

    % update graphics
    set(h1, 'XData', [Ox x1], 'YData', [Oy y1]);
    set(h2, 'XData', [x1 x2], 'YData', [y1 y2]);
    set(pt, 'XData', x1, 'YData', y1);
    title(['t=', num2str(t(k))] );

    drawnow;

    pause(slow * (t(2)-t(1)));
end
disp(t(end));
end
