% Truss Design Analysis Program %
% By: Alfonso Meraz

% Load input file parameters

load('final_design.mat')

% Initialize Variables
[j,m] = size(C);    % joint and member #'s
S = [Sx;Sy];       % Reaction forces
A = zeros(2*j, m);  % Member tension at each joint
members = zeros(m, 1); % member vector

marker = 0;
total_member_length = 0;

% Analysis Logic
for i = 1:m         % looping through members
    for k = 1:j     % looping through joints
        if C(k, i) == 1
            if marker == 0
                x1 = X(k);
                y1 = Y(k);
                r = k + 1;
                for c = r:j
                    if C(c, i) == 1
                        x2 = X(c);
                        y2 = Y(c);
                        member_length = sqrt((x2-x1)^2+(y2-y1)^2);
                        members(i) = member_length;
                        total_member_length = total_member_length + member_length;
                        A(k, i) = ((x2-x1))/member_length;
                        A(c, i) = ((x1-x2))/member_length;
                        A(j+k, i) = ((y2-y1))/member_length;
                        A(j+c, i) = ((y1-y2))/member_length;
                        marker = 1;
                    end
                end
            else
                marker = 0;
            end
        end
    end
end

A = [A, S]; % Combining matrix A(tensions) with S(reaction forces)                        

T = A\L; % Solving the matrix for unknown Tensions

TRat = [];
TBuck = [];

total_cost = total_member_length + 10 * j;
total_load = sum(L);

% Final Output
fprintf('EK301, Section A1, Group: Alfonso M., May A., Karolyn B., 4/9/2022.\n')
fprintf('Load: %d oz\n', total_load)
fprintf('Member forces in oz\n')

for i = 1:m
    TRat = [TRat (T(i)/(2570 / members(i)^2))];
    TBuck = [TBuck (2570 / (members(i))^2)];
    fprintf('m%d: %.2f', i, abs(T(i)))
    if T(i) >= 0
        fprintf(' (T)\n')
    else
        fprintf(' (C)\n')
    end
end

weak_member = find(TRat == min(TRat));
buck_load_coef = abs(1 / TRat(weak_member));
max_load = total_load * buck_load_coef;


fprintf('Reaction forces in oz:\n')
fprintf('Sx1: %.2f\n', T(m+1))
fprintf('Sy1: %.2f\n', T(m+2))
fprintf('Sy2: %.2f\n', T(m+3))
fprintf('Cost of Truss: $%.f\n', total_cost)
fprintf('Theoretical max load/cost ratio in oz/$: %.2f\n', max_load/total_cost)