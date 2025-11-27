%% Load the MAT file
data = load('db.mat');  

%% Get the timeseries from the struct
ts = data.ans;   

%% Create a table from the timeseries data
T = array2table(ts.Data);

% Optional: give generic names S1, S2, ...
nSignals = size(ts.Data, 2);
T.Properties.VariableNames = "S" + string(1:nSignals);

T.Properties.VariableNames = {'Vgrid_A','Vgrid_B','V_grid_C', 'Igrid_A','Igrid_B','I_grid_C', 'F_grid', 'Id','Iq','Te', 'Tload', 'omegaR', 'Vm_A', 'Vm_B', 'Vm_C', 'Im_A', 'Im_B', 'Im_C', 'Ref_Speed'}; 

% Add time column and make it the first column
T.time = ts.Time;
T = movevars(T, 'time', 'Before', 1);

%% Save as CSV
writetable(T, 'db.csv');

disp('CSV file created: db.csv');
