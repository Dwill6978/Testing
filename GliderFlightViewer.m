function GliderFlightViewer()
%GLIDERFLIGHTVIEWER  Browse and plot clipped glider flight logs.
%
%   Pick a clipped flight (produced by the ground-station GUI / clip_flights
%   tool) from the list and it plots the gyroscope, accelerometer and
%   motor-command channels -- each sensor on its own tab -- in the shaded
%   red/blue "direction" style. Each tab can be exported to PNG or JPG, and all
%   major events from the flight's Events.csv are shown in the text box.
%
%   A clipped flight is a set of CSVs sharing a prefix ending in _flightN:
%     <prefix>_Controller.csv    cf_time_s,gyro_roll,gyro_pitch,gyro_yaw,set_roll,set_pitch,set_yaw
%     <prefix>_Accelerometer.csv cf_time_s,acc_x,acc_y,acc_z
%     <prefix>_Motor.csv         cf_time_s,motor_m4,motor_m1,motor_m2,servo_cmd
%     <prefix>_Events.csv        host_time_iso,event,value_1,value_2,value_3
%   Gyro columns are rad/s; motor commands are 0..65535 mapped to a -1..1
%   deflection via (cmd-32767)/32767. Time is shown relative to the clip start.
%
%   Requires MATLAB R2020a+ (uitab, tiledlayout, exportgraphics).
%
%   Usage:  GliderFlightViewer

    % ----- configuration --------------------------------------------------
    S.clippedRoot = fullfile(getHome(), 'Desktop', 'Testing', 'clipped');
    S.SERVO_CENTER = 32767;
    S.SERVO_SPAN   = 32767;
    S.prefixes  = {};   % full path prefixes for each listed flight
    S.currentName = '';
    % Events flagged as "major" (marked with * in the list).
    S.majorEvents = {'MOTOR_ARM','MOTOR_DISARM','FAILSAFE_DISARM', ...
                     'MOTOR_ARM_REJECTED','PID_APPLIED'};

    % Per-channel display: {title, ylabel, positive-label, negative-label}.
    S.gyroCh = { ...
        {'Gyroscope - Roll Axis',  'Radians/s', 'Roll Right', 'Roll Left'}, ...
        {'Gyroscope - Pitch Axis', 'Radians/s', 'Pitch Up',   'Pitch Down'}, ...
        {'Gyroscope - Yaw Axis',   'Radians/s', 'Yaw Right',  'Yaw Left'} };
    S.gyroLegend = {'gyro-roll','gyro-pitch','gyro-yaw'};
    S.accCh = { ...
        {'Accelerometer - X Axis', 'Acceleration (g)', 'Forward (+X)', 'Backward (-X)'}, ...
        {'Accelerometer - Y Axis', 'Acceleration (g)', 'Right (+Y)',   'Left (-Y)'}, ...
        {'Accelerometer - Z Axis', 'Acceleration (g)', 'Up (+Z)',      'Down (-Z)'} };
    S.motCh = { ...
        {'Aileron Deflection',   'Deflection Command', 'Roll Left', 'Roll Right'}, ...
        {'Elevator Deflection',  'Deflection Command', 'Pitch Up',  'Pitch Down'}, ...
        {'Aileron 2 Deflection', 'Deflection Command', 'Roll Left', 'Roll Right'} };

    % ----- build UI -------------------------------------------------------
    f = figure('Name','Glider Flight Viewer','NumberTitle','off', ...
               'Color','w','Units','pixels','Position',[80 80 1250 780]);

    uicontrol(f,'Style','text','String','Clipped flights','FontWeight','bold', ...
        'HorizontalAlignment','left','BackgroundColor','w', ...
        'Units','normalized','Position',[0.015 0.955 0.24 0.03]);
    lb = uicontrol(f,'Style','listbox','Units','normalized', ...
        'Position',[0.015 0.42 0.245 0.535],'Callback',@onListClick);
    uicontrol(f,'Style','pushbutton','String','Browse folder...', ...
        'Units','normalized','Position',[0.015 0.375 0.115 0.04],'Callback',@onBrowse);
    uicontrol(f,'Style','pushbutton','String','Rescan', ...
        'Units','normalized','Position',[0.135 0.375 0.06 0.04],'Callback',@(~,~)populateList());
    uicontrol(f,'Style','pushbutton','String','Plot', ...
        'Units','normalized','Position',[0.20 0.375 0.06 0.04],'Callback',@onPlot);
    uicontrol(f,'Style','pushbutton','String','Save current tab (PNG/JPG)...', ...
        'Units','normalized','Position',[0.015 0.33 0.245 0.04],'Callback',@onSave);

    uicontrol(f,'Style','text','String','Major events','FontWeight','bold', ...
        'HorizontalAlignment','left','BackgroundColor','w', ...
        'Units','normalized','Position',[0.015 0.295 0.24 0.028]);
    evtBox = uicontrol(f,'Style','edit','Max',2,'Min',0,'Enable','inactive', ...
        'HorizontalAlignment','left','FontName','monospaced','BackgroundColor','w', ...
        'Units','normalized','Position',[0.015 0.02 0.245 0.275],'String','');

    tg = uitabgroup('Parent',f,'Units','normalized','Position',[0.275 0.02 0.71 0.965]);
    tabG = uitab(tg,'Title','Gyroscope');
    tabA = uitab(tg,'Title','Accelerometer');
    tabM = uitab(tg,'Title','Motor Commands');

    S.fig = f; S.lb = lb; S.evtBox = evtBox; S.tg = tg;
    S.tabG = tabG; S.tabA = tabA; S.tabM = tabM;
    S.tlG = freshTiles(tabG);
    S.tlA = freshTiles(tabA);
    S.tlM = freshTiles(tabM);

    populateList();

    % ================= nested callbacks / helpers ====================
    function onBrowse(~,~)
        d = uigetdir(S.clippedRoot, 'Select clipped-logs folder');
        if isequal(d,0), return; end
        S.clippedRoot = d;
        populateList();
    end

    function populateList()
        S.prefixes = scanFlights(S.clippedRoot);
        names = cell(1, numel(S.prefixes));
        for i = 1:numel(S.prefixes)
            names{i} = relPath(S.prefixes{i}, S.clippedRoot);
        end
        if isempty(names)
            names = {'(no clipped flights found)'};
        end
        val = min(max(1, get(S.lb,'Value')), numel(names));
        set(S.lb, 'String', names, 'Value', val);
    end

    function onListClick(~,~)
        if strcmp(get(S.fig,'SelectionType'), 'open')   % double-click -> plot
            onPlot();
        end
    end

    function onPlot(~,~)
        idx = get(S.lb,'Value');
        if isempty(S.prefixes) || idx > numel(S.prefixes)
            return;
        end
        prefix = S.prefixes{idx};
        S.currentName = relPath(prefix, S.clippedRoot);
        plotFlight(prefix);
        fillEvents(prefix);
    end

    function onSave(~,~)
        tab = S.tg.SelectedTab;
        switch tab.Title
            case 'Gyroscope',     tl = S.tlG;
            case 'Accelerometer', tl = S.tlA;
            otherwise,            tl = S.tlM;
        end
        base = sanitize(S.currentName);
        if isempty(base), base = 'flight'; end
        defName = sprintf('%s_%s.png', base, strrep(tab.Title,' ','_'));
        [fn, pth] = uiputfile( ...
            {'*.png','PNG image (*.png)'; '*.jpg','JPEG image (*.jpg)'}, ...
            'Save current tab', defName);
        if isequal(fn, 0), return; end
        try
            exportgraphics(tl, fullfile(pth, fn), 'Resolution', 200);
        catch err
            errordlg(sprintf(['Could not save the tab (needs R2020a+ for ' ...
                'exportgraphics):\n%s'], err.message), 'Save failed');
        end
    end

    function plotFlight(prefix)
        ctrl = readNumericCsv([prefix '_Controller.csv']);
        acc  = readNumericCsv([prefix '_Accelerometer.csv']);
        mot  = readNumericCsv([prefix '_Motor.csv']);

        % Common zero so all three tabs share a time origin (the clip start).
        t0 = inf;
        if ~isempty(ctrl), t0 = min(t0, ctrl(1,1)); end
        if ~isempty(acc),  t0 = min(t0, acc(1,1));  end
        if ~isempty(mot),  t0 = min(t0, mot(1,1));  end
        if ~isfinite(t0),  t0 = 0; end

        % --- Gyroscope tab (raw rad/s + dashed setpoint) ---
        S.tlG = freshTiles(S.tabG);
        if ~isempty(ctrl)
            tg_ = ctrl(:,1) - t0;
            for k = 1:3
                ax = nexttile(S.tlG);
                c = S.gyroCh{k};
                styleAxis(ax, tg_, ctrl(:,1+k), ctrl(:,4+k), ...
                          c{1}, c{2}, c{3}, c{4}, [], S.gyroLegend{k});
            end
        else
            noDataAxis(S.tlG, 'No Controller.csv data');
        end

        % --- Accelerometer tab (g) ---
        S.tlA = freshTiles(S.tabA);
        if ~isempty(acc)
            ta_ = acc(:,1) - t0;
            for k = 1:3
                ax = nexttile(S.tlA);
                c = S.accCh{k};
                styleAxis(ax, ta_, acc(:,1+k), [], c{1}, c{2}, c{3}, c{4}, [], '');
            end
        else
            noDataAxis(S.tlA, 'No Accelerometer.csv data');
        end

        % --- Motor Commands tab (deflection -1..1) ---
        S.tlM = freshTiles(S.tabM);
        if ~isempty(mot)
            tm_ = mot(:,1) - t0;
            surfCmd = [defl(mot(:,2)), defl(mot(:,4)), defl(mot(:,3))]; % ail, elev, ail2
            for k = 1:3
                ax = nexttile(S.tlM);
                c = S.motCh{k};
                styleAxis(ax, tm_, surfCmd(:,k), [], c{1}, c{2}, c{3}, c{4}, [-1 1], '');
            end
        else
            noDataAxis(S.tlM, 'No Motor.csv data');
        end
    end

    function d = defl(cmd)
        d = (cmd - S.SERVO_CENTER) ./ S.SERVO_SPAN;
        d = max(-1, min(1, d));
    end

    function fillEvents(prefix)
        E = readEvents([prefix '_Events.csv']);
        lines = {sprintf('Flight: %s', S.currentName), ...
                 repmat('-', 1, 40)};
        if isempty(E)
            lines{end+1} = '(no events logged)';
        else
            for i = 1:size(E,1)
                ts   = E{i,1};
                name = E{i,2};
                extra = strtrim(strjoin(E(i,3:end), ' '));
                marker = ' ';
                if any(strcmp(name, S.majorEvents))
                    marker = '*';
                end
                if isempty(extra)
                    lines{end+1} = sprintf('%s %s  %s', marker, ts, name); %#ok<AGROW>
                else
                    lines{end+1} = sprintf('%s %s  %s  [%s]', marker, ts, name, extra); %#ok<AGROW>
                end
            end
        end
        set(S.evtBox, 'String', lines);
    end

    % ---- axis styling (shaded red/blue halves, direction labels) ----
    function styleAxis(ax, t, y, setp, ttl, ylab, posLabel, negLabel, fixedYl, legName)
        cla(ax);
        title(ax, ttl, 'FontWeight', 'bold');
        xlabel(ax, 'Time (s)');
        ylabel(ax, ylab);
        if isempty(t) || isempty(y)
            return;
        end
        hold(ax, 'on');
        xl = [t(1), t(end)];
        if xl(2) <= xl(1), xl(2) = xl(1) + 1; end
        if ~isempty(fixedYl)
            yl = fixedYl;
        else
            lo = min(y); hi = max(y);
            if hi <= lo, hi = lo + 1; end
            pad = (hi - lo) * 0.10;
            yl = [lo - pad, hi + pad];
        end
        % Shaded halves: red above 0 (positive), blue below 0 (negative).
        yr = max(0, yl(2));  yb = min(0, yl(1));
        patch(ax, [xl(1) xl(2) xl(2) xl(1)], [0 0 yr yr], ...
              [1 0.86 0.86], 'EdgeColor', 'none');
        patch(ax, [xl(1) xl(2) xl(2) xl(1)], [yb yb 0 0], ...
              [0.86 0.86 1], 'EdgeColor', 'none');
        h1 = plot(ax, t, y, 'k-', 'LineWidth', 1.2);
        hasSetp = ~isempty(setp) && any(isfinite(setp));
        h2 = [];
        if hasSetp
            h2 = plot(ax, t, setp, 'k--', 'LineWidth', 1.0);
        end
        set(ax, 'Layer', 'top');
        box(ax, 'on');
        grid(ax, 'on');
        xlim(ax, xl);
        ylim(ax, yl);
        dx = diff(xl);  dy = diff(yl);
        text(ax, xl(2) - 0.02*dx, yl(2) - 0.14*dy, posLabel, ...
             'HorizontalAlignment','right', 'FontWeight','bold', ...
             'BackgroundColor','w', 'EdgeColor','k', 'Margin', 2);
        text(ax, xl(1) + 0.02*dx, yl(1) + 0.14*dy, negLabel, ...
             'HorizontalAlignment','left', 'FontWeight','bold', ...
             'BackgroundColor','w', 'EdgeColor','k', 'Margin', 2);
        if ~isempty(legName) && hasSetp
            legend(ax, [h1 h2], {legName, 'Setpoint'}, 'Location', 'northeast');
        end
        hold(ax, 'off');
    end
end

% ===================== file-scope helpers ============================
function tl = freshTiles(tab)
    delete(allchild(tab));
    tl = tiledlayout(tab, 3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
end

function noDataAxis(tl, msg)
    ax = nexttile(tl, [3 1]);
    text(ax, 0.5, 0.5, msg, 'HorizontalAlignment', 'center', ...
         'Units', 'normalized', 'FontAngle', 'italic', 'Color', [0.4 0.4 0.4]);
    set(ax, 'XTick', [], 'YTick', []);
    box(ax, 'on');
end

function prefixes = scanFlights(root)
    prefixes = {};
    if exist(root, 'dir') ~= 7
        return;
    end
    d = dir(fullfile(root, '**', '*_Accelerometer.csv'));
    suf = '_Accelerometer.csv';
    seen = {};
    for i = 1:numel(d)
        nm = d(i).name;
        if numel(nm) <= numel(suf)
            continue;
        end
        base = nm(1:end-numel(suf));
        if isempty(strfind(base, '_flight')) %#ok<STREMP>
            continue;   % only clipped flight sets (prefix has _flightN)
        end
        p = fullfile(d(i).folder, base);
        if ~any(strcmp(seen, p))
            seen{end+1} = p;         %#ok<AGROW>
            prefixes{end+1} = p;     %#ok<AGROW>
        end
    end
    prefixes = sort(prefixes);
end

function r = relPath(p, root)
    r = p;
    if strncmp(p, root, numel(root))
        r = p(numel(root)+1:end);
        while ~isempty(r) && (r(1) == '/' || r(1) == '\')
            r(1) = [];
        end
    end
end

function s = sanitize(x)
    s = regexprep(x, '[^\w\-]', '_');
end

function h = getHome()
    h = getenv('HOME');
    if isempty(h), h = getenv('USERPROFILE'); end
    if isempty(h), h = pwd; end
end

function M = readNumericCsv(file)
%READNUMERICCSV  Read a cf_time-keyed stream, skipping the schema comment, the
%   column header and any interior "# BREAKPOINT" lines. Returns a numeric
%   matrix (empty if the file is missing/empty).
    M = [];
    fid = fopen(file, 'r');
    if fid < 0
        return;
    end
    closer = onCleanup(@() fclose(fid));
    rows = {};
    ncol = 0;
    while true
        ln = fgetl(fid);
        if ~ischar(ln)
            break;
        end
        if isempty(ln) || ln(1) == '#'
            continue;                       % blank / comment / breakpoint
        end
        first = ln(1);
        if ~(first == '+' || first == '-' || first == '.' || ...
             (first >= '0' && first <= '9'))
            continue;                       % header row (e.g. cf_time_s,...)
        end
        parts = strsplit(ln, ',');
        vals = str2double(parts);
        rows{end+1} = vals;                 %#ok<AGROW>
        ncol = max(ncol, numel(vals));
    end
    if isempty(rows)
        return;
    end
    M = NaN(numel(rows), ncol);
    for i = 1:numel(rows)
        v = rows{i};
        M(i, 1:numel(v)) = v;
    end
end

function E = readEvents(file)
%READEVENTS  Read Events.csv into an Nx5 cell array of strings
%   {host_time, event, value_1, value_2, value_3}, skipping comments/header.
    E = {};
    fid = fopen(file, 'r');
    if fid < 0
        return;
    end
    closer = onCleanup(@() fclose(fid));
    while true
        ln = fgetl(fid);
        if ~ischar(ln)
            break;
        end
        if isempty(ln) || ln(1) == '#'
            continue;
        end
        if strncmp(ln, 'host_time_iso', 13)
            continue;                       % header
        end
        parts = strsplit(ln, ',', 'CollapseDelimiters', false);
        while numel(parts) < 5
            parts{end+1} = '';              %#ok<AGROW>
        end
        E(end+1, :) = parts(1:5);           %#ok<AGROW>
    end
end
