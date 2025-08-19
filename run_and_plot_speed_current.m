function run_and_plot_speed_current()
% Run motor, read SPEED_MEAS + I_ALPHA + I_BETA, and plot them.

%% --- User settings ---
PORT        = "COM4";
BAUD        = 1843200;
MOTOR       = 1;             % 0..7
TARGET_RPM  = 3000;          % can be negative
MONITOR_SEC = 8;             % how long to log
SAMPLE_T    = 0.01;          % ~100 Hz sampling

% IDs as shown by Motor Pilot “Value” list (decimal)
ID_SPEED_MEAS   = 89;        % 0x0059, signed int32 rpm
ID_I_ALPHA      = 3025;      % 0x0BD1, signed int16 raw
ID_I_BETA       = 3089;      % 0x0C11, signed int16 raw

% If you know the scaling of these int16 currents -> set here (A/LSB)
CURR_SCALE = 1.0;            % leave 1.0 to plot raw units

%% --- Open & handshake ---
TIPP = 0.0015;  % header->payload gap
fprintf("Opening %s @ %d...\n", PORT, BAUD);
h = serialport(PORT, BAUD, "Timeout",2,"DataBits",8,"Parity","none","StopBits",1,"FlowControl","none");
cleanupObj = onCleanup(@() cleanup_serial(h)); %#ok<NASGU>
flush(h);

% Handshake (your proven frames)
write(h, uint8([0x85 0xFF 0xFF 0xBF]), "uint8"); pause(0.05); if h.NumBytesAvailable>=4, read(h,4,"uint8"); end
write(h, uint8([0x05 0xC7 0x01 0x14]), "uint8"); pause(0.05); if h.NumBytesAvailable>=4, read(h,4,"uint8"); end
write(h, uint8([0x06 0x00 0x00 0x60]), "uint8"); pause(0.05); if h.NumBytesAvailable>=4, read(h,4,"uint8"); end

% Headers 
HDR_LEN2      = uint8([0x29 0x00 0x00 0xE0]);   % 2B payload (ACK/START/STOP)
HDR_SPEED_LEN = uint8([0xC9 0x00 0x00 0xC0]);   % 12B payload (set speed)
HDR_GETREG    = uint8([0x69 0x00 0x00 0x00]);   % 6B payload (get register)

% MCP words
pay_start     = mcp_word(0x0003, MOTOR);
pay_stop      = mcp_word(0x0004, MOTOR);
pay_fault_ack = mcp_word(0x0007, MOTOR);

% Bring up
send_req(h, HDR_LEN2, pay_fault_ack, TIPP);  recv_pkt_quiet(h);
send_req(h, HDR_LEN2, pay_start,     TIPP);  recv_pkt_quiet(h);
pause(0.2);

% Set speed
send_req(h, HDR_SPEED_LEN, make_speed_payload(TARGET_RPM), TIPP);
recv_pkt_quiet(h);
fprintf("Target = %d rpm\n", TARGET_RPM);

%% --- Logging loop ---
t0   = tic;
tlog = []; slog = []; ialog = []; iblog = [];
while toc(t0) < MONITOR_SEC
    t = toc(t0);

    rpm = read_reg_s32(h, HDR_GETREG, ID_SPEED_MEAS, TIPP);

    ia  = read_reg_s16(h, HDR_GETREG, ID_I_ALPHA, TIPP);
    ib  = read_reg_s16(h, HDR_GETREG, ID_I_BETA,  TIPP);

    if ~isnan(rpm) && ~isnan(ia) && ~isnan(ib)
        tlog(end+1) = t; %#ok<AGROW>
        slog(end+1) = rpm; %#ok<AGROW>
        ialog(end+1) = double(ia)*CURR_SCALE; %#ok<AGROW>
        iblog(end+1) = double(ib)*CURR_SCALE; %#ok<AGROW>
        fprintf("meas: %7d rpm  |  Ia=%7.2f  Ib=%7.2f\n", rpm, ialog(end), iblog(end));
    end
    pause(SAMPLE_T);
end

%% --- Stop & plots ---
send_req(h, HDR_LEN2, pay_stop, TIPP); recv_pkt_quiet(h);

figure('Name','Speed & Currents');
tiledlayout(2,1);
nexttile; plot(tlog, slog); grid on; ylabel('rpm'); title('SPEED\_MEAS (0x0059)');
nexttile; plot(tlog, ialog); hold on; plot(tlog, iblog); hold off; grid on;
ylabel(sprintf('current (x%.3g)', CURR_SCALE)); xlabel('s');
legend('I_\alpha','I_\beta');
end

% ================= helpers =================
function cleanup_serial(h), try, if ~isempty(h), clear h; end, catch, end, end

function out = mcp_word(cmdId, motor)
    w = uint16(bitshift(uint16(cmdId),3) + bitand(uint16(motor),7));
    out = uint8([bitand(w,255) bitshift(w,-8)]);
end

function payload = make_speed_payload(rpm)
    v4 = typecast(int32(rpm), 'uint8');
    payload = uint8([0x08 0x00 0xA9 0x01 0x06 0x00 v4 0x00 0x00]);
end

function send_req(h, header4, payload, TIPP)
    write(h, header4, "uint8"); pause(TIPP);
    if ~isempty(payload), write(h, payload, "uint8"); end
end

function recv_pkt_quiet(h)
    hdr = read_exact(h,4,2); if isempty(hdr), return; end
    w   = uint32(hdr(1)) + bitshift(uint32(hdr(2)),8) + bitshift(uint32(hdr(3)),16) + bitshift(uint32(hdr(4)),24);
    N   = double(bitand(bitshift(w,-4), uint32(8191)));
    if N>0, read_exact(h,N,2); end
end

function rpm = read_reg_s32(h, HDR_GETREG, id_dec, TIPP)
    id16 = uint16(id_dec);
    idb = uint8([bitand(id16,255) bitshift(id16,-8)]);
    pay = uint8([0x11 0x00 idb idb]);
    write(h, HDR_GETREG, "uint8"); pause(TIPP); write(h, pay, "uint8");
    hdr = read_exact(h,4,2); if isempty(hdr), rpm=NaN; return; end
    w   = uint32(hdr(1)) + bitshift(uint32(hdr(2)),8) + bitshift(uint32(hdr(3)),16) + bitshift(uint32(hdr(4)),24);
    N   = double(bitand(bitshift(w,-4), uint32(8191)));
    pl  = read_exact(h,N,2); if isempty(pl) || pl(end)~=0x00, rpm=NaN; return; end
    data = pl(1:end-1);
    if numel(data)==4
        rpm = double(typecast(uint8(data), 'int32'));
    elseif numel(data)==8
        rpm = double(typecast(uint8(data(5:8)), 'int32')); % duplicated value case
    else
        rpm = NaN;
    end
end

function val = read_reg_s16(h, HDR_GETREG, id_dec, TIPP)
    id16 = uint16(id_dec);
    idb = uint8([bitand(id16,255) bitshift(id16,-8)]);
    pay = uint8([0x11 0x00 idb idb]);
    write(h, HDR_GETREG, "uint8"); pause(TIPP); write(h, pay, "uint8");
    hdr = read_exact(h,4,2); if isempty(hdr), val=NaN; return; end
    w   = uint32(hdr(1)) + bitshift(uint32(hdr(2)),8) + bitshift(uint32(hdr(3)),16) + bitshift(uint32(hdr(4)),24);
    N   = double(bitand(bitshift(w,-4), uint32(8191)));
    pl  = read_exact(h,N,2); if isempty(pl) || pl(end)~=0x00, val=NaN; return; end
    data = pl(1:end-1);
    if numel(data)>=2
        val = double(typecast(uint8(data(1:2)), 'int16'));  % take first 2 bytes
    else
        val = NaN;
    end
end

function b = read_exact(h, n, tout_s)
    t0 = tic;
    while h.NumBytesAvailable < n
        if toc(t0) > tout_s, b = []; return; end
        pause(0.01);
    end
    b = read(h, n, "uint8");
end
