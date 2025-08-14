function spin_test_3()
% Start → speed 3000 → wait 5s → speed 4000 → stop
% Uses your captured ASPEP headers. Critical: TIPP (~1–2 ms) between header & payload.

port = "COM4";
baud = 1843200;
TIPP = 0.0015;                  % intra-packet pause (sec). Increase to 0.002–0.003 if needed.

fprintf("Opening %s @ %d...\n", port, baud);
h = serialport(port, baud, "Timeout", 2, ...
    "DataBits", 8, "Parity", "none", "StopBits", 1, "FlowControl", "none");
flush(h);

% ---------------- Handshake ---------------- 
hello_A = uint8([0x85 0xFF 0xFF 0xBF]);  % BEACON (controller)
hello_B = uint8([0x05 0xC7 0x01 0x14]);  % acceptance beacon (as in your capture)
hello_C = uint8([0x06 0x00 0x00 0x60]);  % PING (controller sets C/N/LIID=0)

write(h, hello_A, "uint8"); pause(0.05); if h.NumBytesAvailable>=4, read(h,4,"uint8"); end
write(h, hello_B, "uint8"); pause(0.05); if h.NumBytesAvailable>=4, read(h,4,"uint8"); end
write(h, hello_C, "uint8"); pause(0.05); if h.NumBytesAvailable>=4, read(h,4,"uint8"); end

% ---------------- Headers & payloads ----------------
hdr_len2      = uint8([0x29 0x00 0x00 0xE0]);                     % REQUEST header for 2-byte payload
hdr_speed_len = uint8([0xC9 0x00 0x00 0xC0]);                     % REQUEST header for speed payload

pay_start  = uint8([0x19 0x00]);                                  % START motor#1
pay_stop   = uint8([0x21 0x00]);                                  % STOP  motor#1

% speed 3000 (from Motor Pilot macro)
pay_speed_3000 = uint8([0x08 0x00 0xA9 0x01 0x06 0x00 0xA0 0x0F 0x00 0x00 0x00 0x00]);
% speed 4000 (from Motor Pilot macro)
pay_speed_4000 = uint8([0x08 0x00 0xA9 0x01 0x06 0x00 0x88 0x13 0x00 0x00 0x00 0x00]);


% --- Inject fault type = 5 ---


hdr_fault_len6   = uint8([0x69 0x00 0x00 0x00]);                 % REQUEST header for 6-byte payload
pay_fault_type_5 = uint8([0x08 0x00 0x11 0x1C 0x05 0x00]);       % your Motor Pilot macro payload

fprintf(">> SET fault type = 5 (inject)\n");
send_req(h, hdr_fault_len6, pay_fault_type_5, TIPP);
recv_pkt(h);   % prints RX len and status

pause(1.0);    % (optional) give yourself a moment to observe the faulted state


% --- Clear fault: set type = 0 ---


pay_fault_type_0 = uint8([0x08 0x00 0x11 0x1C 0x00 0x00]);       % your "disable" macro payload

fprintf(">> SET fault type = 0 (clear)\n");
send_req(h, hdr_fault_len6, pay_fault_type_0, TIPP);
recv_pkt(h);

% (optional) if your firmware latches the fault, you may also need an ACK:
% fprintf(">> FAULT_ACK\n");
% send_req(h, uint8([0x29 0x00 0x00 0xE0]), uint8([0x39 0x00]), TIPP);  % 2B header + [39 00]
% recv_pkt(h);

% ---------------- Sequence ----------------
fprintf("\n>> START motor\n");
send_req(h, hdr_len2,      pay_start,       TIPP);   recv_pkt(h);   % expect 1-byte status

fprintf(">> SET speed = 3000\n");
send_req(h, hdr_speed_len, pay_speed_3000,  TIPP);   recv_pkt(h);   % length reported in response

pause(5.0);

fprintf(">> SET speed = 4000\n");
send_req(h, hdr_speed_len, pay_speed_4000,  TIPP);   recv_pkt(h);

pause(2.0);

fprintf(">> STOP motor\n");
send_req(h, hdr_len2,      pay_stop,        TIPP);   recv_pkt(h);

fprintf("\nDone.\n");
clear h;  % release COM port
end

% ---------------- helpers ----------------
function send_req(h, header4, payload, TIPP)
    write(h, header4, "uint8");
    pause(TIPP);                              % mandatory header→payload gap
    if ~isempty(payload), write(h, payload, "uint8"); end
end

function [type,len,payload] = recv_pkt(h)
    % Read one ASPEP packet (header + payload), printing status if present.
    hdr = read(h,4,"uint8");
    w = uint32(hdr(1)) + bitshift(uint32(hdr(2)),8) + ...
        bitshift(uint32(hdr(3)),16) + bitshift(uint32(hdr(4)),24);
    type = bitand(w, uint32(15));                       % 0x0A=RESPONSE
    len  = double(bitand(bitshift(w,-4), uint32(8191)));% 13-bit payload length
    if len>0, payload = read(h, len, "uint8"); else, payload = uint8([]); end

    % Print last byte as status if present (0x00 = OK)
    if ~isempty(payload)
        fprintf("   RX len=%d, status(last)=0x%02X\n", len, payload(end));
    else
        fprintf("   RX len=%d\n", len);
    end
end
