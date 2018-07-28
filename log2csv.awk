#!/usr/bin/awk -F, -f
# timestamp log to (x,y). "timestamp,type,value" -> "x,y,temperature"
#  +-------------+
#  |           ^ | PS_E2W
#  | == ==>== == | PS_EAST
#  | ^           | PS_W2E
#  | == ==<== == | PS_WEST
#  |             |
#  +-------------+
#    0         XMAX
BEGIN {
    status = "WEST";
    XMAX = 10; # [m]
    YSTEP = 3; # [m] distance between light lines
    y = 0; # [m] at bottom end on start
    cnt = 0;
}

# temperature
$2 == "0" {
    data[cnt] = $3 / 100.0;
    cnt++;
}

# turn 90 degree
$2 == "12" && $3 == "90" {
    if (status == "WEST") {
        xdelta = XMAX / cnt;
        for (i = 0; i < cnt; i++) {
            printf("%f,%f,%f\n", XMAX - xdelta * i, y, data[i]);
        }

        cnt = 0;
        status = "W2E";
    } else if (status == "W2E") {
        ydelta = YSTEP / cnt;
        for (i = 0; i < cnt; i++) {
            printf("%f,%f,%f\n", 0.0, y + ydelta * i, data[i]);
        }

        cnt = 0;
        status = "EAST";
        y += YSTEP;
    }
}

# turn -90 degree
$2 == "12" && $3 == "-90" {
    if (status == "EAST") {
        xdelta = XMAX / cnt;
        for (i = 0; i < cnt; i++) {
            printf("%f,%f,%f\n", xdelta * i, y, data[i]);
        }

        cnt = 0;
        status = "E2W";
    } else if (status == "E2W") {
        ydelta = YSTEP / cnt;
        for (i = 0; i < cnt; i++) {
            printf("%f,%f,%f\n", XMAX, y + ydelta * i, data[i]);
        }

        cnt = 0;
        status = "WEST";
        y += YSTEP;
    }
}
