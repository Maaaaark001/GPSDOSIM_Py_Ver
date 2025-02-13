# Thanks to Tom Van Baak (tvb) www.LeapSecond.com/tools for the original C code.
# Maaaaark studied the ideas behind the original C code and rewrote it into a Python script to make it applicable to more environments.
# Additionally, Maaaaark provided example data and reference parameters along with the running scripts.The example data, and running scripts are open source under the LGPLv3 licence.
# As Tom Van Baak's code does not have any open source licence statement, this Python code also does not use any licence, but you are free to read or learn from it. If necessary, I would like this Python code to be open-sourced under the LGPLv3 licence.
# Assumes 1 Hz phase data rate for all files. You can use timelab or Stable32 to anaylze the data.


import sys


def path_to_double(path):
    data = []
    try:
        if path == "-":
            file = sys.stdin
        else:
            file = open(path, "r")

        for line in file:
            data.append(float(line.strip()))

        if file != sys.stdin:
            file.close()

        if len(data) < 2:
            print(f"{sys.argv[0]}: {path}: empty or too small ({len(data)})")
            sys.exit(1)

    except Exception:
        print(f"{sys.argv[0]}: read failed: {path}")
        sys.exit(1)

    return data


def detrend(data, slope, offset):
    n = len(data)
    sum_x = sum_y = 0.0
    for x in range(n):
        sum_x += x
        sum_y += data[x]
    xmean = sum_x / n
    ymean = sum_y / n
    sum_dx2 = sum_dxdy = 0.0

    for x in range(n):
        dx = x - xmean
        dy = data[x] - ymean
        sum_dx2 += dx * dx
        sum_dxdy += dx * dy

    slope[0] = sum_dxdy / sum_dx2
    offset[0] = ymean - slope[0] * xmean

    for x in range(n):
        y = slope[0] * x + offset[0]
        data[x] -= y


def pid(err, kP, kI, kD):
    static_vars = {"sum_err": 0.0, "prev_err": 0.0}
    static_vars["sum_err"] += err
    d_err = err - static_vars["prev_err"]
    static_vars["prev_err"] = err
    return (kP * err) + (kI * static_vars["sum_err"]) + (kD * d_err)


def piid(err, kP, kI, kII, kD):
    static_vars = {"sum_err": 0.0, "sum_sum_err": 0.0, "prev_err": 0.0}
    static_vars["sum_err"] += err
    static_vars["sum_sum_err"] += static_vars["sum_err"]
    d_err = err - static_vars["prev_err"]
    static_vars["prev_err"] = err
    return (
        (kP * err)
        + (kI * static_vars["sum_err"])
        + (kII * static_vars["sum_sum_err"])
        + (kD * d_err)
    )


def tic_resolution(tic, ticres):
    if ticres > 0.0:
        sign = -1 if tic < 0.0 else 1
        count = int(abs(tic) / ticres + 0.5)
        tic = sign * count * ticres
    return tic


def efc_resolution(efc, dacbits):
    if dacbits > 0.0:
        range_val = 10.0
        lsb = range_val / (2**dacbits)
        sign = -1 if efc < 0.0 else 1
        count = int(abs(efc) / lsb + 0.5)
        efc = sign * count * lsb
    return efc


def main():
    Tool = sys.argv[0]
    Verbose = 1

    def FPRINTF(msg):
        return print(msg, file=sys.stderr) if Verbose else None

    # Set parameter defaults.
    params = {
        "ver": 2,  # algorithm version
        "n": 0.0,  # sample count
        "kp": 1e-3,  # PID: proportional
        "ki": 1e-7,  # PID: integral
        "kii": 0,  # PIID: 2nd integral
        "kd": 0,  # PID: derivative
        "dt": 1.0,  # sample interval (seconds)
        "gain": 1e-7,  # OCXO EFC gain (Hz/Hz / volt)
        "avg1": 100,  # filter: GPS short-term
        "avg2": 10000,  # filter: EFC long-term
        "ticres": 0.0,  # simulated finite TIC resolution
        "dacbits": 0.0,  # simulated finite DAC resolution
    }

    if len(sys.argv) > 1 and sys.argv[1] == "/q":
        Verbose = 0
        del sys.argv[1]

    set_fmt = "** setting %s = %g\n"
    # show_fmt = "** %7s = %g\n"

    def SET_PARAM(param_name, param_value):
        params[param_name] = float(param_value)
        FPRINTF(set_fmt % (param_name, params[param_name]))

    while len(sys.argv) > 1 and "=" in sys.argv[1]:
        key, value = sys.argv[1].split("=")
        if key in params:
            SET_PARAM(key, value)
        else:
            print(f"{Tool}: unknown parameter: {sys.argv[1]}")
            sys.exit(1)
        del sys.argv[1]

    if len(sys.argv) != 3:
        print(f"Usage: {Tool} [parameter] gps.dat osc.dat > gpsdo.dat")
        sys.exit(1)

    # Sanity checks.
    if params["avg1"] < 1:
        params["avg1"] = 1
    if params["avg2"] < 1:
        params["avg2"] = 1

    # Read both phase measurement files.
    gps = path_to_double(sys.argv[1])
    FPRINTF("** GPS %d samples in file %s\n" % (len(gps), sys.argv[1]))

    osc = path_to_double(sys.argv[2])
    FPRINTF("** OSC %d samples in file %s\n" % (len(osc), sys.argv[2]))

    # If sample count unspecified, use smaller of both files.
    if params["n"] == 0.0:
        params["n"] = min(len(gps), len(osc))
    if params["n"] > len(gps) or params["n"] > len(osc):
        print(
            "%s: sample count specified (%d) exceeds data counts (%d, %d)"
            % (Tool, int(params["n"]), len(gps), len(osc))
        )
        sys.exit(1)

    # For convenience, detrend and normalize the GPS data file.
    slope = [0.0]
    offset = [0.0]
    detrend(gps[: int(params["n"])], slope, offset)
    FPRINTF("** GPS slope = %g, removed\n" % slope[0])
    FPRINTF("** GPS offset = %g, removed\n" % offset[0])

    # # Echo operating parameters.
    # def SHOW_PARAM(param_name):
    #     return FPRINTF(show_fmt % (param_name, params[param_name]))

    # SHOW_PARAM("ver")
    # SHOW_PARAM("n")
    # SHOW_PARAM("kp")
    # SHOW_PARAM("ki")
    # SHOW_PARAM("kii")
    # SHOW_PARAM("kd")
    # SHOW_PARAM("gain")
    # SHOW_PARAM("avg1")
    # SHOW_PARAM("avg2")
    # if params["ticres"] != 0.0:
    #     SHOW_PARAM("ticres")
    # if params["dacbits"] != 0.0:
    #     SHOW_PARAM("dacbits")

    # Simulate virtual TIC-based GPSDO using real OSC and GPS data, once a second.
    osc_phase = 0.0
    tic_filter = 0.0  # used to smooth GPS noise
    efc_filter = 0.0  # used to handle slow frequency drift
    efc = efc_filter
    print(f"{osc_phase:.6e}")

    for i in range(1, int(params["n"])):
        if params["ver"] == 1:
            # Simple PID.
            freq = efc * params["gain"]
            jitter = osc[i] - osc[i - 1]
            gps_phase = gps[i]
            osc_phase += (freq * params["dt"]) + jitter
            tic = (
                gps_phase - osc_phase
            )  # positive values mean oscillator is behind (slow)
            efc = pid(tic, params["kp"], params["ki"], params["kd"]) / params["gain"]

        elif params["ver"] == 2:
            # PID, with tic filter, slowly self-adjusting set point, finite resolution hack.
            freq = efc * params["gain"]
            jitter = osc[i] - osc[i - 1]
            gps_phase = gps[i]
            osc_phase += (freq * params["dt"]) + jitter
            tic = (
                gps_phase - osc_phase
            )  # positive values mean oscillator is behind (slow)
            tic = tic_resolution(tic, params["ticres"])
            tic_filter = (tic_filter * (params["avg1"] - 1) + tic) / params["avg1"]
            efc = (
                efc_filter
                + pid(tic_filter, params["kp"], params["ki"], params["kd"])
                / params["gain"]
            )
            efc_filter = (efc_filter * (params["avg2"] - 1) + efc) / params["avg2"]
            efc = efc_resolution(efc, params["dacbits"])

        elif params["ver"] == 3:
            # PIID, with tic filter, finite resolution hack.
            freq = efc * params["gain"]
            jitter = osc[i] - osc[i - 1]
            gps_phase = gps[i]
            osc_phase += (freq * params["dt"]) + jitter
            tic = (
                gps_phase - osc_phase
            )  # positive values mean oscillator is behind (slow)
            tic = tic_resolution(tic, params["ticres"])
            tic_filter = (tic_filter * (params["avg1"] - 1) + tic) / params["avg1"]
            efc = (
                piid(
                    tic_filter, params["kp"], params["ki"], params["kii"], params["kd"]
                )
                / params["gain"]
            )
            efc = efc_resolution(efc, params["dacbits"])

        else:
            print(f"{Tool}: algorithm version {params['ver']} not known")
            sys.exit(1)

        print(f"{osc_phase:.6e}")

        if abs(tic) > 1e-4:
            print(f"{Tool}: TIC exceeds 100 usec; use tighter PID parameters?")
            sys.exit(1)


if __name__ == "__main__":
    main()
