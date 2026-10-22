import sys, re, math

THRESH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05   # metros
CONSEC = int(sys.argv[2])   if len(sys.argv) > 2 else 3      # iteraciones seguidas bajo el umbral

num = r"([-+\d.e]+)"
p_iter = re.compile(r"Updating \((\d+)\)")
p_n    = re.compile(r"N=(\d+)\s+downsampling=(\d+)")
p_var  = re.compile(r"Sensor variance: %s\s+Movement variance: %s\s+Resampling variance: %s" % (num, num, num))
p_area = re.compile(r"min_x=%s\s+min_y=%s\s+min_a=%s" % (num, num, num))
p_est  = re.compile(r"Estimated pose: x=%s\s+y=%s\s+a=%s" % (num, num, num))
p_gt   = re.compile(r"Groundtruth pose: x=%s\s+y=%s\s+a=%s" % (num, num, num))

params, iteration, est = {}, 0, None
streak, start, done = 0, None, False

for line in sys.stdin:
    print(line, end="", flush=True)          # deja pasar el log normal

    m = p_n.search(line)
    if m: params["N"], params["ds"] = m.group(1), m.group(2)
    m = p_var.search(line)
    if m: params["s2s"], params["s2m"], params["s2r"] = m.groups()
    m = p_iter.search(line)
    if m: iteration = int(m.group(1))
    m = p_est.search(line)
    if m: est = tuple(map(float, m.groups()))

    m = p_gt.search(line)
    if m and est and not done:
        gt = tuple(map(float, m.groups()))
        err = math.hypot(est[0] - gt[0], est[1] - gt[1])
        err_a = abs((est[2] - gt[2] + math.pi) % (2 * math.pi) - math.pi)
        if err < THRESH:
            streak += 1
            if streak == 1:
                start = (iteration, est, gt, err, err_a)
        else:
            streak = 0
        if streak >= CONSEC:
            it, e, g, er, ea = start
            print("\n" + "=" * 60)
            print(">>> LOCALIZACION CORRECTA (error < %.0f cm durante %d iteraciones)" % (THRESH * 100, CONSEC))
            print("Parametros: N=%s  s2s=%s  s2m=%s  s2r=%s  ds=%s" % (
                params.get("N"), params.get("s2s"), params.get("s2m"), params.get("s2r"), params.get("ds")))
            print("Iteraciones para localizarse: %d" % it)
            print("Pose estimada:   x=%.3f  y=%.3f  a=%.3f" % e)
            print("Pose real:       x=%.3f  y=%.3f  a=%.3f" % g)
            print("Error posicion: %.3f m   Error angular: %.3f rad" % (er, ea))
            print("=" * 60 + "\n", flush=True)
            done = True

if not done:
    print("\n>>> No se localizo (nunca hubo %d iteraciones seguidas con error < %.0f cm)\n" % (CONSEC, THRESH * 100))