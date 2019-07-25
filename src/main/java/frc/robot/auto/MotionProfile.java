public void double[][] accelprofile(v0, v1):
    if v0 == v1:
        return [[v0, 0, 0]]
    accel = 1
    if v1 < v0:
        accel = -1

    ## Do we have time to reach amax?

    deltav = abs(v1-v0)
    T = round(sqrt(deltav/jmax))
    A = jmax*T
    if A < amax:
        t1 = int(T)
        t2 = t1
        t3 = int(2*T)

    else:
        t1 = int( ceil(amax/jmax)) 
        t2 = int( t1 + abs(v1-v0)/amax - amax/jmax) 
        t3 = t2 + t1

    t = 0
    v = v0
    a = 0
    j = 0

    trajectory = [[v,a,j]]
    jm = jmax


    state = POSJERK

    while state != END:
        t += 1
        if state == POSJERK:
            if t > t1:
                if t1 == t2:
                    state = NEGJERK
                    T = abs(a/float(jm))
                    jm = abs(a/ceil(T))

                    j = -accel * jm
                    a += j
                    v += a
                    print T, jm, jmax
                else:
                    state = NOJERK
                    j = 0
                    a = accel * amax
                    v += a
            else:
                j = accel * jm
                a += j
                if abs(a) > amax:
                    a = accel * amax
                v += a
        elif state == NOJERK:
            if t > t2:
                state = NEGJERK
                j = -accel * jm
                a += j
                v += a
            else:
                j = 0
                a += j
                v += a
        elif state == NEGJERK:
            j = -accel*jm
            a += j
            v += a
            if accel * a < 0 or accel * (abs(v)- abs(v1)) > 0:
                j = 0
                a = 0
                v = v1
                state = END

        trajectory.append([v,a,j])
    return trajectory