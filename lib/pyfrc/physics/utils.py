
def subdivide_time(tm_diff: float, timestep_ms: float):
    '''
        Given some amount of time, generate N
        
        :param tm_diff: Amount of time to divide (in seconds)
        :param timestep_ms: Step size in milliseconds
    '''
    
    timestep_ms *= 1000
    
    # split the time difference into timestep_ms steps
    total_time = int(tm_diff * 1000000)
    steps = total_time // timestep_ms
    remainder = total_time % timestep_ms
    step = timestep_ms / 1000000.0
    if remainder:
        last_step = remainder / 1000000.0
        steps += 1
    else:
        last_step = step
    
    while steps != 0:
        if steps == 1:
            yield last_step
        else:
            yield step
        steps -= 1
