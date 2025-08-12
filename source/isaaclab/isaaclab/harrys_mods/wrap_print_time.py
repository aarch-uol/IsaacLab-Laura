import builtins
from datetime import datetime

original_print = builtins.print
start_time = None

def timestamped_print(*args, **kwargs):
    global start_time
    now = datetime.now()
    if start_time is None:
        start_time = now
    elapsed = now - start_time
    timestamp = now.strftime("%H:%M:%S")
    elapsed_str = str(elapsed).split(".")[0]  # remove microseconds

    if args and isinstance(args[0], str):
        new_args = (f"[{timestamp} | +{elapsed_str}] {args[0]}",) + args[1:]
    else:
        new_args = args

    original_print(*new_args, **kwargs)

def activate():
    builtins.print = timestamped_print
